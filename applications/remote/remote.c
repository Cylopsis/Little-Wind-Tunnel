#include <rtthread.h>
#include <sys/socket.h>
#include <netdb.h>
#include <string.h>
#include <sys/errno.h>
#include <stdio.h>
#include "system_vars.h"

#define SERVER_PORT     5000    // 服务器监听的端口
#define RECV_BUFSZ      128     // 接收缓冲区大小
#define SEND_BUFSZ      512     // 发送缓冲区大小
#define MAX_ARGS        8       // 命令行参数最大数量

// 声明在main.c中定义的函数
extern float get_feedforward_speed(float target_height);
// 将浮点数转为字符串，newlib-nano中的sprintf不支持浮点数格式化!!!
void float_to_string(char* buffer, float value, int precision) {
    long integer_part = (long)value;
    long fractional_part = (long)((value - integer_part) * pow(10, precision));
    if (fractional_part < 0) {
        fractional_part = -fractional_part;
    }
    sprintf(buffer, "%ld.%0*ld", integer_part, precision, fractional_part);
}

static rt_thread_t server_thread = RT_NULL;

/**
 * @brief TCP服务器线程入口函数
 * @param parameter 线程参数 (未使用)
 */
static void remote_server_thread_entry(void *parameter)
{
    int sock, connected;
    struct sockaddr_in server_addr, client_addr;
    socklen_t sin_size;
    
    char *recv_buf = RT_NULL;
    char *send_buf = RT_NULL;
    char *argv[MAX_ARGS]; // 用于存放分割后的命令参数指针
    int argc;

    // 分配接收和发送缓冲区
    recv_buf = rt_malloc(RECV_BUFSZ);
    if (recv_buf == RT_NULL)
    {
        rt_kprintf("[Remote] No memory for recv_buf\n");
        return;
    }
    send_buf = rt_malloc(SEND_BUFSZ);
    if (send_buf == RT_NULL)
    {
        rt_kprintf("[Remote] No memory for send_buf\n");
        rt_free(recv_buf);
        return;
    }

    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) == -1)
    {
        rt_kprintf("[Remote] Socket error\n");
        goto __exit;
    }

    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SERVER_PORT);
    server_addr.sin_addr.s_addr = INADDR_ANY;
    rt_memset(&(server_addr.sin_zero), 0, sizeof(server_addr.sin_zero));

    if (bind(sock, (struct sockaddr *)&server_addr, sizeof(struct sockaddr)) == -1)
    {
        rt_kprintf("[Remote] Unable to bind\n");
        goto __exit;
    }

    if (listen(sock, 5) == -1)
    {
        rt_kprintf("[Remote] Listen error\n");
        goto __exit;
    }

    rt_kprintf("[Remote] TCP Server waiting for client on port %d...\n", SERVER_PORT);

    while (1)
    {
        sin_size = sizeof(struct sockaddr_in);
        
        // 接受客户端连接 (阻塞)
        connected = accept(sock, (struct sockaddr *)&client_addr, &sin_size);
        if (connected < 0)
        {
            rt_kprintf("[Remote] Accept connection failed! errno = %d\n", errno);
            continue;
        }
        rt_kprintf("[Remote] Got a connection from (%s, %d)\n", inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));

        // 与客户端交互循环
        while (1)
        {
            int bytes_received = recv(connected, recv_buf, RECV_BUFSZ - 1, 0);
            if (bytes_received <= 0)
            {
                rt_kprintf("[Remote] Client disconnected or recv error.\n");
                closesocket(connected);
                break;
            }

            recv_buf[bytes_received] = '\0';
            char* p = strpbrk(recv_buf, "\r\n");
            if (p) *p = '\0'; // 去掉换行符
            // for (int i = 0; i < bytes_received; i++) {
            //     if (recv_buf[i] == '\r' || recv_buf[i] == '\n') {
            //         recv_buf[i] = '\0';
            //     }
            // }
            // rt_kprintf("[Remote] Received command: '%s'\n", recv_buf);
            argc = 0;
            char *ptr = strtok(recv_buf, " ");
            while (ptr != NULL && argc < MAX_ARGS) {
                argv[argc++] = ptr;
                ptr = strtok(NULL, " ");
            }
 
            if (argc == 0) {
                continue; // 空命令
            }
 
            // --- 根据第一个参数分发命令 ---
            if (strcmp(argv[0], "get_status") == 0)
            {
                // 格式化JSON字符串
                // 为每个浮点数创建临时缓冲区，fk nano优化
                char target_h_str[20], ramped_h_str[20];
                char kp_str[20], ki_str[20], kd_str[20];
                char integral_err_str[20], prev_err_str[20], ff_speed_str[20], total_err_str[20];
            
                // I have to 手动转换所有浮点数 (
                float_to_string(target_h_str, target_height, 1);
                float_to_string(ramped_h_str, ramped_height, 1);
                float_to_string(kp_str, KP, 6);
                float_to_string(ki_str, KI, 6);
                float_to_string(kd_str, KD, 6);
                float_to_string(integral_err_str, integral_error, 4);
                float_to_string(prev_err_str, previous_error, 4);
                float_to_string(ff_speed_str, get_feedforward_speed(ramped_height), 4);
                float_to_string(total_err_str, total_abs_error, 4);
            
                sprintf(send_buf, "{"
                    "\"current_height\":%d,"
                    "\"target_height\":%s,"
                    "\"ramped_height\":%s,"
                    "\"pid_kp\":%s,"
                    "\"pid_ki\":%s,"
                    "\"pid_kd\":%s,"
                    "\"integral_error\":%s,"
                    "\"previous_error\":%s,"
                    "\"feedforward_speed\":%s,"
                    "\"is_evaluating\":%s,"
                    "\"total_abs_error\":%s"
                    "}\r\n",
                    current_height,
                    target_h_str, ramped_h_str,
                    kp_str, ki_str, kd_str,
                    integral_err_str, prev_err_str,
                    ff_speed_str,
                    is_evaluating ? "true" : "false",
                    total_err_str);
                // 发送响应
                if (send(connected, send_buf, strlen(send_buf), 0) < 0) {
                    rt_kprintf("[Remote] Send response failed.\n");
                    closesocket(connected);
                    break;
                }
            }
            else if (strcmp(argv[0], "pid_tune") == 0)
            {         
                pid_tune(argc, argv);
 
                char ok_msg[100];
                sprintf(ok_msg, "OK: '%s' command executed.\r\n", argv);
                send(connected, ok_msg, strlen(ok_msg), 0);
            }
            else
            {
                // 对于未知命令，发送错误信息
                sprintf(send_buf, "ERROR: Unknown command '%s'.\r\n", argv[0]);
                send(connected, send_buf, strlen(send_buf), 0);
            }
        }
    }

__exit:
    if (sock >= 0) closesocket(sock);
    if (recv_buf) rt_free(recv_buf);
    if (send_buf) rt_free(send_buf);
    rt_kprintf("[Remote] Server thread exited.\n");
}

/**
 * @brief MSH命令，用于启动远程控制服务器线程
 */
static void remote_start(int argc, char **argv)
{
    if (server_thread != RT_NULL)
    {
        rt_kprintf("[Remote] Server is already running.\n");
        return;
    }

    server_thread = rt_thread_create("RemoteTCPSrv",
                                     remote_server_thread_entry,
                                     RT_NULL,
                                     2048, // 增加栈空间以应对网络操作
                                     12,
                                     20);

    if (server_thread != RT_NULL)
    {
        rt_thread_startup(server_thread);
        rt_kprintf("[Remote] TCP server started successfully.\n");
    }
    else
    {
        rt_kprintf("[Remote] Failed to create TCP server thread.\n");
    }
}
MSH_CMD_EXPORT(remote_start, Start the remote control TCP server);

