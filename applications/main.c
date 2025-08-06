#include <rtthread.h>
#include <rtdevice.h>
#include "drv_pin.h"
#include "YS4028B12H.h"
#include <stdlib.h> // for atof()
#include <string.h> // for strcmp()

/*******************************************************************************
 * 宏定义
 ******************************************************************************/
#define LED_PIN        ((3*32)+12)
#define SAMPLE_DELAY_MS    50      // 控制周期/采样周期 (ms)，50ms即20Hz

/*******************************************************************************
 * 全局变量
 ******************************************************************************/
rt_thread_t working_indicate = RT_NULL;
/* PID 控制器参数 - 这些值现在可以通过MSH命令动态修改 */
float target_height = 250.0f; // 目标高度 (mm)
float KP = 0.005f;  // 比例系数 (Proportional gain)
float KI = 0.001f;  // 积分系数 (Integral gain)
float KD = 0.002f;  // 微分系数 (Derivative gain)

/* PID 控制器状态变量 */
float integral_error = 0.0f;
float previous_error = 0.0f;

/*******************************************************************************
 * 函数
 ******************************************************************************/

void working_led() {
    rt_pin_mode(LED_PIN, PIN_MODE_OUTPUT);  /* Set GPIO as Output */
    while (1)
    {
        rt_pin_write(LED_PIN, PIN_HIGH);    /* Set GPIO output 1 */
        rt_thread_mdelay(500);              /* Delay 500mS */
        rt_pin_write(LED_PIN, PIN_LOW);     /* Set GPIO output 0 */
        rt_thread_mdelay(500);              /* Delay 500mS */
    }
}
/**
 * @brief MSH command to tune PID parameters.
 *
 * Usage:
 * pid_tune                  - Show current PID parameters.
 * pid_tune -p <value>       - Set Kp value.
 * pid_tune -i <value>       - Set Ki value.
 * pid_tune -d <value>       - Set Kd value.
 * pid_tune -t <value>       - Set target height (mm).
 *
 * Example:
 * pid_tune -p 0.006 -i 0.0015
 */
static void pid_tune(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("Current PID Parameters:\n");
        rt_kprintf("  Target Height: %.2f mm\n", target_height);
        rt_kprintf("  Kp: %f\n", KP);
        rt_kprintf("  Ki: %f\n", KI);
        rt_kprintf("  Kd: %f\n", KD);
        rt_kprintf("\nUsage:\n");
        rt_kprintf("  pid_tune -p <val> -i <val> -d <val> -t <val>\n");
        return;
    }

    /* Parse arguments */
    for (int i = 1; i < argc; i += 2)
    {
        if (i + 1 >= argc) {
            rt_kprintf("Error: Missing value for option %s\n", argv[i]);
            return;
        }

        if (strcmp(argv[i], "-p") == 0) {
            KP = atof(argv[i+1]);
        } else if (strcmp(argv[i], "-i") == 0) {
            KI = atof(argv[i+1]);
        } else if (strcmp(argv[i], "-d") == 0) {
            KD = atof(argv[i+1]);
        } else if (strcmp(argv[i], "-t") == 0) {
            target_height = atof(argv[i+1]);
            // 当目标高度改变时，重置PID状态变量以避免旧的误差累积产生突变
            integral_error = 0.0f;
            previous_error = 0.0f;
            rt_kprintf("PID state reset due to target height change.\n");
        } else {
            rt_kprintf("Error: Unknown option %s\n", argv[i]);
            return;
        }
    }

    rt_kprintf("PID parameters updated.\n");
    // 打印更新后的值
    pid_tune(1, RT_NULL);
}
MSH_CMD_EXPORT(pid_tune, Tune PID parameters for ball suspension);


int main(void)
{
#if defined(__CC_ARM)
    rt_kprintf("using armcc, version: %d\n", __ARMCC_VERSION);
#elif defined(__clang__)
    rt_kprintf("using armclang, version: %d\n", __ARMCC_VERSION);
#elif defined(__ICCARM__)
    rt_kprintf("using iccarm, version: %d\n", __VER__);
#elif defined(__GNUC__)
    rt_kprintf("using gcc, version: %d.%d\n", __GNUC__, __GNUC_MINOR__);
#endif

    rt_kprintf("MCXA156 Ball Suspension Demo\r\n");

    /* 启动工作指示灯线程 */
    working_indicate = rt_thread_create("WorkingIndicate", working_led, RT_NULL, 1024, 10, 20);
    if (working_indicate != RT_NULL) {
        rt_thread_startup(working_indicate);
    } else {
        rt_kprintf("Failed to create working indicate thread.\n");
    }

    /* 初始化风扇 */
    ys4028b12h_cfg_t cfg = &my_ys4028b12h_config;
    if (cfg->name == RT_NULL) {
        ys4028b12h_init(cfg);
    }
    ys4028b12h_set_speed(cfg, 0.0f);
    rt_kprintf("Fan initialized.\n");

    /* 初始化VL53L0X ToF传感器 */
    rt_device_t tof_dev = rt_device_find("tof_vl53l0x");
    if (tof_dev == RT_NULL)
    {
        rt_kprintf("Error: not found tof_vl53l0x device\r\n");
        return -1;
    }

    if (rt_device_open(tof_dev, RT_DEVICE_FLAG_RDONLY) != RT_EOK)
    {
        rt_kprintf("Error: open tof_vl53l0x failed\r\n");
        return -1;
    }
    rt_kprintf("VL53L0X device opened. Initial target height: %.1f mm\n", target_height);
    rt_kprintf("Type 'pid_tune' in MSH for parameter tuning.\n");

    /* 主控制循环 */
    while (1)
    {
        struct rt_sensor_data sensor_data;
        rt_size_t res = rt_device_read(tof_dev, 0, &sensor_data, 1);

        if (res != 1)
        {
            rt_kprintf("Warning: read vl53l0x data failed! res: %d\n", res);
            ys4028b12h_set_speed(cfg, 0.0f);
            rt_thread_mdelay(500);
            continue;
        }

        int32_t current_height = sensor_data.data.proximity;

        if (current_height > 8000) {
            if (previous_error != 9999) { // 避免重复打印
                 rt_kprintf("Ball not detected. Turning off fan.\n");
            }
            ys4028b12h_set_speed(cfg, 0.0f);
            integral_error = 0;
            previous_error = 9999; // 使用一个特殊值标记为未检测到
            rt_thread_mdelay(SAMPLE_DELAY_MS);
            continue;
        }

        /* --- PID 控制器核心计算 --- */
        float error = target_height - (float)current_height;
        integral_error += error;
        float derivative_error = error - previous_error;
        // 如果是从“未检测到”状态恢复，则不计算微分项，避免突变
        if (previous_error == 9999) {
            derivative_error = 0;
        }
        float pid_output = (KP * error) + (KI * integral_error) + (KD * derivative_error);
        previous_error = error;

        /* --- 输出限幅与应用 --- */
        float fan_speed = pid_output;
        if (fan_speed > 1.0f) {
            fan_speed = 1.0f;
            // 积分抗饱和：当输出饱和时，阻止积分项继续增长
            integral_error -= error;
        }
        if (fan_speed < 0.0f) {
            fan_speed = 0.0f;
             // 积分抗饱和：当输出饱和时，阻止积分项继续减小
            integral_error -= error;
        }
        ys4028b12h_set_speed(cfg, fan_speed);

        /* 打印调试信息 (可以考虑使用一个MSH命令来开关此打印，避免刷屏) */
        // rt_kprintf("H:%4d, E:%6.1f, P:%5.2f, I:%5.2f, D:%5.2f, Spd:%.2f\r\n",
        //           current_height, error, (KP * error), (KI * integral_error), (KD * derivative_error), fan_speed);

        rt_thread_mdelay(SAMPLE_DELAY_MS);
    }

    rt_device_close(tof_dev);
    return 0;
}
