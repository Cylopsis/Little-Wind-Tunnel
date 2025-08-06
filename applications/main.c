#include <rtdevice.h>
#include "drv_pin.h"
#include "YS4028B12H.h"

/*******************************************************************************
 * 宏定义
 ******************************************************************************/
#define LED_PIN        ((3*32)+12)

/* PID 控制器参数 - !!! 这些值需要根据实际情况进行调试 !!! */
#define KP                 0.005f  // 比例系数 (Proportional gain)
#define KI                 0.001f  // 积分系数 (Integral gain)
#define KD                 0.002f  // 微分系数 (Derivative gain)
#define SAMPLE_DELAY_MS    50      // 控制周期/采样周期 (ms)，50ms即20Hz

/*******************************************************************************
 * 全局变量
 ******************************************************************************/
rt_thread_t working_indicate = RT_NULL;
float target_height = 250.0f; // 目标高度 (mm)

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
    // 先将风扇速度设置为0
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
    rt_kprintf("VL53L0X device opened. Target height: %.1f mm\n", target_height);

    /* 主控制循环 */
    while (1)
    {
        struct rt_sensor_data sensor_data;
        rt_size_t res = rt_device_read(tof_dev, 0, &sensor_data, 1);

        if (res != 1)
        {
            rt_kprintf("Warning: read vl53l0x data failed! res: %d\n", res);
            // 读取失败时，关闭风扇以策安全
            ys4028b12h_set_speed(cfg, 0.0f);
            rt_thread_mdelay(500); // 等待一段时间再重试
            continue;
        }

        // 获取当前高度 (mm)。VL53L0X超出范围时可能返回8190或8191
        int32_t current_height = sensor_data.data.proximity;

        // 如果没有检测到物体或物体太远，关闭风扇
        if (current_height > 8000) {
            rt_kprintf("Ball not detected. Turning off fan.\n");
            ys4028b12h_set_speed(cfg, 0.0f);
            integral_error = 0; // 重置积分项
            previous_error = 0; // 重置微分项
            rt_thread_mdelay(SAMPLE_DELAY_MS);
            continue;
        }

        /* --- PID 控制器核心计算 --- */

        // 1. 计算误差
        float error = target_height - (float)current_height;

        // 2. 计算积分项 (带抗积分饱和)
        // 只有在风扇未达到饱和状态时才累加积分，防止积分无限制增长（积分饱和）
        // 这里简化处理，可以后续添加更精细的抗饱和逻辑
        integral_error += error;

        // 3. 计算微分项
        float derivative_error = error - previous_error;

        // 4. 计算PID总输出
        float pid_output = (KP * error) + (KI * integral_error) + (KD * derivative_error);

        // 5. 更新上一次的误差
        previous_error = error;

        // 6. 将PID输出转换为风扇速度 (0.0f to 1.0f)
        float fan_speed = pid_output;
        if (fan_speed > 1.0f) {
            fan_speed = 1.0f;
        }
        if (fan_speed < 0.0f) {
            fan_speed = 0.0f;
        }
        ys4028b12h_set_speed(cfg, fan_speed);

        // 打印调试信息
        rt_kprintf("H: %4dmm, E: %6.1f, P: %5.2f, I: %5.2f, D: %5.2f, Speed: %.2f\r\n",
                   current_height, error, (KP * error), (KI * integral_error), (KD * derivative_error), fan_speed);

        // 等待下一个控制周期
        rt_thread_mdelay(SAMPLE_DELAY_MS);
    }

    // 理论上不会执行到这里
    rt_device_close(tof_dev);
    return 0;
}
