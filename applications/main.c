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
#define SAMPLE_DELAY_MS    20      // 控制周期/采样周期 (ms)
#define RAMP_STEP          1.0f    // 值斜坡步长 (mm/cycle)

#define FABS(x) ((x) > 0 ? (x) : -(x))

/*******************************************************************************
 * 全局变量
 ******************************************************************************/
rt_thread_t working_indicate = RT_NULL;

/* PID 控制器参数 */
float final_target_height = 250.0f; // 用户设定的最终目标高度 (mm)
float ramped_target_height = 250.0f; // PID控制器当前正在追踪的、平滑变化的目标高度

float KP = 0.0f;  // 比例系数 (Proportional gain)
float KI = 0.0f;  // 积分系数 (Integral gain)
float KD = 0.0f;  // 微分系数 (Derivative gain)

/* PID 控制器状态变量 */
float integral_error = 0.0f;
float previous_error = 0.0f;

/* PID 控制参数评估 */
rt_bool_t is_evaluating = RT_FALSE;
float total_abs_error = 0.0f;

// 存储每个工作点的PID参数
typedef struct {
    float height;
    float kp;
    float ki;
    float kd;
} pid_profile_t;

// 查找表
const pid_profile_t gain_schedule_table[] = {
    {50.0f,  0.02698591f, 0.0f,          0.01347117f},
    {100.0f, 0.00238823f, 0.00018420f,   0.01759602f},
    {150.0f, 0.00150062f, 0.00013645f,   0.01424987f},
    {200.0f, 0.00273804f, 0.00005707f,   0.01384286f},
    {250.0f, 0.00215347f, 0.00010309f,   0.00857251f},
    {300.0f, 0.00245299f, 0.00008689f,   0.01209851f},
    {350.0f, 0.00255815f, 0.00026372f,   0.01642789f},
    {400.0f, 0.00308520f, 0.00019811f,   0.01500000f},
    {450.0f, 0.00277328f, 0.00019800f,   0.01144115f}
};
const int num_profiles = sizeof(gain_schedule_table) / sizeof(gain_schedule_table[0]);

/*******************************************************************************
 * 函数
 ******************************************************************************/
void update_pid_gains_by_target(float current_target);

void working_led() {
    rt_pin_mode(LED_PIN, PIN_MODE_OUTPUT);
    while (1)
    {
        rt_pin_write(LED_PIN, PIN_HIGH);
        rt_thread_mdelay(500);
        rt_pin_write(LED_PIN, PIN_LOW);
        rt_thread_mdelay(500);
    }
}

static void pid_tune(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("Current PID Parameters:\n");
        rt_kprintf("  Final Target Height: %.2f mm\n", final_target_height);
        rt_kprintf("  Ramped Target Height: %.2f mm\n", ramped_target_height);
        rt_kprintf("  Kp: %f (Proportional)\n", KP);
        rt_kprintf("  Ki: %f (Integral)\n", KI);
        rt_kprintf("  Kd: %f (Derivative)\n", KD);
        rt_kprintf("\nUsage:\n");
        rt_kprintf("  pid_tune -t <val>       (Set final target height)\n");
        rt_kprintf("  pid_tune -p <val> -i <val> -d <val> (Manual override)\n");
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
            rt_kprintf("Manual override: Kp set.\n");
        } else if (strcmp(argv[i], "-i") == 0) {
            KI = atof(argv[i+1]);
            rt_kprintf("Manual override: Ki set.\n");
        } else if (strcmp(argv[i], "-d") == 0) {
            KD = atof(argv[i+1]);
            rt_kprintf("Manual override: Kd set.\n");
        } else if (strcmp(argv[i], "-t") == 0) {
            final_target_height = atof(argv[i+1]);
            rt_kprintf("Final target height set to %.2f mm.\n", final_target_height);
        } else {
            rt_kprintf("Error: Unknown option %s\n", argv[i]);
            return;
        }
    }
}
MSH_CMD_EXPORT(pid_tune, Tune PID parameters for ball suspension);


void update_pid_gains_by_target(float current_target)
{
    int lower_index = -1, upper_index = -1;

    for (int i = 0; i < num_profiles; i++) {
        if (gain_schedule_table[i].height <= current_target) {
            lower_index = i;
        }
        if (gain_schedule_table[i].height >= current_target && upper_index == -1) {
            upper_index = i;
        }
    }

    if (lower_index == -1) {
        KP = gain_schedule_table[0].kp;
        KI = gain_schedule_table[0].ki;
        KD = gain_schedule_table[0].kd;
    } else if (upper_index == -1 || lower_index == upper_index) {
        KP = gain_schedule_table[lower_index].kp;
        KI = gain_schedule_table[lower_index].ki;
        KD = gain_schedule_table[lower_index].kd;
    } else {
        const pid_profile_t* p_lower = &gain_schedule_table[lower_index];
        const pid_profile_t* p_upper = &gain_schedule_table[upper_index];

        float ratio = (current_target - p_lower->height) / (p_upper->height - p_lower->height);

        KP = p_lower->kp + ratio * (p_upper->kp - p_lower->kp);
        KI = p_lower->ki + ratio * (p_upper->ki - p_lower->ki);
        KD = p_lower->kd + ratio * (p_upper->kd - p_lower->kd);
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
    ys4028b12h_set_speed(cfg, 0.0f);
    rt_kprintf("Fan initialized.\n");

    /* 初始化VL53L0X ToF传感器 */
    rt_device_t tof_dev = rt_device_find("tof_vl53l0x");
    if (tof_dev == RT_NULL) { 
        rt_kprintf("Error: VL53L0X device not found!\n");
        return -1; 
    }
    if (rt_device_open(tof_dev, RT_DEVICE_FLAG_RDONLY) != RT_EOK) {
        rt_kprintf("Error: Failed to open VL53L0X device!\n");
        return -1;
    }

    rt_kprintf("VL53L0X device opened.\n");
    rt_kprintf("Initializing PID for default target: %.1f mm\n", final_target_height);
    update_pid_gains_by_target(ramped_target_height);
    pid_tune(1, RT_NULL);


    /* 主控制循环 */
    while (1)
    {
        struct rt_sensor_data sensor_data;
        rt_size_t res = rt_device_read(tof_dev, 0, &sensor_data, 1);

        if (res != 1) {
            rt_kprintf("Warning: read vl53l0x data failed! res: %d\n", res);
            ys4028b12h_set_speed(cfg, 0.0f);
            rt_thread_mdelay(500);
            continue;
        }

        int32_t current_height = sensor_data.data.proximity;

        if (current_height > 8000) {
            if (previous_error != 9999) {
                 rt_kprintf("Ball not detected. Turning off fan.\n");
            }
            ys4028b12h_set_speed(cfg, 0.0f);
            integral_error = 0;
            previous_error = 9999;
            rt_thread_mdelay(SAMPLE_DELAY_MS);
            continue;
        }

        /* --- 设定值斜坡 --- */
        if (FABS(ramped_target_height - final_target_height) > RAMP_STEP)
        {
            if (ramped_target_height < final_target_height) {
                ramped_target_height += RAMP_STEP;
            } else {
                ramped_target_height -= RAMP_STEP;
            }
            // 在斜坡过程中，持续更新PID参数
            update_pid_gains_by_target(ramped_target_height);
        }
        else
        {
            ramped_target_height = final_target_height;
        }

        /* --- PID 控制器核心计算 --- */
        float error = ramped_target_height - (float)current_height;
        
        if (is_evaluating)
        {
            total_abs_error += FABS(error); // 使用宏计算绝对误差
        }

        integral_error += error;
        float derivative_error = error - previous_error;
        if (previous_error == 9999) {
            derivative_error = 0;
        }
        float pid_output = (KP * error) + (KI * integral_error) + (KD * derivative_error);
        previous_error = error;

        /* --- 输出限幅与应用 (带积分抗饱和) --- */
        float fan_speed = pid_output;
        if (fan_speed > 1.0f) {
            fan_speed = 1.0f;
            integral_error -= error;
        }
        if (fan_speed < 0.0f) {
            fan_speed = 0.0f;
            integral_error -= error;
        }
        ys4028b12h_set_speed(cfg, fan_speed);

        rt_thread_mdelay(SAMPLE_DELAY_MS);
    }

    rt_device_close(tof_dev);
    return 0;
}

void pid_eval(int argc, char **argv)
{
    if (argc != 2) {
        rt_kprintf("Usage: pid_eval <duration_ms>\n");
        return;
    }
 
    rt_uint32_t duration = atoi(argv[1]);
    rt_kprintf("Starting evaluation for %d ms...\n", duration);
 
    integral_error = 0;
    previous_error = 0;
    total_abs_error = 0;
    is_evaluating = RT_TRUE;
 
    rt_thread_mdelay(duration);
 
    is_evaluating = RT_FALSE;
    rt_kprintf("EVAL_RESULT:%f\n", total_abs_error);
}
MSH_CMD_EXPORT(pid_eval, Evaluate current PID performance);
