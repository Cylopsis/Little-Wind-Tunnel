#include <rtthread.h>
#include <rtdevice.h>
#include "drv_pin.h"
#include "YS4028B12H.h"
#include <stdlib.h> // for atof()
#include <string.h> // for strcmp()
#include <system_vars.h>

/*******************************************************************************
 * 宏定义
 ******************************************************************************/
#define LED_PIN        ((3*32)+12)      // 工作指示灯引脚
#define SAMPLE_DELAY_MS    20           // 控制周期/采样周期 (ms)
#define RAMP_STEP          0.5f         // 设定值斜坡步长 (mm/cycle)
#define MIN_HEIGHT        100.0f        // 最小高度 (mm)
#define FABS(x) ((x) > 0 ? (x) : -(x))  // 绝对值宏

/*******************************************************************************
 * 全局变量
 ******************************************************************************/
rt_thread_t working_indicate = RT_NULL;
// rt_thread_t remote_thread = RT_NULL;

/* PID 控制器参数 */
int32_t current_height = 20;  // 当前高度 (mm)
float target_height = 250.0f; // 用户设定的最终目标高度 (mm)
float ramped_height = 250.0f; // PID控制器当前正在追踪的、平滑变化的目标高度

float KP = 0.0f;
float KI = 0.0f;
float KD = 0.0f;

/* PID 控制器状态变量 */
float integral_error = 0.0f;
float previous_error = 0.0f;

/* PID 控制参数评估 */
static void pid_tune(int argc, char **argv);
rt_bool_t is_evaluating = RT_FALSE;
float total_abs_error = 0.0f;
typedef struct {
    float height;
    float kp;
    float ki;
    float kd;
} pid_profile_t;

/* --- PID增益调度表 --- PS：每个高度点只运行了30s,60个迭代的贝叶斯算法，还可以再优化优化 */
const pid_profile_t gain_schedule_table[] = {
    // {高度, Kp, Ki, Kd}
    {50.0f, 0.03042419f, 0.00000174f, 0.00651698f}, // Best Score: 2148.0
    {100.0f, 0.00199807f, 0.00028206f, 0.01910421f}, // Best Score: 2012.0
    {150.0f, 0.00150062f, 0.00013645f, 0.01424987f},
    {200.0f, 0.00108072f, 0.00004870f, 0.00508059f}, // Best Score: 5962.0
    {250.0f, 0.00188518f, 0.00010055f, 0.00835555f}, // Best Score: 8428.0
    {300.0f, 0.00290016f, 0.00010323f, 0.01117709f}, // Best Score: 6893.0
    {350.0f, 0.00369505f, 0.00012367f, 0.02000000f}, // Best Score: 4952.0
    {400.0f, 0.00485637f, 0.00022979f, 0.01500000f}, // Best Score: 7465.0
    {450.0f, 0.00284322f, 0.00024262f, 0.01463256f}, // Best Score: 7254.0
};
const int num_pid_profiles = sizeof(gain_schedule_table) / sizeof(gain_schedule_table[0]);

typedef struct {
    float height;
    float base_fan_speed;
} feedforward_profile_t;
/* --- 前馈速度表 --- PS:0.35这个速度是可以让小球缓慢上升的速度，这个风扇的硬件条件就找不到一个能让小球悬停的速度 */
feedforward_profile_t ff_table[] = {
    {50.0f,  0.35f},
    {100.0f, 0.35f},
    {150.0f, 0.35f},
    {200.0f, 0.35f},
    {250.0f, 0.3f},
    {300.0f, 0.3f},
    {350.0f, 0.3f},
    {400.0f, 0.3f},
    {450.0f, 0.3f}
};
const int num_ff_profiles = sizeof(ff_table) / sizeof(ff_table[0]);

/*******************************************************************************
 * 函数
 ******************************************************************************/
void update_pid_gains_by_target(float current_target);
float get_feedforward_speed(float target_height); // 前馈计算
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

void update_pid_gains_by_target(float current_target)
{
    int lower_index = -1, upper_index = -1;
    for (int i = 0; i < num_pid_profiles; i++) {
        if (gain_schedule_table[i].height <= current_target) lower_index = i;
        if (gain_schedule_table[i].height >= current_target && upper_index == -1) upper_index = i;
    }

    if (lower_index == -1) { // 低于范围
        KP = gain_schedule_table[0].kp; KI = gain_schedule_table[0].ki; KD = gain_schedule_table[0].kd;
    } else if (upper_index == -1 || lower_index == upper_index) { // 高于范围或精确匹配
        KP = gain_schedule_table[lower_index].kp; KI = gain_schedule_table[lower_index].ki; KD = gain_schedule_table[lower_index].kd;
    } else { // 插值
        const pid_profile_t* p_lower = &gain_schedule_table[lower_index];
        const pid_profile_t* p_upper = &gain_schedule_table[upper_index];
        float ratio = (current_target - p_lower->height) / (p_upper->height - p_lower->height);
        KP = p_lower->kp + ratio * (p_upper->kp - p_lower->kp);
        KI = p_lower->ki + ratio * (p_upper->ki - p_lower->ki);
        KD = p_lower->kd + ratio * (p_upper->kd - p_lower->kd);
    }
}

float get_feedforward_speed(float target_height)
{
    int lower_index = -1, upper_index = -1;
    for (int i = 0; i < num_ff_profiles; i++) {
        if (ff_table[i].height <= target_height) lower_index = i;
        if (ff_table[i].height >= target_height && upper_index == -1) upper_index = i;
    }

    if (lower_index == -1) {
        return ff_table[0].base_fan_speed;
    } else if (upper_index == -1 || lower_index == upper_index) {
        return ff_table[lower_index].base_fan_speed;
    } else {
        const feedforward_profile_t* p_lower = &ff_table[lower_index];
        const feedforward_profile_t* p_upper = &ff_table[upper_index];
        if (p_upper->height == p_lower->height) return p_lower->base_fan_speed;
        float ratio = (target_height - p_lower->height) / (p_upper->height - p_lower->height);
        return p_lower->base_fan_speed + ratio * (p_upper->base_fan_speed - p_lower->base_fan_speed);
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
    working_indicate = rt_thread_create("WorkingIndicate", working_led, RT_NULL, 256, 11, 20);
    // remote_thread = rt_thread_create("RemoteComm", remote_main, RT_NULL, 1024, 11, 20);
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
    
    rt_kprintf("Initializing PID and Feedforward for default target: %.1f mm\n", target_height);
    update_pid_gains_by_target(ramped_height);
    pid_tune(1, RT_NULL);

    /* 主控制循环 */
    while (1)
    {
        struct rt_sensor_data sensor_data;
        rt_size_t res = rt_device_read(tof_dev, 0, &sensor_data, 1);
        if (res != 1) { rt_kprintf("Error: Failed to read sensor data!\n"); continue; }
        current_height = sensor_data.data.proximity;
        if (current_height > 8000) { rt_kprintf("Warning: Height exceeds 8000\n"); continue; }

        /* --- 设定值斜坡 --- PS：这里会有0.5的误差，懒得调了 */
        if (FABS(ramped_height - target_height) > RAMP_STEP) {
            if (ramped_height < target_height) ramped_height += RAMP_STEP;
            else ramped_height -= RAMP_STEP;
            update_pid_gains_by_target(ramped_height);
        } else {
            ramped_height = target_height;
        }

        /* --- PID 控制器核心计算 --- */
        float error = ramped_height - (float)current_height;
        if (is_evaluating) { total_abs_error += FABS(error); }

        integral_error += error;
        float derivative_error = error - previous_error;
        if (previous_error == 9999) derivative_error = 0;
        float pid_output = (KP * error) + (KI * integral_error) + (KD * derivative_error);
        previous_error = error;

        /* --- 前馈与PID输出合并 --- */
        float ff_speed = get_feedforward_speed(ramped_height);
        float final_fan_speed = ff_speed + pid_output;

        /* --- 输出限幅与积分抗饱和 --- */
        if (final_fan_speed > 1.0f) {
            final_fan_speed = 1.0f;
            integral_error -= error; // 抗饱和
        }
        if (final_fan_speed < 0.0f) {
            final_fan_speed = 0.0f;
            integral_error -= error; // 抗饱和
        }
        ys4028b12h_set_speed(cfg, final_fan_speed);

        rt_thread_mdelay(SAMPLE_DELAY_MS);
    }

    rt_device_close(tof_dev);
    return 0;
}

/*******************************************************************************
 * 评测用
 ******************************************************************************/

static void pid_tune(int argc, char **argv)
{
    if (argc < 2) {
        rt_kprintf("--- PID & Feedforward Status ---\n");
        rt_kprintf("  Final Target: %.2f mm, Ramped Target: %.2f mm\n", target_height, ramped_height);
        rt_kprintf("  Kp: %f, Ki: %f, Kd: %f\n", KP, KI, KD);
        rt_kprintf("\n--- Usage ---\n");
        rt_kprintf("  pid_tune -t <val>                    (Set target height)\n");
        rt_kprintf("  pid_tune -p <val> -i <val> -d <val>  (Manual PID override)\n");
        rt_kprintf("  pid_tune -ff                         (Show Feedforward table)\n");
        rt_kprintf("  pid_tune -ff_set <idx> <h> <spd>     (Set Feedforward entry)\n");
        rt_kprintf("    e.g., pid_tune -ff_set 2 100.0 0.45\n");
        return;
    }

    if (strcmp(argv[1], "-ff") == 0) {
        rt_kprintf("--- Feedforward Table ---\n");
        rt_kprintf("Idx | Height (mm) | Base Speed\n");
        rt_kprintf("----|-------------|-----------\n");
        for (int i = 0; i < num_ff_profiles; i++) {
            rt_kprintf("%-3d | %-11.1f | %.4f\n", i, ff_table[i].height, ff_table[i].base_fan_speed);
        }
        return;
    }

    if (strcmp(argv[1], "-ff_set") == 0) {
        if (argc != 5) {
            rt_kprintf("Error: Incorrect arguments for -ff_set.\n");
            rt_kprintf("Usage: pid_tune -ff_set <index> <height> <speed>\n");
            return;
        }
        int index = atoi(argv[2]);
        if (index < 0 || index >= num_ff_profiles) {
            rt_kprintf("Error: Index %d is out of bounds (0-%d).\n", index, num_ff_profiles - 1);
            return;
        }
        ff_table[index].height = atof(argv[3]);
        ff_table[index].base_fan_speed = atof(argv[4]);
        rt_kprintf("Feedforward table entry %d updated to: Height=%.1f, Speed=%.4f\n",
                   index, ff_table[index].height, ff_table[index].base_fan_speed);
        return;
    }

    for (int i = 1; i < argc; i += 2) {
        if (strcmp(argv[i], "-p") == 0) { KP = atof(argv[i+1]); }
        else if (strcmp(argv[i], "-i") == 0) { KI = atof(argv[i+1]); }
        else if (strcmp(argv[i], "-d") == 0) { KD = atof(argv[i+1]); }
        else if (strcmp(argv[i], "-t") == 0) {
            target_height = atof(argv[i+1]);
            if (target_height < MIN_HEIGHT) {
                rt_kprintf("Warning: Target height is below minimum (%f mm). Clamping to %f mm.\n", target_height, MIN_HEIGHT);
                target_height = MIN_HEIGHT;
            }
        }
        else { rt_kprintf("Error: Unknown option %s\n", argv[i]); return; }
    }
    rt_kprintf("Parameters updated. Current status:\n");
    pid_tune(1, RT_NULL); // 显示更新后的状态
}
MSH_CMD_EXPORT(pid_tune, Tune PID and Feedforward parameters);

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

static void get_status(int argc, char **argv)
{
    rt_kprintf("--- System Status ---\n");
    rt_kprintf("Current Height: %d mm\n", current_height);
    rt_kprintf("Final Target Height: %.2f mm\n", target_height);
    rt_kprintf("Ramped Target Height: %.2f mm\n", ramped_height);
    rt_kprintf("PID Gains: Kp=%.6f, Ki=%.6f, Kd=%.6f\n", KP, KI, KD);
    rt_kprintf("Integral Error: %.4f\n", integral_error);
    rt_kprintf("Previous Error: %.4f\n", previous_error);
    rt_kprintf("Feedforward Speed: %.4f\n", get_feedforward_speed(ramped_height));
    rt_kprintf("PID Evaluation: %s\n", is_evaluating ? "ON" : "OFF");
    rt_kprintf("Total Abs Error: %.4f\n", total_abs_error);
}
MSH_CMD_EXPORT(get_status, Get current ball height for testing);