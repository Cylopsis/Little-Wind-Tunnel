#ifndef SYSTEM_VARS_H
#define SYSTEM_VARS_H

#include <rtthread.h>

/* 系统状态变量声明 */
extern int32_t current_height;      // 当前高度 (mm)
extern float target_height;         // 最终目标高度 (mm)  
extern float ramped_height;         // PID控制器当前正在追踪的、平滑变化的目标高度

extern float KP;                    // PID参数
extern float KI;
extern float KD;

extern float integral_error;        // PID 控制器状态变量
extern float previous_error;

extern rt_bool_t is_evaluating;     // PID 控制参数评估状态
extern float total_abs_error;

// 远程控制还是调用这个函数，懒得写专门的远程控制代码了
void pid_tune(int argc, char **argv);
// OLED显示
void screen_on();

#endif /* SYSTEM_VARS_H */
