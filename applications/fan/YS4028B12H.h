#include <rtthread.h>
#include <rtdevice.h>
#include <stdlib.h>

#define YS4028B12H_ANTICLOCKWISE          1       /* PWM顺时针 */
#define YS4028B12H_CLOCKWISE              0       /* PWM逆时针 */

typedef struct ys4028b12h_cfg
{   
    int period; /* 周期,单位是纳秒 */
    int pulse;  /* PWM脉冲宽度值，单位为纳秒ns */
    int channel; /* PWM通道 */
    struct rt_device_pwm *name;
} ys4028b12h_cfg,*ys4028b12h_cfg_t;

/* 初始化结构体 */
extern  ys4028b12h_cfg my_ys4028b12h_config;


rt_err_t ys4028b12h_init(ys4028b12h_cfg_t cfg);
rt_err_t ys4028b12h_deinit(ys4028b12h_cfg_t cfg);
rt_err_t ys4028b12h_set_speed(ys4028b12h_cfg_t cfg,float speed);
rt_err_t ys4028b12h_control(ys4028b12h_cfg_t cfg,int speed,int dir);
rt_err_t ys4028b12h_get(ys4028b12h_cfg_t cfg);
float ys4028b12h_get_speed(ys4028b12h_cfg_t cfg);
