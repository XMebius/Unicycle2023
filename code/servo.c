#include "servo.h"

void MotorCtrl3W (sint32 Motor_A, sint32 Motor_B, sint32 Motor_C)
{
    if(Motor_A>0){
        pwm_set_duty(PWM_CH1,10000-Motor_A);
//        gpio_init(dir1, GPO, GPIO_LOW, GPO_PUSH_PULL);
        gpio_set_level(dir1,0);
    }
    else{
        pwm_set_duty(PWM_CH1,10000+Motor_A);
//        gpio_init(dir1, GPO, GPIO_HIGH, GPO_PUSH_PULL);
        gpio_set_level(dir1,1);
    }
    if(Motor_B>0){
        pwm_set_duty(PWM_CH2,10000-Motor_B);
//        gpio_init(dir2, GPO, GPIO_LOW, GPO_PUSH_PULL);
        gpio_set_level(dir2,0);
    }
    else{
        pwm_set_duty(PWM_CH2,10000+Motor_B);
//        gpio_init(dir2, GPO, GPIO_HIGH, GPO_PUSH_PULL);
        gpio_set_level(dir2,1);
    }
    if(Motor_C>0){
        pwm_set_duty(PWM_CH3,Motor_C);
//        gpio_init(dir3, GPO, GPIO_LOW, GPO_PUSH_PULL);
        gpio_set_level(dir3,0);
    }
    else{
        pwm_set_duty(PWM_CH3,(0-Motor_C));
//        gpio_init(dir3, GPO, GPIO_HIGH, GPO_PUSH_PULL);
        gpio_set_level(dir3,1);
    }
}
