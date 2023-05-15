#include "servo.h"

void MotorCtrl3W (sint32 Motor_A, sint32 Motor_B, sint32 Motor_C)
{
    if(Motor_A>0){
        pwm_set_duty(PWM_CH1,10000-Motor_A);
    }
    else{
        pwm_set_duty(PWM_CH1,10000+Motor_A);
    }
    if(Motor_B>0){
        pwm_set_duty(PWM_CH2,10000-Motor_B);
    }
    else{
        pwm_set_duty(PWM_CH2,10000+Motor_B);
    }
    if(Motor_C>0){
        pwm_set_duty(PWM_CH3,Motor_C);
    }
    else{
        pwm_set_duty(PWM_CH3,(0-Motor_C));
    }
}
