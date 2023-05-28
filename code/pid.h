/*
 * pid.h
 *
 *  Created on: 2023年5月5日
 *      Author: Mebius
 */

#ifndef USER_PID_H_
#define USER_PID_H_
#include <stdint.h>
#include "head.h"

typedef struct
{
    float                kp;         //P
    float                ki;         //I
    float                kd;         //D
    float                imax;       //积分限幅

    float                out_p;  //KP输出
    float                out_i;  //KI输出
    float                out_d;  //KD输出
    float                out;    //pid输出

    float                integrator; //< 积分值
    float                last_error; //< 上次误差
    float                last_derivative;//< 上次误差与上上次误差之差
    unsigned long        last_t;     //< 上次时间
}pid_param_t;

void PidInit(pid_param_t * pid);

void My_Pid_P_Init(void);
void My_Pid_R_Init(void);
float P_Cascade_Pid_Ctrl(float zhongzhi);
float R_Cascade_Pid_Ctrl(float zhongzhi);
void P_Acc_Pid_Init(pid_param_t * pid);
void P_Angle_Pid_Init(pid_param_t * pid);
void P_Vel_Pid_Init(pid_param_t * pid);
void R_Acc_Pid_Init(pid_param_t * pid);
void R_Angle_Pid_Init(pid_param_t * pid);

float constrain_float(float amt, float low, float high);
short constrain_short(short amt, short low, short high);
float PidLocCtrl(pid_param_t * pid, float error/*, float dot_error*/);

float PidIncCtrl(pid_param_t * pid, float error);

#endif /* USER_PID_H_ */
