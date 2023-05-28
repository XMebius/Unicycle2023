/*
 * pid.h
 *
 *  Created on: 2023��5��5��
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
    float                imax;       //�����޷�

    float                out_p;  //KP���
    float                out_i;  //KI���
    float                out_d;  //KD���
    float                out;    //pid���

    float                integrator; //< ����ֵ
    float                last_error; //< �ϴ����
    float                last_derivative;//< �ϴ���������ϴ����֮��
    unsigned long        last_t;     //< �ϴ�ʱ��
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
