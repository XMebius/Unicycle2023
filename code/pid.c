/*
 * pid.c
 *
 *  Created on: 2023��5��5��
 *      Author: Mebius
 */


#include "pid.h"
#include "zf_common_headfile.h"

//
extern float Pitch,Roll,Yaw;
extern float gyro_yaw,gyro_pitch,gyro_roll;
extern float Move_distance;


/*!
  * @brief    �޷�����
  *
  * @param    amt   �� ����
  * @param    low   �� ���ֵ
  * @param    high  �� ���ֵ
  *
  * @return   ��
  *
  * @note     ��
  *
  * @see      ��
  *
  * @date     2020/6/8
  */
float constrain_float(float amt, float low, float high)
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

short constrain_short(short amt, short low, short high)
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

pid_param_t Roll_acc_pid;      //C������ٶȻ�
pid_param_t Roll_angle_pid;    //C����ǶȻ�
pid_param_t Roll_vel_pid;      //C����ٶȻ�
pid_param_t Pitch_acc_pid;      //AB������ٶȻ�
pid_param_t Pitch_angle_pid;    //AB����ǶȻ�

extern short Encoder_C;

void P_Acc_Pid_Init(pid_param_t * pid);
void P_Angle_Pid_Init(pid_param_t * pid);
void P_Vel_Pid_Init(pid_param_t * pid);
void R_Acc_Pid_Init(pid_param_t * pid);
void R_Angle_Pid_Init(pid_param_t * pid);

void My_Pid_P_Init(void)
{
    P_Acc_Pid_Init(&Roll_acc_pid);
    P_Angle_Pid_Init(&Roll_angle_pid);
    P_Vel_Pid_Init(&Roll_vel_pid);
}
void My_Pid_R_Init(void)
{
    R_Acc_Pid_Init(&Pitch_acc_pid);
    R_Angle_Pid_Init(&Pitch_angle_pid);
}

float R_Cascade_Pid_Ctrl(float zhongzhi)
{
    static int16_t Pid_t;
    Pid_t = Pid_t+2;
    if(Pid_t % 32==0){
        Pid_t = 0;
        Encoder_C = encoder_get_count(TIM6_ENCOEDER_CH1_P20_3);   //�н����������
        /*16ms����һ��*/
        PidLocCtrl(&Roll_vel_pid, (Encoder_C*5/32)-Move_distance);   //�ٶȻ�,�����ٶ�ΪMove_distance
    }
    if(Pid_t % 8 == 0)
        /*4ms����һ��*/
        PidLocCtrl(&Roll_angle_pid,Roll_vel_pid.out - Roll + zhongzhi);//�ǶȻ�

    /*2ms����һ��*/
    PidLocCtrl(&Roll_acc_pid,-gyro_pitch + Roll_angle_pid.out);  //���ٶȻ�

    return Roll_acc_pid.out;
}

float P_Cascade_Pid_Ctrl(float zhongzhi)
{
    static int16_t Pid_t;
    Pid_t = Pid_t+2;
    if(Pid_t % 4 == 0){
        Pid_t = 0;
        PidLocCtrl(&Pitch_angle_pid,Pitch - zhongzhi);//�ǶȻ�
    }

    PidLocCtrl(&Pitch_acc_pid,+gyro_yaw + Pitch_angle_pid.out);  //���ٶȻ�

    return Pitch_acc_pid.out;
}

/*����PID����*/
void P_Acc_Pid_Init(pid_param_t * pid)
{
    pid->kp        = 1.9; //10
    pid->ki        = 0;
    pid->kd        = 0;
    pid->imax      = 100;
    pid->out_p     = 0;
    pid->out_i     = 0;
    pid->out_d     = 0;
    pid->out       = 0;
    pid->integrator= 0;
    pid->last_error= 0;
    pid->last_derivative   = 0;
    pid->last_t    = 0;
}

/*����PID����*/
void P_Angle_Pid_Init(pid_param_t * pid)
{
    pid->kp        = 80.0;//90
    pid->ki        = 0.0;
    pid->kd        = 0;
    pid->imax      = 50;
    pid->out_p     = 0;
    pid->out_i     = 0;
    pid->out_d     = 0;
    pid->out       = 0;
    pid->integrator= 0;
    pid->last_error= 0;
    pid->last_derivative   = 0;
    pid->last_t    = 0;
}

/*����PID����*/
void P_Vel_Pid_Init(pid_param_t * pid)
{
    pid->kp        = 0.047;
    pid->ki        = 0.0;
    pid->kd        = 0.005;
    pid->imax      = 0.0;
    pid->out_p     = 0;
    pid->out_i     = 0;
    pid->out_d     = 0;
    pid->out       = 0;
    pid->integrator= 0;
    pid->last_error= 0;
    pid->last_derivative   = 0;
    pid->last_t    = 0;
}

/*����PID����*/
void R_Acc_Pid_Init(pid_param_t * pid)
{
    pid->kp        = 45.0; //10
    pid->ki        = 0;
    pid->kd        = 0;
    pid->imax      = 100;
    pid->out_p     = 0;
    pid->out_i     = 0;
    pid->out_d     = 0;
    pid->out       = 0;
    pid->integrator= 0;
    pid->last_error= 0;
    pid->last_derivative   = 0;
    pid->last_t    = 0;
}

/*����PID����*/
void R_Angle_Pid_Init(pid_param_t * pid)
{
    pid->kp        = 155.0;//90
    pid->ki        = 0.0;
    pid->kd        = 0;
    pid->imax      = 50;
    pid->out_p     = 0;
    pid->out_i     = 0;
    pid->out_d     = 0;
    pid->out       = 0;
    pid->integrator= 0;
    pid->last_error= 0;
    pid->last_derivative   = 0;
    pid->last_t    = 0;
}

/*!
  * @brief    pid������ʼ������
  *
  * @param    ��
  *
  * @return   ��
  *
  * @note     ��
  *
  * @see      ��
  *
  * @date     2020/6/8
  */
void PidInit(pid_param_t * pid)
{
    pid->kp        = 0;
    pid->ki        = 0;
    pid->kd        = 0;
    pid->imax      = 0;
    pid->out_p     = 0;
    pid->out_i     = 0;
    pid->out_d     = 0;
    pid->out       = 0;
    pid->integrator= 0;
    pid->last_error= 0;
    pid->last_derivative   = 0;
    pid->last_t    = 0;
}


/*!
  * @brief    pidλ��ʽ���������
  *
  * @param    pid     pid����
  * @param    error   pid�������
  *
  * @return   PID������
  *
  * @note     ��
  *
  * @see      ��
  *
  * @date     2020/6/8
  */
float PidLocCtrl(pid_param_t * pid, float error)
{
    /* �ۻ���� */
    pid->integrator += error;

    /* ����޷� */
    constrain_float(pid->integrator, -pid->imax, pid->imax);


    pid->out_p = pid->kp * error;
    pid->out_i = pid->ki * pid->integrator;
    pid->out_d = pid->kd * (error - pid->last_error);

    pid->last_error = error;

    pid->out = pid->out_p + pid->out_i + pid->out_d;

    return pid->out;
}


/*!
  * @brief    pid����ʽ���������
  *
  * @param    pid     pid����
  * @param    error   pid�������
  *
  * @return   PID������   ע���������Ѿ��������ϴν��
  *
  * @note     ��
  *
  * @see      ��
  *
  * @date     2020/6/8
  */
float PidIncCtrl(pid_param_t * pid, float error)
{

    pid->out_p = pid->kp * error;
    pid->out_i = pid->ki * (error - pid->last_error);
    pid->out_d = pid->kd * ((error - pid->last_error) - pid->last_derivative);

    pid->last_derivative = error - pid->last_error;
    pid->last_error = error;

    pid->out += pid->out_p + pid->out_i + pid->out_d;

    return pid->out;
}

