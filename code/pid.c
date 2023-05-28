#include "pid.h"
#include "zf_common_headfile.h"

extern float Pitch,Roll,Yaw;
extern float gyro_yaw,gyro_pitch,gyro_roll;
extern float Move_distance;
/*!
  * @brief    �޷�����
  * @param    amt   �� ����
  * @param    low   �� ���ֵ
  * @param    high  �� ���ֵ
  */
float constrain_float(float amt, float low, float high)
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

short constrain_short(short amt, short low, short high)
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

pid_param_t Pitch_acc_pid;      //C������ٶȻ�
pid_param_t Pitch_angle_pid;    //C����ǶȻ�
pid_param_t Pitch_vel_pid;      //C����ٶȻ�
pid_param_t Roll_acc_pid;      //AB������ٶȻ�
pid_param_t Roll_angle_pid;    //AB����ǶȻ�

extern short Encoder_C;

void P_Acc_Pid_Init(pid_param_t * pid);
void P_Angle_Pid_Init(pid_param_t * pid);
void P_Vel_Pid_Init(pid_param_t * pid);
void R_Acc_Pid_Init(pid_param_t * pid);
void R_Angle_Pid_Init(pid_param_t * pid);

void My_Pid_P_Init(void)
{
    P_Acc_Pid_Init(&Pitch_acc_pid);
    P_Angle_Pid_Init(&Pitch_angle_pid);
    P_Vel_Pid_Init(&Pitch_vel_pid);
}
void My_Pid_R_Init(void)
{
    R_Acc_Pid_Init(&Roll_acc_pid);
    R_Angle_Pid_Init(&Roll_angle_pid);
}
// ���ڻ���Ӧ�ٶ���죬���⻷��Ӧ�ٶ����������ٶ����ڻ����Ƕ��м价���ٶ����⻷
float P_Cascade_Pid_Ctrl(float zhongzhi)
{
    static int16_t Pid_t;
    Pid_t = Pid_t+5;
    if(Pid_t % 20==0){
        Pid_t = 0;
        Encoder_C =encoder_get_count(ENCODER_3_QUADDEC);
        PidLocCtrl(&Pitch_vel_pid, (Encoder_C*5/32)-Move_distance);
    }

    PidLocCtrl(&Pitch_angle_pid,Pitch_vel_pid.out-Pitch+zhongzhi);//�ٶȻ�
    return Pitch_angle_pid.out;
}

float R_Cascade_Pid_Ctrl(float zhongzhi)
{
    static int16_t Pid_t;
    Pid_t = Pid_t+5;
    if(Pid_t % 10 == 0){
        Pid_t = 0;
        PidLocCtrl(&Roll_angle_pid,Roll- zhongzhi);//�ǶȻ�
    }
    PidLocCtrl(&Roll_acc_pid,Roll_acc_pid.out+gyro_roll);//���ٶȻ�
    return Roll_angle_pid.out;
}

void R_Angle_Pid_Init(pid_param_t * pid)
{
    pid->kp        = -15400;//90
    pid->ki        = 0.0;
    pid->kd        = -415/*-4150*/;
    pid->imax      = 120;

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
    pid->kp        = 2350;
    pid->ki        = 0.0;
    pid->kd        = 0;
    pid->imax      = 160;

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

/*!
  * @brief    pid������ʼ������
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
  * @param    pid     pid����
  * @param    error   pid�������
  */
float PidLocCtrl(pid_param_t* pid,float error)
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
