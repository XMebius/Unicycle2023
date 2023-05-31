/*
 * balance.c
 *
 *  Created on: 2023��5��5��
 *      Author: Mebius
 */
#include "balance.h"

#define MOTORA_OFF        gpio_set_level(stop1,  GPIO_LOW);       // ֹͣ��ɲ���ߣ�
#define MOTORB_OFF        gpio_set_level(stop2,  GPIO_LOW);       // ֹͣ��ɲ���ߣ�
#define MOTORC_OFF        gpio_set_level(Nsleep, GPIO_LOW);        // �н������ʹ��
#define MOTORA_ON         gpio_set_level(stop1,  GPIO_HIGH);       // ֹͣ��ɲ���ߣ�
#define MOTORB_ON         gpio_set_level(stop2,  GPIO_HIGH);       // ֹͣ��ɲ���ߣ�
#define MOTORC_ON         gpio_set_level(Nsleep, GPIO_HIGH);       // �н����ʹ��

float R_balance_Control(float Angle,float *Angle_Zero,float Gyro);
float P_balance_Control(float Angle,float Angle_Zero,float Gyro);
float Y_balance_Control(float Angle,float Angle_Zero,float Gyro);
//float Yaw_balance_Control(float Gyro,float Gyro_control);
float Velocity_Control_A(int encoder);
float Velocity_Control_B(int encoder);
float Velocity_Control_C(int encoder);
float constrain_float(float num, float min, float max);
short constrain_short(short num, short min, short max);

int Start_Flag=3;                    //������־
// unsigned char  show_flag=0;                     //��ʾ��־

// Status_car status_car = idle;                   //С��״̬ һ��ʼΪ����״̬
// Element element = nothing;                      //����Ԫ�� һ��ʼΪĬ��

int count_size,count_sum;
int count_A,count_B,count_C,count_D,count_E,count_F,count_G;            //������
int Flag_A,Flag_B,Flag_C,Flag_D,Flag_E,Flag_F,Flag_G,Flag_H,Flag_I;     //��־λ

short Motor_A,Motor_B,Motor_C,Motor_D;                                  //���PWM����

short Encoder_A=2;
short Encoder_B,Encoder_C,Encoder_D;                          //���������������
int32_t Encoder_C_Sum;                                                     //�н�������������ۼ�ֵ

int PWM_R=0,PWM_Y=0,PWMA_accel,PWMB_accel,PWMC_accel;                       //PWM�м���

// int Voltage;  

//���ú�  Pitch_Zero=1.35  ������ Pitch_Zero=3.82
float Roll_Zero=0.03/*0.15~0.2*//*0.094850~0.094860*//*, Roll_Zero1=5.63*/;       //��ǰ��Զ��������λ�ã���ǰRoll��С������roll���
float Pitch_Zero=-0.56/*-0.55~-0.75*/;     // ��ǰ��Զ�������Ƿ��ã�����ƫ���С������Խ��
//float Roll_Zero=0.0038, Roll_Zero1=5.63;      //Roll_Zero1����¼ֵ������Ĵ�����Roll_Zero=-3,3,�������Ƿ�����б�Ǹ�������֮Ϊ����
//float Pitch_Zero=-0.024,Pitch_Zero1=5.63;     //Pitch_Zero1:
float Yaw_Zero=0.0;                     //XY��Ƕ���㣬���е�йأ�Ӱ���ȶ���

float Roll_error=0,Pitch_error=0,Yaw_error=0;
float error_Camera, error_Blance;       //����ͷ�͵�����
float error_of_CameraOrBalance;         //ת�����

/****************************************�ٶ�Ϊ160����*************************************************************/
float R_Balance_KP=-15400/*-11000,-13400��ʼ���ֽϴ������*/, R_Balance_KI=0/*-10*/   ,  R_Balance_KD=-4150/*-2250*/  ;          //AB����������4500 15 275
float Y_Balance_KP=-1.05  ,  Y_Balance_KI=00.0  ,  Y_Balance_KD=-25.0  ;      //AB�����ת�Ƕ�
float R_Velocity_KP=00  ,  R_Velocity_KI=0   ;                               //AB����ٶȻ�
float P_Balance_KP=-2350/*>2250 -1450 -2850���ִ������*/, P_Balance_KI=0,P_Balance_KD=-40/*<-180*//*300*/;                //C���ǰ�����
float P_Velocity_KP=0/*-0.287*//*2.287*/,P_Velocity_KI=0/*-0.05*/;                                //C����ٶȻ�

///****************************************�ٶ�Ϊ160����*************************************************************/
//float R_Balance_KP=-7600/*-8000/-3600,��ʼ���ֽϴ������*/, R_Balance_KI=1   ,  R_Balance_KD=-760/*-820/420*/  ;          //AB����������4500 15 275
//float Y_Balance_KP=-1.05  ,  Y_Balance_KI=00.0  ,  Y_Balance_KD=-25.0  ;      //AB�����ת�Ƕ�
//float R_Velocity_KP=00  ,  R_Velocity_KI=0   ;                               //AB����ٶȻ�
//float P_Balance_KP=-1400/*-850*/, P_Balance_KI=1,P_Balance_KD=-125;                //C���ǰ�����
//float P_Velocity_KP=9.15,P_Velocity_KI=0.105;                                //C����ٶȻ�

float Yaw_control_1,Yaw_control_2 = 0;
float Yaw_mark;
int P_Move_distance=100 , R_Move_distance = 0/*, Move_distance_MAX=10*/;

float Pitch=0.0f,Roll=0.0f,Yaw=0.0f,Pitch_temp=0.0f,Roll_temp=0.0f,Yaw_temp=0.0f;    //�����������˲������̬��Ϣ
float gyro_yaw=0.0f,gyro_pitch=0.0f,gyro_roll=0.0f,gyro_yaw_temp=0.0f,gyro_pitch_temp=0.0f,gyro_roll_temp=0.0f;

void Balance(void)
{
    static int num;
/*****************������Ϣ�ɼ�***************************************************************************/
#ifndef cascade_pid
    Encoder_A = encoder_get_count(ENCODER_1_QUADDEC);   //���� ĸ���ϱ�����1��С��ǰ��Ϊ��ֵ
    Encoder_B = encoder_get_count(ENCODER_2_QUADDEC);   //�ҵ�� ĸ���ϱ�����2��С��ǰ��Ϊ��ֵ
    Encoder_C = encoder_get_count(ENCODER_3_QUADDEC);   //�н���� ĸ���ϱ�����3��С��ǰ��Ϊ��ֵ
#endif
//    printf("%hd,%hd,%hd,",Encoder_A,Encoder_B,Encoder_C);  //-3500��PWM��Ӧ��encoder��ֵΪ-3330
    // printf("%f,",Encoder_C);
//    Encoder_C_Sum += Encoder_C;                     //�н�����������ۼ�

    getKalmanPosition(&Pitch_temp,&Roll_temp,&Yaw_temp,&gyro_yaw_temp,&gyro_pitch_temp,&gyro_roll_temp);//��ȡ��̬��Ϣ
    Pitch=Pitch_temp*40;
    Roll=Roll_temp*40;
    gyro_pitch=gyro_pitch_temp*20;
    gyro_roll=gyro_roll_temp*40;
//    printf("%f,%f",Pitch,gyro_pitch);
    float temp=Pitch-Pitch_Zero;
//    printf("%f,%f,",temp,Roll);
/************************����ֱ���Լ�ת������*********************************************************************************************************************************/
#ifdef cascade_pid   //����pid
    PWM_R = R_Cascade_Pid_Ctrl(Roll_Zero);
#else
    PWM_R = R_balance_Control(Roll, &Roll_Zero, gyro_roll);
#endif

    if(++Flag_I==30)
    {
        PWM_Y = Y_balance_Control(/*error_of_CameraOrBalance*/0, 0,gyro_yaw);     //A B�������Z��ת��
        Flag_I=0;
    }
    Motor_A = (short)+PWM_R ;//+ PWM_Y /*+ PWMA_accel*/;                                //���տ�����   PWM_Y?
    Motor_B = (short)-PWM_R ;//+ PWM_Y /*+ PWMA_accel*/;                                //���տ�����
#ifdef cascade_pid
    Motor_C = (short)P_Cascade_Pid_Ctrl(Pitch_Zero);
#else
    PWMC_accel = Velocity_Control_C(Encoder_C);                             //C����ٶȻ�������
    Motor_C = (short)-P_balance_Control(Pitch, Pitch_Zero, gyro_pitch) + PWMC_accel;    //C�������ǰ�����
//    printf("%hd,%f,%d\n",Motor_C,gyro_pitch,Start_Flag);
#endif

/*************************�������޷�******************************************************************************************************************************/
    Roll_error = Roll - Roll_Zero;
    Pitch_error = Pitch - Pitch_Zero;
    //�����ض��Ƕ�ֹͣ����
    if(fabs(Roll_error)>15 || fabs(Pitch_error)>15){ Motor_A =0,Motor_B =0,Motor_C =0,Start_Flag=0;MOTORA_OFF;MOTORB_OFF;MOTORC_OFF;}
    Motor_A = constrain_short(Motor_A, -10000, 10000);                     //PWM�޷�
    Motor_B = constrain_short(Motor_B, -10000, 10000);                     //PWM�޷�
//    if(Motor_C>0) Motor_C = Motor_C + 120;
//    else if(Motor_C<0) Motor_C = Motor_C - 120;
//    printf("%f,%f\n",gyro_pitch,Motor_C);
    Motor_C = constrain_short(Motor_C, -3500, 3500);                       //PWM�޷�
//    printf("%hd,%hd,%hd\n",Motor_A,Motor_B,Motor_C);
    /************************�������****************************************************************************************************************************/
    if(Start_Flag==0)
    {
        MOTORA_OFF;
        MOTORB_OFF;                                       //ɲ��
        MOTORC_OFF;
        MotorCtrl3W(0,0,0);                             //���� ���ֶ�ֹͣ
    }
    else if(Start_Flag==1)
    {
        MOTORA_OFF;
        MOTORB_OFF;                                       //ɲ��
        MOTORC_ON;
        MotorCtrl3W(0,0,Motor_C);                       //����ɲ�� ��������
    }
    else if(Start_Flag==2)
    {
        MOTORA_ON;
        MOTORB_ON;                                       //��������
        MOTORC_OFF;
        MotorCtrl3W(Motor_A,Motor_B,0);                 //�������� ����ɲ��
    }
    else if(Start_Flag==3)
    {
        MOTORA_ON;
        MOTORB_ON;
        MOTORC_ON;
        MotorCtrl3W(Motor_A, Motor_B, Motor_C);         //���� ���ֶ�����
    }

    encoder_clear_count(ENCODER_1_QUADDEC);
    encoder_clear_count(ENCODER_2_QUADDEC);
    encoder_clear_count(ENCODER_3_QUADDEC);

}
/**************************************************************************
X��ƽ��PID����,�ǶȻ�
**************************************************************************/
float R_balance_Control(float Angle,float *Angle_Zero,float Gyro)
{
//    static unsigned int n;                         //�����������ı����
    float PWM,Bias;
    static float error;
    Bias=Angle-*Angle_Zero;                                              //��ȡƫ��
    error+=Bias;                                                         //ƫ���ۻ�
    error = constrain_float(error, -120, 120);                            //�����޷�
    PWM=R_Balance_KP*Bias + R_Balance_KI*error + Gyro*R_Balance_KD/10;   //��ȡ������ֵ
//    printf("%f,%f,%f,%f,%f\n",PWM,R_Balance_KP,Bias,R_Balance_KI,error);
    PWM = constrain_float(PWM, -9000, 9000);                            //����޷�
    if(Start_Flag==0) PWM=0,error=0;                                     //ֹͣʱ��������

    return PWM;
}
/**************************************************************************
Y��ƽ��PID����,�ǶȻ�
**************************************************************************/
float P_balance_Control(float Angle,float Angle_Zero,float Gyro)
{
    float PWM,Bias;
    static float error;
    Bias=Angle-Angle_Zero;                                               //��ȡƫ��
    error+=Bias;                                                         //ƫ���ۻ�
    error = constrain_float(error, -30, 30);                            //�����޷�
    PWM=P_Balance_KP*Bias + P_Balance_KI*error + Gyro*P_Balance_KD;   //��ȡ������ֵ
//    printf("%f,%f,",P_Balance_KP,Bias);
    if(Start_Flag==0) PWM=0,error=0;                                     //ֹͣʱ��������
//    printf("%f,%f\n",PWM,Start_Flag);
    return PWM;
}
/**************************************************************************
Z��ƽ��PID����,�ǶȻ�
**************************************************************************/
float Y_balance_Control(float Angle,float Angle_Zero,float Gyro)
{
    float PWM,Bias;
    static float error;
    Gyro = constrain_float(Gyro, -3000, 3000);                            //ȡֵ�޷�
    Bias=Angle-Angle_Zero;                                               //��ȡƫ�ƫ��ֵ��Ҫ���¼��㣬��Ȼ�������䣩
    error+=Bias;                                                         //ƫ���ۻ�
    error = constrain_float(error, -30, 30);                            //�����޷�
    PWM=Y_Balance_KP*Bias + Y_Balance_KI*error + Gyro*Y_Balance_KD/10;   //��ȡ������ֵ
    if(Start_Flag==0) PWM=0,error=0;                                     //ֹͣʱ��������

    return PWM;
}

/**************************************************************************
�ٶ�PI����,�ٶ���������
**************************************************************************/
float Velocity_Control_A(int encoder)
{
    static float Encoder,Encoder_Integral;
    float Velocity,Encoder_Least;

    Encoder_Least = (float)encoder;                                                   //�ٶ��˲�
    Encoder *= 0.7;                                                              //һ�׵�ͨ�˲���
    Encoder += Encoder_Least*0.3;                                                //һ�׵�ͨ�˲���
    Encoder_Integral += Encoder;                                               //���ֳ�λ��
    Encoder_Integral = constrain_float(Encoder_Integral, -2600, 2600);        //�����޷�
    Velocity = Encoder * R_Velocity_KP + Encoder_Integral * R_Velocity_KI/100; //��ȡ������ֵ
    Velocity = constrain_float(Velocity, -5000, 5000);
    if(Start_Flag==0) Encoder_Integral=0,Encoder=0,Velocity=0;                 //ֹͣʱ��������
    return Velocity;
}

/**************************************************************************
�ٶ�PI����,�ٶ���������
**************************************************************************/
float Velocity_Control_B(int encoder)
{
    static float Encoder,Encoder_Integral;
    float Velocity,Encoder_Least;

    Encoder_Least = (float)encoder;                                                   //�ٶ��˲�
    Encoder *= 0.7;                                                              //һ�׵�ͨ�˲���
    Encoder += Encoder_Least*0.3;                                                //һ�׵�ͨ�˲���
    Encoder_Integral += Encoder;                                               //���ֳ�λ��
    Encoder_Integral = constrain_float(Encoder_Integral, -2600, 2600);        //�����޷�
    Velocity = Encoder * R_Velocity_KP + Encoder_Integral * R_Velocity_KI/100; //��ȡ������ֵ
    Velocity = constrain_float(Velocity, -5000, 5000);
    if(Start_Flag==0) Encoder_Integral=0,Encoder=0,Velocity=0;                 //ֹͣʱ��������
    return Velocity;
}

/**************************************************************************
C����ٶ�PI����,�ٶ���������
**************************************************************************/
//float Velocity_Control_C(int encoder)
//{
//    static float Encoder,Encoder_Integral;
//    float Velocity,Encoder_Least;
//
//    Encoder_Least = (float)encoder- (float)Move_distance;                                  //�ٶ��˲�
//    Encoder *= 0.7;                                                            //һ�׵�ͨ�˲���
//    Encoder += Encoder_Least*0.3;                                              //һ�׵�ͨ�˲���

//    Encoder_Integral += Encoder - Move_distance;                             //���ֳ�λ��
//    Encoder_Integral = constrain_float(Encoder_Integral, -500, 500);        //�����޷�
//    Velocity = Encoder * P_Velocity_KP + Encoder_Integral * P_Velocity_KI;   //��ȡ������ֵ
//
//    if(Start_Flag==0) Encoder_Integral=0,Encoder=0,Velocity=0;               //ֹͣʱ��������
//    return Velocity;
//}

