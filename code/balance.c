/*
 * balance.c
 *
 *  Created on: 2023��5��5��
 *      Author: Mebius
 */
#include "balance.h"
#define DIN_ON      gpio_init(Nsleep, GPO, GPIO_HIGH, GPO_PUSH_PULL);       // ������ɲ���ߣ�
#define DIN_OFF     gpio_init(Nsleep, GPO, GPIO_LOW, GPO_PUSH_PULL);       // ֹͣ��ɲ���ߣ�

unsigned char  Start_Flag=0;                    //������־
unsigned char  show_flag=0;                     //��ʾ��־
unsigned char  delay_30,delay_flag;             //30�����־

// Status_car status_car = idle;                   //С��״̬ һ��ʼΪ����״̬
// Element element = nothing;                      //����Ԫ�� һ��ʼΪĬ��

int count_size,count_sum;
int count_A,count_B,count_C,count_D,count_E,count_F,count_G;            //������
int Flag_A,Flag_B,Flag_C,Flag_D,Flag_E,Flag_F,Flag_G,Flag_H,Flag_I;     //��־λ

short Motor_A,Motor_B,Motor_C,Motor_D;                                  //���PWM����

short Encoder_A,Encoder_B,Encoder_C,Encoder_D;                          //���������������
int32_t Encoder_C_Sum;                                                     //�н�������������ۼ�ֵ

int PWM_R=0,PWM_Y=0,PWMA_accel,PWMB_accel,PWMC_accel;                       //PWM�м���

// int Voltage;  

//���ú�  Pitch_Zero=1.35  ������ Pitch_Zero=3.82
float Roll_Zero=-0.03, Roll_Zero1=5.63;      //Roll_Zero1����¼ֵ������Ĵ�����Roll_Zero=-3,3
float Pitch_Zero=1.20,Pitch_Zero1=5.63;     //Pitch_Zero1:
float Yaw_Zero=0.0;                     //XY��Ƕ���㣬���е�йأ�Ӱ���ȶ���

float Roll_error=0,Pitch_error=0,Yaw_error=0;
float error_Camera, error_Blance;       //����ͷ�͵�����
float error_of_CameraOrBalance;         //ת�����

// extern int OFFSET0, OFFSET1, OFFSET2;   //������������ֵ 8-23 24-39 40-56
// extern sint16 temp0_L, temp0_R;        //��ʱ��ֵ
// extern sint16 temp1_L, temp1_R;        //��ʱ��ֵ
// extern sint16 temp2_L, temp2_R;        //��ʱ��ֵ

/****************************************�ٶ�Ϊ160����*************************************************************/
float R_Balance_KP=7000 ,  R_Balance_KI=80   ,  R_Balance_KD=450  ;          //AB����������4500 15 275
float Y_Balance_KP=-1.05  ,  Y_Balance_KI=00.0  ,  Y_Balance_KD=-25.0  ;      //AB�����ת�Ƕ�
float R_Velocity_KP=00  ,  R_Velocity_KI=0   ;                               //AB����ٶȻ�
float P_Balance_KP=152.5, P_Balance_KI=1.12,P_Balance_KD=20.5;                //C���ǰ�����
//float P_Balance_KP=420,P_Balance_KI=0.5,P_Balance_KD=8;
float P_Velocity_KP=9.15,P_Velocity_KI=0.105;                                //C����ٶȻ�

float Yaw_control_1,Yaw_control_2 = 0;
float Yaw_mark;
int Move_distance=30 , Move_distance_MAX=170;

float Pitch=0.0f,Roll=0.0f,Yaw=0.0f;    //�����������˲������̬��Ϣ
float gyrox=0.0f,gyroy=0.0f,gyroz=0.0f;

void Balance(void)
{
    static int num;
/*****************������Ϣ�ɼ�***************************************************************************/
    Encoder_A = encoder_get_count(TIM5_ENCOEDER_CH1_P10_3);   //���� ĸ���ϱ�����1��С��ǰ��Ϊ��ֵ
        Encoder_B = encoder_get_count(TIM4_ENCOEDER_CH1_P02_8);   //�ҵ�� ĸ���ϱ�����2��С��ǰ��Ϊ��ֵ
    #ifndef cascade_pid           //����PID���ٶȼ����pid������
        Encoder_C = encoder_get_count(TIM6_ENCOEDER_CH1_P20_3);   //�н���� ĸ���ϱ�����3��С��ǰ��Ϊ��ֵ
    #endif
        Encoder_C_Sum += Encoder_C;                     //�н�����������ۼ�

    getKalmanPosition(&Pitch,&Roll,&Yaw,&gyrox,&gyroy,&gyroz);//��ȡ��̬��Ϣ

/************************����ֱ���Լ�ת������*********************************************************************************************************************************/
    PWM_R = P_balance_Control(Roll, Roll_Zero, gyroy);
    // float temp=Roll-Roll_Zero;
    // printf("%f,%f,%f,%f,",Roll,Roll_Zero,temp,P_Balance_KP);
    Motor_A = (short)+PWM_R + PWM_Y /*+ PWMA_accel*/;                                //���տ�����
    Motor_B = (short)-PWM_R + PWM_Y /*+ PWMA_accel*/;                                //���տ�����
#ifdef cascade_pid
    Motor_C = R_Cascade_Pid_Ctrl(Roll_Zero);
#else
    PWMC_accel = Velocity_Control_C(Encoder_C);                             //C����ٶȻ�������
    // Motor_C = -P_balance_Control(Roll, Roll_Zero, gyro[1]) + PWMC_accel;    //C�������ǰ�����
#endif

/*************************�������޷�******************************************************************************************************************************/
    Roll_error = Roll - Roll_Zero;
    Pitch_error = Pitch - Pitch_Zero;
    //�����ض��Ƕ�ֹͣ����
    if(fabs(Roll_error)>20 || fabs(Pitch_error)>15){ Motor_A =0,Motor_B =0,Motor_C =0,Start_Flag=0;DIN_OFF;}
    Motor_A = constrain_short(Motor_A, -10000, 10000);                     //PWM�޷�
    Motor_B = constrain_short(Motor_B, -10000, 10000);                     //PWM�޷�
    if(Motor_C>0) Motor_C = Motor_C + 120;
    else if(Motor_C<0) Motor_C = Motor_C - 120;
    Motor_C = constrain_short(Motor_C, -3500, 3500);                       //PWM�޷�

/************************�������****************************************************************************************************************************/
    if(Start_Flag==0)
    {
        DIN_OFF;                                        //ɲ��
        MotorCtrl3W(0,0,0);                             //���� ���ֶ�ֹͣ
    }
    else if(Start_Flag==1)
    {
        DIN_OFF;                                        //ɲ��
        MotorCtrl3W(0,0,Motor_C);                       //����ɲ�� ��������
    }
    else if(Start_Flag==2)
    {
        DIN_ON;                                         //����
        MotorCtrl3W(Motor_A, Motor_B, Motor_C);         //���� ���ֶ�����
    }
    if(delay_flag==1)
    {
        if(++delay_30==6)   delay_30=0,delay_flag=0;    //���������ṩ30ms�ľ�׼��ʱ
    }

    // pwm_set_duty(PWM_CH1,Motor_A);
    // pwm_set_duty(PWM_CH2,Motor_A);
    // printf("%f\n",Motor_A);
}

/*Roll�������*/
float R_balance_Control(float Angle,float *Angle_Zero,float Gyro)
{
//    static unsigned int n;                         //�����������ı����
    float PWM,Bias;
    static float error;
    Bias=Angle-*Angle_Zero;                                              //��ȡƫ��
    error+=Bias;                                                         //ƫ���ۻ�
    error = constrain_float(error, -120, 120);                            //�����޷�
    PWM=R_Balance_KP*Bias + R_Balance_KI*error + Gyro*R_Balance_KD/10;   //��ȡ������ֵ
    PWM = constrain_float(PWM, -9000, 9000);                            //����޷�
    if(Start_Flag==0) PWM=0,error=0;                                     //ֹͣʱ��������

    return PWM;
}

float P_balance_Control(float Angle,float Angle_Zero,float Gyro)
{
    float PWM,Bias;
    static float error;
    Bias=Angle-Angle_Zero;                                               //��ȡƫ��
    error+=Bias;                                                         //ƫ���ۻ�
    error = constrain_float(error, -30, 30);                            //�����޷�
    PWM=P_Balance_KP*Bias + P_Balance_KI*error + Gyro*P_Balance_KD/10;   //��ȡ������ֵ
    if(Start_Flag==0) PWM=0,error=0;                                     //ֹͣʱ��������
    return PWM;
}

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

float Velocity_Control_C(int encoder)
{
    static float Encoder,Encoder_Integral;
    float Velocity,Encoder_Least;

    Encoder_Least = (float)encoder- (float)Move_distance;                                  //�ٶ��˲�
    Encoder *= 0.7;                                                            //һ�׵�ͨ�˲���
    Encoder += Encoder_Least*0.3;                                              //һ�׵�ͨ�˲���
    Encoder_Integral += Encoder - Move_distance;                             //���ֳ�λ��
    Encoder_Integral = constrain_float(Encoder_Integral, -500, 500);        //�����޷�
    Velocity = Encoder * P_Velocity_KP + Encoder_Integral * P_Velocity_KI;   //��ȡ������ֵ
    if(Start_Flag==0) Encoder_Integral=0,Encoder=0,Velocity=0;               //ֹͣʱ��������
    return Velocity;
}
