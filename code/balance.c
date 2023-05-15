/*
 * balance.c
 *
 *  Created on: 2023��5��5��
 *      Author: Mebius
 */
#include "balance.h"

int Encoder_A,Encoder_B,Encoder_C,Encoder_D;                          //���������������
int Encoder_C_Sum;                                                  //�н�������������ۼ�ֵ

int PWM_R,PWM_Y,PWMA_accel,PWMB_accel,PWMC_accel;                       //PWM�м���

float P_Balance_KP=2000.0, P_Balance_KI=0,P_Balance_KD=0;                //���ҷ���
float R_Balance_KP=7000 ,  R_Balance_KI=80   ,  R_Balance_KD=450  ;          //AB����������4500 15 275
float Y_Balance_KP=-1.05  ,  Y_Balance_KI=00.0  ,  Y_Balance_KD=-25.0  ;      //AB�����ת�Ƕ�

float R_Velocity_KP=00  ,  R_Velocity_KI=0   ;                               //AB����ٶȻ�
float P_Velocity_KP=9.15,P_Velocity_KI=0.105;                                //C����ٶȻ�
int Move_distance=30 , Move_distance_MAX=170;
//���ú�  Pitch_Zero=1.35  ������ Pitch_Zero=3.82
float Roll_Zero=0.03, Roll_Zero1=5.63;      //Roll_Zero1����¼ֵ
float Pitch_Zero=1.20,Pitch_Zero1=5.63;     //Pitch_Zero1:
float Yaw_Zero=0.0;                     //XY��Ƕ���㣬���е�йأ�Ӱ���ȶ���

int Flag_A,Flag_B,Flag_C,Flag_D,Flag_E,Flag_F,Flag_G,Flag_H,Flag_I;     //��־λ
float Motor_A,Motor_B,Motor_C,Motor_D;                                  //���PWM����

float Roll_error=0,Pitch_error=0,Yaw_error=0;
unsigned char  Start_Flag=1;                    //������־
unsigned char  delay_30,delay_flag;             //30�����־

float Pitch=0.0f,Roll=0.0f,Yaw=0.0f;
float gyrox=0.0f,gyroy=0.0f,gyroz=0.0f;

void Balance(void)
{
    static int num;

    Encoder_A = encoder_get_count(TIM5_ENCOEDER_CH1_P10_3);   //���� ĸ���ϱ�����1��С��ǰ��Ϊ��ֵ
        Encoder_B = encoder_get_count(TIM4_ENCOEDER_CH1_P02_8);   //�ҵ�� ĸ���ϱ�����2��С��ǰ��Ϊ��ֵ
    #ifndef cascade_pid           //����PID���ٶȼ����pid������
        Encoder_C = encoder_get_count(TIM6_ENCOEDER_CH1_P20_3);   //�н���� ĸ���ϱ�����3��С��ǰ��Ϊ��ֵ
    #endif
        Encoder_C_Sum += Encoder_C;                     //�н�����������ۼ�

    getKalmanPosition(&Pitch,&Roll,&Yaw,&gyrox,&gyroy,&gyroz);//��ȡ��̬��Ϣ

    Motor_A = P_balance_Control(Roll, Roll_Zero, gyroy);
    Motor_A = constrain_short(Motor_A, -10000, 10000);
    float temp=Roll-Roll_Zero;
    printf("%f,%f,%f,%f,",Roll,Roll_Zero,temp,P_Balance_KP);
    if(Motor_A<0){
        gpio_set_level(dir1,GPIO_HIGH);
        gpio_set_level(dir2,GPIO_LOW);
        Motor_A=10000+Motor_A;
    }
    else{
        gpio_set_level(dir1,GPIO_LOW);
        gpio_set_level(dir2,GPIO_HIGH);
        Motor_A=10000-Motor_A;
    }
    pwm_set_duty(PWM_CH1,Motor_A);
    pwm_set_duty(PWM_CH2,Motor_A);
    printf("%f\n",Motor_A);
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

    return PWM*60;
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
