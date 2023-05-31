/*
 * balance.c
 *
 *  Created on: 2023年5月5日
 *      Author: Mebius
 */
#include "balance.h"

#define MOTORA_OFF        gpio_set_level(stop1,  GPIO_LOW);       // 停止（刹车线）
#define MOTORB_OFF        gpio_set_level(stop2,  GPIO_LOW);       // 停止（刹车线）
#define MOTORC_OFF        gpio_set_level(Nsleep, GPIO_LOW);        // 行进电机不使能
#define MOTORA_ON         gpio_set_level(stop1,  GPIO_HIGH);       // 停止（刹车线）
#define MOTORB_ON         gpio_set_level(stop2,  GPIO_HIGH);       // 停止（刹车线）
#define MOTORC_ON         gpio_set_level(Nsleep, GPIO_HIGH);       // 行进电机使能

float R_balance_Control(float Angle,float *Angle_Zero,float Gyro);
float P_balance_Control(float Angle,float Angle_Zero,float Gyro);
float Y_balance_Control(float Angle,float Angle_Zero,float Gyro);
//float Yaw_balance_Control(float Gyro,float Gyro_control);
float Velocity_Control_A(int encoder);
float Velocity_Control_B(int encoder);
float Velocity_Control_C(int encoder);
float constrain_float(float num, float min, float max);
short constrain_short(short num, short min, short max);

int Start_Flag=3;                    //启动标志
// unsigned char  show_flag=0;                     //显示标志

// Status_car status_car = idle;                   //小车状态 一开始为空闲状态
// Element element = nothing;                      //赛道元素 一开始为默认

int count_size,count_sum;
int count_A,count_B,count_C,count_D,count_E,count_F,count_G;            //计数器
int Flag_A,Flag_B,Flag_C,Flag_D,Flag_E,Flag_F,Flag_G,Flag_H,Flag_I;     //标志位

short Motor_A,Motor_B,Motor_C,Motor_D;                                  //电机PWM变量

short Encoder_A=2;
short Encoder_B,Encoder_C,Encoder_D;                          //编码器的脉冲计数
int32_t Encoder_C_Sum;                                                     //行进电机编码器的累计值

int PWM_R=0,PWM_Y=0,PWMA_accel,PWMB_accel,PWMC_accel;                       //PWM中间量

// int Voltage;  

//静置后  Pitch_Zero=1.35  不静置 Pitch_Zero=3.82
float Roll_Zero=0.03/*0.15~0.2*//*0.094850~0.094860*//*, Roll_Zero1=5.63*/;       //面前，远离陀螺仪位置，往前Roll变小，往后roll变大
float Pitch_Zero=-0.56/*-0.55~-0.75*/;     // 面前，远离陀螺仪放置，往左偏零点小，往右越大
//float Roll_Zero=0.0038, Roll_Zero1=5.63;      //Roll_Zero1：记录值，龙邱的代码中Roll_Zero=-3,3,往陀螺仪方向倾斜是负数，反之为正数
//float Pitch_Zero=-0.024,Pitch_Zero1=5.63;     //Pitch_Zero1:
float Yaw_Zero=0.0;                     //XY轴角度零点，与机械有关，影响稳定性

float Roll_error=0,Pitch_error=0,Yaw_error=0;
float error_Camera, error_Blance;       //摄像头和电磁误差
float error_of_CameraOrBalance;         //转向环误差

/****************************************速度为160左右*************************************************************/
float R_Balance_KP=-15400/*-11000,-13400开始出现较大幅度振荡*/, R_Balance_KI=0/*-10*/   ,  R_Balance_KD=-4150/*-2250*/  ;          //AB电机左右倾角4500 15 275
float Y_Balance_KP=-1.05  ,  Y_Balance_KI=00.0  ,  Y_Balance_KD=-25.0  ;      //AB电机旋转角度
float R_Velocity_KP=00  ,  R_Velocity_KI=0   ;                               //AB电机速度环
float P_Balance_KP=-2350/*>2250 -1450 -2850出现大幅度振荡*/, P_Balance_KI=0,P_Balance_KD=-40/*<-180*//*300*/;                //C电机前后倾角
float P_Velocity_KP=0/*-0.287*//*2.287*/,P_Velocity_KI=0/*-0.05*/;                                //C电机速度环

///****************************************速度为160左右*************************************************************/
//float R_Balance_KP=-7600/*-8000/-3600,开始出现较大幅度振荡*/, R_Balance_KI=1   ,  R_Balance_KD=-760/*-820/420*/  ;          //AB电机左右倾角4500 15 275
//float Y_Balance_KP=-1.05  ,  Y_Balance_KI=00.0  ,  Y_Balance_KD=-25.0  ;      //AB电机旋转角度
//float R_Velocity_KP=00  ,  R_Velocity_KI=0   ;                               //AB电机速度环
//float P_Balance_KP=-1400/*-850*/, P_Balance_KI=1,P_Balance_KD=-125;                //C电机前后倾角
//float P_Velocity_KP=9.15,P_Velocity_KI=0.105;                                //C电机速度环

float Yaw_control_1,Yaw_control_2 = 0;
float Yaw_mark;
int P_Move_distance=100 , R_Move_distance = 0/*, Move_distance_MAX=10*/;

float Pitch=0.0f,Roll=0.0f,Yaw=0.0f,Pitch_temp=0.0f,Roll_temp=0.0f,Yaw_temp=0.0f;    //经过卡尔曼滤波后的姿态信息
float gyro_yaw=0.0f,gyro_pitch=0.0f,gyro_roll=0.0f,gyro_yaw_temp=0.0f,gyro_pitch_temp=0.0f,gyro_roll_temp=0.0f;

void Balance(void)
{
    static int num;
/*****************基本信息采集***************************************************************************/
#ifndef cascade_pid
    Encoder_A = encoder_get_count(ENCODER_1_QUADDEC);   //左电机 母板上编码器1，小车前进为负值
    Encoder_B = encoder_get_count(ENCODER_2_QUADDEC);   //右电机 母板上编码器2，小车前进为正值
    Encoder_C = encoder_get_count(ENCODER_3_QUADDEC);   //行进电机 母板上编码器3，小车前进为正值
#endif
//    printf("%hd,%hd,%hd,",Encoder_A,Encoder_B,Encoder_C);  //-3500的PWM对应的encoder的值为-3330
    // printf("%f,",Encoder_C);
//    Encoder_C_Sum += Encoder_C;                     //行进电机编码器累加

    getKalmanPosition(&Pitch_temp,&Roll_temp,&Yaw_temp,&gyro_yaw_temp,&gyro_pitch_temp,&gyro_roll_temp);//获取姿态信息
    Pitch=Pitch_temp*40;
    Roll=Roll_temp*40;
    gyro_pitch=gyro_pitch_temp*20;
    gyro_roll=gyro_roll_temp*40;
//    printf("%f,%f",Pitch,gyro_pitch);
    float temp=Pitch-Pitch_Zero;
//    printf("%f,%f,",temp,Roll);
/************************保持直立以及转向运算*********************************************************************************************************************************/
#ifdef cascade_pid   //串级pid
    PWM_R = R_Cascade_Pid_Ctrl(Roll_Zero);
#else
    PWM_R = R_balance_Control(Roll, &Roll_Zero, gyro_roll);
#endif

    if(++Flag_I==30)
    {
        PWM_Y = Y_balance_Control(/*error_of_CameraOrBalance*/0, 0,gyro_yaw);     //A B电机控制Z轴转动
        Flag_I=0;
    }
    Motor_A = (short)+PWM_R ;//+ PWM_Y /*+ PWMA_accel*/;                                //最终控制量   PWM_Y?
    Motor_B = (short)-PWM_R ;//+ PWM_Y /*+ PWMA_accel*/;                                //最终控制量
#ifdef cascade_pid
    Motor_C = (short)P_Cascade_Pid_Ctrl(Pitch_Zero);
#else
    PWMC_accel = Velocity_Control_C(Encoder_C);                             //C电机速度环正反馈
    Motor_C = (short)-P_balance_Control(Pitch, Pitch_Zero, gyro_pitch) + PWMC_accel;    //C电机控制前后倾角
//    printf("%hd,%f,%d\n",Motor_C,gyro_pitch,Start_Flag);
#endif

/*************************保护与限幅******************************************************************************************************************************/
    Roll_error = Roll - Roll_Zero;
    Pitch_error = Pitch - Pitch_Zero;
    //超过特定角度停止运行
    if(fabs(Roll_error)>15 || fabs(Pitch_error)>15){ Motor_A =0,Motor_B =0,Motor_C =0,Start_Flag=0;MOTORA_OFF;MOTORB_OFF;MOTORC_OFF;}
    Motor_A = constrain_short(Motor_A, -10000, 10000);                     //PWM限幅
    Motor_B = constrain_short(Motor_B, -10000, 10000);                     //PWM限幅
//    if(Motor_C>0) Motor_C = Motor_C + 120;
//    else if(Motor_C<0) Motor_C = Motor_C - 120;
//    printf("%f,%f\n",gyro_pitch,Motor_C);
    Motor_C = constrain_short(Motor_C, -3500, 3500);                       //PWM限幅
//    printf("%hd,%hd,%hd\n",Motor_A,Motor_B,Motor_C);
    /************************电机控制****************************************************************************************************************************/
    if(Start_Flag==0)
    {
        MOTORA_OFF;
        MOTORB_OFF;                                       //刹车
        MOTORC_OFF;
        MotorCtrl3W(0,0,0);                             //飞轮 独轮都停止
    }
    else if(Start_Flag==1)
    {
        MOTORA_OFF;
        MOTORB_OFF;                                       //刹车
        MOTORC_ON;
        MotorCtrl3W(0,0,Motor_C);                       //飞轮刹车 独轮启动
    }
    else if(Start_Flag==2)
    {
        MOTORA_ON;
        MOTORB_ON;                                       //飞轮启动
        MOTORC_OFF;
        MotorCtrl3W(Motor_A,Motor_B,0);                 //飞轮启动 独轮刹车
    }
    else if(Start_Flag==3)
    {
        MOTORA_ON;
        MOTORB_ON;
        MOTORC_ON;
        MotorCtrl3W(Motor_A, Motor_B, Motor_C);         //飞轮 独轮都启动
    }

    encoder_clear_count(ENCODER_1_QUADDEC);
    encoder_clear_count(ENCODER_2_QUADDEC);
    encoder_clear_count(ENCODER_3_QUADDEC);

}
/**************************************************************************
X轴平衡PID控制,角度环
**************************************************************************/
float R_balance_Control(float Angle,float *Angle_Zero,float Gyro)
{
//    static unsigned int n;                         //计数，用来改变零点
    float PWM,Bias;
    static float error;
    Bias=Angle-*Angle_Zero;                                              //获取偏差
    error+=Bias;                                                         //偏差累积
    error = constrain_float(error, -120, 120);                            //积分限幅
    PWM=R_Balance_KP*Bias + R_Balance_KI*error + Gyro*R_Balance_KD/10;   //获取最终数值
//    printf("%f,%f,%f,%f,%f\n",PWM,R_Balance_KP,Bias,R_Balance_KI,error);
    PWM = constrain_float(PWM, -9000, 9000);                            //输出限幅
    if(Start_Flag==0) PWM=0,error=0;                                     //停止时参数清零

    return PWM;
}
/**************************************************************************
Y轴平衡PID控制,角度环
**************************************************************************/
float P_balance_Control(float Angle,float Angle_Zero,float Gyro)
{
    float PWM,Bias;
    static float error;
    Bias=Angle-Angle_Zero;                                               //获取偏差
    error+=Bias;                                                         //偏差累积
    error = constrain_float(error, -30, 30);                            //积分限幅
    PWM=P_Balance_KP*Bias + P_Balance_KI*error + Gyro*P_Balance_KD;   //获取最终数值
//    printf("%f,%f,",P_Balance_KP,Bias);
    if(Start_Flag==0) PWM=0,error=0;                                     //停止时参数清零
//    printf("%f,%f\n",PWM,Start_Flag);
    return PWM;
}
/**************************************************************************
Z轴平衡PID控制,角度环
**************************************************************************/
float Y_balance_Control(float Angle,float Angle_Zero,float Gyro)
{
    float PWM,Bias;
    static float error;
    Gyro = constrain_float(Gyro, -3000, 3000);                            //取值限幅
    Bias=Angle-Angle_Zero;                                               //获取偏差（偏差值需要重新计算，不然会有跳变）
    error+=Bias;                                                         //偏差累积
    error = constrain_float(error, -30, 30);                            //积分限幅
    PWM=Y_Balance_KP*Bias + Y_Balance_KI*error + Gyro*Y_Balance_KD/10;   //获取最终数值
    if(Start_Flag==0) PWM=0,error=0;                                     //停止时参数清零

    return PWM;
}

/**************************************************************************
速度PI控制,速度正反馈环
**************************************************************************/
float Velocity_Control_A(int encoder)
{
    static float Encoder,Encoder_Integral;
    float Velocity,Encoder_Least;

    Encoder_Least = (float)encoder;                                                   //速度滤波
    Encoder *= 0.7;                                                              //一阶低通滤波器
    Encoder += Encoder_Least*0.3;                                                //一阶低通滤波器
    Encoder_Integral += Encoder;                                               //积分出位移
    Encoder_Integral = constrain_float(Encoder_Integral, -2600, 2600);        //积分限幅
    Velocity = Encoder * R_Velocity_KP + Encoder_Integral * R_Velocity_KI/100; //获取最终数值
    Velocity = constrain_float(Velocity, -5000, 5000);
    if(Start_Flag==0) Encoder_Integral=0,Encoder=0,Velocity=0;                 //停止时参数清零
    return Velocity;
}

/**************************************************************************
速度PI控制,速度正反馈环
**************************************************************************/
float Velocity_Control_B(int encoder)
{
    static float Encoder,Encoder_Integral;
    float Velocity,Encoder_Least;

    Encoder_Least = (float)encoder;                                                   //速度滤波
    Encoder *= 0.7;                                                              //一阶低通滤波器
    Encoder += Encoder_Least*0.3;                                                //一阶低通滤波器
    Encoder_Integral += Encoder;                                               //积分出位移
    Encoder_Integral = constrain_float(Encoder_Integral, -2600, 2600);        //积分限幅
    Velocity = Encoder * R_Velocity_KP + Encoder_Integral * R_Velocity_KI/100; //获取最终数值
    Velocity = constrain_float(Velocity, -5000, 5000);
    if(Start_Flag==0) Encoder_Integral=0,Encoder=0,Velocity=0;                 //停止时参数清零
    return Velocity;
}

/**************************************************************************
C电机速度PI控制,速度正反馈环
**************************************************************************/
//float Velocity_Control_C(int encoder)
//{
//    static float Encoder,Encoder_Integral;
//    float Velocity,Encoder_Least;
//
//    Encoder_Least = (float)encoder- (float)Move_distance;                                  //速度滤波
//    Encoder *= 0.7;                                                            //一阶低通滤波器
//    Encoder += Encoder_Least*0.3;                                              //一阶低通滤波器

//    Encoder_Integral += Encoder - Move_distance;                             //积分出位移
//    Encoder_Integral = constrain_float(Encoder_Integral, -500, 500);        //积分限幅
//    Velocity = Encoder * P_Velocity_KP + Encoder_Integral * P_Velocity_KI;   //获取最终数值
//
//    if(Start_Flag==0) Encoder_Integral=0,Encoder=0,Velocity=0;               //停止时参数清零
//    return Velocity;
//}

