/*
 * balance.c
 *
 *  Created on: 2023年5月5日
 *      Author: Mebius
 */
#include "balance.h"

int Encoder_A,Encoder_B,Encoder_C,Encoder_D;                          //编码器的脉冲计数
int Encoder_C_Sum;                                                  //行进电机编码器的累计值

int PWM_R,PWM_Y,PWMA_accel,PWMB_accel,PWMC_accel;                       //PWM中间量

float P_Balance_KP=2000.0, P_Balance_KI=0,P_Balance_KD=0;                //左右方向
float R_Balance_KP=7000 ,  R_Balance_KI=80   ,  R_Balance_KD=450  ;          //AB电机左右倾角4500 15 275
float Y_Balance_KP=-1.05  ,  Y_Balance_KI=00.0  ,  Y_Balance_KD=-25.0  ;      //AB电机旋转角度

float R_Velocity_KP=00  ,  R_Velocity_KI=0   ;                               //AB电机速度环
float P_Velocity_KP=9.15,P_Velocity_KI=0.105;                                //C电机速度环
int Move_distance=30 , Move_distance_MAX=170;
//静置后  Pitch_Zero=1.35  不静置 Pitch_Zero=3.82
float Roll_Zero=0.03, Roll_Zero1=5.63;      //Roll_Zero1：记录值
float Pitch_Zero=1.20,Pitch_Zero1=5.63;     //Pitch_Zero1:
float Yaw_Zero=0.0;                     //XY轴角度零点，与机械有关，影响稳定性

int Flag_A,Flag_B,Flag_C,Flag_D,Flag_E,Flag_F,Flag_G,Flag_H,Flag_I;     //标志位
float Motor_A,Motor_B,Motor_C,Motor_D;                                  //电机PWM变量

float Roll_error=0,Pitch_error=0,Yaw_error=0;
unsigned char  Start_Flag=1;                    //启动标志
unsigned char  delay_30,delay_flag;             //30毫秒标志

float Pitch=0.0f,Roll=0.0f,Yaw=0.0f;
float gyrox=0.0f,gyroy=0.0f,gyroz=0.0f;

void Balance(void)
{
    static int num;

    Encoder_A = encoder_get_count(TIM5_ENCOEDER_CH1_P10_3);   //左电机 母板上编码器1，小车前进为负值
        Encoder_B = encoder_get_count(TIM4_ENCOEDER_CH1_P02_8);   //右电机 母板上编码器2，小车前进为正值
    #ifndef cascade_pid           //串级PID的速度检测在pid函数中
        Encoder_C = encoder_get_count(TIM6_ENCOEDER_CH1_P20_3);   //行进电机 母板上编码器3，小车前进为正值
    #endif
        Encoder_C_Sum += Encoder_C;                     //行进电机编码器累加

    getKalmanPosition(&Pitch,&Roll,&Yaw,&gyrox,&gyroy,&gyroz);//获取姿态信息

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

/*Roll方向调节*/
float R_balance_Control(float Angle,float *Angle_Zero,float Gyro)
{
//    static unsigned int n;                         //计数，用来改变零点
    float PWM,Bias;
    static float error;
    Bias=Angle-*Angle_Zero;                                              //获取偏差
    error+=Bias;                                                         //偏差累积
    error = constrain_float(error, -120, 120);                            //积分限幅
    PWM=R_Balance_KP*Bias + R_Balance_KI*error + Gyro*R_Balance_KD/10;   //获取最终数值
    PWM = constrain_float(PWM, -9000, 9000);                            //输出限幅
    if(Start_Flag==0) PWM=0,error=0;                                     //停止时参数清零

    return PWM*60;
}

float P_balance_Control(float Angle,float Angle_Zero,float Gyro)
{
    float PWM,Bias;
    static float error;
    Bias=Angle-Angle_Zero;                                               //获取偏差
    error+=Bias;                                                         //偏差累积
    error = constrain_float(error, -30, 30);                            //积分限幅
    PWM=P_Balance_KP*Bias + P_Balance_KI*error + Gyro*P_Balance_KD/10;   //获取最终数值
    if(Start_Flag==0) PWM=0,error=0;                                     //停止时参数清零
    return PWM;
}

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

float Velocity_Control_C(int encoder)
{
    static float Encoder,Encoder_Integral;
    float Velocity,Encoder_Least;

    Encoder_Least = (float)encoder- (float)Move_distance;                                  //速度滤波
    Encoder *= 0.7;                                                            //一阶低通滤波器
    Encoder += Encoder_Least*0.3;                                              //一阶低通滤波器
    Encoder_Integral += Encoder - Move_distance;                             //积分出位移
    Encoder_Integral = constrain_float(Encoder_Integral, -500, 500);        //积分限幅
    Velocity = Encoder * P_Velocity_KP + Encoder_Integral * P_Velocity_KI;   //获取最终数值
    if(Start_Flag==0) Encoder_Integral=0,Encoder=0,Velocity=0;               //停止时参数清零
    return Velocity;
}
