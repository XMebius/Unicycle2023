/*
 * Kalman.c
 *
 *  Created on: 2023年5月5日
 *      Author: Mebius
 */

#include "Kalman.h"

/***********************卡尔曼滤波参数*************************/
const float     dt = 0.01;
const float     Q_angle = 0.0002;
float           Q_gyro  = 0.0003;                 //角速度方差
float           R_angle = 0.02;                     //测量噪声的方差
const char      C_0     = 1;
float q_bias = 0;                                   //角速度偏差
float angle_err;                                    //角度偏差
float PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
float P[2][2] = {{ 1, 0 },{ 0, 1 }};

/**
 * @brief 获取位姿数据（角度和角速度）
 * @param angle 卡尔曼滤波估计的角度
 * @param gyro 卡尔曼滤波估计的角速度
 */
void getKalmanPosition(float* Pitch,float* Roll,float* Yaw, float* gyro_x,float* gyro_Pitch,float* gyro_Roll)
{
    icm20602_get_acc();
    icm20602_get_gyro();

    float gyrox,gyroy,gyroz;
    float accx,accy,accz;

    gyrox=-(float)((float)icm20602_gyro_x*2000/(32768) )*(float)Gyro_Parameter;
    gyroy=-(float)((float)icm20602_gyro_y*2000/(32768) )*(float)Gyro_Parameter;
    gyroz=-(float)((float)icm20602_gyro_z*2000/(32768) )*(float)Gyro_Parameter;

    accx=(float) icm20602_acc_x/4096;
    accy=(float) icm20602_acc_y/4096;
    accz=(float) icm20602_acc_z/4096;

    tjrc_kalman(accy,-gyroz,Pitch,gyro_Pitch);
    tjrc_kalman(accz,gyroy,Roll,gyro_Roll);

//    printf("%f,%f,%f,%f,%f,%f\n",accy*40,*Pitch,accz*40,*Roll,*gyro_Pitch,*gyro_Roll);

//    tjrc_kalman(accz,gyroy,Pitch,gyro_Pitch);
//    tjrc_kalman(accy,-gyroz,Roll,gyro_Roll);

//    printf("%f,%f,",*Pitch,accy);
//    printf("%f,%f\n",*Roll,accz);
}

/**
 * @brief 卡尔曼滤波在这里的主要思想是，使用陀螺仪测得的角速度作为系统输入，加速度计可以通过分解重量得到角度作为量测量。
 * @param angle_m 原始角度
 * @param gyro_m 原始角速度
 * @param angle_kalman （输出）卡尔曼滤波后的角度
 * @param angle_dot  （输出）卡尔曼滤波后的角速度
 */
void tjrc_kalman(float angle_m, float gyro_m , float* angle_kalman,float*angle_dot)
{
    /* 先验估计，即求角加速度积分 */
    *angle_kalman += (gyro_m - q_bias) * dt;

    /* 求先验估计的协方差矩阵 */
    P[0][0] += (Q_angle - P[0][1] - P[1][0]) * dt;
    P[0][1] += (-P[1][1]) * dt;
    P[1][0] += (-P[1][1]) * dt;
    P[1][1] += Q_gyro * dt;

    angle_err = angle_m - *angle_kalman;

    PCt_0 = C_0 * P[0][0];
    PCt_1 = C_0 * P[1][0];

    /* 计算卡尔曼增益系数 */
    E = R_angle + C_0 * PCt_0;
    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;

    /* 更新协方差矩阵 */
    t_0 = PCt_0;
    t_1 = C_0 * P[0][1];
    P[0][0] -= K_0 * t_0;
    P[0][1] -= K_0 * t_1;
    P[1][0] -= K_1 * t_0;
    P[1][1] -= K_1 * t_1;

    /* 卡尔曼最优估计 */
    *angle_kalman += K_0 * angle_err;
    q_bias += K_1 * angle_err;
    *angle_dot = gyro_m - q_bias;
}



