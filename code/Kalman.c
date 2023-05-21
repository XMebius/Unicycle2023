/*
 * Kalman.c
 *
 *  Created on: 2023��5��5��
 *      Author: Mebius
 */

#include "Kalman.h"

/***********************�������˲�����*************************/
const float     dt = 0.01;
const float     Q_angle = 0.0002;
float           Q_gyro  = 0.0003;                 //���ٶȷ���
float           R_angle = 0.02;                     //���������ķ���
const char      C_0     = 1;
float q_bias = 0;                                   //���ٶ�ƫ��
float angle_err;                                    //�Ƕ�ƫ��
float PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
float P[2][2] = {{ 1, 0 },{ 0, 1 }};

/**
 * @brief ��ȡλ�����ݣ��ǶȺͽ��ٶȣ�
 * @param angle �������˲����ƵĽǶ�
 * @param gyro �������˲����ƵĽ��ٶ�
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
 * @brief �������˲����������Ҫ˼���ǣ�ʹ�������ǲ�õĽ��ٶ���Ϊϵͳ���룬���ٶȼƿ���ͨ���ֽ������õ��Ƕ���Ϊ��������
 * @param angle_m ԭʼ�Ƕ�
 * @param gyro_m ԭʼ���ٶ�
 * @param angle_kalman ��������������˲���ĽǶ�
 * @param angle_dot  ��������������˲���Ľ��ٶ�
 */
void tjrc_kalman(float angle_m, float gyro_m , float* angle_kalman,float*angle_dot)
{
    /* ������ƣ�����Ǽ��ٶȻ��� */
    *angle_kalman += (gyro_m - q_bias) * dt;

    /* ��������Ƶ�Э������� */
    P[0][0] += (Q_angle - P[0][1] - P[1][0]) * dt;
    P[0][1] += (-P[1][1]) * dt;
    P[1][0] += (-P[1][1]) * dt;
    P[1][1] += Q_gyro * dt;

    angle_err = angle_m - *angle_kalman;

    PCt_0 = C_0 * P[0][0];
    PCt_1 = C_0 * P[1][0];

    /* ���㿨��������ϵ�� */
    E = R_angle + C_0 * PCt_0;
    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;

    /* ����Э������� */
    t_0 = PCt_0;
    t_1 = C_0 * P[0][1];
    P[0][0] -= K_0 * t_0;
    P[0][1] -= K_0 * t_1;
    P[1][0] -= K_1 * t_0;
    P[1][1] -= K_1 * t_1;

    /* ���������Ź��� */
    *angle_kalman += K_0 * angle_err;
    q_bias += K_1 * angle_err;
    *angle_dot = gyro_m - q_bias;
}



