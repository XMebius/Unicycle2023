/*
 * Kalman.h
 *
 *  Created on: 2023��5��5��
 *      Author: Mebius
 */

#ifndef CODE_KALMAN_H_
#define CODE_KALMAN_H_

#include "head.h"


//��ȡ�������˲������������
void getKalmanPosition(float* Pitch,float* Roll,float* Yaw, float* gyro_x,float* gyro_y,float* gyro_z);
//����Ŀ������˲��㷨
void tjrc_kalman(float angle_m, float gyro_m , float* angle_kalman,float*angle_dot);

#endif /* CODE_KALMAN_H_ */
