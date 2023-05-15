/*
 * Kalman.h
 *
 *  Created on: 2023年5月5日
 *      Author: Mebius
 */

#ifndef CODE_KALMAN_H_
#define CODE_KALMAN_H_

#include "head.h"


//获取卡尔曼滤波后的六个参数
void getKalmanPosition(float* Pitch,float* Roll,float* Yaw, float* gyro_x,float* gyro_y,float* gyro_z);
//具体的卡尔曼滤波算法
void tjrc_kalman(float angle_m, float gyro_m , float* angle_kalman,float*angle_dot);

#endif /* CODE_KALMAN_H_ */
