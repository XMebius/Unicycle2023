/*
 * balance.h
 *
 *  Created on: 2023Äê5ÔÂ5ÈÕ
 *      Author: Mebius
 */

#ifndef CODE_BALANCE_H_
#define CODE_BALANCE_H_
//#define cascade_pid
#include "head.h"



void Balance(void);
float R_balance_Control(float Angle,float *Angle_Zero,float Gyro);
float P_balance_Control(float Angle,float Angle_Zero,float Gyro);
float Y_balance_Control(float Angle,float Angle_Zero,float Gyro);
//float Yaw_balance_Control(float Gyro,float Gyro_control);
float Velocity_Control_A(int encoder);
float Velocity_Control_B(int encoder);
float Velocity_Control_C(int encoder);

#endif /* CODE_BALANCE_H_ */
