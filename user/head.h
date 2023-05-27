/*
 * @Author: Yixuan Chen mebiusccc@proton.me
 * @Date: 2023-05-25 00:44:47
 * @LastEditors: Yixuan Chen mebiusccc@proton.me
 * @LastEditTime: 2023-05-27 17:04:24
 * @FilePath: \Unicycle\ReUnicycle\user\head.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*
 * head.h
 *
 *  Created on: 2023��5��5��
 *      Author: Mebius
 */

#ifndef USER_HEAD_H_
#define USER_HEAD_H_

#include "zf_common_headfile.h"
#include "isr_config.h"
#include "Kalman.h"
#include "pid.h"
#include "balance.h"
#include "talk.h"
#include "tjrc_st7735.h"
#include "tjrc_imageProc.h"

/*ָʾ��*/
#define LED1                    (P20_9)
#define LED2                    (P20_8)
#define LED3                    (P21_5)
#define LED4                    (P21_4)

/*�ж�*/
#define PIT0                            (CCU60_CH0 )
#define PIT1                            (CCU61_CH0 )

/*������*/
#define ENCODER_1_QUADDEC                 (TIM2_ENCOEDER)
#define ENCODER_1_QUADDEC_A               (TIM2_ENCOEDER_CH1_P33_7)
#define ENCODER_1_QUADDEC_B               (TIM2_ENCOEDER_CH2_P33_6)

#define ENCODER_2_QUADDEC                 (TIM6_ENCOEDER)
#define ENCODER_2_QUADDEC_A               (TIM6_ENCOEDER_CH1_P20_3)
#define ENCODER_2_QUADDEC_B               (TIM6_ENCOEDER_CH2_P20_0)

#define ENCODER_3_QUADDEC                 (TIM5_ENCOEDER)
#define ENCODER_3_QUADDEC_A               (TIM5_ENCOEDER_CH1_P10_3)
#define ENCODER_3_QUADDEC_B               (TIM5_ENCOEDER_CH2_P10_1)

/*���ߴ���*/
#define WIRELESS_UART_INDEX         (UART_2)
#define WIRELESS_UART_BUAD_RATE     (115200)
#define WIRELESS_UART_TX_PIN        (UART2_RX_P10_6)
#define WIRELESS_UART_RX_PIN        (UART2_TX_P10_5)
#define WIRELESS_UART_RTS_PIN       (P10_2)

/*��ͨ��PWM*/
#define PWM_CH1         (ATOM2_CH7_P11_12)
#define PWM_CH2         (ATOM2_CH4_P11_9)
#define PWM_CH3         (ATOM1_CH0_P15_5)
//#define PWM_CH3         (P15_6)
#define dir1                    (P02_5)
#define stop1                   (P02_6)
#define dir2                    (P20_12)
#define stop2                   (P20_11)
#define dir3                    (P15_6)
#define Nsleep                  (P15_2)


/*������*/
/*ʹ������IIC����,ע��Ҫ��libraries���ͷ�ļ����!!!*/
#define GRAVITY 9.81    //�������ٶ�
#define Gyro_Parameter ( PI / 180 )

#endif /* USER_HEAD_H_ */
