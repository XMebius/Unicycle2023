/*
 * talk.c
 *
 *  Created on: 2023年5月5日
 *      Author: Mebius
 */

#include "talk.h"
extern float P_Balance_KP, P_Balance_KI,P_Balance_KD;

static void tip()
{
    gpio_toggle_level(LED1);
    gpio_toggle_level(LED2);
    gpio_toggle_level(LED3);
    gpio_toggle_level(LED4);
}

static int int_pow(int src, int p){
    int base = src;
    if(p==0){
        return 1;
    }
    else{
        for(int i=1;i<p;i++){
            src *= base;
        }
    }
    return src;
}

/*
 * 形式为(p12300),数字不超过五位数
 * */
void wirelessRecCallBack()
{
    char buffer[28];
    static uint8_t rx_cnt = 0;
    static uint8_t state = 0;
    int data = 0;
    wireless_uart_read_buff((uint8_t *)&buffer[rx_cnt], 28);
    if (buffer[rx_cnt] == '(') {
        state = 1;
        rx_cnt++;
    }
    else if (state == 1 && ((buffer[rx_cnt] >= 'a' && buffer[rx_cnt] <= 'z') || (buffer[rx_cnt] >= 'A' && buffer[rx_cnt] <= 'Z'))) {
        rx_cnt++;
        state = 2;
    }
    else if (state == 2 && buffer[rx_cnt] >= '0' && buffer[rx_cnt] <= '9' && buffer[rx_cnt] != ')'){
        rx_cnt++;
    }
    else if (buffer[rx_cnt] == ')'){
        for (uint8_t i = 0; i + 2 < rx_cnt; i++){
                data += (buffer[2 + i] - 48) * int_pow(10, rx_cnt - 3 - i);
                printf("%c,",buffer[2+i]);
        }
//        printf("%d",data);
        tip();
        switch(buffer[1]){
            case 'p':
                P_Balance_KP=data;
//                printf("%f,%f\n",P_Balance_KP,data);
                tjrc_st7735_dispInt32(0,0,data,0XFFFF);
                break;
            case 'i':
                P_Balance_KI=data;
//                printf("%f\n",P_Balance_KI);
                tjrc_st7735_dispInt32(0,10,data,0XFFFF);
                break;
            case 'd':
                P_Balance_KD=data;
//                printf("%f\n",P_Balance_KD);
                tjrc_st7735_dispInt32(0,20,data,0XFFFF);
                break;
        }
        rx_cnt = 0;
        state = 0;
    }
}
