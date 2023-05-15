#pragma section all "cpu0_dsram"
#include "head.h"

void device_init()
{
    /*��ʼ���ĸ�ָʾ��*/
    gpio_init(LED1, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(LED2, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(LED3, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(LED4, GPO, GPIO_LOW, GPO_PUSH_PULL);

    /*��ʼ�������ֵ�ת������GPIO*/
    gpio_init(dir1, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(dir2, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(dir3, GPO, GPIO_LOW, GPO_PUSH_PULL);
//    gpio_init(PWM_CH3, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    // pwm_set_duty(PWM_CH3, 500);
    // gpio_init(Nsleep, GPO, GPIO_HIGH, GPO_PUSH_PULL);


    /*��ʼ��������״̬*/
    gpio_init(stop1, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(stop2, GPO, GPIO_HIGH, GPO_PUSH_PULL);


    /*��ʼ���н����ֵ�ʹ������,Ϊ��ʱ���Թ���*/

    /*��ʼ������������*/
    encoder_dir_init(ENCODER_1_DIR, ENCODER_1_DIR_PULSE, ENCODER_1_DIR_DIR);
    encoder_dir_init(ENCODER_2_DIR, ENCODER_2_DIR_PULSE, ENCODER_2_DIR_DIR);
//    encoder_dir_init(ENCODER_3_DIR, ENCODER_3_DIR_PULSE, ENCODER_3_DIR_DIR);

    /*��Ļ��ʼ��*/
    tjrc_setSt7735();
    tjrc_st7735_clean(0X5458);//���ñ�����ɫΪGBLUE

    /*��ʼ��������*/
    icm20602_init();

    /*��ʼ�����ߴ���ͨ��*/
    wireless_uart_init();

    /*��ʼ��PWM��ͨ��*/
    pwm_init(PWM_CH1, 17000, 8000);
    pwm_init(PWM_CH2, 17000, 8000);
    pwm_init(PWM_CH3, 17000, 8000);

// //    pwm_init(PWM_CH3, 17000, 8000);
//     pwm_set_duty(PWM_CH1, 10000);
//     pwm_set_duty(PWM_CH2, 10000);


    /*��ʼ����ʱ���ж�*/
    pit_ms_init(PIT0,100);
    pit_us_init(PIT1,5000);     // 5ms

//    /*��ʼ�����ڽ����ж�*/
    uart_init(UART_2,115200,WIRELESS_UART_RX_PIN,WIRELESS_UART_TX_PIN);
    uart_rx_interrupt(UART_2,1);
}

int core0_main(void)
{
    clock_init();                   // ��ȡʱ��Ƶ��
    debug_init();                   // ��ʼ��Ĭ�ϵ��Դ���
    device_init();
    cpu_wait_event_ready();         // �ȴ����к��ĳ�ʼ�����
	while (TRUE) {

	}
}

#pragma section all restore
