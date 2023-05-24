#pragma section all "cpu0_dsram"
#include "head.h"

void cpu0_device_init()
{
    /*初始化四个指示灯*/
    gpio_init(LED1, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(LED2, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(LED3, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(LED4, GPO, GPIO_LOW, GPO_PUSH_PULL);

    /*初始化三个轮的转动方向GPIO*/
    gpio_init(dir1, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(dir2, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(dir3, GPO, GPIO_LOW, GPO_PUSH_PULL);

    /*初始化动量轮状态*/
    gpio_init(stop1, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(stop2, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(Nsleep, GPO, GPIO_HIGH, GPO_PUSH_PULL); /*初始化行进车轮的使能引脚,为高时可以工作*/

    /*初始化三个编码器*/
    encoder_quad_init(ENCODER_1_QUADDEC, ENCODER_1_QUADDEC_A, ENCODER_1_QUADDEC_B);
    encoder_quad_init(ENCODER_2_QUADDEC, ENCODER_2_QUADDEC_A, ENCODER_2_QUADDEC_B);
    encoder_quad_init(ENCODER_3_QUADDEC, ENCODER_3_QUADDEC_A, ENCODER_3_QUADDEC_B);

    /*初始化陀螺仪*/
    icm20602_init();

    /*初始化无线串口通信*/
    wireless_uart_init();

    /*初始化PWM波通道*/
    pwm_init(PWM_CH1, 17000, 10000);
    pwm_init(PWM_CH2, 17000, 10000);
    pwm_init(PWM_CH3, 17000, 0);

    /*初始化定时器中断*/
//    pit_ms_init(PIT0,100);
    pit_ms_init(PIT1,15);     // 10ms

    /*初始化串口接收中断*/
    uart_init(UART_2,115200,WIRELESS_UART_RX_PIN,WIRELESS_UART_TX_PIN);
    uart_rx_interrupt(UART_2,1);

}


int core0_main(void)
{
    clock_init();                   // 获取时钟频率
    debug_init();                   // 初始化默认调试串口
    cpu0_device_init();
    cpu_wait_event_ready();         // 等待所有核心初始化完毕
	while (TRUE) {
	}
}

#pragma section all restore
