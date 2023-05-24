#include "zf_common_headfile.h"
#pragma section all "cpu1_dsram"

void cpu1_device_init()
{
    /*��Ļ��ʼ��*/
    tft180_set_dir(TFT180_CROSSWISE);                                           // ��Ҫ�Ⱥ��� ��Ȼ��ʾ����
    tft180_init();

    /*����ͷ��ʼ��*/
    mt9v03x_init();

    /*��Ŵ��г�ʼ��*/

}

void core1_main(void)
{
    disable_Watchdog();                     // �رտ��Ź�
    interrupt_global_enable(0);             // ��ȫ���ж�

    cpu_wait_event_ready();
    while (TRUE)
    {
        tft180_displayimage03x((const uint8 *)mt9v03x_image, 160, 128);
    }
}
#pragma section all restore
