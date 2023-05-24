#include "zf_common_headfile.h"
#pragma section all "cpu1_dsram"

void cpu1_device_init()
{
    /*屏幕初始化*/
    tft180_set_dir(TFT180_CROSSWISE);                                           // 需要先横屏 不然显示不下
    tft180_init();

    /*摄像头初始化*/
    mt9v03x_init();

    /*电磁传感初始化*/

}

void core1_main(void)
{
    disable_Watchdog();                     // 关闭看门狗
    interrupt_global_enable(0);             // 打开全局中断

    cpu_wait_event_ready();
    while (TRUE)
    {
        tft180_displayimage03x((const uint8 *)mt9v03x_image, 160, 128);
    }
}
#pragma section all restore
