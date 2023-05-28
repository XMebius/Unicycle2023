#include "head.h"
#pragma section all "cpu1_dsram"

void cpu1_device_init()
{
    tft180_set_dir(TFT180_CROSSWISE);                                           
    tft180_init();

    mt9v03x_init();


}
void camera_service(){
//    tft180_displayimage03x((uint8 *)mt9v03x_image[0], 100, 60);
    uint8* image_in=(uint8*)mt9v03x_image[0];

    mt9v03x_finish_flag = 0;
    uint8* image_out = tjrc_imageProc(image_in);
    tft180_displayimage03x((const uint8 *)image_out, 150, 110);

}
void core1_main(void)
{
    disable_Watchdog();                     
    interrupt_global_enable(0);             
    cpu1_device_init();

    cpu_wait_event_ready();
    while (TRUE)
    {
//        tft180_displayimage03x((const uint8 *)mt9v03x_image, 160, 128);
        camera_service();
    }
}
#pragma section all restore
