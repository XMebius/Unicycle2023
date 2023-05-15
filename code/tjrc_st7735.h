/*
 * st7735.h
 *
 *  Created on: 2023��3��21��
 *      Author: Mebius
 */

#ifndef CODE_TJRC_ST7735_H_
#define CODE_TJRC_ST7735_H_

#include "stdio.h"
#include "stdint.h"

#define ST7735_SCL &MODULE_P33,9
#define ST7735_SDA &MODULE_P33,8
#define ST7735_RES &MODULE_P33,12
#define ST7735_DC &MODULE_P33,11
#define ST7735_CS &MODULE_P33,10
#define ST7735_BL &MODULE_P33,13

#define PIXEL_WIDTH 128
#define PIXEL_HEIGHT 160

/* 565��ɫ��� */
/*   |-----|------|-----|
       r(5)  g(6)   b(5)      */
#define RGB565_WHITE 0xFFFF
#define RGB565_BLACK 0x0000
#define RGB565_BLUE 0x001F
#define RGB565_BRED 0XF81F
#define RGB565_GRED 0XFFE0
#define RGB565_GBLUE 0X07FF
#define RGB565_RED 0xF800
#define RGB565_MAGENTA 0xF81F
#define RGB565_GREEN 0x07E0
#define RGB565_CYAN 0x7FFF
#define RGB565_YELLOW 0xFFE0
#define RGB565_BROWN 0XBC40     //��ɫ
#define RGB565_BRRED 0XFC07     //�غ�ɫ
#define RGB565_GRAY 0X8430      //��ɫ
//GUI��ɫ
#define RGB565_DARKBLUE 0X01CF     //����ɫ
#define RGB565_LIGHTBLUE 0X7D7C //ǳ��ɫ
#define RGB565_GRAYBLUE 0X5458  //����ɫ

/* ������Ļ */
void tjrc_setSt7735(void);
/* �����Ļ�������ñ���ɫ */
void tjrc_st7735_clean(uint16_t color);
/* ��ʾ�ַ��� */
void tjrc_st7735_dispStr612(uint16_t dx, uint16_t dy, uint8_t *str, uint16_t color);
/* ��ʾ���� */
void tjrc_st7735_dispInt32(uint16_t dx, uint16_t dy, int num, uint16_t color);
/* ��ʾ������(3λС��) */
void tjrc_st7735_dispFloat32(uint16_t dx, uint16_t dy, float num, uint16_t color);
/* ��ͼ���� */
void tjrc_st7735_drawPoint(uint16_t x, uint16_t y, uint16_t color);
void tjrc_st7735_drawRectangle(uint16_t x, uint16_t y,uint16_t width, uint16_t height, uint16_t color);
void tjrc_st7735_drawBox(uint16_t x, uint16_t y,uint16_t width, uint16_t height, uint16_t color);
/* ��ʾ��ͼ�� */
void tjrc_st7735_dispImage(uint8_t *image, uint16_t width, uint16_t height, uint16_t x, uint16_t y, uint16_t color);
void tjrc_st7735_dispbin(uint8_t *image, uint16_t width, uint16_t height, uint16_t x, uint16_t y, uint16_t color,uint8_t th);
void tjrc_st7735_dispImage_gray(uint8_t *image, uint16_t width, uint16_t height, uint16_t x, uint16_t y);
/* ��ȡ��Ļ��ǰ״̬ */
uint8_t tjrc_st7735_getBusy(void);
void tjrc_st7735_setBusy(uint8_t state);

#endif /* CODE_TJRC_ST7735_H_ */
