/**
 * @file tjrc_imageProc.h
 * @author YYYDS team. (1951578@tongji.edu.cn)
 * @brief 
 * @version 2.6.1
 * @date 2022-07-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef __TJRC_IMAGEPROC_H__
#define __TJRC_IMAGEPROC_H__

#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <stdarg.h>
#include "head.h"

/**
 * @brief ͼ������߶�
 */
#define IMAGE_WIDTH     120
#define IMAGE_HEIGHT    80


/**
 * @brief �����������ೣ���������߸��ţ������Ҫ������в���(3,width-3)
 */
#define IMAGE_COL_KEEPOUT_PIXEL 3

#define IMAGE_ROW_KEEPOUT_PIXEL 3

/**
 * @brief ͼ�����Ȥ�ĸ߶ȣ�0-i��
 */
#define IMAGE_INTEREST_REGION 73

/**
 * @brief ͼ����ɫ�궨��
 */
#define IMAGE_COLOR_BLACK 0x00
#define IMAGE_COLOR_WHITE 0xff

 /**
  * @brief �жϹյ����ֵ��Ӱ���жϹյ��������
  */
#define INFLECTION_THRESHOLD 10

#define INFLECTION_ADJUST_PIXEL 2
  /**
   * @brief �������յ���С��ֱ����
   */
#define INFLECTION_MINIMUM_DISTANCE 10


#define LINE_INFO_MAX_PIXEL 80

#define tjrc_abs(X) (X >= 0 ? X : -X)

#define WEIGHT_MODE_NORMOL 0x01
#define WEIGHT_MODE_LOWER 0x02

typedef struct
{
    uint8_t pixel_cnt;
    uint8_t x_left_edgeCnt;
    uint8_t x_right_edgeCnt;
    /* index��Ӧͼ�����ص�y���꣬�統ǰ��Ϊ(10,20)����x_left[20] = 10 */
    uint8_t x_left[LINE_INFO_MAX_PIXEL];
    uint8_t x_right[LINE_INFO_MAX_PIXEL];
    /* ���ұ����ѵ��߱�־������Ϊ0 */
    uint8_t x_left_findEdge[LINE_INFO_MAX_PIXEL];
    uint8_t x_right_findEdge[LINE_INFO_MAX_PIXEL];
}line_info;

enum inflection_bitSet
{
    inflection_upper_left = 0x01,
    inflection_lower_left = 0x02,
    inflection_upper_right = 0x04,
    inflection_lower_right = 0x08,
    inflection_upper = 0x05,
    inflection_lower = 0x0A,
    inflection_left = 0x03,
    inflection_right = 0x0C,
    inflection_all = 0x0F
};

typedef struct
{
    uint8_t findFlag;
    uint8_t x_upper_left;
    uint8_t y_upper_left;
    uint8_t x_lower_left;
    uint8_t y_lower_left;
    uint8_t x_upper_right;
    uint8_t y_upper_right;
    uint8_t x_lower_right;
    uint8_t y_lower_right;
}inflection_info;


uint8_t* tjrc_imageProc(const uint8_t* image_p);

/* ��ֵ�������� */
uint8_t tjrc_binarization_otsu(const uint8_t* image, uint16_t col, uint16_t row);
uint8_t tjrc_binarization_avg(const uint8_t* image, uint16_t col, uint16_t row);
void tjrc_binarization_getBinImage(uint8_t threshold, const uint8_t* image_in, uint8_t* image_out, uint16_t width, uint16_t height);
void tjrc_sobel_autoThreshold(const uint8_t* imageIn, uint8_t* imageOut, uint16_t width, uint16_t height);

/* ���ߣ��յ㣬���ߺ����� */
void tjrc_imageProc_searchEdge_x(const uint8_t* image, line_info* line_info_out);
void tjrc_imageProc_fineInflection(const line_info* line_info_in, inflection_info* inflection_info_out);
uint8_t tjrc_imageProc_patchLine(line_info* line_info_out, inflection_info* inflection_info_in);
void tjrc_imageProc_updateImage(uint8_t* image, line_info* line_info_in, inflection_info* inflection_info_in);

/* ����Ԫ���жϺ����� */
uint8_t check_gridLine(const uint8_t* image);
uint8_t check_separate(const uint8_t* image);
int check_roundabout(const inflection_info* inflection_info_in, const line_info* line_info_in);
int check_roundabout_left(const inflection_info* inflection_info_in, const line_info* line_info_in);
uint8_t check_120turn(const line_info* line_info_in, uint8_t mode, uint8_t find[2]);

/* ͼ��log��Ϣ���Ժ�����һ��Ϊ�գ�����ʱ��������� */
int tjrc_log(char* format, ...);

#endif // !__TJRC_IMAGEPROC_H__
