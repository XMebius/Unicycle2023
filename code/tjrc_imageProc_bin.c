/**
 * @file tjrc_binarization.c
 * @author YYYDS team (1951578@tongji.edu.cn)
 * @brief 图像二值化相关代码，如OTSU, AVG, SOBEL等
 * @version 0.1
 * @date 2022-04-01
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "tjrc_imageProc.h"
#include "stdlib.h"

/**
 * @brief OTSU（大津法）求图像阈值（减1/2分辨率采样）
 *
 * @param image 输入灰度图像指针
 * @param col 图像列数
 * @param row 图像行数
 * @return uint8_t 二值化阈值（范围：th_min-th_max(uint8_t) 注意阈值的取值受内部限幅常量控制）
 */
uint8_t tjrc_binarization_otsu(const uint8_t *image, uint16_t col, uint16_t row)
{
    /* 定义灰度调整等级：128档 */
#define GrayScale 128
    static uint32_t pixelCount[GrayScale];
    static float pixelPro[GrayScale];
    /* 阈值限幅 */
    uint16_t th_max = 150, th_min = 20;
    uint32_t i, j, pixelSum = col * row;
    uint8_t threshold = 0;
    for (i = 0; i < GrayScale; i++)
    {
        pixelCount[i] = 0;
        pixelPro[i] = 0;
    }
    /* 统计灰度级中每个像素在整幅图像中的个数 */
    for (i = 0; i < row; i += 2)
    {
        for (j = 0; j < col; j += 2)
        {
            /* 将像素值作为计数数组的下标 */
            pixelCount[(int32_t)image[i * col + j] / 2]++;
        }
    }
    /* 计算每个像素在整幅图像中的比例 */
    float maxPro = 0.0;
    for (i = 0; i < GrayScale; i++)
    {
        pixelPro[i] = (float)pixelCount[i] / pixelSum;
        if (pixelPro[i] > maxPro)
        {
            maxPro = pixelPro[i];
        }
    }
    /* 遍历灰度级[0,255] */
    float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
    for (i = 0; i < GrayScale; i++) // i作为阈值
    {
        w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
        for (j = 0; j < GrayScale; j++)
        {
            if (j <= i) //背景部分
            {
                w0 += pixelPro[j];
                u0tmp += j * pixelPro[j];
            }
            else //前景部分
            {
                w1 += pixelPro[j];
                u1tmp += j * pixelPro[j];
            }
        }
        u0 = u0tmp / w0;
        u1 = u1tmp / w1;
        u = u0tmp + u1tmp;
        deltaTmp = w0 * (u0 - u) * (u0 - u) + w1 * (u1 - u) * (u1 - u);
        if (deltaTmp > deltaMax)
        {
            deltaMax = deltaTmp;
            threshold = (uint8_t)i;
        }
    }
    threshold -= 1;

    if (threshold * 2 > th_max)
        threshold = th_max / 2;
    if (threshold * 2 < th_min)
        threshold = th_min / 2;

    return threshold * 2;
}

/**
 * @brief 求图像平均灰度为阈值
 *
 * @param image 灰度图像指针
 * @param col 图像列数
 * @param row 图像行数
 * @return uint8_t 图像平均灰度
 */
uint8_t tjrc_binarization_avg(const uint8_t *image, uint16_t col, uint16_t row)
{
    uint32_t tv = 0;
    for (uint16_t i = 0; i < row; i++)
    {
        for (uint16_t j = 0; j < col; j++)
        {
            tv += image[i * col + j]; //累加
        }
    }
    uint8_t threshold = tv / col / row; //求平均值,光线越暗越小，全黑约35，对着屏幕约160，一般情况下大约100
    return threshold;
}

/**
 * @brief 将256级灰度图像按照阈值进行二值化（0=0x00 1=0xff）
 *
 * @param threshold 图像阈值
 * @param image_in (in)输入灰度图像起始地址
 * @param image_out (out)输出二值化图像起始地址
 * @param width 图像宽度
 * @param height 图像高度
 */
void tjrc_binarization_getBinImage(uint8_t threshold, const uint8_t *image_in, uint8_t *image_out, uint16_t width, uint16_t height)
{
    for (uint16_t i = 0; i < height; i++)
    {
        for (uint16_t j = 0; j < width; j++)
        {
            if (image_in[i * width + j] < threshold)
            {
                /* 小于阈值赋值为0，显示为黑色 */
                image_out[i * width + j] = 0x00;
            }
            else
            {
                /* 大于阈值赋值为ff，显示为白色 */
                image_out[i * width + j] = 0xff;
            }
        }
    }
}

/**
 * @brief 基于soble边沿检测算子的一种自动阈值边沿检测算法
 *
 * @param imageIn (in)输入256级灰度图像起始地址
 * @param imageOut (out)输出二值化边沿信息的图像起始地址
 * @param width 图像宽度
 * @param height 图像高度
 */
void tjrc_sobel_autoThreshold(const uint8_t *imageIn, uint8_t *imageOut, uint16_t width, uint16_t height)
{
    /** 卷积核大小 */
    const int32_t KERNEL_SIZE = 3;
    const int32_t xStart = KERNEL_SIZE / 2;
    const int32_t xEnd = width - KERNEL_SIZE / 2;
    const int32_t yStart = KERNEL_SIZE / 2;
    const int32_t yEnd = height - KERNEL_SIZE / 2;

    int32_t temp[4];
    for (int32_t i = yStart; i < yEnd; i++)
    {
        for (int32_t j = xStart; j < xEnd; j++)
        {
            /* 计算不同方向梯度幅值  */
            /* 90 deg  */
            temp[0] = -(int32_t)imageIn[(i - 1) * width + j - 1] + (int32_t)imageIn[(i - 1) * width + j + 1]   // -1,  0,  1
                      - (int32_t)imageIn[i * width + j - 1] + (int32_t)imageIn[i * width + j + 1]              // -1,  0,  1
                      - (int32_t)imageIn[(i + 1) * width + j - 1] + (int32_t)imageIn[(i + 1) * width + j + 1]; // -1,  0,  1
            /* 0 deg  */
            temp[1] = -(int32_t)imageIn[(i - 1) * width + j - 1] + (int32_t)imageIn[(i + 1) * width + j - 1]   // -1, -1, -1
                      - (int32_t)imageIn[(i - 1) * width + j] + (int32_t)imageIn[(i + 1) * width + j]          //  0,  0,  0
                      - (int32_t)imageIn[(i - 1) * width + j + 1] + (int32_t)imageIn[(i + 1) * width + j + 1]; //  1,  1,  1

            /* 45 deg  */
            temp[2] = -(int32_t)imageIn[(i - 1) * width + j] + (int32_t)imageIn[i * width + j - 1]             //  0, -1, -1
                      - (int32_t)imageIn[i * width + j + 1] + (int32_t)imageIn[(i + 1) * width + j]            //  1,  0, -1
                      - (int32_t)imageIn[(i - 1) * width + j + 1] + (int32_t)imageIn[(i + 1) * width + j - 1]; //  1,  1,  0
            /* 135 deg  */
            temp[3] = -(int32_t)imageIn[(i - 1) * width + j] + (int32_t)imageIn[i * width + j + 1]             // -1, -1,  0
                      - (int32_t)imageIn[i * width + j - 1] + (int32_t)imageIn[(i + 1) * width + j]            // -1,  0,  1
                      - (int32_t)imageIn[(i - 1) * width + j - 1] + (int32_t)imageIn[(i + 1) * width + j + 1]; //  0,  1,  1

            temp[0] = abs(temp[0]);
            temp[1] = abs(temp[1]);
            temp[2] = abs(temp[2]);
            temp[3] = abs(temp[3]);

            /* 找出梯度幅值最大值  */
            for (uint16_t k = 1; k < 4; k++)
            {
                if (temp[0] < temp[k])
                {
                    temp[0] = temp[k];
                }
            }

            /* 使用像素点邻域内像素点之和的一定比例    作为阈值  */
            temp[3] = (int32_t)imageIn[(i - 1) * width + j - 1] + (int32_t)imageIn[(i - 1) * width + j] + (int32_t)imageIn[(i - 1) * width + j + 1] + (int32_t)imageIn[i * width + j - 1] + (int32_t)imageIn[i * width + j] + (int32_t)imageIn[i * width + j + 1] + (int32_t)imageIn[(i + 1) * width + j - 1] + (int32_t)imageIn[(i + 1) * width + j] + (int32_t)imageIn[(i + 1) * width + j + 1];

            if (temp[0] > temp[3] / 20.0f)
            {
                imageOut[i * width + j] = IMAGE_COLOR_BLACK;
                imageOut[i * width + j + 1] = IMAGE_COLOR_BLACK;
                imageOut[i * width + j - 1] = IMAGE_COLOR_BLACK;

            }
            else
            {
                imageOut[i * width + j] = IMAGE_COLOR_WHITE;
            }
        }
    }
}

///**
//  * @brief    基于soble边沿检测算子的一种边沿检测
//  * @param    imageIn    输入数组
//  * @param    imageOut   输出数组      保存的二值化后的边沿信息
//  * @param    Threshold  阈值
//  * @return
//  * @note
//  * @date     2020/5/15
//  */
// void SobelThreshold(uint8_t imageIn[MT9V03X_H][MT9V03X_W], uint8_t Threshold)
//{
//    /** 卷积核大小 */
//    int32_t KERNEL_SIZE = 3;
//    int32_t xStart = KERNEL_SIZE / 2;
//    int32_t xEnd = MT9V03X_W - KERNEL_SIZE / 2;
//    int32_t yStart = KERNEL_SIZE / 2;
//    int32_t yEnd = MT9V03X_H - KERNEL_SIZE / 2;
//    int32_t i, j, k;
//    int32_t temp[4];
//    uint8_t imageOut[MT9V03X_H][MT9V03X_W];
//    for (i = yStart; i < yEnd; i++)
//    {
//        for (j = xStart; j < xEnd; j++)
//        {
//            /* 计算不同方向梯度幅值  */
//            /* 90 deg  */
//            temp[0] = -(int32_t)imageIn[i - 1][j - 1] + (int32_t)imageIn[i - 1][j + 1]     // -1,  0,  1
//                - (int32_t)imageIn[i][j - 1] + (int32_t)imageIn[i][j + 1]                  // -1,  0,  1
//                - (int32_t)imageIn[i + 1][j - 1] + (int32_t)imageIn[i + 1][j + 1];         // -1,  0,  1
//            /* 0 deg  */
//            temp[1] = -(int32_t)imageIn[i - 1][j - 1] + (int32_t)imageIn[i + 1][j - 1]     // -1, -1, -1
//                - (int32_t)imageIn[i - 1][j] + (int32_t)imageIn[i + 1][j]                  //  0,  0,  0
//                - (int32_t)imageIn[i - 1][j + 1] + (int32_t)imageIn[i + 1][j + 1];         //  1,  1,  1
//
//            /* 45 deg  */
//            temp[2] = -(int32_t)imageIn[i - 1][j] + (int32_t)imageIn[i][j - 1]             //  0, -1, -1
//                - (int32_t)imageIn[i][j + 1] + (int32_t)imageIn[i + 1][j]                  //  1,  0, -1
//                - (int32_t)imageIn[i - 1][j + 1] + (int32_t)imageIn[i + 1][j - 1];         //  1,  1,  0
//            /* 135 deg  */
//            temp[3] = -(int32_t)imageIn[i - 1][j] + (int32_t)imageIn[i][j + 1]             // -1, -1,  0
//                - (int32_t)imageIn[i][j - 1] + (int32_t)imageIn[i + 1][j]                  // -1,  0,  1
//                - (int32_t)imageIn[i - 1][j - 1] + (int32_t)imageIn[i + 1][j + 1];         //  0,  1,  1
//
//            temp[0] = abs(temp[0]);
//            temp[1] = abs(temp[1]);
//            temp[2] = abs(temp[2]);
//            temp[3] = abs(temp[3]);
//
//            /* 找出梯度幅值最大值  */
//            for (k = 1; k < 4; k++)
//            {
//                if (temp[0] < temp[k])
//                {
//                    temp[0] = temp[k];
//                }
//            }
//
//            if (temp[0] > Threshold)
//            {
//                imageOut[i][j] = 255;
//            }
//            else
//            {
//                imageOut[i][j] = 1;
//            }
//        }
//    }
//    for (int32_t i = 0; i < MT9V03X_H; i++)
//    {
//        for (int32_t j = 0; j < MT9V03X_W; j++)
//        {
//            imageIn[i][j] = imageOut[i][j];
//        }
//    }
//}
