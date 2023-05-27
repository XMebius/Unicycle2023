/**
 * @file tjrc_binarization.c
 * @author YYYDS team (1951578@tongji.edu.cn)
 * @brief ͼ���ֵ����ش��룬��OTSU, AVG, SOBEL��
 * @version 0.1
 * @date 2022-04-01
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "tjrc_imageProc.h"
#include "stdlib.h"

/**
 * @brief OTSU����򷨣���ͼ����ֵ����1/2�ֱ��ʲ�����
 *
 * @param image ����Ҷ�ͼ��ָ��
 * @param col ͼ������
 * @param row ͼ������
 * @return uint8_t ��ֵ����ֵ����Χ��th_min-th_max(uint8_t) ע����ֵ��ȡֵ���ڲ��޷��������ƣ�
 */
uint8_t tjrc_binarization_otsu(const uint8_t *image, uint16_t col, uint16_t row)
{
    /* ����Ҷȵ����ȼ���128�� */
#define GrayScale 128
    static uint32_t pixelCount[GrayScale];
    static float pixelPro[GrayScale];
    /* ��ֵ�޷� */
    uint16_t th_max = 150, th_min = 20;
    uint32_t i, j, pixelSum = col * row;
    uint8_t threshold = 0;
    for (i = 0; i < GrayScale; i++)
    {
        pixelCount[i] = 0;
        pixelPro[i] = 0;
    }
    /* ͳ�ƻҶȼ���ÿ������������ͼ���еĸ��� */
    for (i = 0; i < row; i += 2)
    {
        for (j = 0; j < col; j += 2)
        {
            /* ������ֵ��Ϊ����������±� */
            pixelCount[(int32_t)image[i * col + j] / 2]++;
        }
    }
    /* ����ÿ������������ͼ���еı��� */
    float maxPro = 0.0;
    for (i = 0; i < GrayScale; i++)
    {
        pixelPro[i] = (float)pixelCount[i] / pixelSum;
        if (pixelPro[i] > maxPro)
        {
            maxPro = pixelPro[i];
        }
    }
    /* �����Ҷȼ�[0,255] */
    float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
    for (i = 0; i < GrayScale; i++) // i��Ϊ��ֵ
    {
        w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
        for (j = 0; j < GrayScale; j++)
        {
            if (j <= i) //��������
            {
                w0 += pixelPro[j];
                u0tmp += j * pixelPro[j];
            }
            else //ǰ������
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
 * @brief ��ͼ��ƽ���Ҷ�Ϊ��ֵ
 *
 * @param image �Ҷ�ͼ��ָ��
 * @param col ͼ������
 * @param row ͼ������
 * @return uint8_t ͼ��ƽ���Ҷ�
 */
uint8_t tjrc_binarization_avg(const uint8_t *image, uint16_t col, uint16_t row)
{
    uint32_t tv = 0;
    for (uint16_t i = 0; i < row; i++)
    {
        for (uint16_t j = 0; j < col; j++)
        {
            tv += image[i * col + j]; //�ۼ�
        }
    }
    uint8_t threshold = tv / col / row; //��ƽ��ֵ,����Խ��ԽС��ȫ��Լ35��������ĻԼ160��һ������´�Լ100
    return threshold;
}

/**
 * @brief ��256���Ҷ�ͼ������ֵ���ж�ֵ����0=0x00 1=0xff��
 *
 * @param threshold ͼ����ֵ
 * @param image_in (in)����Ҷ�ͼ����ʼ��ַ
 * @param image_out (out)�����ֵ��ͼ����ʼ��ַ
 * @param width ͼ����
 * @param height ͼ��߶�
 */
void tjrc_binarization_getBinImage(uint8_t threshold, const uint8_t *image_in, uint8_t *image_out, uint16_t width, uint16_t height)
{
    for (uint16_t i = 0; i < height; i++)
    {
        for (uint16_t j = 0; j < width; j++)
        {
            if (image_in[i * width + j] < threshold)
            {
                /* С����ֵ��ֵΪ0����ʾΪ��ɫ */
                image_out[i * width + j] = 0x00;
            }
            else
            {
                /* ������ֵ��ֵΪff����ʾΪ��ɫ */
                image_out[i * width + j] = 0xff;
            }
        }
    }
}

/**
 * @brief ����soble���ؼ�����ӵ�һ���Զ���ֵ���ؼ���㷨
 *
 * @param imageIn (in)����256���Ҷ�ͼ����ʼ��ַ
 * @param imageOut (out)�����ֵ��������Ϣ��ͼ����ʼ��ַ
 * @param width ͼ����
 * @param height ͼ��߶�
 */
void tjrc_sobel_autoThreshold(const uint8_t *imageIn, uint8_t *imageOut, uint16_t width, uint16_t height)
{
    /** ����˴�С */
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
            /* ���㲻ͬ�����ݶȷ�ֵ  */
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

            /* �ҳ��ݶȷ�ֵ���ֵ  */
            for (uint16_t k = 1; k < 4; k++)
            {
                if (temp[0] < temp[k])
                {
                    temp[0] = temp[k];
                }
            }

            /* ʹ�����ص����������ص�֮�͵�һ������    ��Ϊ��ֵ  */
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
//  * @brief    ����soble���ؼ�����ӵ�һ�ֱ��ؼ��
//  * @param    imageIn    ��������
//  * @param    imageOut   �������      ����Ķ�ֵ����ı�����Ϣ
//  * @param    Threshold  ��ֵ
//  * @return
//  * @note
//  * @date     2020/5/15
//  */
// void SobelThreshold(uint8_t imageIn[MT9V03X_H][MT9V03X_W], uint8_t Threshold)
//{
//    /** ����˴�С */
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
//            /* ���㲻ͬ�����ݶȷ�ֵ  */
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
//            /* �ҳ��ݶȷ�ֵ���ֵ  */
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
