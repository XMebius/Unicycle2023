#include "tjrc_imageProc.h"

static void patchLine(uint8_t y0, uint8_t y1, uint8_t *line_x_out);
static float fit_slope(const uint8_t *line, line_info *line_info_in);
static float weighted_line(const uint8_t *line, line_info *line_info_in, uint8_t mode);

uint8_t circle_flag = 0;
static uint8_t cross_flag = 0;
static uint8_t separate_flag = 0;
static uint8_t gridline_flag = 0;

extern uint16_t cnt;
uint8_t left_turn_count = 0;
uint8_t right_turn_count = 0;

/**
 * @brief ͼ������ȫ�ֱ��������ڴ���cpu0�����˶�ѧ����
 *
 */
float line_bias = 0;
float camera_slope = 0, camera_pixelError = 0;
uint8_t weighted_mode = 0;
uint8_t is_separate = 0;
uint8_t is_gridline = 0;

/**
 * @brief ��ֵ��ͼ����󣨶�ֵ������Ҳ�����ڻ��Ʊ��ߣ����ߵ���Ϣ��
 * �ߴ�Ϊ120*80=9600Byte=9.375KB(x2)
 *
 */
uint8_t image_bin[IMAGE_HEIGHT][IMAGE_WIDTH];

uint8_t roundabout_flag = 0;
uint8_t separate_turn = 0;

/**
 * @brief
 *
 * @param image_p
 * @return uint8_t*
 */
uint8_t *tjrc_imageProc(const uint8_t *image_p)
{
    uint8_t *image_bin_p = image_bin[0];
    static uint8_t midline[IMAGE_HEIGHT];
    static line_info edge_line;
    static inflection_info inflections;

    is_separate = 0;

    /* ��һ������ֵ�� */
    static uint8_t threshold_smooth = 30;
    uint8_t threshold = tjrc_binarization_otsu(image_p, IMAGE_WIDTH, IMAGE_HEIGHT);
    uint8_t threshold_bias = 7;
    threshold += threshold_bias;
    threshold_smooth = (uint8_t)(0.8 * threshold_smooth + 0.2 * threshold);
    tjrc_binarization_getBinImage(threshold_smooth, image_p, image_bin_p, IMAGE_WIDTH, IMAGE_HEIGHT);
    /* ������⻷�����߷���ǿ�����ʹ��sobel���� */
    //    tjrc_sobel_autoThreshold(image_p, image_bin_p, IMAGE_WIDTH, IMAGE_HEIGHT);

    /* �ڶ��������ߣ���յ㣬���� */
    is_separate = check_separate(image_bin_p);
    tjrc_imageProc_searchEdge_x(image_bin_p, &edge_line);
    tjrc_imageProc_fineInflection(&edge_line, &inflections);
    is_gridline = check_gridLine(image_bin_p);
    extern float Pitch;
    if(tjrc_abs(Pitch)<0.2f)
    {
        uint8_t patch_res = tjrc_imageProc_patchLine(&edge_line, &inflections);
    }
    //    printf("[patch]%d\r\n",patch_res);

    /* ������������ͼ�� */
    tjrc_imageProc_updateImage(image_bin_p, &edge_line, &inflections);

    /* ���Ĳ�����������Ϣ�洢��ȫ�ֱ������ں�0���� */
    for (uint8_t i = IMAGE_HEIGHT - IMAGE_ROW_KEEPOUT_PIXEL; i >= IMAGE_HEIGHT - IMAGE_INTEREST_REGION; i--)
    {
        if (IMAGE_HEIGHT - edge_line.pixel_cnt > i)
            continue;
        /* �������� */
        midline[i] = (edge_line.x_left[i] + edge_line.x_right[i]) / 2;
    }
    /* �����ǰ����С��10�����ж�Ϊ���ߣ���������� */
    if (edge_line.pixel_cnt > 10)
    {
        if (separate_flag == 1)
        {
            weighted_mode = WEIGHT_MODE_LOWER;
        }
        else
        {
            weighted_mode = WEIGHT_MODE_NORMOL;
        }
        /* ����������б�ʣ�����90�ȣ������߶��μ�Ȩƽ��ƫ�� */
        camera_pixelError = weighted_line(midline, &edge_line, weighted_mode) + line_bias;
        camera_slope = fit_slope(midline, &edge_line);
    }

//    char str2[40];
//    sprintf(str2, "c:%f", camera_pixelError);
//    tjrc_st7735_dispStr612(88, 146, (uint8_t *)str2, RGB565_GREEN);
//    tjrc_st7735_dispStr612(88, 146, (uint8_t *)str2, RGB565_GREEN);

    return image_bin_p;
}

/**
 * @brief ���м���������չ���������߷���
 *
 * @param image [in](uint8*)ͼ��ָ��(IMAGE_HEIGHT*IMAGE_WIDTH)
 * @param line_info_out [out]������Ϣ�ṹ�壬�洢���ߺ����꣬�Ƿ��ߵ���Ϣ
 */
void tjrc_imageProc_searchEdge_x(const uint8_t *image, line_info *line_info_out)
{
    /* ��ά�����ع���������� */
    uint8_t *image_p[IMAGE_HEIGHT];
    for (uint8_t i = 0; i < IMAGE_HEIGHT; i++)
    {
        image_p[i] = (uint8_t *)&image[i * IMAGE_WIDTH];
    }
    line_info_out->x_left_edgeCnt = 0;
    line_info_out->x_right_edgeCnt = 0;

    /* �������Ͻ���ˮƽ���ߣ���ײ����м�Ϊ��� */
    uint8_t p_start = IMAGE_WIDTH / 2;
    uint8_t P_left_findEdgeCnt = 0;
    uint8_t P_right_findEdgeCnt = 0;

    /* ΢����ʼ���ߵ㣬����ʼ���Ǻ��ߣ������ҷ���Ѱ�ҵ�һ���׵� */
    if (image_p[IMAGE_HEIGHT - IMAGE_ROW_KEEPOUT_PIXEL][p_start] == IMAGE_COLOR_BLACK)
    {
        uint8_t p_start_bias = 0;
        while (p_start_bias < IMAGE_WIDTH / 2 - IMAGE_COL_KEEPOUT_PIXEL)
        {
            p_start_bias++;
            if (image_p[IMAGE_HEIGHT - IMAGE_ROW_KEEPOUT_PIXEL][p_start - p_start_bias] != IMAGE_COLOR_BLACK)
            {
                p_start -= p_start_bias;
                break;
            }
            if (image_p[IMAGE_HEIGHT - IMAGE_ROW_KEEPOUT_PIXEL][p_start + p_start_bias] != IMAGE_COLOR_BLACK)
            {
                p_start += p_start_bias;
                break;
            }
        }
        if (p_start_bias >= IMAGE_WIDTH / 2 - IMAGE_COL_KEEPOUT_PIXEL)
        {
            return;
        }
    }
    /* ����ÿһ��Ѱ�ұ��� */
    for (uint16_t i = IMAGE_HEIGHT - IMAGE_ROW_KEEPOUT_PIXEL; i >= IMAGE_HEIGHT - IMAGE_INTEREST_REGION; i--)
    {
        /* �������ұ߽�x������������� */
        uint8_t p_left = p_start;
        uint8_t p_right = p_start + 1;
        uint8_t p_mid = p_start;

        /* ���ұ߽��������߱�־λ */
        uint8_t p_left_findEdge = 0;
        uint8_t p_right_findEdge = 0;

        /* ������ߣ��˳��оݣ�����ͼ��߽��������������ص�Ϊ�� */
        while (p_left - 1 > IMAGE_COL_KEEPOUT_PIXEL)
        {
            if (image_p[i][p_left] == IMAGE_COLOR_BLACK &&
                image_p[i][p_left - 1] == IMAGE_COLOR_BLACK)
            {
                P_left_findEdgeCnt++;
                p_left_findEdge = 1;
                break;
            }
            p_left--;
        }
        /* �Ҳ����ߣ��˳��оݣ�����ͼ��߽��������������ص�Ϊ�� */
        while (p_right + 1 < IMAGE_WIDTH - IMAGE_COL_KEEPOUT_PIXEL)
        {
            if (image_p[i][p_right] == IMAGE_COLOR_BLACK &&
                image_p[i][p_right + 1] == IMAGE_COLOR_BLACK)
            {
                P_right_findEdgeCnt++;
                p_right_findEdge = 1;
                break;
            }
            p_right++;
        }

        p_mid = (p_right + p_left) / 2;
        /* ������һ�����ߵ����ĺ�����Ϊ�������ߵ��е� */
        p_start = p_mid;

        /* �����˳����оݣ�����������Ϊblack������ҲΪblack�˳�ѭ�� */
        if (tjrc_abs(p_right - p_left) <= 2 && i < IMAGE_HEIGHT - 20)
        {
            tjrc_log("[edge_x]line break row:%d\n", i);
            break;
        }
        /* �洢��������㵽line_info */
        line_info_out->pixel_cnt = IMAGE_HEIGHT - i;
        line_info_out->x_left[i] = p_left;
        line_info_out->x_right[i] = p_right;
        line_info_out->x_left_findEdge[i] = p_left_findEdge;
        line_info_out->x_right_findEdge[i] = p_right_findEdge;
        line_info_out->x_left_edgeCnt = P_left_findEdgeCnt;
        line_info_out->x_right_edgeCnt = P_right_findEdgeCnt;
        tjrc_log("[edge_x]left:%d,right:%d,mid:%d\n", p_left, p_right, (p_left + p_right) / 2);
        // image_p[i][(p_left + p_right) / 2] = IMAGE_COLOR_BLACK;
    }
}

/**
 * @brief ͨ��������Ϣ���ķ���Ѱ�ҹյ������������������յ�ķ���������ߵĶ��׵���
 *
 * @param line_info_in [in]������Ϣ�ṹ�壬�洢���ߺ����꣬�Ƿ��ߵ���Ϣ
 * @param inflection_info_out [out]�յ�ṹ�壬�洢4�ǹյ��������Ƿ���ڵı�־λ
 */
void tjrc_imageProc_fineInflection(const line_info *line_info_in, inflection_info *inflection_info_out)
{
    const int16_t slope_threshold = 3;
    const uint8_t search_tolerance = 5;
    const uint8_t line_tolerance = 3;
    const uint8_t line_tolerance_t = 60;
    const uint8_t turn_tolerance = 13;
    const uint8_t rate = 2;
    /* ��չյ���Ѱ��־λ����λ��δ�ѵ��յ�״̬ */
    inflection_info_out->findFlag = 0;
    /* ��������ֹ������ */
    uint8_t search_begin = 0, search_end = 0;
    /* ������ȡ����������Ϣ */
    const uint8_t *line_left = line_info_in->x_left;
    const uint8_t *line_right = line_info_in->x_right;
    /* ���ڼ�¼���ڱ��ߺ�����Ĳ�ֵ */
    int8_t rate_slope[LINE_INFO_MAX_PIXEL];
    int8_t line_slope[LINE_INFO_MAX_PIXEL];

    uint8_t left_is_line = 1; // TODO
    uint8_t right_is_line = 1;

    uint8_t left_is_turn = 0;
    uint8_t right_is_turn = 0;
    left_turn_count = 0;
    right_turn_count = 0;

    uint8_t check_left_120turn = 0;
    uint8_t check_right_120turn = 0;

    /*______LEFT_______ */
    /* ��������ߵĺ������ֵ(��ʼ��涨Ϊ0) */
    for (uint8_t i = IMAGE_HEIGHT - IMAGE_INTEREST_REGION; i < IMAGE_HEIGHT; i++)
    {
        rate_slope[i] = (i == IMAGE_HEIGHT - IMAGE_INTEREST_REGION) ? 0 : line_left[i] - line_left[i - rate];
        line_slope[i] = (i == IMAGE_HEIGHT - IMAGE_INTEREST_REGION) ? 0 : line_left[i] - line_left[i - 1];
        tjrc_log("%d, ", rate_slope[i]);
    }
    tjrc_log("\n");
    for (uint8_t i = IMAGE_HEIGHT - IMAGE_INTEREST_REGION + 2; i < IMAGE_HEIGHT - 2; i++)
    {
        if (line_info_in->x_left_edgeCnt > line_tolerance_t)
        {
            left_is_line = 1;
            break;
        }
        if ((line_slope[i] >= 0 ? line_slope[i] : -line_slope[i]) >= line_tolerance)
        {
            left_is_line = 0; // TODO
            break;
        }
    }
    for (uint8_t i = IMAGE_HEIGHT - IMAGE_INTEREST_REGION; i < IMAGE_HEIGHT; i++)
    {
        if (line_left[i] > IMAGE_WIDTH / 2 && line_info_in->x_left_findEdge[i] == 1)
            left_turn_count++;
        if (line_right[i] < IMAGE_WIDTH / 2 && line_info_in->x_right_findEdge[i] == 1)
            right_turn_count++;
    }
    if (left_turn_count >= turn_tolerance)
        left_is_turn = 1;
    if (right_turn_count >= turn_tolerance)
        right_is_turn = 1;

//    uint8_t str[30];
//    sprintf(str,"%d,%d\n",left_turn_count,right_turn_count);
//    tjrc_asclin1_sendStr(str);
    if (!left_is_line && !left_is_turn)
    {
        /* ����һ����������Ѱ�����յ� */
        /* ����ȷ���յ��������Χ�����������м�ضϣ���ӽضϴ���ʼ��--->�ײ�-��Ե�����У� */
        if (IMAGE_HEIGHT - line_info_in->pixel_cnt > IMAGE_HEIGHT - IMAGE_INTEREST_REGION + 1)
        {
            search_begin = IMAGE_HEIGHT - line_info_in->pixel_cnt;
        }
        else
        {
            search_begin = IMAGE_HEIGHT - IMAGE_INTEREST_REGION + 1;
        }
        search_end = IMAGE_HEIGHT - search_tolerance;
        for (uint8_t i = search_begin; i < search_end - 2; i++)
        {
            /* �оݣ�����3�е�б������������������Ҫ������ͷ�����ܼܸߣ����ӣ� */
            if (rate_slope[i] >= -slope_threshold &&
                rate_slope[i + 1] >= -slope_threshold &&
                rate_slope[i + 2] < -slope_threshold)
            {
                /* ��־���ҵ��յ� */
                inflection_info_out->findFlag |= inflection_upper_left;
                inflection_info_out->x_upper_left = line_left[i + 1];
                inflection_info_out->y_upper_left = i + 1;
                tjrc_log("find inflection[LU] (%d,%d)\n", line_left[i + 1], i + 1);
                break;
            }
        }
        /* ���������������Ѱ�����յ� */
        search_begin = IMAGE_HEIGHT - search_tolerance;
        search_end = (inflection_info_out->findFlag & inflection_upper_left) ? inflection_info_out->y_upper_left + 10 : IMAGE_HEIGHT - IMAGE_INTEREST_REGION;
        for (uint8_t i = search_begin; i >= search_end + 2; i--)
        {

            /* �оݣ�����3�е�б������������������Ҫ������ͷ�����ܼܸߣ����ӣ� */
            if (rate_slope[i] <= slope_threshold &&
                rate_slope[i - 1] <= slope_threshold &&
                rate_slope[i - 2] > slope_threshold)
            {
                /* ��־���ҵ��յ� */
                inflection_info_out->findFlag |= inflection_lower_left;
                inflection_info_out->x_lower_left = line_left[i - 2];
                inflection_info_out->y_lower_left = i - 2;
                tjrc_log("find inflection[LL] (%d,%d)\n", line_left[i - 2], i - 2);
                break;
            }
        }
        /* ������·�ڵĹյ�ǿ������ */
        if (!(inflection_info_out->findFlag & inflection_lower_left))
        {
            uint8_t max = 0;
            uint8_t maxi = 0;
            uint8_t find_left[2];
            find_left[0] = 0;
            find_left[1] = 0;

            check_left_120turn = check_120turn(line_info_in, 0, find_left);
            if (check_left_120turn)
            {
                for (uint8_t i = find_left[1]; i < find_left[0]; i++)
                {
                    if (line_info_in->x_left[i] >= max)
                    {
                        max = line_info_in->x_left[i];
                        maxi = i;
                    }
                }
                inflection_info_out->findFlag |= inflection_lower_left;
                inflection_info_out->x_lower_left = max;
                inflection_info_out->y_lower_left = maxi;
                if (maxi < inflection_info_out->y_upper_left)
                {
                    inflection_info_out->findFlag &= 0x0E;
                    inflection_info_out->x_upper_left = 0;
                    inflection_info_out->y_upper_left = 0;
                }
                tjrc_log("find inflection[LL] (%d,%d)\n", max, maxi);
            }
        }
    }

    /*______RIGHT_______ */
    /* ��������ߵĺ������ֵ(��ʼ��涨Ϊ0) */
    for (uint8_t i = IMAGE_HEIGHT - IMAGE_INTEREST_REGION; i < IMAGE_HEIGHT; i++)
    {
        rate_slope[i] = (i == IMAGE_HEIGHT - IMAGE_INTEREST_REGION) ? 0 : line_right[i] - line_right[i - rate];
        line_slope[i] = (i == IMAGE_HEIGHT - IMAGE_INTEREST_REGION) ? 0 : line_right[i] - line_right[i - 1];
        tjrc_log("%d, ", rate_slope[i]);
    }
    tjrc_log("\n");
    for (uint8_t i = IMAGE_HEIGHT - IMAGE_INTEREST_REGION + 2; i < IMAGE_HEIGHT - 2; i++)
    {
        if (line_info_in->x_right_edgeCnt > line_tolerance_t)
        {
            right_is_line = 1;
            break;
        }
        if ((line_slope[i] >= 0 ? line_slope[i] : -line_slope[i]) >= line_tolerance)
        {
            right_is_line = 0; // TODO
            break;
        }
    }

    if (!right_is_line && !right_is_turn)
    {
        /* ����������������Ѱ���Ҳ�յ� */
        if (IMAGE_HEIGHT - line_info_in->pixel_cnt > IMAGE_HEIGHT - IMAGE_INTEREST_REGION + 1)
        {
            search_begin = IMAGE_HEIGHT - line_info_in->pixel_cnt;
        }
        else
        {
            search_begin = IMAGE_HEIGHT - IMAGE_INTEREST_REGION + 1;
        }
        search_end = IMAGE_HEIGHT - search_tolerance;
        for (uint8_t i = search_begin; i < search_end - 2; i++)
        {
            /* �оݣ�����3�е�б������������������Ҫ������ͷ�����ܼܸߣ����ӣ� */
            if (rate_slope[i] <= slope_threshold &&
                rate_slope[i + 1] <= slope_threshold &&
                rate_slope[i + 2] > slope_threshold)
            {
                /* ��־���ҵ��յ� */
                inflection_info_out->findFlag |= inflection_upper_right;
                inflection_info_out->x_upper_right = line_right[i + 1];
                inflection_info_out->y_upper_right = i + 1;
                tjrc_log("find inflection[RU] (%d,%d)\n", line_right[i + 1], i + 1);
                break;
            }
        }
        /* �����ģ���������Ѱ���Ҳ�յ� */
        search_begin = IMAGE_HEIGHT - search_tolerance;
        search_end = (inflection_info_out->findFlag & inflection_upper_right) ? inflection_info_out->y_upper_right + 10 : IMAGE_HEIGHT - IMAGE_INTEREST_REGION;
        for (uint8_t i = search_begin; i >= search_end + 2; i--)
        {
            /* �оݣ�����3�е�б������������������Ҫ������ͷ�����ܼܸߣ����ӣ� */
            if (rate_slope[i] >= -slope_threshold &&
                rate_slope[i - 1] >= -slope_threshold &&
                rate_slope[i - 2] < -slope_threshold)
            {
                /* ��־���ҵ��յ� */
                inflection_info_out->findFlag |= inflection_lower_right;
                inflection_info_out->x_lower_right = line_right[i - 2];
                inflection_info_out->y_lower_right = i - 2;
                tjrc_log("find inflection[RL] (%d,%d)\n", line_right[i - 2], i - 2);
                break;
            }
        }
        /* ������·�ڵĹյ�ǿ������ */
        if (!(inflection_info_out->findFlag & inflection_lower_right))
        {
            uint8_t min = 120;
            uint8_t mini = 0;
            uint8_t find_right[2];
            find_right[0] = 0;
            find_right[1] = 0;
            check_right_120turn = check_120turn(line_info_in, 1, find_right);
            if (check_right_120turn)
            {
                for (uint8_t i = find_right[1]; i < find_right[0]; i++)
                {
                    if (line_info_in->x_right[i] <= min)
                    {
                        min = line_info_in->x_right[i];
                        mini = i;
                    }
                }
                inflection_info_out->findFlag |= inflection_lower_right;
                inflection_info_out->x_lower_right = min;
                inflection_info_out->y_lower_right = mini;
                if (mini < inflection_info_out->y_upper_right)
                {
                    inflection_info_out->findFlag &= 0x0B;
                    inflection_info_out->x_upper_right = 0;
                    inflection_info_out->y_upper_right = 0;
                }
                tjrc_log("find inflection[RL] (%d,%d)\n", min, mini);
            }
        }
    }
}

/* �ڶ�Ȧ��־λ */
static uint8_t second_cycle_flag = 1;
/* �뻷���������� */
uint8_t in_roundabout_chance = 1;
static uint8_t in_separate_chance = 2;

/**
 * @brief ���ݹյ���Ϣ���Ե�ǰ������з�����в���
 *
 * @param line_info_out [out]������Ϣ�ṹ�壬�洢���ߺ����꣬�Ƿ��ߵ���Ϣ
 * @param inflection_info_in [in]�յ�ṹ�壬�洢4�ǹյ��������Ƿ���ڵı�־λ
 */
/* ���������Ի������д��¼�ʱ */
static uint16_t roundabout_cnt = 0;
static uint8_t count = 0, count1 = 0;
uint8_t tjrc_imageProc_patchLine(line_info *line_info_out, inflection_info *inflection_info_in){
    
}
/**
 * @brief ��������Ϣ���յ���Ϣ�����߸��µ��ӵ�ͼ��
 *
 * @param image [out]ͼ��
 * @param line_info_in [in]������Ϣ
 * @param inflection_info_in [in]�յ���Ϣ
 */
void tjrc_imageProc_updateImage(uint8_t *image, line_info *line_info_in, inflection_info *inflection_info_in)
{
    for (uint8_t i = IMAGE_HEIGHT - IMAGE_ROW_KEEPOUT_PIXEL; i >= IMAGE_HEIGHT - IMAGE_INTEREST_REGION; i--)
    {
        if (IMAGE_HEIGHT - line_info_in->pixel_cnt > i)
            continue;
        /* �������� */
        uint8_t x_mid = (line_info_in->x_left[i] + line_info_in->x_right[i]) / 2;
        image[i * IMAGE_WIDTH + x_mid] = 0x00;
        /* ���Ʊ��� */
        image[i * IMAGE_WIDTH + line_info_in->x_left[i]] = 0xAA;
        image[i * IMAGE_WIDTH + line_info_in->x_right[i]] = 0xAA;
    }
    /* ��ӹյ㣨0x33����ӳ�䵽��ɫ�� */
    if (inflection_info_in->findFlag & inflection_upper_left)
        image[inflection_info_in->y_upper_left * IMAGE_WIDTH + inflection_info_in->x_upper_left] = 0x33;
    if (inflection_info_in->findFlag & inflection_upper_right)
        image[inflection_info_in->y_upper_right * IMAGE_WIDTH + inflection_info_in->x_upper_right] = 0x33;
    if (inflection_info_in->findFlag & inflection_lower_left)
        image[inflection_info_in->y_lower_left * IMAGE_WIDTH + inflection_info_in->x_lower_left] = 0x33;
    if (inflection_info_in->findFlag & inflection_lower_right)
        image[inflection_info_in->y_lower_right * IMAGE_WIDTH + inflection_info_in->x_lower_right] = 0x33;
}

/**
 * @brief ���ڲ������������������������
 *
 * @param y0 ��ʼ���������
 * @param y1 ��ֹ���������
 * @param line_x_out [io]�߽ṹ��
 */
static void patchLine(uint8_t y0, uint8_t y1, uint8_t *line_x_out)
{
    if (y0 > IMAGE_HEIGHT || y0 < IMAGE_HEIGHT - IMAGE_INTEREST_REGION ||
        y1 > IMAGE_HEIGHT || y1 < IMAGE_HEIGHT - IMAGE_INTEREST_REGION)
    {
        tjrc_log("[error]patchLine invaild param %d,%d\n", y0, y1);
        return;
    }
    if (y0 > y1)
    {
        uint8_t temp;
        temp = y0;
        y0 = y1;
        y1 = temp;
    }
    float k = ((float)(line_x_out[y1] - line_x_out[y0])) / ((float)(y1 - y0));
    for (uint8_t i = y0; i < y1; i++)
    {
        line_x_out[i] = (uint8_t)((int8_t)line_x_out[y0] + (int8_t)(k * (float)(i - y0)));
    }
}

/**
 * @brief �жϵ�ǰ�������ް�����
 *
 * @param image
 * @return uint8_t
 */
uint8_t check_gridLine(const uint8_t *image)
{
    /* ����ɨ����ֹ�У���0��80�� */
    const uint8_t row_begin = 50, row_end = 75;
    /* ����ɨ������ߺ��߿��������ֵ */
    const uint8_t grid_min = 3, grid_max = 8; // sobel 5 10; otsu 3 8
    /* ��������ߺڿ�����ж���ֵ(������7��) */
    const uint8_t blocks_min = 5, blocks_max = 10;
    /* ���������ж�Ϊ�����ߵ���ȷ�жϴ��� */
    const uint8_t times_min = 3;
    /* ��¼��ȷ�жϴ��� */
    uint8_t times = 0;

    for (uint8_t y = row_begin; y <= row_end; y++)
    {
        uint8_t blocks = 0;
        uint8_t cursor = 0; //ָ��ջ�����α�
        const uint8_t *row_ptr = &(image[IMAGE_WIDTH * y]);

        /* ���������ǰ�� */
        for (uint8_t x = IMAGE_COL_KEEPOUT_PIXEL; x < IMAGE_WIDTH - IMAGE_COL_KEEPOUT_PIXEL; x++)
        {
            /* ����ǰΪ��ɫ���أ����ۼӺ�ɫ���ؿ�ȣ���Ϊ��ɫ�����������ۼӣ�
                            ���㵱ǰ�ڿ��ȣ������Ⱥ��ʣ�������һ�κϸ��ɫ������ */
            if (row_ptr[x] == IMAGE_COLOR_BLACK)
            {
                cursor++;
            }
            else
            {
                if (cursor >= grid_min && cursor <= grid_max)
                    blocks++;
                cursor = 0;
            }
        }
        /* �ڿ������������Ҫ�� */
        if (blocks >= blocks_min && blocks <= blocks_max)
        {
            times++;
        }
    }
    /* ��ȷ�жϴ�����������Ҫ�� */
    if (times >= times_min)
        return 1;
    else
        return 0;
}

/**
 * @brief ͼ��״̬���жϻ����������
 *
 * @param inflection_info_in �յ���Ϣ
 * @param line_info_in ������Ϣ
 * @return int 1-�ұ��߷��ϻ������� 0-������
 */
int check_roundabout(const inflection_info *inflection_info_in, const line_info *line_info_in)
{
#define EDGE_STATE_INIT 0
#define EDGE_STATE_LOWER_FIND 1
#define EDGE_STATE_UPPER_FIND 2
#define EDGE_STATE_MIDDLE_LOSS 3
#define EDGE_STATE_ERROR 4

    /* �о�1������޹յ㣬�Ҳ��Ϸ��йյ�, �Ҳ�ײ����� */
    if ((inflection_info_in->findFlag & inflection_upper_right) &&
        (inflection_info_in->findFlag & inflection_lower_right) &&
        !(inflection_info_in->findFlag & inflection_upper_left) &&
        !(inflection_info_in->findFlag & inflection_lower_left))
    {
        uint8_t edge_state = EDGE_STATE_INIT;
        /* ����״̬���жϱ����Ƕ��ߣ����ߣ����� */
        for (uint16_t i = IMAGE_HEIGHT - IMAGE_COL_KEEPOUT_PIXEL; i >= IMAGE_HEIGHT - IMAGE_INTEREST_REGION; i--)
        {
            switch (edge_state)
            {
            case EDGE_STATE_INIT:
                if (!line_info_in->x_right_findEdge[i] || i == IMAGE_HEIGHT - IMAGE_COL_KEEPOUT_PIXEL)
                    edge_state = EDGE_STATE_INIT;
                else
                    edge_state = EDGE_STATE_LOWER_FIND;
                break;
            case EDGE_STATE_LOWER_FIND:
                if (!line_info_in->x_right_findEdge[i])
                    edge_state = EDGE_STATE_MIDDLE_LOSS;
                break;
            case EDGE_STATE_MIDDLE_LOSS:
                if (line_info_in->x_right_findEdge[i])
                    edge_state = EDGE_STATE_UPPER_FIND;
                break;
            case EDGE_STATE_UPPER_FIND:
                if (!line_info_in->x_right_findEdge[i])
                    edge_state = EDGE_STATE_ERROR;
                break;
            default:
                break;
            }
        }
        if (edge_state == EDGE_STATE_UPPER_FIND || edge_state == EDGE_STATE_MIDDLE_LOSS)
        {
            roundabout_flag = 1;
            printf("find roundabout!\n");
            return 1;
        }
        else
        {
            return 0;
        }
    }
    return 0;
}


int check_roundabout_left(const inflection_info *inflection_info_in, const line_info *line_info_in)
{
#define EDGE_STATE_INIT 0
#define EDGE_STATE_LOWER_FIND 1
#define EDGE_STATE_UPPER_FIND 2
#define EDGE_STATE_MIDDLE_LOSS 3
#define EDGE_STATE_ERROR 4

    /* �о�1������޹յ㣬�Ҳ��Ϸ��йյ�, �Ҳ�ײ����� */
    if ((inflection_info_in->findFlag & inflection_upper_left) &&
        (inflection_info_in->findFlag & inflection_lower_left) &&
        !(inflection_info_in->findFlag & inflection_upper_right) &&
        !(inflection_info_in->findFlag & inflection_lower_right))
    {
        uint8_t edge_state = EDGE_STATE_INIT;
        /* ����״̬���жϱ����Ƕ��ߣ����ߣ����� */
        for (uint16_t i = IMAGE_HEIGHT - IMAGE_COL_KEEPOUT_PIXEL; i >= IMAGE_HEIGHT - IMAGE_INTEREST_REGION; i--)
        {
            switch (edge_state)
            {
            case EDGE_STATE_INIT:
                if (!line_info_in->x_left_findEdge[i] || i == IMAGE_HEIGHT - IMAGE_COL_KEEPOUT_PIXEL)
                    edge_state = EDGE_STATE_INIT;
                else
                    edge_state = EDGE_STATE_LOWER_FIND;
                break;
            case EDGE_STATE_LOWER_FIND:
                if (!line_info_in->x_left_findEdge[i])
                    edge_state = EDGE_STATE_MIDDLE_LOSS;
                break;
            case EDGE_STATE_MIDDLE_LOSS:
                if (line_info_in->x_left_findEdge[i])
                    edge_state = EDGE_STATE_UPPER_FIND;
                break;
            case EDGE_STATE_UPPER_FIND:
                if (!line_info_in->x_left_findEdge[i])
                    edge_state = EDGE_STATE_ERROR;
                break;
            default:
                break;
            }
        }
        if (edge_state == EDGE_STATE_UPPER_FIND || edge_state == EDGE_STATE_MIDDLE_LOSS)
        {
            roundabout_flag = 1;
            printf("find roundabout!\n");
            return 1;
        }
        else
        {
            return 0;
        }
    }
    return 0;
}

/**
 * @brief ��ǿ����·��120��յ���
 *
 * @param line_info_in ������Ϣ
 * @param mode 0-left 1-right
 * @param find �յ��ⷶΧ�������յ��������
 * @return uint8_t
 */
uint8_t check_120turn(const line_info *line_info_in, uint8_t mode, uint8_t find[2])
{
#define TURN_STATE_INIT 0x00
#define TURN_STATE_LOWER_LOSS 0x01
#define TURN_STATE_LOWER_HALF 0x02
#define TURN_STSTE_UPPER_HALF 0x03
#define TURN_STATE_UPPER_LOSS 0x04
#define TURN_STATE_ERROR 0x10
#define TURN_MODE_LEFT 0
#define TURN_MODE_RIGHT 1
    if (mode == TURN_MODE_LEFT)
    {
        uint8_t turn_state = TURN_STATE_INIT;
        uint8_t left_120turn_find = 0;
        /* ___________left______ */
        for (uint16_t i = IMAGE_HEIGHT - IMAGE_COL_KEEPOUT_PIXEL; i >= IMAGE_HEIGHT - IMAGE_INTEREST_REGION; i--)
        {
            switch (turn_state)
            {
            case TURN_STATE_INIT:
                if (!line_info_in->x_left_findEdge[i])
                    turn_state = TURN_STATE_LOWER_LOSS;
                else
                {
                    turn_state = TURN_STATE_LOWER_HALF;
                    find[0] = i;
                }

                break;
            case TURN_STATE_LOWER_LOSS:
                if (line_info_in->x_left_findEdge[i])
                {
                    turn_state = TURN_STATE_LOWER_HALF;
                    find[0] = i;
                }

                break;
            case TURN_STATE_LOWER_HALF:
                if (!line_info_in->x_left_findEdge[i])
                    turn_state = TURN_STATE_ERROR;
                else
                {
                    if (line_info_in->x_left[i - 1] >= line_info_in->x_left[i])
                        turn_state = TURN_STATE_LOWER_HALF;
                    else
                        turn_state = TURN_STSTE_UPPER_HALF;
                }

                break;
            case TURN_STSTE_UPPER_HALF:
                if (!line_info_in->x_left_findEdge[i])
                {
                    turn_state = TURN_STATE_UPPER_LOSS;
                    find[1] = i;
                }

                else
                {
                    if (line_info_in->x_left[i - 1] <= line_info_in->x_left[i])
                        turn_state = TURN_STSTE_UPPER_HALF;
                    else
                        turn_state = TURN_STATE_ERROR;
                }
                break;
            case TURN_STATE_UPPER_LOSS:
                left_120turn_find = 1;
                return 1;
                break;
            case TURN_STATE_ERROR:
                break;
            default:
                break;
            }
        }
        return 0;
    }

    if (mode == TURN_MODE_RIGHT)
    {
        uint8_t turn_state = TURN_STATE_INIT;
        uint8_t right_120turn_find = 0;
        /* ___________right______ */
        for (uint16_t i = IMAGE_HEIGHT - IMAGE_COL_KEEPOUT_PIXEL; i >= IMAGE_HEIGHT - IMAGE_INTEREST_REGION; i--)
        {
            switch (turn_state)
            {
            case TURN_STATE_INIT:
                if (!line_info_in->x_right_findEdge[i])
                    turn_state = TURN_STATE_LOWER_LOSS;
                else
                {
                    turn_state = TURN_STATE_LOWER_HALF;
                    find[0] = i;
                }

                break;
            case TURN_STATE_LOWER_LOSS:
                if (line_info_in->x_right_findEdge[i])
                {
                    turn_state = TURN_STATE_LOWER_HALF;
                    find[0] = i;
                }

                break;
            case TURN_STATE_LOWER_HALF:
                if (!line_info_in->x_right_findEdge[i])
                    turn_state = TURN_STATE_ERROR;
                else
                {
                    if (line_info_in->x_right[i - 1] <= line_info_in->x_right[i])
                        turn_state = TURN_STATE_LOWER_HALF;
                    else
                        turn_state = TURN_STSTE_UPPER_HALF;
                }

                break;
            case TURN_STSTE_UPPER_HALF:
                if (!line_info_in->x_right_findEdge[i])
                {
                    turn_state = TURN_STATE_UPPER_LOSS;
                    find[1] = i;
                }

                else
                {
                    if (line_info_in->x_right[i - 1] >= line_info_in->x_right[i])
                        turn_state = TURN_STSTE_UPPER_HALF;
                    else
                        turn_state = TURN_STATE_ERROR;
                }
                break;
            case TURN_STATE_UPPER_LOSS:
                right_120turn_find = 1;
                return 1;
                break;
            case TURN_STATE_ERROR:
                break;
            default:
                break;
            }
        }
        return 0;
    }
    return 120;
}

/**
 * @brief ��������ϲ��ֵ�������Ϣ
 *
 * @param image ͼ������
 * @return uint8_t 1-��⵽ 0-δ����
 */
uint8_t check_separate(const uint8_t *image)
{
    /* ����ɨ����ֹ�У���0��80�� */
    const uint8_t row_start = 20, row_end = 32;
    /* ����ɨ����ֹ�У�0-119�� */
    const uint8_t col_start = 3, col_end = 117;
    /* ����������� */
    int8_t count[12];
    uint8_t num = row_end - row_start;

    uint8_t up_cnt = 0, down_cnt = 0;
    //    up_cnt = 0, down_cnt = 0;

    for (int i = 0; i < num; i++)
    {
        count[i] = 0;
    }
    uint8_t m = 0;
    for (uint8_t x = row_start; x < row_end; x++)
    {
        for (uint8_t y = col_start; y < col_end; y++)
        {
            if (image[x * IMAGE_WIDTH + y] == IMAGE_COLOR_BLACK &&
                image[x * IMAGE_WIDTH + y + 1] == IMAGE_COLOR_BLACK)
            {
                count[m++] = y;
                break;
            }
        }
    }
    m = 0;
    for (uint8_t x = row_start; x < row_end; x++)
    {
        for (uint8_t y = col_end; y > col_start; y--)
        {
            if (image[x * IMAGE_WIDTH + y] == IMAGE_COLOR_BLACK &&
                image[x * IMAGE_WIDTH + y - 1] == IMAGE_COLOR_BLACK)
            {
                count[m] = y - count[m];
                m++;
                break;
            }
        }
    }

    for (uint8_t i = 0; i < num - 1; i++)
    {
        if (count[i] > count[i + 1] && count[i] != 0 && count[i + 1] != 0)
            up_cnt++;
    }

    // char str[40];
    // sprintf(str, "u:%d", up_cnt);
    // tjrc_st7735_dispStr612(56, 132, (uint8_t *)str, RGB565_GREEN);
    // tjrc_st7735_dispStr612(56, 132, (uint8_t *)str, RGB565_GREEN);

    if (up_cnt >= 9)
        return 1;
    else
        return 0;
}

/**
 * @brief ��ǰֱ��б��
 *
 * @param line ���ߺ���������mid[height]��ָ��
 * @param line_info_in ������Ϣ
 * @return float б�ʣ�-90deg ~ 90deg
 */
static float fit_slope(const uint8_t *line, line_info *line_info_in)
{
    float sum_xy = 0.0f, sum_xx = 0.0f;
    float sum_x = 0.0f, sum_y = 0.0f;
    uint8_t n = 0;
    for (uint8_t i = IMAGE_HEIGHT - IMAGE_ROW_KEEPOUT_PIXEL; i >= IMAGE_HEIGHT - IMAGE_INTEREST_REGION; i--)
    {
        if (IMAGE_HEIGHT - line_info_in->pixel_cnt > i)
            break;
        sum_xy += i * line[i];
        sum_xx += line[i] * line[i];
        sum_x += line[i];
        sum_y += i;
        n += 1;
    }
    /* ���޷���ȡб����Ϣ���򷵻�0 */
    if (n * sum_xx - sum_x * sum_x == 0)
    {
        return 0;
    }
    else
    {
        /* ��С������ϣ����ؽǶ�ֵ */
        float k = -(n * sum_xy - sum_x * sum_y) / (n * sum_xx - sum_x * sum_x);
        k = atan(k) / 3.1415926 * 180;
        k = (k >= 0) ? k : 180 + k;
        k -= 90;
        return k;
    }
}

/**
 * @brief ��Ȩ[Ȩֵ���ߣ�-0.02(x-40)^2+20]ƽ�����ߺ�����
 *
 * @param line ���ߺ���������mid[height]��ָ��
 * @param line_info_in ������Ϣ
 * @return float -60~60
 */
static float weighted_line(const uint8_t *line, line_info *line_info_in, uint8_t mode)
{
    static uint8_t weighted_table[IMAGE_HEIGHT];
    static uint8_t current_mode = 0;
    float table_sum = 0;

    /* �״ε��ã�����Ȩֵ�� */
    if (current_mode != mode)
    {
        switch (mode)
        {
        case WEIGHT_MODE_NORMOL:
            for (uint8_t i = IMAGE_HEIGHT - IMAGE_ROW_KEEPOUT_PIXEL; i >= IMAGE_HEIGHT - IMAGE_INTEREST_REGION; i--)
            {
                if (i < IMAGE_HEIGHT && i >= IMAGE_HEIGHT - 20)
                    weighted_table[i] = 40;
                if (i < IMAGE_HEIGHT - 20 && i >= IMAGE_HEIGHT - 40)
                    weighted_table[i] = 30;
                if (i < IMAGE_HEIGHT - 40 && i >= IMAGE_HEIGHT - 60)
                    weighted_table[i] = 20;
                if (i < IMAGE_HEIGHT - 60)
                    weighted_table[i] = 10;
            }
            break;
        case WEIGHT_MODE_LOWER:
            for (uint8_t i = IMAGE_HEIGHT - IMAGE_ROW_KEEPOUT_PIXEL; i >= IMAGE_HEIGHT - IMAGE_INTEREST_REGION; i--)
            {
                if (i < IMAGE_HEIGHT && i >= IMAGE_HEIGHT - 40)
                    weighted_table[i] = 1;
                else
                    weighted_table[i] = 0;
            }
            break;
        }
        current_mode = mode;
    }
    float lsum = 0.0f;
    for (uint8_t i = IMAGE_HEIGHT - IMAGE_ROW_KEEPOUT_PIXEL; i >= IMAGE_HEIGHT - IMAGE_INTEREST_REGION; i--)
    {
        if (IMAGE_HEIGHT - line_info_in->pixel_cnt > i)
            break;
        lsum += line[i] * weighted_table[i];
        table_sum += weighted_table[i];
    }
    lsum /= table_sum;
    if (table_sum != 0)
    {
        return lsum - IMAGE_WIDTH / 2;
    }
    else
    {
        return 0;
    }
}

/**
 * @brief ͼ���������Ϣ���������ͨ��ע��Ӱ�ص�����Ϣ��
 *
 * @param format
 * @param ...
 * @return int
 */
int tjrc_log(char *format, ...)
{
    /*static char buff[128];
    va_list ap;
    va_start(ap, format);
    vsnprintf(buff, sizeof(buff), format, ap);
    va_end(ap);
    printf("%s", buff);*/
    return 0;
}
