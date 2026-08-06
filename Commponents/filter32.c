/**
 ******************************************************************************
 * @file    filter32.c
 * @author  Wang Hongxi
 * @version V1.0.1
 * @date    2020/7/7
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "filter32.h"
#include "user_lib.h"

#if (__CORTEX_M == (4U))
/**
 * @brief  一阶低通滤波器初始化
 * @param  first_order_filter  滤波器结构体指针
 * @param  dt                  采样周期，单位 s
 * @param  cutoff_freq         截止频率，单位 Hz
 * @param  point_num           滤波通道数，0 时自动取 1
 */

void First_Order_Filter_Init(First_Order_Filter_t *first_order_filter, float dt, float cutoff_freq, uint8_t point_num)
{
    if (point_num == 0)
    {
        point_num = 1;
    }

    first_order_filter->dt = dt;
    first_order_filter->RC = 1.0f / (2 * 3.14159f * cutoff_freq);
    first_order_filter->aphha = float_constrain(dt / (dt + first_order_filter->RC), 0.0f, 1.0f);
    first_order_filter->PointNum = point_num;
    first_order_filter->Input = 0.0f;
    first_order_filter->Output = 0.0f;
    first_order_filter->InputArray = (float *)user_malloc(sizeof(float) * point_num);
    first_order_filter->OutputArray = (float *)user_malloc(sizeof(float) * point_num);
    memset(first_order_filter->InputArray, 0, sizeof(float) * point_num);
    memset(first_order_filter->OutputArray, 0, sizeof(float) * point_num);
}

/**
 * @brief  一阶低通滤波器计算（单通道）
 * @param  first_order_filter  滤波器结构体指针
 * @param  input               输入值
 * @return float               滤波后的输出值
 */
float First_Order_Filter_Calculate(First_Order_Filter_t *first_order_filter, float input)
{
    First_Order_Filter_Calculate_Array(first_order_filter, &input, &first_order_filter->Output);
    first_order_filter->Input = first_order_filter->InputArray[0];
    first_order_filter->Output = first_order_filter->OutputArray[0];

    return float_deadband(first_order_filter->Output, -1e-5,1e-5);
}

/**
 * @brief  一阶低通滤波器计算（多通道数组版本）
 * @param  first_order_filter  滤波器结构体指针
 * @param  input               多通道输入数组
 * @param  output              多通道输出数组
 */
void First_Order_Filter_Calculate_Array(First_Order_Filter_t *first_order_filter, const float *input, float *output)
{
    for (uint8_t i = 0; i < first_order_filter->PointNum; i++)
    {
        float prev_output = first_order_filter->OutputArray[i];

        first_order_filter->InputArray[i] = input[i];
        first_order_filter->OutputArray[i] +=
            float_deadband(first_order_filter->InputArray[i] - prev_output, -1e-5, 1e-5) * first_order_filter->aphha;

        if (isnan(first_order_filter->OutputArray[i]))
        {
            first_order_filter->OutputArray[i] = prev_output;
        }

        output[i] = float_deadband(first_order_filter->OutputArray[i], -1e-5, 1e-5);
    }

    first_order_filter->Input = first_order_filter->InputArray[0];
    first_order_filter->Output = first_order_filter->OutputArray[0];
}

/**
 * @brief         窗口滤波器初始化
 * @param[in]     window_filter  窗口滤波器结构体指针
 * @param[in]     windowSize     窗口大小
 * @retval        无
 */
void Window_Filter_Init(Window_Filter_t *window_filter, uint8_t windowSize)
{
    window_filter->WindowNum = 0;
    window_filter->WindowSize = windowSize;
    window_filter->WindowBuffer = (float *)user_malloc(sizeof(float) * windowSize);
    memset(window_filter->WindowBuffer, 0, windowSize);
}

/**
 * @brief         窗口滤波器计算
 * @param[in]     window_filter  窗口滤波器结构体指针
 * @param[in]     input          输入值
 * @retval        窗口滤波结果
 */
float Window_Filter_Calculate(Window_Filter_t *window_filter, float input)
{
    window_filter->Input = input;
    window_filter->Output = 0;

    window_filter->WindowBuffer[window_filter->WindowNum++] = input;
    if (window_filter->WindowNum >= window_filter->WindowSize)
        window_filter->WindowNum = 0;

    for (uint8_t i = 0; i < window_filter->WindowSize; i++)
        window_filter->Output += window_filter->WindowBuffer[i];

    window_filter->Output /= window_filter->WindowSize;

    return window_filter->Output;
}

/**
 * @brief         IIR 滤波器初始化
 * @param[in]     iir_filter  IIR 滤波器结构体指针
 * @param[in]     num         分子系数数组
 * @param[in]     den         分母系数数组
 * @param[in]     order       滤波器阶数
 * @retval        无
 */
void IIR_Filter_Init(IIR_Filter_t *iir_filter, float *num, float *den, uint8_t order)
{
    iir_filter->Order = order;
    iir_filter->Num = (float *)user_malloc(sizeof(float) * order);
    iir_filter->Den = (float *)user_malloc(sizeof(float) * order);
    iir_filter->xbuf = (float *)user_malloc(sizeof(float) * order);
    iir_filter->ybuf = (float *)user_malloc(sizeof(float) * order);
    memcpy(iir_filter->Num, num, sizeof(float) * order);
    memcpy(iir_filter->Den, den, sizeof(float) * order);
}

/**
 * @brief         IIR 滤波器计算
 * @param[in]     iir_filter  IIR 滤波器结构体指针
 * @param[in]     input       输入值
 * @retval        IIR 滤波结果
 */
float IIR_Filter_Calculate(IIR_Filter_t *iir_filter, float input)
{
    iir_filter->Input = input;
    for (uint8_t i = iir_filter->Order - 1; i > 0; i--)
    {
        iir_filter->xbuf[i] = iir_filter->xbuf[i - 1];
        iir_filter->ybuf[i] = iir_filter->ybuf[i - 1];
    }
    iir_filter->xbuf[0] = input;
    iir_filter->ybuf[0] = iir_filter->Num[0] * iir_filter->xbuf[0];
    for (uint8_t i = 1; i < iir_filter->Order; i++)
    {
        iir_filter->ybuf[0] += iir_filter->Num[i] * iir_filter->xbuf[i] - iir_filter->Den[i] * iir_filter->ybuf[i];
    }
    iir_filter->Output = iir_filter->ybuf[0];
    return iir_filter->Output;
}

#endif
