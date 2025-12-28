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
 * @brief
 *
 * @param first_order_filter һ�׵�ͨ�˲���
 * @param dt �������ڣ���λs
 * @param cutoff_freq ��ֹƵ��
 */

void First_Order_Filter_Init(First_Order_Filter_t *first_order_filter, float dt, float cutoff_freq)
{
    first_order_filter->dt = dt;
    first_order_filter->RC = 1.0f / (2 * 3.14159f * cutoff_freq);
    first_order_filter->aphha = float_constrain(dt / (dt + first_order_filter->RC), 0.0f, 1.0f);
    first_order_filter->Input = 0.0f;
    first_order_filter->Output = 0.0f;
}

/**
 * @brief
 *
 * @param first_order_filter һ�׵�ͨ�˲����ṹ��
 * @param input ����
 * @return float ���
 */
float First_Order_Filter_Calculate(First_Order_Filter_t *first_order_filter, float input)
{
    first_order_filter->Input = input;

    first_order_filter->Output +=
        (first_order_filter->Input - first_order_filter->Output) * first_order_filter->aphha;

    first_order_filter->Output = float_deadband(first_order_filter->Output, -1e-5,1e-5);
    return first_order_filter->Output;
}

/**
 * @brief          �����˲���ʼ��
 * @param[in]      �����˲��ṹ��
 * @param[in]      ���ڴ�С
 * @retval         ���ؿ�
 */
void Window_Filter_Init(Window_Filter_t *window_filter, uint8_t windowSize)
{
    window_filter->WindowNum = 0;
    window_filter->WindowSize = windowSize;
    window_filter->WindowBuffer = (float *)user_malloc(sizeof(float) * windowSize);
    memset(window_filter->WindowBuffer, 0, windowSize);
}

/**
 * @brief          �����˲�����
 * @param[in]      �����˲��ṹ��
 * @param[in]      ����ֵ
 * @retval         �����˲����
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
 * @brief          IIR�˲���ʼ��
 * @param[in]      IIR�˲��ṹ��
 * @param[in]      �����ʱ�䣬��λ s
 * @param[in]      �˲�ϵ��
 * @retval         ���ؿ�
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
 * @brief          IIR�˲�����
 * @param[in]      IIR�˲��ṹ��
 * @param[in]      ����ֵ
 * @retval         �����˲����
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
