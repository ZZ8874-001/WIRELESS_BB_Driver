/**
 ******************************************************************************
 * @file    filter32.h
 * @author  Wang Hongxi
 * @version V1.0.0
 * @date    2020/3/17
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef __FILTER32_H
#define __FILTER32_H

#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "main.h"

#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif

#if (__CORTEX_M == (4U))

typedef __packed struct
{
    float Input;  //��������
    float Output; //�˲����������
    float RC;     //�˲����� RC = 1/(2*pi*f_c)
    float dt;     //�˲���ʱ���� ��λ s
    float aphha;
} First_Order_Filter_t;

typedef __packed struct window_filter
{
    float Input;         //��������
    float Output;        //�˲����������
    uint8_t WindowSize;  //���ڴ�С
    uint8_t WindowNum;   //��Ҫ���µĴ���ֵ
    float *WindowBuffer; //�������ݻ�����
} Window_Filter_t;

typedef __packed struct
{
    float Input;   //��������
    float Output;  //�˲����������
    uint8_t Order; //�˲�������
    float *Num;    // Numerator
    float *Den;    // Denominator
    float *xbuf;
    float *ybuf;
} IIR_Filter_t;

void First_Order_Filter_Init(First_Order_Filter_t *first_order_filter, float dt, float cutoff_freq);
float First_Order_Filter_Calculate(First_Order_Filter_t *first_order_filter, float input);
void Window_Filter_Init(Window_Filter_t *window_filter, uint8_t windowSize);
float Window_Filter_Calculate(Window_Filter_t *window_filter, float input);
void IIR_Filter_Init(IIR_Filter_t *iir_filter, float *num, float *den, uint8_t order);
float IIR_Filter_Calculate(IIR_Filter_t *iir_filter, float input);
#endif
#endif

