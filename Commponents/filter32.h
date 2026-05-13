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

typedef struct
{
    float Input;         // 滤波器输入值
    float Output;        // 滤波器输出值
    float RC;            // 滤波时间常数 RC = 1/(2*pi*f_c)
    float dt;            // 采样周期，单位 s
    float aphha;         // 滤波系数 alpha = dt/(dt+RC)
    uint8_t PointNum;    // 滤波通道数（支持多通道并行滤波）
    float *InputArray;   // 多通道输入缓冲区
    float *OutputArray;  // 多通道输出缓冲区
} First_Order_Filter_t;

typedef struct window_filter
{
    float Input;         // 滤波器输入值
    float Output;        // 滤波器输出值
    uint8_t WindowSize;  // 窗口大小
    uint8_t WindowNum;   // 当前更新的窗口索引
    float *WindowBuffer; // 窗口数据缓冲区
} Window_Filter_t;

typedef struct
{
    float Input;   // 滤波器输入值
    float Output;  // 滤波器输出值
    uint8_t Order; // 滤波器阶数
    float *Num;    // Numerator
    float *Den;    // Denominator
    float *xbuf;   // 输入历史缓冲区
    float *ybuf;   // 输出历史缓冲区
} IIR_Filter_t;

void First_Order_Filter_Init(First_Order_Filter_t *first_order_filter, float dt, float cutoff_freq, uint8_t point_num);
float First_Order_Filter_Calculate(First_Order_Filter_t *first_order_filter, float input);
void First_Order_Filter_Calculate_Array(First_Order_Filter_t *first_order_filter, const float *input, float *output);
void Window_Filter_Init(Window_Filter_t *window_filter, uint8_t windowSize);
float Window_Filter_Calculate(Window_Filter_t *window_filter, float input);
void IIR_Filter_Init(IIR_Filter_t *iir_filter, float *num, float *den, uint8_t order);
float IIR_Filter_Calculate(IIR_Filter_t *iir_filter, float input);
#endif
#endif
