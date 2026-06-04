/**
 ******************************************************************************
 * @file    bsp_adc.h
 * @author  Shuchen Tao
 * @version V3.0.0
 * @brief   ADC BSP 层 — Plan B：DMA 循环直接填满大缓冲区
 *          - ADC1 交织缓冲区 [Vout0,Vin0, Vout1,Vin1, ...]  DMA 循环搬运
 *          - ADC2 独立缓冲区                                DMA 循环搬运
 *          - Bsp_ADC_ProcessSample() 在 TIM 回调中批量解交织 + 滤波
 ******************************************************************************
 */
#ifndef __BSP_ADC_H__
#define __BSP_ADC_H__

#include "main.h"
#include "adc.h"

/* 各通道采样数（独立定义，暂定 20；ADC1 双通道须保持一致） */
#define ADC_VOUT_BUF_SIZE  10
#define ADC_VIN_BUF_SIZE   10
#define ADC_CURR_BUF_SIZE  10

/* DMA 循环缓冲区长度 */
#define ADC1_DMA_BUF_SIZE  (ADC_VOUT_BUF_SIZE * 2)   /* 交织：40 half-words */
#define ADC2_DMA_BUF_SIZE  (ADC_CURR_BUF_SIZE)        /* 独立：20 half-words */

void Bsp_ADC_Init(void);
void Bsp_ADC_ProcessSample(void);        /* TIM 回调中批量解交织 + 工程值转换 + 滤波 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);
void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef *hadc);

#endif
