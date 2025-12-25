#ifndef __BSP_ADC_H__
#define __BSP_ADC_H__

#include "main.h"
#include "adc.h"

void Bsp_ADC_Init(void);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);
void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc);

#endif