#include "bsp_adc.h"
#include "bsp_uart.h"

#include "adc.h"
#include <stdlib.h>

#include "bb_control.h"

static uint16_t ADC1_Rx[2];
static uint16_t ADC2_Rx;

void Bsp_ADC_Init(void)
{
    // 滤波器初始化
    First_Order_Filter_Init(&bb.voltage_in_filter_,1/Frequency,30);
    First_Order_Filter_Init(&bb.voltage_out_filter_,1/Frequency,20);
    First_Order_Filter_Init(&bb.current_out_filter_,1/Frequency,20);
    
    // 开启ADC
    while(HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED) != HAL_OK)
    {
    }
    while(HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED) != HAL_OK)
    {
    }
    while(HAL_ADC_Start_DMA(&hadc1,ADC1_Rx,2) != HAL_OK)
    {
    }
    while(HAL_ADC_Start_DMA(&hadc2,&ADC2_Rx,1) != HAL_OK)
    {
    }
    ADC1->AWD2CR = ADC_AWD2CR_AWD2CH_2;
    ADC1->IER |= ADC_IER_AWD2IE;

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc->Instance == ADC1)
    {
        float voltage_out_;
        float voltage_in_;
        if(USART_Debug_Flag)
        {
            voltage_in_ = (float)(atoi(Rx_Buf.adc1)) * ADC_Ratio * Voltage_Ratio - Voltage_Out_Offset;
            voltage_out_ = (float)(atoi(Rx_Buf.adc2)) * ADC_Ratio * Voltage_Ratio;
        }
        else
        {
            voltage_out_ = (float)ADC1_Rx[0] * ADC_Ratio * Voltage_Ratio - Voltage_Out_Offset;
            voltage_in_ = (float)ADC1_Rx[1] * ADC_Ratio * Voltage_Ratio;
        }
        
    
        bb.voltage_out_f_ = First_Order_Filter_Calculate(&bb.voltage_out_filter_,voltage_out_);
        bb.voltage_in_f_ = First_Order_Filter_Calculate(&bb.voltage_in_filter_,voltage_in_);
    }
    else if(hadc->Instance == ADC2)
    {
        float current_out_ = (float)(ADC2_Rx * ADC_Ratio - Current_Out_Offset) * Current_Ratio;

        bb.current_out_f_ = First_Order_Filter_Calculate(&bb.current_out_filter_,current_out_);
    }
}

void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc)
{
    // __ASM("bx lr");
    if(hadc->Instance == ADC1)
    {
    }
    else if(hadc->Instance == ADC2)
    {
    }
}

void HAL_ADCEx_LevelOutOfWindow2Callback(ADC_HandleTypeDef* hadc)
{
    if(hadc->Instance == ADC1)
    {
    }
}