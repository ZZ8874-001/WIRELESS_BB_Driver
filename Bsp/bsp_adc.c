#include "bsp_adc.h"
#include "bsp_uart.h"

#include "adc.h"
#include <stdlib.h>

#include "bb_control.h"
#include "detect_task.h"

#define ADC_RATIO 3.251f/4096.0f
#define CURRENT_OUT_OFFSET 1.65554738f//6.753255f
#define VOLTAGE_OUT_OFFSET 0.0f
#define VOLTAGE_RATIO 19.967254f
#define CURRENT_RATIO 1.0f/0.235911906f//4.052521f
#define adc_volt_watchdog_min (VOLTAGE_IN_MIN * 3.896f)
#define adc_volt_watchdog_max (VOLTAGE_IN_MAX * 3.896f)

static uint16_t ADC1_Rx[2];
static uint16_t ADC2_Rx;

static void Change_ADC_AWD_Threshold(uint32_t *ADCx_TRx,uint16_t high_threshold,uint16_t low_threshold);

void Bsp_ADC_Init(void)
{
    // 滤波器初始化
    First_Order_Filter_Init(&bb.voltage_in_filter_,1/Frequency,300);
    First_Order_Filter_Init(&bb.voltage_out_filter_,1/Frequency,200);
    First_Order_Filter_Init(&bb.current_out_filter_,1/Frequency,200);
    
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
    ADC1->AWD2CR = 1 << 2;
    // 1020-3000
    Change_ADC_AWD_Threshold(&ADC1->TR2,adc_volt_watchdog_min,adc_volt_watchdog_max);//ADC1_WATCHDOG1_TOE
    ADC1->IER |= ADC_IER_AWD2IE;

}

static void Change_ADC_AWD_Threshold(uint32_t *ADCx_TRx,uint16_t low_threshold,uint16_t high_threshold)
{
    *ADCx_TRx = (high_threshold << 16) | low_threshold;
}

static uint16_t Char_To_Uint16(uint8_t *buf,uint8_t size)
{
    uint16_t ret = 0;
    for(uint8_t i = 0;i<size;i++)
    {
        ret *= 10;
        ret += (buf[i]-'0')>0?(buf[i]-'0'):0;
    }
    return ret;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc->Instance == ADC1)
    {
        float voltage_out_;
        float voltage_in_;

        voltage_out_ = (float) (USART_Debug_Flag?Char_To_Uint16(Rx_Buf.voltage_out,4):ADC1_Rx[0]) * ADC_RATIO * VOLTAGE_RATIO - VOLTAGE_OUT_OFFSET;
        voltage_in_ = (float) (USART_Debug_Flag?Char_To_Uint16(Rx_Buf.voltage_in,4):ADC1_Rx[1]) * ADC_RATIO * VOLTAGE_RATIO;
        
    
        bb.voltage_out_f_ = First_Order_Filter_Calculate(&bb.voltage_out_filter_,voltage_out_);
        bb.voltage_in_f_ = First_Order_Filter_Calculate(&bb.voltage_in_filter_,voltage_in_);
    }
    else if(hadc->Instance == ADC2)
    {
        float current_out_ = (float)(ADC2_Rx * ADC_RATIO - CURRENT_OUT_OFFSET) * CURRENT_RATIO;

        bb.current_out_f_ = First_Order_Filter_Calculate(&bb.current_out_filter_,current_out_);
    }
}

void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc)
{
    // __ASM("bx lr");
    if(hadc->Instance == ADC1)
    {
        Detect_Hook(ADC1_WATCHDOG1_TOE);
    }
    else if(hadc->Instance == ADC2)
    {
        Detect_Hook(ADC2_WATCHDOG1_TOE);
    }
}

void HAL_ADCEx_LevelOutOfWindow2Callback(ADC_HandleTypeDef* hadc)
{
    if(hadc->Instance == ADC1)
    {
        HRTIM1->sCommonRegs.ODISR = 0xFFFF;
        Detect_Hook(ADC1_WATCHDOG2_TOE);
    }
}