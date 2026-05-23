/**
 ******************************************************************************
 * @file    bsp_adc.c
 * @author  Shuchen Tao (Claude_Version(Doge))
 * @version V3.0.0
 * @brief   ADC BSP 层实现 — Plan B：DMA 循环直接搬运大缓冲区
 *          - ADC1: DMA 循环写入交织缓冲区 ADC1_DMA_Buf[40]
 *                  布局 [Vout0,Vin0, Vout1,Vin1, ..., Vout19,Vin19]
 *          - ADC2: DMA 循环写入独立缓冲区 ADC2_DMA_Buf[20]
 *          - HAL_ADC_ConvCpltCallback 仅更新位置索引，不做处理
 *          - Bsp_ADC_ProcessSample() 在 TIM 回调中批量解交织，每个采样点滤一次
 ******************************************************************************
 */
#include "bsp_adc.h"
#include "bsp_uart.h"

#include "adc.h"
#include <stdlib.h>

#include "bb_control.h"
#include "board_id.h"
#include "detect_task.h"

/* 各板卡 ADC 校准系数 */
float ADC_RATIO[BOARD_NUM]           = {  2.9832f/4096.0f,  2.9827f/4096.0f,  2.9799f/4096.0f,  2.9899f/4096.0f,  2.9859f/4096.0f };
float CURRENT_OUT_OFFSET[BOARD_NUM]  = { -8.020f,         -15.9082f,        -11.9225f,        -14.8929f,        -16.937f         };
float VOLTAGE_OUT_OFFSET[BOARD_NUM]  = { -0.0129f,         -0.0129f,          0.037000f,        0.037000f,        -0.022158f       };
float VOLTAGE_RATIO[BOARD_NUM]       = { 22.227f,          22.227f,          20.134f,          20.648f,          21.471f          };
float CURRENT_RATIO[BOARD_NUM]       = { -10.200f,        -20.3865f,        -15.3139f,        -19.1205f,        -21.277f         };

#define adc_volt_watchdog_min (VOLTAGE_IN_MIN * 3.896f)
#define adc_volt_watchdog_max (VOLTAGE_IN_MAX * 3.896f)

/* ADC 触发频率，与 HRTIM 对齐 */
#define ADC_SAMPLING_FREQUENCY (100000.0f)

/* DMA 循环目标缓冲区（DMA 硬件自动循环搬运，无需软件写指针维护） */
uint16_t ADC1_DMA_Buf[ADC1_DMA_BUF_SIZE];   /* 交织: [V0,Vi0, V1,Vi1, ...] */
uint16_t ADC2_DMA_Buf[ADC2_DMA_BUF_SIZE];   /* 独立: [I0, I1, I2, ...]      */

static void Change_ADC_AWD_Threshold(volatile uint32_t *ADCx_TRx, int16_t low_threshold, int16_t high_threshold);

/* ========================== ADC 初始化 ========================== */

void Bsp_ADC_Init(void)
{
    /* 一阶低通滤波器初始化：截止频率 8kHz，采样频率 200kHz（与 HRTIM 一致） */
    First_Order_Filter_Init(&bb.voltage_in_filter_,  1.0f / ADC_SAMPLING_FREQUENCY, 8000, ADC_VIN_BUF_SIZE);
    First_Order_Filter_Init(&bb.voltage_out_filter_, 1.0f / ADC_SAMPLING_FREQUENCY, 8000, ADC_VOUT_BUF_SIZE);
    First_Order_Filter_Init(&bb.current_out_filter_, 1.0f / ADC_SAMPLING_FREQUENCY, 8000, ADC_CURR_BUF_SIZE);

    /* ADC 校准 */
    while (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) {}
    while (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK) {}

    /*
     * 启动 DMA 循环搬运（DMA 已在 CubeMX 配置为 CIRCULAR 模式）
     * ADC1: 40 half-words，交织存放 20 组 Vout/Vin
     * ADC2: 20 half-words，存放 20 个电流采样
     */
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC1_DMA_Buf, ADC1_DMA_BUF_SIZE);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)ADC2_DMA_Buf, ADC2_DMA_BUF_SIZE);

    /* 禁用 DMA 半传输/全传输中断，仅使用 ADC EOS 中断（无需软件参与 DMA 管理） */
    DMA1_Channel1->CCR &= ~(DMA_CCR_HTIE | DMA_CCR_TCIE);
    DMA1_Channel4->CCR &= ~(DMA_CCR_HTIE | DMA_CCR_TCIE);

    /* 配置模拟看门狗 */
    ADC1->AWD2CR = 1 << 2;
    Change_ADC_AWD_Threshold(&ADC1->TR2, adc_volt_watchdog_min,
        (adc_volt_watchdog_max < 255 ? adc_volt_watchdog_max : 255));

    ADC1->IER |= ADC_IER_AWD2IE | ADC_IER_EOSIE;
    ADC2->IER |= ADC_IER_EOSIE;
}

/* ========================== 辅助函数 ========================== */

static void Change_ADC_AWD_Threshold(volatile uint32_t *ADCx_TRx, int16_t low_threshold, int16_t high_threshold)
{
    if (high_threshold > 4095) high_threshold = 4095;
    if (low_threshold  < 0)    low_threshold  = 0;
    *ADCx_TRx = ((uint32_t)high_threshold << 16) | (uint32_t)low_threshold;
}

/* ========================== ADC 数据处理（在 TIM 回调中调用） ========================== */

/**
 * @brief  解交织 DMA 缓冲区 → float 数组 → Calculate_Array 并行滤波
 * @note   在 HAL_TIM_PeriodElapsedCallback (10kHz) 中调用
 *         20 通道并行滤波，每个通道对应 DMA 缓冲区一个固定偏移位置
 *         滤波器 dt = 1/200000，与 HRTIM ADC 触发频率对齐
 *
 *         时序关系：
 *         200kHz ADC 触发 × 20 采样 = 100µs = 1 个 TIM2 周期
 *         Calculate_Array 一次性处理全部 20 个采样
 */
void Bsp_ADC_ProcessSample(void)
{
    if (!BoardID_IsValid(IDCard)) return;

    float vout_in[ADC_VOUT_BUF_SIZE];
    float vin_in[ADC_VIN_BUF_SIZE];

    /* ADC1：解交织 DMA 缓冲区 → 两个独立 float 数组 */
    for (uint8_t i = 0; i < ADC_VOUT_BUF_SIZE; i++)
    {
        vout_in[i] = (float)ADC1_DMA_Buf[i * 2]
                   * ADC_RATIO[IDCard] * VOLTAGE_RATIO[IDCard] + VOLTAGE_OUT_OFFSET[IDCard];
        vin_in[i]  = (float)ADC1_DMA_Buf[i * 2 + 1]
                   * ADC_RATIO[IDCard] * VOLTAGE_RATIO[IDCard] + VOLTAGE_OUT_OFFSET[IDCard];
    }

    /* 一次性并行滤波 20 个采样（每采样点一个独立 filter channel） */
    First_Order_Filter_Calculate_Array(&bb.voltage_out_filter_, vout_in, vout_in);
    First_Order_Filter_Calculate_Array(&bb.voltage_in_filter_,  vin_in,  vin_in);

    bb.voltage_out_f_ = vout_in[ADC_VOUT_BUF_SIZE - 1];
    bb.voltage_in_f_  = vin_in[ADC_VIN_BUF_SIZE - 1];

    /* ADC2：DMA 缓冲区 → float 数组，并行滤波 */
    float curr_in[ADC_CURR_BUF_SIZE];
    for (uint8_t i = 0; i < ADC_CURR_BUF_SIZE; i++)
    {
        curr_in[i] = (float)((2048 - ADC2_DMA_Buf[i]) * ADC_RATIO[IDCard])
                   * CURRENT_RATIO[IDCard] + CURRENT_OUT_OFFSET[IDCard];
    }

    First_Order_Filter_Calculate_Array(&bb.current_out_filter_, curr_in, curr_in);
    bb.current_out_f_ = curr_in[ADC_CURR_BUF_SIZE - 1];
}

/* ========================== HAL 回调 ========================== */

/**
 * @brief  ADC 转换完成回调（由 HRTIM 触发，~200kHz）
 * @note   DMA 已自动将数据写入循环缓冲区，此回调仅维护看门狗钩子
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    /* DMA 循环模式自动搬运，无需在此做数据拷贝 */
    (void)hadc;
}

void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        Detect_Hook(ADC1_WATCHDOG1_TOE);
    }
    else if (hadc->Instance == ADC2)
    {
        Detect_Hook(ADC2_WATCHDOG1_TOE);
    }
}

void HAL_ADCEx_LevelOutOfWindow2Callback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        Detect_Hook(ADC1_WATCHDOG2_TOE);
    }
}
