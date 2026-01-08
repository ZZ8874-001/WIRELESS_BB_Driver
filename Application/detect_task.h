#ifndef __DETECT_TASK_H__
#define __DETECT_TASK_H__

#include "controller.h"
#include <stdbool.h>

#define USER_GetTick HAL_GetTick

#ifdef _CMSIS_OS_H
#define USER_Delay_ms vTaskDelay
#else
#define USER_Delay_ms HAL_Delay
#endif

typedef struct 
{ 
    bool is_Lost;
    bool Overtime_Exit;

    float Overtime_ms;
    float new_time;
    float dt_ms;
    
}Detect_t;

enum errorlist
{
    ADC1_WATCHDOG1_TOE,
    ADC1_WATCHDOG2_TOE,
    ADC2_WATCHDOG1_TOE,
    VoltIpt_Error_TOE,
    CURRENT_TO_VOLTAGE_TOE,
    DETECT_LIST_LENGTH,
};

void Detect_Init();
void Detect_Task();
void Detect_Hook(uint8_t toe);
uint8_t is_TOE_Overtime(uint8_t toe);

#endif