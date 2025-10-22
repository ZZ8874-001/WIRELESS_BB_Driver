#ifndef BB_CONTROL_H
#define BB_CONTROL_H

#include "stdint.h"
#include "adc.h"
#include "bsp_dwt.h"
#include "hrtim.h"
#include "controller.h"
#include "filter32.h"
#include "usart.h"

// #define Frequency 75000.0f
#define ADC_Ratio 3.3f/4096.0f
#define Current_Out_Offset 0.1f
#define Kp_FFB 0.1f     //前馈增量系数
#define Voltage_Ratio 12.370f
#define Current_Ratio 10.0f
#define Voltage_Out_Ref 20.0f
#define Current_Out_Max 1.0f
#define Power_Out_Limit 88.0f


typedef struct
{
    float voltage_out_;
    float voltage_in_;
    float current_out_;
    float power_out_;

    float voltage_out_f_;
    float voltage_in_f_;
    float current_out_f_;

    float I_Ref_;
    float duty_;
    float duty_min_;
    float duty_max_;

    float voltage_gain_measure_;
    float voltage_gain_ref_;

    float voltage_out_ref_;
    float current_out_ref_;
    
    float buck_duty_cycle_;
    float boost_duty_cycle_;

    float dutyoutput_PID;

    PID_t voltage_out_PID_;
    PID_t current_out_PID_;

    First_Order_Filter_t voltage_out_filter_;
    First_Order_Filter_t voltage_in_filter_;
    First_Order_Filter_t current_out_filter_;

    uint8_t state_;

}Buck_Boost_Str;

enum Buck_Boost_State
{
    Buck = 0,
    Boost,
    Buck_Boost,
    None,
};

void BB_Control_Init(void);
void Buck_Boost_Task();
void Data_Handle();
void Duty_Set();
void Duty_Set_FFB();
void MOS_PWM_Set();

#endif