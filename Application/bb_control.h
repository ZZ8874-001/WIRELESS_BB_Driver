#ifndef BB_CONTROL_H
#define BB_CONTROL_H

#include "controller.h"

#define ADC_Ratio 3.251f/4096.0f
#define Current_Out_Offset 1.65554738f//6.753255f
#define Voltage_Out_Offset 0.0f
#define Kp_FFB 12.0f     //前馈增量系数
#define Voltage_Ratio 19.967254f
#define Current_Ratio 1.0f/0.235911906f//4.052521f
#define Voltage_Out_Ref 24.1f
#define Current_Out_Max 5.1f
#define Power_Out_Limit 122.4f
#define Voltage_In_Max 48.0f
#define Voltage_In_Min 16.0f
#define VoltProt_Delay 180000000.0f // 2.5*72mHz

enum Buck_Boost_State
{
    Buck = 0,
    Boost,
    Buck_Boost,
    VoltIpt_Error,
    Ext_Err,
    None,
};

typedef struct
{
    float voltage_out_f_;
    float voltage_in_f_;
    float current_out_f_;

    float duty_;
    float duty_min_;
    float duty_max_;
    float duty_changing_min_;   // 动态占空比防止输出电压过高
    float duty_changing_max_;   

    float voltage_gain_measure_;
    float voltage_gain_ref_;
    float voltage_gain_min_;
    float voltage_gain_max_;

    float voltage_out_ref_;
    float current_out_ref_;
    
    float buck_duty_cycle_;
    float boost_duty_cycle_;

    float duty_PID_output_;
    float duty_FFB_output_;

    PID_t voltage_gain_PID_;
    PID_t current_out_PID_;

    First_Order_Filter_t voltage_out_filter_;
    First_Order_Filter_t voltage_in_filter_;
    First_Order_Filter_t current_out_filter_;

    // enum Buck_Boost_State status_;

}Buck_Boost_Str;

extern Buck_Boost_Str bb;

void BB_Control_Init(void);
void Buck_Boost_Task();

#endif