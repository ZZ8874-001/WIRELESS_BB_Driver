//#ifndef BB_CONTROL_H
#define BB_CONTROL_H

#include "controller.h"

#define Kp_FFB 0.1f     //前馈增量系数
#define VOLTAGE_OUT_REF 24.1f
#define CURRENT_OUT_MAX 1.1f
#define VOLTAGE_IN_MAX 48.0f
#define VOLTAGE_IN_MIN 14.0f
#define VoltProt_Delay 180000000.0f // 2.5*72mHz
#define CURRENT_TO_VOLTAGE_DELAY 10  //ms


enum Buck_Boost_State
{
    Buck = 0,
    Boost,
    Buck_Boost,
    VoltIpt_Error,
    Soft_Start,
    None,
};

typedef struct
{
    float voltage_out_f_;
    float voltage_in_f_;
    float current_out_f_;
    
    float voltage_gain_measure_;
    float voltage_gain_ref_;

    float voltage_out_ref_;
    float current_out_ref_;
    
    float buck_duty_cycle_;
    // float boost_duty_cycle_;

    float gain_PID_output_;
    float gain_FFB_output_;

    PID_t voltage_gain_PID_;
    PID_t current_out_PID_;

    float voltage_gain_NFB_;
    float current_gain_NFB_;
    float current_gain_NFB_f_;

    First_Order_Filter_t voltage_out_filter_;
    First_Order_Filter_t voltage_in_filter_;
    First_Order_Filter_t current_out_filter_;
    First_Order_Filter_t current_gain_NFB_filter_;

    // enum Buck_Boost_State status_;

}Buck_Boost_Str;

extern Buck_Boost_Str bb;
extern enum Buck_Boost_State bb_state;
extern uint8_t USART_Debug_Flag;

void BB_Control_Init(void);
void Buck_Boost_Task();

//#endif