#include "bb_control.h"
#include "detect_task.h"

#include <stdbool.h>
#include "stdint.h"
#include "adc.h"
#include "hrtim.h"
#include "usart.h"

#include "bsp_dwt.h"
#include "bsp_uart.h"

#include "filter32.h"

static void Choose_State(void);
static void Data_Handle(void);
static void Duty_Calculate();
static void MOS_PWM_Set();

static float kp_ffb1;
static bool Debug_Mode = 0;
Buck_Boost_Str bb = {0};
static float square_ratio_a = 3;
enum Buck_Boost_State bb_state = VoltIpt_Error;
enum Buck_Boost_State last_bb_state = VoltIpt_Error;
uint8_t USART_Debug_Flag = 0;

static float voltage_gain_final_output = 0;
static float enter_soft_start_time = 0;

void BB_Control_Init(void)
{
    bb.current_out_ref_ = 0;
    bb.voltage_out_ref_ = VOLTAGE_OUT_REF;
    kp_ffb1 = Kp_FFB;
    Debug_Mode = 0;

    PID_Init(&bb.voltage_gain_PID_,1.5f,0.5f,  1.0f,-1.0f,  0.001f,  2.0f,1.0f,0,  1,1,  0,0.5,  0,Integral_Limit | DerivativeFilter );
    PID_Init(&bb.current_out_PID_,1.5f,0.5f,  1.0f,-1.0f,  0.001f,  0.3f,1.6f,0,  1,1,  0,0.5,  0,Integral_Limit | DerivativeFilter );//0.5  0.2

    // 开启hrtim
    while(HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2) != HAL_OK)
    {
    }
    while(HAL_HRTIM_WaveformCountStart(&hhrtim1, HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_A) != HAL_OK)
    {
    }

    // 初始化bben指示灯
    BBEN_Indicator_GPIO_Port->BSRR = BBEN_Indicator_Pin;
}

void Buck_Boost_Task()
{
    Choose_State();
    Data_Handle();
    Duty_Calculate();

    if((!is_TOE_Overtime(ADC1_WATCHDOG1_TOE) 
    || !is_TOE_Overtime(ADC1_WATCHDOG2_TOE) 
    || !is_TOE_Overtime(ADC2_WATCHDOG1_TOE)) 
    && USART_Debug_Flag == 0)
    {
        bb.buck_duty_cycle_ = 0;
    }
    else
    {
        MOS_PWM_Set();
    }
    
}

static void Choose_State(void)
{
    switch(bb_state)
    {
        case Buck:
            if(bb.voltage_in_f_ < VOLTAGE_IN_MIN || VOLTAGE_IN_MAX < bb.voltage_in_f_)
            {
                last_bb_state = bb_state;
                bb_state = VoltIpt_Error;

                GPIOA->BRR = GPIO_PIN_7|GPIO_PIN_6;
                Detect_Hook(VoltIpt_Error_TOE);
            }
            else
            {
                last_bb_state = bb_state;
            }
            break;
        case None:
            last_bb_state = bb_state;
            bb_state = VoltIpt_Error;

            GPIOA->BRR = GPIO_PIN_7|GPIO_PIN_6;
            Detect_Hook(VoltIpt_Error_TOE);
            break;
        case VoltIpt_Error:
            if(bb.voltage_in_f_ < VOLTAGE_IN_MIN || VOLTAGE_IN_MAX < bb.voltage_in_f_)
            {
                Detect_Hook(VoltIpt_Error_TOE);
            }
            else if(is_TOE_Overtime(VoltIpt_Error_TOE))
            {
                last_bb_state = bb_state;
                bb_state = Soft_Start;

                GPIOA->BSRR = GPIO_PIN_6 | GPIO_PIN_7;
                GPIOA->BRR = 0.2f*CURRENT_OUT_MAX<bb.current_out_f_ ? 0:GPIO_PIN_6;
            }
            break;
        case Soft_Start:
            if(bb.voltage_in_f_ < VOLTAGE_IN_MIN || VOLTAGE_IN_MAX < bb.voltage_in_f_)
            {
                last_bb_state = bb_state;
                bb_state = VoltIpt_Error;

                GPIOA->BRR = GPIO_PIN_7|GPIO_PIN_6;
                Detect_Hook(VoltIpt_Error_TOE);
            }
            else if(last_bb_state != Soft_Start)
            {
                last_bb_state = bb_state;
                enter_soft_start_time = USER_GetTick();
            }
            else if(USER_GetTick() - enter_soft_start_time > 5000)
            {
                last_bb_state = bb_state;
                bb_state = Buck;
            }
            break;
        default:
            break;
    }
}
   
static void Data_Handle(void)
{
    static float k_current,k_voltage = 0;
    static float voltage_gain_PID_output,voltage_gain_FFB_output = 0;
    
    bb.voltage_gain_measure_ = bb.voltage_out_f_ / bb.voltage_in_f_;
    bb.voltage_gain_ref_ = float_constrain(VOLTAGE_OUT_REF / bb.voltage_in_f_,0.0f,5.0f);

    PID_Calculate(&bb.voltage_gain_PID_, bb.voltage_gain_measure_, bb.voltage_gain_ref_);//10k
    PID_Calculate(&bb.current_out_PID_,bb.current_out_f_, bb.current_out_ref_);//10k

    k_current = pow(bb.current_out_f_/bb.current_out_ref_,square_ratio_a);
    k_current = float_constrain(k_current,0,1.0f);
    
    k_voltage = 1 - k_current;

    voltage_gain_PID_output = k_current * bb.current_out_PID_.Output + k_voltage * bb.voltage_gain_PID_.Output;
        
    //前馈赋值
    voltage_gain_FFB_output = (bb.voltage_gain_ref_ - bb.voltage_gain_measure_) * k_voltage;
    voltage_gain_final_output = float_constrain(voltage_gain_FFB_output * Kp_FFB + voltage_gain_PID_output,0.8f * bb.voltage_gain_ref_,1.2f * bb.voltage_gain_ref_);
}

static void Duty_Calculate()
{   
    static float soft_start_gain = 0;

    switch (bb_state)
    {
    case Buck:
        //电压增益->占空比
        bb.buck_duty_cycle_ = float_constrain(voltage_gain_final_output,0.05f,0.95f) ;
        break;
    case VoltIpt_Error:
        bb.buck_duty_cycle_ = 0;
        break;

    case Soft_Start:
        //电压增益->占空比
        soft_start_gain = float_constrain(voltage_gain_final_output,0.025f,0.95f) * (USER_GetTick() - enter_soft_start_time) / 5000.0f + 0.026f;
        bb.buck_duty_cycle_ = float_constrain(soft_start_gain,0.05f,0.95f);
        break;
    
    default:
        break;
    }
}

static void MOS_PWM_Set()
{
    // if(USART_Debug_Flag)
    // {
    //     Tx_Buf.duty1.data[0] = (uint8_t)(bb.buck_duty_cycle_ * 10) + '0';
    //     Tx_Buf.duty1.data[1] = (uint8_t)(bb.buck_duty_cycle_ * 100) % 10 + '0';
    //     // Tx_Buf.duty2.data[0] = (uint8_t)(bb.boost_duty_cycle_ * 10) + '0';
    //     // Tx_Buf.duty2.data[1] = (uint8_t)(bb.boost_duty_cycle_ * 100) % 10 + '0';
    // }
    if(is_TOE_Overtime(ADC1_WATCHDOG2_TOE))
    {
        if(!is_TOE_Overtime(VoltIpt_Error))
        {
            HRTIM1->sMasterRegs.MCMP1R = Hrtim_Period;
            HRTIM1->sMasterRegs.MCMP2R = 96;
        }
        else if(bb_state == Buck_Boost)
        {
            HRTIM1->sMasterRegs.MCMP1R = (1 + bb.buck_duty_cycle_  ) / 2 * Hrtim_Period;  // buck low d2
            HRTIM1->sMasterRegs.MCMP2R = (1 - bb.buck_duty_cycle_  ) / 2 * Hrtim_Period;  // buck high d1
        }
        else
        {
            HRTIM1->sMasterRegs.MCMP1R = (1 - (1 - bb.buck_duty_cycle_  )) / 2 * Hrtim_Period;  // buck low d2
            HRTIM1->sMasterRegs.MCMP2R = (1 + (1 - bb.buck_duty_cycle_  )) / 2 * Hrtim_Period;  // buck high d1
        }
        
        HRTIM1->sCommonRegs.OENR = 0xF;
    }
  
}
 
 