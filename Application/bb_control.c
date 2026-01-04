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


static void Data_Handle();
static void Duty_Calculate();
static void MOS_PWM_Set();
static void BB_Error_Handler();

static float dt = 0, t = 0;
static float kp_ffb1;
static uint32_t DWT_Count;
static bool Prot_Delay_Flag = 0;
static bool Debug_Mode = 0;
Buck_Boost_Str bb = {0};
static float square_ratio_a = 3;
enum Buck_Boost_State bb_state = Boost;
enum Buck_Boost_State last_bb_state = Boost;
uint8_t USART_Debug_Flag = 0;


void BB_Control_Init(void)
{
    bb.current_out_ref_ = Current_Out_Max;
    bb.voltage_out_ref_ = Voltage_Out_Ref;
    kp_ffb1 = Kp_FFB;
    Debug_Mode = 0;

    PID_Init(&bb.voltage_gain_PID_,1.5f,0.5f,1.0f,-1.0f,0.001f,1.0f,2.0f,-0.1f,0,0,0,0.5,0,Integral_Limit | OutputFilter | DerivativeFilter);
    PID_Init(&bb.current_out_PID_,1.5f,0.5f,1.0f,-1.0f,0.001f,0.3f,1.6f,0,0,0,0,0.5,0,Integral_Limit | OutputFilter | DerivativeFilter);//0.5  0.2

    // 开启hrtim
    HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2);
    HAL_HRTIM_WaveformCountStart(&hhrtim1, HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B);

    // 初始化bben指示灯
    HAL_GPIO_WritePin(BBEN_Indicator_GPIO_Port, BBEN_Indicator_Pin, GPIO_PIN_SET);
}

void Buck_Boost_Task()
{
    BB_Error_Handler();
    dt=DWT_GetDeltaT(&DWT_Count);
    t += dt;
    if(!is_TOE_Overtime(ADC1_WATCHDOG1_TOE) || !is_TOE_Overtime(ADC1_WATCHDOG2_TOE) || !is_TOE_Overtime(ADC2_WATCHDOG1_TOE))
    {
    
    }
    else
    {
        Data_Handle();
        Duty_Calculate();
        MOS_PWM_Set();
    }
    if(Debug_Mode)
    {
        Prot_Delay_Flag = 0;
        Data_Handle();
        Duty_Calculate();
        MOS_PWM_Set();
    }
    
}
static void Data_Handle()
{
    switch(last_bb_state)
    {
    case Buck:
        if(bb.voltage_in_f_ < 1.1*Voltage_Out_Ref)
        {
            last_bb_state = bb_state;
            bb_state = Buck_Boost;
        }
        break;
    case Boost:
        if(0.9*Voltage_Out_Ref < bb.voltage_in_f_)
        {
            last_bb_state = bb_state;
            bb_state = Buck_Boost;
        }
        break;
    case Buck_Boost:
        if(1.2*Voltage_Out_Ref < bb.voltage_in_f_ )
        {
            last_bb_state = bb_state;
            bb_state = Buck;
        }
        else if(bb.voltage_in_f_ < 0.8*Voltage_Out_Ref)
        {
            last_bb_state = bb_state;
            bb_state = Boost;
        }
        break;
    case None:
        last_bb_state = Boost;
        bb_state = Boost;
        break;
    case VoltIpt_Error:
        if(Prot_Delay_Flag)
        {
            last_bb_state = bb_state;
        }
        else if(bb.voltage_in_f_ < 0.9*Voltage_Out_Ref)
        {
            last_bb_state = bb_state;
            bb_state = Boost;
        }
        else if (1.1*Voltage_Out_Ref < bb.voltage_in_f_)
        {
            last_bb_state = bb_state;
            bb_state = Buck;
        }
        else
        {
            last_bb_state = bb_state;
            bb_state = Buck_Boost;
        }
        break;
    case Ext_Err:
        if(0)
        {
            last_bb_state = bb_state;
        }
        break;
    default:
        break;
    }

    // buck连vin,boost连vout
    if(!bb.voltage_in_f_)
    {
        bb.voltage_gain_measure_ = 0;
        bb.voltage_gain_ref_ = 0;
    }
    else
    {
        bb.voltage_gain_measure_ = bb.voltage_out_f_ / bb.voltage_in_f_;
        bb.voltage_gain_ref_ = Voltage_Out_Ref / bb.voltage_in_f_;
    }

    
}
static void Duty_Calculate()
{   
    static float voltage_gain_measure;
    static float voltage_gain_ref;
    static float voltage_gain_max;
    static float voltage_gain_min;
    static float duty_cyc1;
	static float duty;
    static float k_current;
    static float k_voltage;
    static float voltage_gain_PID_output;
    static float voltage_gain_FFB_output;
    static float voltage_gain_final_output;
    static float voltage_gain_buck_output;
    static float voltage_gain_boost_output;
    static float voltage_gain_bb_output;

    Inc_PID_Calculate(&bb.voltage_gain_PID_, bb.voltage_gain_measure_, bb.voltage_gain_ref_);//10k
    Inc_PID_Calculate(&bb.current_out_PID_,bb.current_out_f_, bb.current_out_ref_);//10k

    k_current = pow(bb.current_out_f_/bb.current_out_ref_,square_ratio_a);
    k_current = float_constrain(k_current,0,1.0f);
    
    k_voltage = 1 - k_current;
    // voltage_gain_PID_output = bb.voltage_gain_PID_.Output > bb.current_out_PID_.Output ? bb.current_out_PID_.Output : bb.voltage_gain_PID_.Output;
    voltage_gain_PID_output = k_current * bb.current_out_PID_.Output + k_voltage * bb.voltage_gain_PID_.Output;
    // dutyoutput = 0.25;

    // 变量转换&动态限幅
    voltage_gain_measure = bb.voltage_gain_measure_;
    voltage_gain_ref = bb.voltage_gain_ref_;
    voltage_gain_max = 1.2f * voltage_gain_ref;
    voltage_gain_min = 0.8f * voltage_gain_ref;
        
    //前馈赋值
    voltage_gain_FFB_output = (voltage_gain_ref - voltage_gain_measure) * k_voltage;
    voltage_gain_final_output = float_constrain(voltage_gain_FFB_output * Kp_FFB + voltage_gain_PID_output,voltage_gain_min,voltage_gain_max);

    switch (bb_state)
    {
    case Buck:
        //本地限幅
        voltage_gain_buck_output = float_constrain(voltage_gain_final_output,0.05f,0.95f);
        //电压增益->占空比
        bb.buck_duty_cycle_ = voltage_gain_buck_output;
        bb.boost_duty_cycle_ = 0.95f;
        break;
    
    case Boost:
        //本地限幅
        voltage_gain_boost_output = float_constrain(voltage_gain_final_output,1.05f,2.45f);
        //电压增益->占空比&反占空比转换
        bb.boost_duty_cycle_ = 1.0f - 1.0f / voltage_gain_boost_output;
        bb.buck_duty_cycle_ = 0.95f;
        break;
    
    case Buck_Boost:

        //本地限幅
        voltage_gain_bb_output = float_constrain(voltage_gain_final_output,0.50f,1.50f);

        //电压增益->占空比    
        bb.buck_duty_cycle_ = voltage_gain_bb_output / (1.0f + voltage_gain_bb_output);
        bb.boost_duty_cycle_ = 1.0f - bb.buck_duty_cycle_;
        break;
    case VoltIpt_Error:

    break;
    
    default:
        break;
    }
}

static void MOS_PWM_Set()
{
    if(USART_Debug_Flag)
    {
        Tx_Buf.duty1.data[0] = (uint8_t)(bb.buck_duty_cycle_ * 10) + '0';
        Tx_Buf.duty1.data[1] = (uint8_t)(bb.buck_duty_cycle_ * 100) % 10 + '0';
        Tx_Buf.duty2.data[0] = (uint8_t)(bb.boost_duty_cycle_ * 10) + '0';
        Tx_Buf.duty2.data[1] = (uint8_t)(bb.boost_duty_cycle_ * 100) % 10 + '0';
    }
    if(is_TOE_Overtime(ADC1_WATCHDOG2_TOE))
    {
        if(Prot_Delay_Flag)
        {
            HRTIM1->sMasterRegs.MCMP1R = Hrtim_Period;
            HRTIM1->sMasterRegs.MCMP2R = 0;
            HRTIM1->sMasterRegs.MCMP3R = Hrtim_Period;
            HRTIM1->sMasterRegs.MCMP4R = 0;
        }
        else if(bb_state == Buck_Boost)
        {
            HRTIM1->sMasterRegs.MCMP1R = (1 - (1 - bb.buck_duty_cycle_  ) / 2) * Hrtim_Period;  // buck low d2
            HRTIM1->sMasterRegs.MCMP2R = (1 - bb.buck_duty_cycle_  ) / 2 * Hrtim_Period;  // buck high d1
            HRTIM1->sMasterRegs.MCMP3R = (1 - (1 - bb.boost_duty_cycle_  )) / 2 * Hrtim_Period;;  // boost low d3
            HRTIM1->sMasterRegs.MCMP4R = (1 + (1 - bb.boost_duty_cycle_  )) / 2 * Hrtim_Period;  // boost high d4
        }
        else
        {
            HRTIM1->sMasterRegs.MCMP1R = (1 - (1 - bb.buck_duty_cycle_  )) / 2 * Hrtim_Period;  // buck low d2
            HRTIM1->sMasterRegs.MCMP2R = (1 + (1 - bb.buck_duty_cycle_  )) / 2 * Hrtim_Period;  // buck high d1
            HRTIM1->sMasterRegs.MCMP3R = (1 - (1 - bb.boost_duty_cycle_  )) / 2 * Hrtim_Period;;  // boost low d3
            HRTIM1->sMasterRegs.MCMP4R = (1 + (1 - bb.boost_duty_cycle_  )) / 2 * Hrtim_Period;  // boost high d4
        }
        
        HRTIM1->sCommonRegs.OENR = 0xF;
    }
  
}

static void BB_Error_Handler()
{
    if(bb.voltage_in_f_ < Voltage_In_Min || Voltage_In_Max < bb.voltage_in_f_ )
    {
        last_bb_state = bb_state;
        bb_state = VoltIpt_Error;
        GPIOA->BRR = GPIO_PIN_7|GPIO_PIN_6;
        Prot_Delay_Flag = 1;
        Detect_Hook(VoltIpt_Error_TOE);
    }
    else if(Voltage_In_Min <= bb.voltage_in_f_ && bb.voltage_in_f_ <= Voltage_In_Max)
    {
        if(is_TOE_Overtime(VoltIpt_Error_TOE))
        {
            Prot_Delay_Flag = 0;
            GPIOA->BSRR = GPIO_PIN_7;
        }
        
    }
    if(0.2f*Current_Out_Max < bb.current_out_f_ && Prot_Delay_Flag == 0)
    {
        GPIOA->BSRR = GPIO_PIN_7|GPIO_PIN_6;
    }
    else if(Prot_Delay_Flag == 0)   
    {
        GPIOA->BSRR = GPIO_PIN_7;
        GPIOA->BRR = GPIO_PIN_6;
    }
}
    