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
static uint32_t Last_VoltProt_Time = 0;
static bool Prot_Delay_Flag = 0;
Buck_Boost_Str bb = {0};
static float square_ratio_a = 3;
enum Buck_Boost_State bb_state = Boost;
enum Buck_Boost_State last_bb_state = Boost;
uint8_t USART_Debug_Flag = 0;


void BB_Control_Init(void)
{
    bb.duty_max_ = 0.83;
    bb.duty_min_ = 0.2;
    bb.duty_ = bb.duty_min_;
    bb.current_out_ref_ = Current_Out_Max;
    bb.voltage_out_ref_ = Voltage_Out_Ref;
    kp_ffb1 = Kp_FFB;

    PID_Init(&bb.voltage_gain_PID_,1.5f,0.5f,1.0f,-1.0f,0.001f,1000.0f,2000.0f,-0.1f,0,0,0,0.5,0,Integral_Limit | OutputFilter | DerivativeFilter);
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
        bb.voltage_gain_min_ = 0;
        bb.voltage_gain_max_ = 0;
    }
    else
    {
        bb.voltage_gain_measure_ = bb.voltage_out_f_ / bb.voltage_in_f_;
        bb.voltage_gain_ref_ = Voltage_Out_Ref / bb.voltage_in_f_;
        bb.voltage_gain_min_ = 0.8 * Voltage_Out_Ref / bb.voltage_in_f_;
        bb.voltage_gain_max_ = 1.6 * Voltage_Out_Ref / bb.voltage_in_f_;
    }

    
}
static void Duty_Calculate()
{   
    static float duty_voltage_gain_measure;
    static float duty_voltage_gain_ref;
    static float duty_cyc1;
	static float duty;
    static float k_current;
    static float k_voltage;
    static float voltage_gain_PID_compete;
    static float duty_FFB_output_noconstrain;

    Inc_PID_Calculate(&bb.voltage_gain_PID_, bb.voltage_gain_measure_, bb.voltage_gain_ref_);//10k
    Inc_PID_Calculate(&bb.current_out_PID_,bb.current_out_f_,bb.current_out_ref_);

    k_current = pow(bb.current_out_f_/bb.current_out_ref_,square_ratio_a);
    k_current = float_constrain(k_current,0,1.0f);
    
    k_voltage = 1 - k_current;
    // voltage_gain_PID_compete = bb.voltage_gain_PID_.Output > bb.current_out_PID_.Output ? bb.current_out_PID_.Output : bb.voltage_gain_PID_.Output;
    voltage_gain_PID_compete = k_current * bb.current_out_PID_.Output + k_voltage * bb.voltage_gain_PID_.Output;
    // dutyoutput = 0.25;
    switch (bb_state)
    {
    case Buck:
        // pid输出
        duty_cyc1 = voltage_gain_PID_compete;
        bb.duty_PID_output_ = float_constrain(duty_cyc1,0,1.0f);//0 -- 1.0f

        // 前馈输出
        duty_voltage_gain_measure = bb.voltage_gain_measure_;
        duty_voltage_gain_ref = bb.voltage_gain_ref_;
        duty_FFB_output_noconstrain = (duty_voltage_gain_ref - duty_voltage_gain_measure) * k_voltage;
        bb.duty_FFB_output_ = float_constrain(duty_FFB_output_noconstrain,0,1.0f);
        

        // 动态占空比
        bb.duty_changing_min_ = bb.voltage_gain_min_;
        bb.duty_changing_max_ = bb.voltage_gain_max_;

        // 占空比约束
        duty = bb.duty_PID_output_ + bb.duty_FFB_output_;
        bb.duty_ = float_constrain(duty,bb.duty_changing_min_,bb.duty_changing_max_);
        bb.duty_ = float_constrain(bb.duty_,bb.duty_min_,bb.duty_max_);// 0.2f -- 0.83f

        bb.buck_duty_cycle_ = bb.duty_;
        bb.boost_duty_cycle_ = 0;
        break;
    
    case Boost:
        
        duty_cyc1 = 1 - 1.0f/voltage_gain_PID_compete;
        bb.duty_PID_output_ = float_constrain(duty_cyc1,0,1.0f);//0 -- 1.0f

        duty_voltage_gain_measure = 1 - 1.0f/bb.voltage_gain_measure_;
        if(isnan(duty_voltage_gain_measure))
        {
            duty_voltage_gain_measure = 0;
        }
        duty_voltage_gain_ref = 1 - 1.0f/bb.voltage_gain_ref_;
        if(isnan(duty_voltage_gain_ref))
        {
            duty_voltage_gain_ref = 0;
        }
        duty_FFB_output_noconstrain = (duty_voltage_gain_ref - duty_voltage_gain_measure) * k_voltage;
        bb.duty_FFB_output_ = float_constrain(duty_FFB_output_noconstrain,0,1.0f);

        bb.duty_changing_min_ = 1 - 1.0f/bb.voltage_gain_min_;
        bb.duty_changing_max_ = 1 - 1.0f/bb.voltage_gain_max_;
        if(isnan(bb.duty_changing_min_) || isnan(bb.duty_changing_max_))
        {
            bb.duty_changing_min_ = 0;
            bb.duty_changing_max_ = 0;
        }

        duty = bb.duty_PID_output_ + bb.duty_FFB_output_;
        bb.duty_ = float_constrain(duty,bb.duty_changing_min_,bb.duty_changing_max_);
        bb.duty_ = float_constrain(bb.duty_,bb.duty_min_,bb.duty_max_);// 0.2f -- 0.83f

        bb.buck_duty_cycle_ = 0;
        bb.boost_duty_cycle_ = bb.duty_;
        break;
    
    case Buck_Boost:

        duty_cyc1 = voltage_gain_PID_compete/(1.0f + voltage_gain_PID_compete);
        bb.duty_PID_output_ = float_constrain(duty_cyc1,0,1.0f);//0 -- 1.0f

        duty_voltage_gain_measure = bb.voltage_gain_measure_ / (bb.voltage_gain_measure_ + 1.0f);
        duty_voltage_gain_ref = bb.voltage_gain_ref_ / (bb.voltage_gain_ref_ + 1.0f);
        duty_FFB_output_noconstrain = (duty_voltage_gain_ref - duty_voltage_gain_measure) * k_voltage;
        bb.duty_FFB_output_ = float_constrain(duty_FFB_output_noconstrain,0,1.0f);

        bb.duty_changing_min_ = bb.voltage_gain_min_ / (bb.voltage_gain_min_ + 1.0f);
        bb.duty_changing_max_ = bb.voltage_gain_max_ / (bb.voltage_gain_max_ + 1.0f);

        duty = bb.duty_PID_output_ + bb.duty_FFB_output_;
        bb.duty_ = float_constrain(duty,bb.duty_changing_min_,bb.duty_changing_max_);
        bb.duty_ = float_constrain(bb.duty_,bb.duty_min_,bb.duty_max_);// 0.2f -- 0.83f

        bb.buck_duty_cycle_ = bb.duty_;
        bb.boost_duty_cycle_ = bb.duty_;
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
        else
        {
            HRTIM1->sMasterRegs.MCMP1R = (1 - (1 - bb.buck_duty_cycle_  )) / 2 * Hrtim_Period;  // buck low
            HRTIM1->sMasterRegs.MCMP2R = (1 + (1 - bb.buck_duty_cycle_  )) / 2 * Hrtim_Period;  // buck high d1
            HRTIM1->sMasterRegs.MCMP3R = (1 - bb.boost_duty_cycle_ ) / 2 * Hrtim_Period;  // boost low d3
            HRTIM1->sMasterRegs.MCMP4R = (1 + bb.boost_duty_cycle_ ) / 2 * Hrtim_Period;  // boost high
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
        Last_VoltProt_Time = DWT_Count;
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
    