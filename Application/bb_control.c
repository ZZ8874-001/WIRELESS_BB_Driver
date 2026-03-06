//**************************************************************************************/
/*
 * **************************************************************************
 * ********************                                  ********************
 * ********************      COPYRIGHT INFORMATION       ********************
 * ********************                                  ********************
 * **************************************************************************
 *                                                                          *
 *                                   _oo8oo_                                *
 *                                  o8888888o                               *
 *                                  88" . "88                               *
 *                                  (| -_- |)                               *
 *                                  0\  =  /0                               *
 *                                ___/'==='\___                             *
 *                              .' \\|     |// '.                           *
 *                             / \\|||  :  |||// \                          *
 *                            / _||||| -:- |||||_ \                         *
 *                           |   | \\\  -  /// |   |                        *
 *                           | \_|  ''\---/''  |_/ |                        *
 *                           \  .-\__  '-'  __/-.  /                        *
 *                         ___'. .'  /--.--\  '. .'___                      *
 *                      ."" '<  '.___\_<|>_/___.'  >' "".                   *
 *                     | | :  `- \`.:`\ _ /`:.`/ -`  : | |                  *
 *                     \  \ `-.   \_ __\ /__ _/   .-` /  /                  *
 *                 =====`-.____`.___ \_____/ ___.`____.-`=====              *
 *                                   `=---=`                                *
 * **************************************************************************
 * ********************                                  ********************
 * ********************                                  ********************
 * ********************         佛祖保佑 永远无BUG        ********************
 * ********************                                  ********************
 * **************************************************************************
 */

#include "bb_control.h"
#include "detect_task.h"

#include <stdbool.h>
#include "stdint.h"
#include "adc.h"
#include "hrtim.h"

#include "bsp_dwt.h"
#include "bsp_uart.h"

#include "filter32.h"

#define NFB_CALCULATING_FREQUENCY (10000.0f)
#define BUCK_HIGH 52.0f

static void Choose_State(void);
static void Data_Handle(void);
static void Duty_Calculate();
static void MOS_PWM_Set();
static void NFB_Calculate();

static float kp_ffb1;
static float Kp_Volt_NFB = -0.1f;
static float Kp_Curr_NFB = -0.4f;

static bool Wireless_data = false;
static bool Wireless_EN_flag = false;

static bool is_CC = 0;
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
    bb.current_out_ref_ = CURRENT_OUT_MAX;
    kp_ffb1 = Kp_FFB;
    Debug_Mode = 0;

    //PID_Init(&bb.voltage_gain_PID_,1.5f,0.5f,  1.0f,-1.0f,  0.001f,  2.0f,1.0f,0,  1,1,  0,0.5,  0,Integral_Limit | DerivativeFilter );
    //PID_Init(&bb.current_out_PID_,1.5f,0.5f,  1.0f,-1.0f,  0.001f,  0.3f,1.6f,0,  1,1,  0,0.5,  0,Integral_Limit | DerivativeFilter );//0.5  0.2
    

    First_Order_Filter_Init(&bb.current_gain_NFB_filter_,1.0f/NFB_CALCULATING_FREQUENCY,50);
    // 开启hrtim
    
    
    while(HAL_HRTIM_WaveformCountStart(&hhrtim1, HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B) != HAL_OK)
    {
    }
    while(HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2) != HAL_OK)
    {
    }


    // 初始化bben指示灯
    BBEN_Indicator_GPIO_Port->BSRR = BBEN_Indicator_Pin;
}

void Buck_Boost_Task(void)
{
    if(is_TOE_Overtime(USART3_BUCKEN_TOE))
    {
        Wireless_EN_flag = false;
    }
    else if(!Wireless_data)
    {
        Wireless_EN_flag = false;
    }
    else
    {
        Wireless_EN_flag = true;
    }

    Choose_State();
    Data_Handle();
    Duty_Calculate();

    if((!is_TOE_Overtime(ADC1_WATCHDOG1_TOE) 
    || !is_TOE_Overtime(ADC1_WATCHDOG2_TOE)) 
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
            HRTIM1->sCommonRegs.OENR = 0xF;
            GPIOA->BSRR = GPIO_PIN_6 | GPIO_PIN_7;
            GPIOA->BRR = 0.2f*CURRENT_OUT_MAX<bb.current_out_f_ ? 0:GPIO_PIN_6;

            float buck_into_error_flag = 0;
            if(is_TOE_Overtime(USART3_BUCKEN_TOE))
            {
                if(bb.voltage_in_f_ < VOLTAGE_IN_MIN || VOLTAGE_IN_MAX < bb.voltage_in_f_)
                {
                    buck_into_error_flag = 1;
                }
            }
            else if(!Wireless_EN_flag)
            {
                buck_into_error_flag = 1;
            }
            else
            {
                last_bb_state = bb_state;
            }

            // 癫疯之作1
            // buck_into_error_flag = is_TOE_Overtime(USART3_BUCKEN_TOE) ? ((bb.voltage_in_f_ < VOLTAGE_IN_MIN || VOLTAGE_IN_MAX < bb.voltage_in_f_)?1:0) : ((!Wireless_EN_flag)?1:0);

            if(buck_into_error_flag && bb.voltage_in_f_ < BUCK_HIGH)
            {
                last_bb_state = bb_state;
                bb_state = VoltIpt_Error;

                Detect_Hook(VoltIpt_Error_TOE);
            }
            break;
        case None:
            last_bb_state = bb_state;
            bb_state = VoltIpt_Error;

            GPIOA->BRR = GPIO_PIN_7|GPIO_PIN_6;
            Detect_Hook(VoltIpt_Error_TOE);
            break;
        case VoltIpt_Error:
            HRTIM1->sCommonRegs.ODISR = 0xF;
            GPIOA->BRR = GPIO_PIN_7|GPIO_PIN_6;

            float error_into_ss_flag = 0;
            if(bb.voltage_in_f_ < VOLTAGE_IN_MIN || VOLTAGE_IN_MAX < bb.voltage_in_f_)
            {
                Detect_Hook(VoltIpt_Error_TOE);
            }

            if(is_TOE_Overtime(USART3_BUCKEN_TOE))
            {
                if(is_TOE_Overtime(VoltIpt_Error_TOE))
                {
                    error_into_ss_flag = 1;
                }
            }
            else if(Wireless_EN_flag == 1)
            {
                error_into_ss_flag = 1;
            }

            // 癫疯之作2
            // error_into_ss_flag = is_TOE_Overtime(USART3_BUCKEN_TOE) ? (is_TOE_Overtime(VoltIpt_Error_TOE) ? 1:0) : (Wireless_EN_flag == 1 ? 1:0);

            if(bb.voltage_in_f_ > BUCK_HIGH)
            {
                last_bb_state = bb_state;
                bb_state = Soft_Start;
            }
            else if(error_into_ss_flag)
            {
                last_bb_state = bb_state;
                bb_state = Soft_Start;
            }

            break;
        case Soft_Start:
            HRTIM1->sCommonRegs.OENR = 0xF;
            GPIOA->BSRR = GPIO_PIN_6 | GPIO_PIN_7;
            GPIOA->BRR = 0.2f*CURRENT_OUT_MAX<bb.current_out_f_ ? 0:GPIO_PIN_6;

            float ss_into_error_flag = 0;
            if(is_TOE_Overtime(USART3_BUCKEN_TOE))
            {
                if(bb.voltage_in_f_ < VOLTAGE_IN_MIN || VOLTAGE_IN_MAX < bb.voltage_in_f_)
                {
                    ss_into_error_flag = 1;
                }
            }
            else if(!Wireless_EN_flag)
            {
                ss_into_error_flag = 1;
            }

            // 癫疯之作3,其实我感觉三个都大差不差)
            // ss_into_error_flag = is_TOE_Overtime(USART3_BUCKEN_TOE) ? (bb.voltage_in_f_ < VOLTAGE_IN_MIN || VOLTAGE_IN_MAX < bb.voltage_in_f_ ? 1:0) : ((!Wireless_EN_flag) ? 1:0);

            if(ss_into_error_flag)
            {
                last_bb_state = bb_state;
                bb_state = VoltIpt_Error;

                Detect_Hook(VoltIpt_Error_TOE);
            }
            
            if(last_bb_state != Soft_Start)
            {
                last_bb_state = bb_state;
                enter_soft_start_time = USER_GetTick();
            }
            else if(USER_GetTick() - enter_soft_start_time > 1000)
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
    //static float k_current,k_voltage = 0;
    static float voltage_gain_PID_output,voltage_gain_FFB_output = 0;
    
    bb.voltage_gain_measure_ = float_constrain(bb.voltage_out_f_ / bb.voltage_in_f_,0.05f,0.95f);
    bb.voltage_gain_ref_ = float_constrain(VOLTAGE_OUT_REF / bb.voltage_in_f_,0.05f,0.95f);


    if(bb.current_out_f_ > 0.05f + bb.current_out_ref_||is_CC == 1)
    {
        is_CC = 1;
        Kp_Curr_NFB = float_constrain(Kp_Curr_NFB * 1.001f,-0.6f,-0.01f);

    }
    if(bb.current_out_f_ < bb.current_out_ref_)
    {
        is_CC = 0;
        Kp_Curr_NFB = float_constrain(Kp_Curr_NFB * 0.95f,-0.6f,-0.01f);
    }


    NFB_Calculate();//10k

    if(bb.current_gain_NFB_f_ > bb.voltage_gain_NFB_ * 1.05f && is_TOE_Overtime(CURRENT_TO_VOLTAGE_TOE))
    {
        voltage_gain_PID_output = bb.voltage_gain_NFB_;
        
    }
    else if(bb.current_gain_NFB_f_ < bb.voltage_gain_NFB_ )
    {
        voltage_gain_PID_output = bb.current_gain_NFB_f_;
        Detect_Hook(CURRENT_TO_VOLTAGE_TOE);
    }
    else
    {
        voltage_gain_PID_output = (bb.voltage_gain_NFB_ + bb.current_gain_NFB_f_) / 2.0f;
    }

    //k_current = pow(bb.current_out_f_/bb.current_out_ref_,square_ratio_a);
    //k_current = float_constrain(k_current,0,1.0f);
    
    //k_voltage = 1 - k_current;

    //voltage_gain_PID_output = k_current * bb.current_gain_NFB_ + k_voltage * bb.voltage_gain_NFB_;
        
    //前馈赋值
    //voltage_gain_FFB_output = (bb.voltage_gain_ref_ - bb.voltage_gain_measure_) * k_voltage;
    //voltage_gain_final_output = float_constrain(voltage_gain_FFB_output * Kp_FFB + voltage_gain_PID_output,0.8f * bb.voltage_gain_ref_,1.2f * bb.voltage_gain_ref_);
    voltage_gain_final_output = voltage_gain_PID_output;
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
        soft_start_gain = float_constrain(voltage_gain_final_output,0.025f,0.95f) * (USER_GetTick() - enter_soft_start_time) / 1000.0f + 0.026f;
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
        if(!is_TOE_Overtime(VoltIpt_Error)
        || (last_bb_state != VoltIpt_Error && bb_state == VoltIpt_Error)
        || (bb_state == Soft_Start && USER_GetTick() - enter_soft_start_time < 1))
        {
            HRTIM1->sMasterRegs.MCMP1R = Hrtim_Period;
            HRTIM1->sMasterRegs.MCMP2R = 0;
        }
        else if((bb_state == Buck || last_bb_state == Soft_Start))
        {
            HRTIM1->sMasterRegs.MCMP1R = (1 - (1 - bb.buck_duty_cycle_  )) / 2 * Hrtim_Period;  // buck low d2
            HRTIM1->sMasterRegs.MCMP2R = (1 + (1 - bb.buck_duty_cycle_  )) / 2 * Hrtim_Period;  // buck high d1
        }
        
    }
  
}
 
static void NFB_Calculate()
{
    
    float Gain_Limit = fmaxf(0.95f - bb.voltage_gain_ref_,bb.voltage_gain_ref_ - 0.05f);

    //电压部分
    bb.voltage_gain_NFB_ = float_constrain(bb.voltage_gain_ref_ + Kp_Volt_NFB * sinf((bb.voltage_gain_measure_ - bb.voltage_gain_ref_)/Gain_Limit * PI * 0.5f),0.05f,0.95f);

    //电流部分
    float Curr_Normalized = float_constrain((bb.current_out_f_ - bb.current_out_ref_) / bb.current_out_ref_,-0.95,0.95);
    bb.current_gain_NFB_ = float_constrain(bb.voltage_gain_NFB_ + Kp_Curr_NFB * sinf(Curr_Normalized / Gain_Limit * PI * 0.5f),0.05f,0.95f);

    bb.current_gain_NFB_f_ = First_Order_Filter_Calculate(&bb.current_gain_NFB_filter_,bb.current_gain_NFB_);
    
}

void WirelessRx_DataHandle(uint8_t *data)
{
    Wireless_data = (*data == 0xAA || *(data + 1) == 0xAA) ? true : false;
}