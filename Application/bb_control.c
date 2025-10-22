#include "bb_control.h"

static float dt = 0, t = 0;
uint32_t DWT_Count;
uint32_t ADC1_Rx[3];
Buck_Boost_Str bb;
static GPIO_PinState debug[4];

void BB_Control_Init(void)
{
    bb.duty_max_ = H_duty - deadtime/2;
    bb.duty_min_ = L_duty + deadtime/2;
    bb.duty_ = bb.duty_min_;
    bb.current_out_ref_ = Current_Out_Max;
    bb.voltage_out_ref_ = Voltage_Out_Ref;
    // bb.state_ = None;

    // PID初始化
    // PID_Init(&bb.voltage_out_PID_,20.0f,0.04f,0.001f,0,1.9f,0.5f,1.9f,0.5f,0.001f);
    // PID_Init(&bb.current_out_PID_,6.0f,0,0,0,1.9f,0.5f,1.9f,0.5f,0.001f);

    PID_Init(&bb.voltage_out_PID_,0.6f,0.2f,0.001f,-0.001f,0.001f,1.0f,0.8f,0.8f,0,0,0,0.5,0,Integral_Limit | OutputFilter | DerivativeFilter);
    PID_Init(&bb.current_out_PID_,0.6f,0.2f,0.001f,-0.001f,0.001f,0.5f,0,0,0,0,0,1/Frequency,0,Integral_Limit | OutputFilter);//0.5  0.2

    
    // 滤波器初始化
    First_Order_Filter_Init(&bb.voltage_in_filter_,1/Frequency,30);
    First_Order_Filter_Init(&bb.voltage_out_filter_,1/Frequency,5);
    First_Order_Filter_Init(&bb.current_out_filter_,1/Frequency,5);

    // 开启ADC
    HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);
    HAL_ADC_Start_DMA(&hadc1,ADC1_Rx,3);

    // 开启hrtim
    HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2);
    HAL_HRTIM_WaveformCountStart(&hhrtim1, HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B);
    // __HAL_HRTIM_MASTER_ENABLE_IT(&hhrtim1,HRTIM_MASTER_IT_MREP);
}

void Buck_Boost_Task()
{
    dt=DWT_GetDeltaT(&DWT_Count);
    t += dt;
    Data_Handle();
    bb.duty_max_ = 0.6;
    bb.duty_min_ = 0.2;
    Duty_Set();
    Duty_Set_FFB();
    MOS_PWM_Set();
}
// float current_out_offset=0.0;
void Data_Handle()
{
    // static uint32_t count = 0;
    // static int flag = 1;
    // if (flag)
    // {
    //     count++;    
    //     current_out_offset+=bb.current_out_;
    //     if(count >= 100000)
    //     {
    //         current_out_offset /= count;
    //         flag = 0;
    //     }
    // }

    bb.voltage_out_ = (float)ADC1_Rx[0] * ADC_Ratio * Voltage_Ratio;
    bb.voltage_in_ = (float)ADC1_Rx[1] * ADC_Ratio * Voltage_Ratio;
    bb.current_out_ = (float)(ADC1_Rx[2]) * ADC_Ratio * Current_Ratio - Current_Out_Offset;
    
    bb.voltage_out_f_ = First_Order_Filter_Calculate(&bb.voltage_out_filter_,bb.voltage_out_);
    bb.voltage_in_f_ = First_Order_Filter_Calculate(&bb.voltage_in_filter_,bb.voltage_in_);
    bb.current_out_f_ = First_Order_Filter_Calculate(&bb.current_out_filter_,bb.current_out_);

    // buck连vin,boost连vout
    bb.voltage_gain_measure_ = bb.voltage_out_f_ / bb.voltage_in_f_;
    bb.voltage_gain_ref_ = Voltage_Out_Ref / bb.voltage_in_f_;
    
    bb.power_out_ = bb.voltage_out_f_ * bb.current_out_f_;


}

void Duty_Set()
{
    static float duty_cyc1;
	static float duty_cyc2;
	static float output;
    static float dutyoutput;
    static float count_duty_set = 0;

    duty_cyc1 = Inc_PID_Calculate(&bb.voltage_out_PID_, bb.voltage_out_f_, bb.voltage_out_ref_);//10k
    if(count_duty_set >= 10)
        duty_cyc2 = Inc_PID_Calculate(&bb.current_out_PID_, bb.current_out_f_, bb.current_out_ref_);//1k

    // if(duty_cyc1 < duty_cyc2)
    //     bb.dutyoutput_PID = duty_cyc1;
    // else
    //     bb.dutyoutput_PID = duty_cyc2;



    // if(duty_cyc1 < duty_cyc2)
    //     output = duty_cyc1;
    // else
    //     output = duty_cyc2;

    // bb.dutyoutput_PID = output/(1 + output);//0.333 - 0.655
    

    bb.dutyoutput_PID = float_constrain(duty_cyc1, 0 ,2.0f * bb.duty_max_);
    // bb.buck_duty_cycle_ = bb.dutyoutput_PID;
    // bb.boost_duty_cycle_ = bb.dutyoutput_PID;

    count_duty_set++;
}

void Duty_Set_FFB()
{   
    static float dutyoutput;
    static float duty_voltage_gain_measure;
    static float duty_voltage_gain_ref;
    duty_voltage_gain_measure = bb.voltage_gain_measure_ / (bb.voltage_gain_measure_ + 1.0f);
    duty_voltage_gain_ref = bb.voltage_gain_ref_ / (bb.voltage_gain_ref_ + 1.0f);
    dutyoutput = bb.dutyoutput_PID + (duty_voltage_gain_ref - duty_voltage_gain_measure) * Kp_FFB;
    dutyoutput = float_constrain(dutyoutput,bb.duty_min_,bb.duty_max_);
    bb.buck_duty_cycle_ = float_constrain(dutyoutput,bb.duty_min_,bb.duty_max_);
    bb.boost_duty_cycle_ = bb.buck_duty_cycle_;
}

void MOS_PWM_Set()
{
    if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_10) == GPIO_PIN_RESET)
    {
        HRTIM1->sMasterRegs.MCMP1R = 0;
        HRTIM1->sMasterRegs.MCMP2R = (1 + 1) / 2 * Hrtim_Period;
        HRTIM1->sMasterRegs.MCMP3R = 0;
        HRTIM1->sMasterRegs.MCMP4R = (1 + 1) / 2 * Hrtim_Period;
    }
    else if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_10) == GPIO_PIN_SET)
    {
        HRTIM1->sMasterRegs.MCMP1R = (1 - (1 - bb.buck_duty_cycle_  )) / 2 * Hrtim_Period;
        HRTIM1->sMasterRegs.MCMP2R = (1 + (1 - bb.buck_duty_cycle_  )) / 2 * Hrtim_Period;
        HRTIM1->sMasterRegs.MCMP3R = (1 + (1 - bb.boost_duty_cycle_ )) / 2 * Hrtim_Period;
        HRTIM1->sMasterRegs.MCMP4R = (1 - (1 - bb.boost_duty_cycle_ )) / 2 * Hrtim_Period;
    }
    if (bb.voltage_out_f_ > bb.voltage_out_ref_)
    {
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
    }
    else{
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
    }
    
    
}