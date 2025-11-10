#include "bb_control.h"

static float dt = 0, t = 0;
float kp_ffb1;
uint32_t DWT_Count;
uint32_t ADC1_Rx[2];
uint32_t ADC2_Rx;
Buck_Boost_Str bb = {0};


void BB_Control_Init(void)
{
    bb.duty_max_ = 0.83;
    bb.duty_min_ = 0.2;
    bb.duty_ = bb.duty_min_;
    bb.current_out_ref_ = Current_Out_Max;
    bb.voltage_out_ref_ = Voltage_Out_Ref;
    kp_ffb1 = Kp_FFB;

    PID_Init(&bb.voltage_gain_PID_,1.5f,0.5f,1.0f,-1.0f,0.001f,1000.0f,2000.0f,-0.1f,0,0,0,0.5,0,Integral_Limit | OutputFilter | DerivativeFilter);
    PID_Init(&bb.current_out_PID_,1.5f,0.5f,1.0f,-1.0f,0.001f,0,100.0f,1.0f,0,0,0,0.5,0,Integral_Limit | OutputFilter | DerivativeFilter);//0.5  0.2

    
    // 滤波器初始化
    First_Order_Filter_Init(&bb.voltage_in_filter_,1/Frequency,30);
    First_Order_Filter_Init(&bb.voltage_out_filter_,1/Frequency,5);
    First_Order_Filter_Init(&bb.current_out_filter_,1/Frequency,20);
    
    // 开启ADC
    HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED);
    HAL_ADC_Start_DMA(&hadc1,ADC1_Rx,2);
    HAL_ADC_Start_DMA(&hadc2,&ADC2_Rx,1);

    // 开启hrtim
    HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2);
    HAL_HRTIM_WaveformCountStart(&hhrtim1, HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B);

    // 初始化bben指示灯
    HAL_GPIO_WritePin(BBEN_Indicator_GPIO_Port, BBEN_Indicator_Pin, GPIO_PIN_SET);
}

void Buck_Boost_Task()
{
    dt=DWT_GetDeltaT(&DWT_Count);
    t += dt;
    Data_Handle();
    Duty_Calculate();
    MOS_PWM_Set();
}
void Data_Handle()
{

    bb.voltage_out_ = (float)ADC1_Rx[0] * ADC_Ratio * Voltage_Ratio - Voltage_Out_Offset;
    bb.voltage_in_ = (float)ADC1_Rx[1] * ADC_Ratio * Voltage_Ratio;
    bb.current_out_ = (float)(ADC2_Rx * ADC_Ratio - Current_Out_Offset) * Current_Ratio;
    
    bb.voltage_out_f_ = First_Order_Filter_Calculate(&bb.voltage_out_filter_,bb.voltage_out_);
    bb.voltage_in_f_ = First_Order_Filter_Calculate(&bb.voltage_in_filter_,bb.voltage_in_);
    bb.current_out_f_ = First_Order_Filter_Calculate(&bb.current_out_filter_,bb.current_out_);

    if(bb.voltage_in_f_ > 1.2*Voltage_Out_Ref)
    {
        bb.status_ = Buck;
    }
    else if(bb.voltage_in_f_ < 0.8*Voltage_Out_Ref)
    {
        bb.status_ = Boost;
    }
    else
    {
        bb.status_ = Buck_Boost;
    }

    // buck连vin,boost连vout
    bb.voltage_gain_measure_ = bb.voltage_out_f_ / bb.voltage_in_f_;
    bb.voltage_gain_ref_ = Voltage_Out_Ref / bb.voltage_in_f_;
    bb.voltage_gain_min_ = 0.8 * Voltage_Out_Ref / bb.voltage_in_f_;
    bb.voltage_gain_max_ = 1.6 * Voltage_Out_Ref / bb.voltage_in_f_;
    
}
void Duty_Calculate()
{   
    static float duty_voltage_gain_measure;
    static float duty_voltage_gain_ref;
    static float duty_cyc1;
	static float duty;
    static float voltage_gain_PID_compete;
    static float duty_FFB_output_noconstrain;
    static uint8_t count_duty_set_current = 0;
    static uint8_t flag_shi = 0;

    Inc_PID_Calculate(&bb.voltage_gain_PID_, bb.voltage_gain_measure_, bb.voltage_gain_ref_);//10k
    Inc_PID_Calculate(&bb.current_out_PID_,bb.current_out_f_,bb.current_out_ref_);
    // voltage_gain_PID_compete = bb.voltage_gain_PID_.Output > bb.current_out_PID_.Output ? bb.current_out_PID_.Output : bb.voltage_gain_PID_.Output;
    if(bb.voltage_gain_PID_.Output - bb.current_out_PID_.Output > 0.01f)
    {
        voltage_gain_PID_compete = bb.current_out_PID_.Output;
        kp_ffb1 = 0;
        flag_shi = 0;
    }
    else if(bb.voltage_gain_PID_.Output - bb.current_out_PID_.Output < -0.01f)
    {
        voltage_gain_PID_compete = bb.voltage_gain_PID_.Output;
        kp_ffb1 = Kp_FFB;
        flag_shi = 1;
    }
    else
    {
        if(flag_shi)
        {
            voltage_gain_PID_compete = bb.voltage_gain_PID_.Output;
        }
        else
        {
            voltage_gain_PID_compete = bb.current_out_PID_.Output;
        }
    }
    // dutyoutput = 0.25;
    switch (bb.status_)
    {
    case Buck:
        // pid输出
        duty_cyc1 = voltage_gain_PID_compete;
        bb.duty_PID_output_ = float_constrain(duty_cyc1,0,1.0f);//0 -- 1.0f

        // 前馈输出
        duty_voltage_gain_measure = bb.voltage_gain_measure_;
        duty_voltage_gain_ref = bb.voltage_gain_ref_;
        duty_FFB_output_noconstrain = (duty_voltage_gain_ref - duty_voltage_gain_measure) * kp_ffb1;
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
        duty_voltage_gain_ref = 1 - 1.0f/bb.voltage_gain_ref_;
        duty_FFB_output_noconstrain = (duty_voltage_gain_ref - duty_voltage_gain_measure) * kp_ffb1;
        bb.duty_FFB_output_ = float_constrain(duty_FFB_output_noconstrain,0,1.0f);

        bb.duty_changing_min_ = 1 - 1.0f/bb.voltage_gain_min_;
        bb.duty_changing_max_ = 1 - 1.0f/bb.voltage_gain_max_;

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
        duty_FFB_output_noconstrain = (duty_voltage_gain_ref - duty_voltage_gain_measure) * kp_ffb1;
        bb.duty_FFB_output_ = float_constrain(duty_FFB_output_noconstrain,0,1.0f);

        bb.duty_changing_min_ = bb.voltage_gain_min_ / (bb.voltage_gain_min_ + 1.0f);
        bb.duty_changing_max_ = bb.voltage_gain_max_ / (bb.voltage_gain_max_ + 1.0f);

        duty = bb.duty_PID_output_ + bb.duty_FFB_output_;
        bb.duty_ = float_constrain(duty,bb.duty_changing_min_,bb.duty_changing_max_);
        bb.duty_ = float_constrain(bb.duty_,bb.duty_min_,bb.duty_max_);// 0.2f -- 0.83f

        bb.buck_duty_cycle_ = bb.duty_;
        bb.boost_duty_cycle_ = bb.duty_;
        break;
    
    default:
        break;
    }
}

void MOS_PWM_Set()
{
    if (bb.voltage_out_f_ > bb.voltage_out_ref_)
    {
        HAL_GPIO_WritePin(VCC_Indicator_GPIO_Port,VCC_Indicator_Pin,GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(VCC_Indicator_GPIO_Port,VCC_Indicator_Pin,GPIO_PIN_RESET);
    }
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET);

    HRTIM1->sMasterRegs.MCMP1R = (1 - (1 - bb.buck_duty_cycle_  )) / 2 * Hrtim_Period;  // buck low
    HRTIM1->sMasterRegs.MCMP2R = (1 + (1 - bb.buck_duty_cycle_  )) / 2 * Hrtim_Period;  // buck high d1
    HRTIM1->sMasterRegs.MCMP3R = (1 - bb.boost_duty_cycle_ ) / 2 * Hrtim_Period;  // boost low d3
    HRTIM1->sMasterRegs.MCMP4R = (1 + bb.boost_duty_cycle_ ) / 2 * Hrtim_Period;  // boost high

}