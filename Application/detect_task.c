#include "detect_task.h"
#include "bb_control.h"

static Detect_t Detect_List[DETECT_LIST_LENGTH + 1]={0};

void Detect_Init()
{
    float set_item[DETECT_LIST_LENGTH] = 
    {
        1000,// ms
        1000,
        1000,
        2500,
        CURRENT_TO_VOLTAGE_DELAY,
        1000,
    };

    for(uint8_t i = 0;i<DETECT_LIST_LENGTH;i++)
    {
        Detect_List[i].Overtime_ms = set_item[i];
        Detect_List[i].is_Lost = 1;
        Detect_List[i].Overtime_Exit = 1;
        Detect_List[i].dt_ms = 0.0f;
    }
}

void Detect_Task()
{
    Detect_List[DETECT_LIST_LENGTH].is_Lost = 0;
    Detect_List[DETECT_LIST_LENGTH].Overtime_Exit = 0;
    
    for(uint8_t i = 0;i<DETECT_LIST_LENGTH;i++)
    {
        Detect_List[i].dt_ms = (float)(USER_GetTick() - Detect_List[i].new_time);
        if(Detect_List[i].dt_ms > Detect_List[i].Overtime_ms)
        {
            if(!Detect_List[i].Overtime_Exit)
            {
                Detect_List[i].is_Lost = 1;
                Detect_List[i].Overtime_Exit = 1;
            }
            Detect_List[DETECT_LIST_LENGTH].is_Lost = 1;
            Detect_List[DETECT_LIST_LENGTH].Overtime_Exit = 1;
        }
        else
        {
            Detect_List[i].is_Lost = 0;
            Detect_List[i].Overtime_Exit = 0;
        }
    
    }
}

void Detect_Hook(uint8_t toe)
{
    Detect_List[toe].new_time = USER_GetTick();
    Detect_List[toe].is_Lost = 0;
}

uint8_t is_TOE_Overtime(uint8_t toe)
{
    return Detect_List[toe].Overtime_Exit;
}
