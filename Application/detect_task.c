/**
 ******************************************************************************
 * @file    detect task.h
 * @author  Wang Hongxi
 * @version V1.0.0
 * @date    2020/1/30
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "detect_task.h"
#include "stddef.h"

uint32_t resetCount = 0;

Detect_t Detect_List[DETECT_LIST_LENGHT + 1];
BoardState_t BoardState;

static void Detect_Init(uint32_t time);

void Detect_Task_Init(void)
{
    static uint32_t system_time;
    system_time = USER_GetTick();

    Detect_Init(system_time);

    USER_Delay_ms(60);
}

void Detect_Task(void)
{
    static uint32_t system_time;
    static uint8_t error_num_display = 0;
    system_time = USER_GetTick();

    error_num_display = DETECT_LIST_LENGHT;
    Detect_List[DETECT_LIST_LENGHT].is_Lost = 0;
    Detect_List[DETECT_LIST_LENGHT].Error_Exist = 0;

    for (int i = 0; i < DETECT_LIST_LENGHT; i++)
    {
        if (Detect_List[i].Enable == 0)
        {
            continue;
        }

        // 掉线判断
        if (system_time - Detect_List[i].New_Time > Detect_List[i].Offline_Time)
        {
            if (Detect_List[i].Error_Exist == 0)
            {
                // 记录错误以及掉线时间
                Detect_List[i].is_Lost = 1;
                Detect_List[i].Error_Exist = 1;
                Detect_List[i].Lost_Time = system_time;
            }

            // 保存优先级最高的错误码
            if (Detect_List[i].Priority > Detect_List[error_num_display].Priority)
            {
                error_num_display = i;
            }

            Detect_List[DETECT_LIST_LENGHT].is_Lost = 1;
            Detect_List[DETECT_LIST_LENGHT].Error_Exist = 1;

            // 如果提供解决函数，运行解决函数
            if (Detect_List[i].f_Solve_Lost != NULL)
            {
                Detect_List[i].f_Solve_Lost();
            }
        }
        else if (system_time - Detect_List[i].Work_Time < Detect_List[i].Online_Time)
        {
            // 刚刚上线，可能存在数据不稳定，只记录不丢失，
            Detect_List[i].is_Lost = 0;
            Detect_List[i].Error_Exist = 1;
        }
        else
        {
            Detect_List[i].is_Lost = 0;
            // 判断是否存在数据错误
            if (Detect_List[i].is_Data_Error != NULL)
            {
                Detect_List[i].Error_Exist = 1;
            }
            else
            {
                Detect_List[i].Error_Exist = 0;
            }

            // 计算频率
            if (Detect_List[i].New_Time > Detect_List[i].Last_Time)
            {
                Detect_List[i].frequency_d = 1000.0f / (float)(Detect_List[i].New_Time - Detect_List[i].Last_Time);
            }
        }
    }
}

uint8_t is_TOE_Error(uint8_t toe)
{
    return (Detect_List[toe].Error_Exist == 1);
}

void Detect_Hook(uint8_t toe)
{
    Detect_List[toe].Last_Time = Detect_List[toe].New_Time; // 更新上一次记录时间
    Detect_List[toe].New_Time = USER_GetTick();             // 记录本次时间

    // 进入此函数证明没有掉线
    if (Detect_List[toe].is_Lost)
    {
        Detect_List[toe].is_Lost = 0;
        Detect_List[toe].Work_Time = Detect_List[toe].New_Time;
    }

    if (Detect_List[toe].f_is_Data_Error != NULL)
    {
        if (Detect_List[toe].f_is_Data_Error())
        {
            Detect_List[toe].Error_Exist = 1;
            Detect_List[toe].is_Data_Error = 1;

            if (Detect_List[toe].f_Solve_Data_Error != NULL)
            {
                Detect_List[toe].f_Solve_Data_Error();
            }
        }
        else
        {
            Detect_List[toe].is_Data_Error = 0;
        }
    }
    else
    {
        Detect_List[toe].is_Data_Error = 0;
    }
}

static void Detect_Init(uint32_t time)
{
    // 设置离线时间，上线稳定工作时间，优先级 offlineTime onlinetime Priority
    uint16_t set_item[DETECT_LIST_LENGHT][3] =
        {
            {250, 100, 15},
        };

    for (uint8_t i = 0; i < DETECT_LIST_LENGHT; i++)
    {
        Detect_List[i].Offline_Time = set_item[i][0];
        Detect_List[i].Online_Time = set_item[i][1];
        Detect_List[i].Priority = set_item[i][2];
        Detect_List[i].f_is_Data_Error = NULL;
        Detect_List[i].f_Solve_Lost = NULL;
        Detect_List[i].f_Solve_Data_Error = NULL;

        Detect_List[i].Enable = 1;
        Detect_List[i].Error_Exist = 1;
        Detect_List[i].is_Lost = 1;
        Detect_List[i].is_Data_Error = 1;
        Detect_List[i].frequency_d = 0.0f;
        Detect_List[i].New_Time = time;
        Detect_List[i].Last_Time = time;
        Detect_List[i].Lost_Time = time;
        Detect_List[i].Work_Time = time;
    }
}
