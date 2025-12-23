#ifndef __BSP_UART_H__
#define __BSP_UART_H__

#include "usart.h"

void Bsp_UART_Init(void);
// void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void Bsp_USART_RxCallBack(UART_HandleTypeDef *huart);

#endif