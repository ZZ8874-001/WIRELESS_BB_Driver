#ifndef __BSP_UART_H__
#define __BSP_UART_H__

// #include "usart.h"
#include "main.h"

typedef struct{
    uint8_t interger;
    uint8_t point;
    uint8_t data[2];
    uint8_t space;
}Data_t;

typedef struct{
    uint8_t voltage_in[4];
    uint8_t space1;
    uint8_t voltage_out[4];
}USART_Rx_Buf_t;

extern USART_Rx_Buf_t Rx_Buf;

void Bsp_UART_Init(void);
void USER_USART_InterruptCallback(UART_HandleTypeDef *huart);

#endif