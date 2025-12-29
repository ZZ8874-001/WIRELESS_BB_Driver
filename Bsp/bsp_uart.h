#ifndef __BSP_UART_H__
#define __BSP_UART_H__

#include "usart.h"


typedef struct{
    uint8_t interger;
    uint8_t point;
    uint8_t data[2];
    uint8_t space;
}Data_t;
typedef struct{
    Data_t duty1;
    Data_t duty2;

}USART_Tx_Buf_t;

typedef struct{
    uint8_t voltage_in[4];
    uint8_t space;
    uint8_t voltage_out[4];
}USART_Rx_Buf_t;

extern USART_Rx_Buf_t Rx_Buf;
extern USART_Tx_Buf_t Tx_Buf;

void Bsp_UART_Init(void);
// void Bsp_USART_RxCallBack(UART_HandleTypeDef *huart);

#endif