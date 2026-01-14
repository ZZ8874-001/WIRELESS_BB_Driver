#include "bsp_uart.h"

#include "bb_control.h"
#include "detect_task.h"

#include <string.h>

USART_Tx_Buf_t Tx_Buf = {
    .duty1.interger = '0',
    .duty1.point = '.',
    .duty1.data = "00",
    .duty1.space = ' ',
    .duty2.interger = '0',
    .duty2.point = '.',
    .duty2.data = "00",
    .duty2.space = '\n',
};
USART_Rx_Buf_t Rx_Buf = {0};
static DMA_TypeDef *dma1 = DMA1; 

uint8_t Rx_data[255] = {0};

void Bsp_UART_Init(void)
{
    DMA1_Channel3->CCR = 0; 
    USART3->CR1 = 0;

    DMA1_Channel3->CCR |= DMA_CCR_MINC;
    DMA1_Channel3->CNDTR = sizeof(Rx_data)/sizeof(Rx_data[0]);
    DMA1_Channel3->CPAR = (uint32_t)&USART3->RDR;
    DMA1_Channel3->CMAR = (uint32_t)Rx_data;

    USART3->CR3 = 0;
    USART3->CR3 |= USART_CR3_DMAR;

    USART3->ICR |= USART_ICR_IDLECF;

    DMA1_Channel3->CCR |= DMA_CCR_EN;
    USART3->CR1 |= USART_CR1_IDLEIE | USART_CR1_RE | USART_CR1_UE;
}

void USER_USART_InterruptCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART3)
    {
        if(USART3->ISR & USART_ISR_IDLE)
        {
            DMA1_Channel3->CCR &= ~DMA_CCR_EN;
            DMA1_Channel3->CNDTR = sizeof(Rx_data)/sizeof(Rx_data[0]);

            WirelessRx_DataHandle(Rx_data);
            Detect_Hook(USART3_BUCKEN_TOE);

            USART3->ICR |= USART_ICR_IDLECF;
            DMA1_Channel3->CCR |= DMA_CCR_EN;
            
        }
    }
}


