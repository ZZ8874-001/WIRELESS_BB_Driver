#include "bsp_uart.h"

#include "main.h"

#include <string.h>

USART_Rx_Buf_t Rx_Buf = {0};
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
static uint8_t aaa[16] = {0};
static DMA_TypeDef *dma1 = DMA1; 

void Bsp_UART_Init(void)
{
    DMA1_Channel2->CCR = 0;
    DMA1_Channel2->CCR |= DMA_CCR_CIRC|DMA_CCR_MINC|DMA_CCR_DIR;
    DMA1_Channel2->CNDTR = sizeof(USART_Tx_Buf_t);
    DMA1_Channel2->CPAR = (uint32_t)&USART3->TDR;
    DMA1_Channel2->CMAR = (uint32_t)&Tx_Buf;

    DMA1_Channel3->CCR = 0; 
    DMA1_Channel3->CCR |= DMA_CCR_CIRC|DMA_CCR_MINC;
    DMA1_Channel3->CNDTR = sizeof(USART_Rx_Buf_t);
    DMA1_Channel3->CPAR = (uint32_t)&USART3->RDR;
    DMA1_Channel3->CMAR = (uint32_t)&Rx_Buf;

    USART3->CR1 = 0;
    USART3->CR1 |= USART_CR1_RE|USART_CR1_TE;
    USART3->CR1 |= USART_CR1_UE;

    USART3->CR3 = 0;
    USART3->CR3 |= USART_CR3_DMAR|USART_CR3_DMAT;

    DMA1_Channel2->CCR |= DMA_CCR_EN;
    DMA1_Channel3->CCR |= DMA_CCR_EN;
    
}
