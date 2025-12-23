#include "bsp_uart.h"

#include "main.h"

#include <string.h>

static uint8_t Rx_Buf[255];
static uint8_t Tx_Buf[] = "hello world";
static uint8_t aaa[16] = {0};
static DMA_TypeDef *dma1 = DMA1; 

void Bsp_UART_Init(void)
{
    // while(HAL_UART_Transmit_DMA(&huart3, (uint8_t *)Tx_Buf, sizeof(Tx_Buf)/sizeof(Tx_Buf[0])) != HAL_OK)
    // {
    // }
    DMA1_Channel3->CCR = 0;

    DMA1_Channel3->CCR |= DMA_CCR_CIRC|DMA_CCR_MINC;
    DMA1_Channel3->CNDTR = sizeof(Rx_Buf)/sizeof(Rx_Buf[0]);
    DMA1_Channel3->CPAR = (uint32_t)&USART3->RDR;
    DMA1_Channel3->CMAR = (uint32_t)Rx_Buf;

    USART3->ICR |= USART_ICR_IDLECF;

    USART3->CR1 |= USART_CR1_IDLEIE;
    USART3->CR3 |= USART_CR3_DMAR;

    DMA1_Channel3->CCR |= DMA_CCR_EN;
    
    
//     DMA1_Channel6->CCR &= ~DMA_CCR_HTIE;
//     DMA1_Channel6->CCR |= DMA_CCR_EN;
}

void Bsp_USART_RxCallBack(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART3)
    {
        if(USART3->ISR & USART_ISR_IDLE)
        {

            USART3->ICR |= USART_ICR_IDLECF;
            DMA1_Channel3->CCR &= ~DMA_CCR_EN;
            // DMA1_Channel3->CNDTR = sizeof(Rx_Buf)/sizeof(Rx_Buf[0]);
            DMA1_Channel3->CCR |= DMA_CCR_EN;
            aaa[0]++;
        }
        
        // USART3->CR1
        // USART3->ICR |= USART_ICR_IDLECF;
    }
}
