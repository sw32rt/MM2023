/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

#define PACKET_DATA_SIZE 1024
#define UART_DMA_RECV_BUFFER_SIZE 1024
#define UART_DMA_SEND_BUFFER_SIZE 1024

typedef struct {
  uint8_t data[PACKET_DATA_SIZE];
  uint16_t size;
}packet_t;

typedef struct
{
    unsigned short dataPoolWriteIndex;
    unsigned short dataPoolReadIndex;
    unsigned char pool[UART_DMA_RECV_BUFFER_SIZE + 2]; /* +2はヘッダサイズ */
}dataPool_t;

extern dataPool_t g_UartDataPool;

/* USER CODE END Private defines */

extern uint8_t g_uartDMARecvBuff[UART_DMA_RECV_BUFFER_SIZE];
extern uint8_t g_uartDMASendBuff[UART_DMA_SEND_BUFFER_SIZE];


void MX_USART1_UART_Init(void);

/* USER CODE BEGIN Prototypes */
void uartRecvStart(void);
void uartSendStart(void);
void uartSend(const uint8_t* const sendData, uint16_t sendSize);
void usart1_isr(void);
void usart1DmaRx_isr(void);
void usart1DmaTx_isr(void);


/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

