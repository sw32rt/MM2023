/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "app_freertos.h"
#include "stream_buffer.h"
#include <stdbool.h>
#include <string.h>
#include "common.h"

void MX_USART1_UART_Init(void);
void g_UartCommandTask(void *argument);
size_t uartCommandAnalysis(uint8_t* recvBuff, size_t recvSize, uint8_t* sendBuff);
void uartRecvStart(void);
void uartSendStart( void );
void uartSend(const uint8_t* const sendData, uint16_t sendSize);
static void DataPoolInit(dataPool_t* dataPool);
static void DataPoolResetIndex(dataPool_t* dataPool);
static uint16_t DataPoolRead(dataPool_t* dataPool, uint8_t* readBuff, uint16_t maxCopySize);
static bool DataPoolWrite(dataPool_t* dataPool, uint8_t* storeData, uint16_t size);

void usart1_isr(void);
void usart1DmaRx_isr(void);
void usart1DmaTx_isr(void);


/* DMAに渡すバッファは32byteアライメント */
__attribute__ ((aligned(32))) uint8_t g_uartDMARecvBuff[UART_DMA_RECV_BUFFER_SIZE] = {0};
__attribute__ ((aligned(32))) uint8_t g_uartDMASendBuff[UART_DMA_SEND_BUFFER_SIZE] = {0};

dataPool_t g_UartDataPool = {0};



/* USER CODE END 0 */

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  /**USART1 GPIO Configuration
  PB6   ------> USART1_TX
  PB7   ------> USART1_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USART1 DMA Init */

  /* USART1_RX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_5, LL_DMAMUX_REQ_USART1_RX);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_5, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_5, (uint32_t)&(USART1->RDR));

  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_5, (uint32_t)&g_uartDMARecvBuff);

  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_5, (uint32_t)sizeof(g_uartDMARecvBuff));

  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_5);
  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_5);


  /* USART1_TX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_6, LL_DMAMUX_REQ_USART1_TX);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_6, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_6, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_6, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_6, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_6, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_6, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_6, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_6, (uint32_t)&(USART1->TDR));

  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_6);
  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_6);


  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
  USART_InitStruct.BaudRate = 2000000;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_SetTXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_SetRXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_DisableFIFO(USART1);
  LL_USART_ConfigAsyncMode(USART1);

  /* 受信タイムアウト有効 */
  LL_USART_SetRxTimeout(USART1, 30);
  LL_USART_EnableRxTimeout(USART1);

  /* USER CODE BEGIN WKUPType USART1 */

  /* USER CODE END WKUPType USART1 */

  LL_USART_Enable(USART1);

  /* Polling USART1 initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(USART1))) || (!(LL_USART_IsActiveFlag_REACK(USART1))))
  {
  }
  /* USER CODE BEGIN USART1_Init 2 */
  /* 受信開始 */
  uartRecvStart();

  /* USER CODE END USART1_Init 2 */
}

/* USER CODE BEGIN 1 */

/**
  * @brief  uart command task
  * @param  argument : not used
  * @retval None
  */
void g_UartCommandTask(void *argument)
{
  static uint8_t recvBuff[UART_DMA_RECV_BUFFER_SIZE];
  static uint8_t sendBuff[UART_DMA_SEND_BUFFER_SIZE];
  size_t recvSize = 0;
  size_t sendSize = 0;

  while (1)
  {
    osSemaphoreAcquire(uartRecvCompleteSemaphoreHandle, osWaitForever);
    recvSize = DataPoolRead(&g_UartDataPool, recvBuff, sizeof(recvBuff));
    DataPoolResetIndex(&g_UartDataPool);

    /* 解析 */
    sendSize = uartCommandAnalysis(recvBuff, recvSize, sendBuff);
    if(sendSize > 0)
    {
      /* 応答 */
      uartSend(sendBuff, sendSize);
    }
  }
}

/**
  * @brief  uart command analysis
  * @param  recvBuff : 受信データ
  * @param  recvSize : 受信データサイズ
  * @param  sendBuff : 送信データ
  * @retval send data size
  */
size_t uartCommandAnalysis(uint8_t* recvBuff, size_t recvSize, uint8_t* sendBuff)
{
  size_t sendSize = 0;

  memcpy(sendBuff, recvBuff, recvSize);
  sendSize = recvSize;

  return sendSize;
}

void uartRecvStart(void)
{
    /* 受信タイムアウトフラグクリア */
    LL_USART_ClearFlag_RTO(USART1);
    /* DMA転送完了フラグクリア */
    LL_DMA_ClearFlag_TC5(DMA1);
    /* パリティエラー割り込み許可 */
    LL_USART_EnableIT_PE(USART1);
    /* エラー割り込み許可 */
    LL_USART_EnableIT_ERROR(USART1);
    /* 受信タイムアウト割り込み許可 */
    LL_USART_EnableIT_RTO(USART1);
    /* DMA送信エラー割り込み許可 */
    LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_5);
    /* DMA転送完了割り込み許可 */
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_5);
    /* 受信バッファポインタセット */
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_5, (uint32_t)&g_uartDMARecvBuff);
    /* 受信サイズセット */
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_5, (uint32_t)sizeof(g_uartDMARecvBuff));
    /* DMA 受信モード 有効化リクエスト */
    LL_USART_EnableDMAReq_RX(USART1);
    /* DMA Stream 有効化 */
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_5);
}

void uartSendStart(void)
{
    /* TCフラグをクリア */
    LL_USART_ClearFlag_TC(USART1);
    /* 送信完了割り込み許可 */
    LL_USART_EnableIT_TC(USART1);
    /* エラー割り込み許可 */
    LL_USART_EnableIT_ERROR(USART1);
    /* パリティエラー割り込み許可 */
    LL_USART_EnableIT_PE(USART1);
    /* Enable USART DMA TX Requsts */
    LL_USART_EnableDMAReq_TX(USART1);
    /* Enable DMA1 Stream3 UART Line A Tx */
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_6);
}

void uartSend(const uint8_t* const sendData, uint16_t sendSize)
{
    memcpy(g_uartDMASendBuff, sendData, sendSize);

    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_6, (uint32_t)g_uartDMASendBuff);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_6, (uint32_t)sendSize);

    /* 送信開始 */
    uartSendStart();
}

/*-------------------------------------------------------------------------------------------------------------------*/
/** 
 *@fn    void DataPoolInit(dataPool_t* dataPool)
 *@brief  受信データプール初期化
 *@param  dataPool_t* dataPool データプール構造体
 *@return なし
 */
/*-------------------------------------------------------------------------------------------------------------------*/
static void DataPoolInit(dataPool_t* dataPool)
{
    memset(dataPool->pool, 0, sizeof(dataPool->pool));
    dataPool->dataPoolWriteIndex = 0;
    dataPool->dataPoolReadIndex = 0;
}

/*-------------------------------------------------------------------------------------------------------------------*/
/** 
 *@fn    void DataPoolResetIndex(dataPool_t* dataPool)
 *@brief  受信データプール 読み書きOffset初期化
 *@param  dataPool_t* dataPool データプール構造体
 *@return なし
 */
/*-------------------------------------------------------------------------------------------------------------------*/
static void DataPoolResetIndex(dataPool_t* dataPool)
{ /* すべて読み終わったのでオフセットを0に戻す。リングバッファではないのでオフセットを戻さないとバッファオーバーランする */
    dataPool->dataPoolWriteIndex = 0;
    dataPool->dataPoolReadIndex = 0;
}


/*-------------------------------------------------------------------------------------------------------------------*/
/** 
 *@fn    uint16_t DataPoolRead(dataPool_t* dataPool, uint8_t* readBuff, uint16_t maxCopySize)
 *@brief  データプール 読み出し
 *@param  dataPool_t* dataPool : データプール構造体
 *@param  char* pchReadBuff : 読み込み先バッファ
 *@param  uint16_t ushMaxCopyLen : 読み込み先バッファへコピーする最大サイズ
 *@return 読み込みサイズ
 */
/*-------------------------------------------------------------------------------------------------------------------*/
static uint16_t DataPoolRead(dataPool_t* dataPool, uint8_t* readBuff, uint16_t maxCopySize)
{
    uint16_t readSize = 0;
    size_t copySize = 0;
    if(dataPool->dataPoolReadIndex == dataPool->dataPoolWriteIndex)
    { /* パケットなし */
        return 0;
    }

    /* ヘッダ読み出し */
    readSize = *((uint16_t*)&(dataPool->pool[ dataPool->dataPoolReadIndex ]));
    dataPool->dataPoolReadIndex += 2;

    /* バッファオーバーラン対策 */
    copySize = min(maxCopySize, readSize);
    /* データ読み出し */
    memcpy(readBuff, &(dataPool->pool[ dataPool->dataPoolReadIndex ]), copySize);

    /* コピーしたサイズに関わらずパケットの長さ文オフセットする。 */
    dataPool->dataPoolReadIndex += readSize;

    return readSize;
}

/*-------------------------------------------------------------------------------------------------------------------*/
/** 
 *@fn    static bool DataPoolWrite(dataPool_t* dataPool, uint8_t* storeData, uint16_t size)
 *@brief  データプール 書き込み
 *@param  dataPool_t* dataPool : データプール構造体
 *@param  char* pchStorePacket : 読み込み先バッファ
 *@param  uint16_t ushPacketLen : 読み込み先バッファへコピーする最大サイズ
 *@return 書き込み成功 : true, poolの空きサイズが少なくて書き込めなかった : false
 */
/*-------------------------------------------------------------------------------------------------------------------*/
static bool DataPoolWrite(dataPool_t* dataPool, uint8_t* storeData, uint16_t size)
{
    size_t poolFreeSpaceSize = 0;
    if(dataPool->dataPoolReadIndex == dataPool->dataPoolWriteIndex)
    {
        DataPoolResetIndex(dataPool);
    }

    poolFreeSpaceSize = (sizeof(dataPool->pool) - dataPool->dataPoolWriteIndex);
    if(poolFreeSpaceSize < (size + 2))
    {/* 空き容量が少なくてパケットが入りきらない */
        return false;
    }

    /* ヘッダ書き込み ヘッダは2Byte*/
    *((uint16_t*)&dataPool->pool[ dataPool->dataPoolWriteIndex ]) = size;
    dataPool->dataPoolWriteIndex += 2;
    /* データ書き込み */
    memcpy(&dataPool->pool[ dataPool->dataPoolWriteIndex ], storeData, size);
    dataPool->dataPoolWriteIndex += size;

    return true;
}

void usart1_isr(void)
{
  uint16_t recvSize = 0;
  /* 受信タイムアウト割り込みか */
  if((LL_USART_IsActiveFlag_RTO(USART1) != 0) && (LL_USART_IsEnabledIT_RTO(USART1) != 0))
  {
      LL_USART_DisableIT_PE(USART1);
      LL_USART_DisableIT_RXNE(USART1);
      LL_USART_DisableIT_ERROR(USART1);
      LL_USART_DisableIT_RTO(USART1);
      LL_USART_ClearFlag_RTO(USART1);

      /* Disable DMA UART Line A Rx */
      LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_5);

      recvSize = sizeof(g_uartDMARecvBuff) - LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_5);
          
      DataPoolWrite(&g_UartDataPool, g_uartDMARecvBuff, recvSize);

      /* 受信開始 */
      uartRecvStart();

      osSemaphoreRelease(uartRecvCompleteSemaphoreHandle);
  }
    /* 送信完了割り込みか */
  if (LL_USART_IsActiveFlag_TC(USART1) == SET)
  {
      /* フラグをクリア */
      LL_USART_ClearFlag_TC(USART1);

      /* 送信完了割り込み禁止 */
      LL_USART_DisableIT_TC(USART1);

      /* 送信完了通知 */
      osSemaphoreRelease(uartSendCompleteSemaphoreHandle);
  }

  if (LL_USART_IsActiveFlag_PE(USART1) == SET ||
      LL_USART_IsActiveFlag_FE(USART1) == SET ||
      LL_USART_IsActiveFlag_NE(USART1) == SET)
  {
      LL_USART_ClearFlag_PE(USART1);
      LL_USART_ClearFlag_FE(USART1);
      LL_USART_ClearFlag_NE(USART1);
  }
  
  if(LL_USART_IsActiveFlag_ORE(USART1) == SET)
  {
      LL_USART_ClearFlag_ORE(USART1);
  }
  
}

void usart1DmaRx_isr(void)
{
  uint16_t recvSize = 0;

  /* Test on DMA Stream Transfer Complete interrupt */
  if (LL_DMA_IsActiveFlag_TC5(DMA1) == SET)
  {
      /* Clear DMA Stream Transfer Complete interrupt pending bit */
      LL_DMA_ClearFlag_TC5(DMA1);
      LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_5);

      recvSize = sizeof(g_uartDMARecvBuff) - LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_5);
      
      DataPoolWrite(&g_UartDataPool, g_uartDMARecvBuff, recvSize);
      
      /* 受信開始 */
      uartRecvStart();

      osSemaphoreRelease(uartRecvCompleteSemaphoreHandle);
  }

  if (LL_DMA_IsActiveFlag_HT5(DMA1) == SET)
  {
      /* Clear DMA Stream half Transfer complete interrupt pending bit */
      LL_DMA_ClearFlag_HT5(DMA1);
  }
  if (LL_DMA_IsActiveFlag_TE5(DMA1) == SET)
  {
      /* Clear DMA Stream Transfer Error interrupt pending bit */
      LL_DMA_ClearFlag_TE5(DMA1);
  }

  if (LL_DMA_IsActiveFlag_GI5(DMA1) == SET)
  {
      /* Clear DMA Stream global interrupt pending bit */
      LL_DMA_ClearFlag_GI5(DMA1);
  }
}

void usart1DmaTx_isr(void)
{
  /* On DMA Stream Transfer Complete interrupt */
  if (LL_DMA_IsActiveFlag_TC6(DMA1) == SET)
  {
      /* Clear DMA Stream Transfer Complete interrupt pending bit */
      LL_DMA_ClearFlag_TC6(DMA1);
      LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_6);

      osSemaphoreRelease(uartSendCompleteSemaphoreHandle);
  }

  if (LL_DMA_IsActiveFlag_HT6(DMA1) == SET)
  {
      /* Clear DMA Stream half Transfer complete interrupt pending bit */
      LL_DMA_ClearFlag_HT6(DMA1);
  }

  if (LL_DMA_IsActiveFlag_TE6(DMA1) == SET)
  {
      /* Clear DMA Stream Transfer Error interrupt pending bit */
      LL_DMA_ClearFlag_TE6(DMA1);
  }

  if (LL_DMA_IsActiveFlag_GI6(DMA1) == SET)
  {
      /* Clear DMA Stream global interrupt pending bit */
      LL_DMA_ClearFlag_GI6(DMA1);
  }
}

/* USER CODE END 1 */
