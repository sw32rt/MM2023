/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stream_buffer.h"
#include "app_freertos.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "custom_ranging_sensor.h"
#include "sensor.h"
#include "tof.h"
#include "log.h"
#include "sound.h"

#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticQueue_t osStaticMessageQDef_t;
typedef StaticTimer_t osStaticTimerDef_t;
typedef StaticSemaphore_t osStaticSemaphoreDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_RECV_STREAM_BUFFER_SIZE_BYTES 1023
#define UART_SEND_STREAM_BUFFER_SIZE_BYTES 1023
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* Definitions for memoryPool */
StreamBufferHandle_t uartRecvStreamBufferHandle;
static uint8_t ucUartRecvStreamBufferStorage[ UART_RECV_STREAM_BUFFER_SIZE_BYTES + 1 ]; 
StaticStreamBuffer_t uartRecvStreamBufferStruct;
const size_t uartRecvTriggerLevel = 256; 

StreamBufferHandle_t uartSendStreamBufferHandle;
static uint8_t ucUartSendStreamBufferStorage[ UART_SEND_STREAM_BUFFER_SIZE_BYTES + 1 ]; 
StaticStreamBuffer_t uartSendStreamBufferStruct;
const size_t uartSendTriggerLevel = 256; 


/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 128 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for soundTask */
osThreadId_t soundTaskHandle;
uint32_t soundTaskBuffer[ 128 ];
osStaticThreadDef_t soundTaskControlBlock;
const osThreadAttr_t soundTask_attributes = {
  .name = "soundTask",
  .stack_mem = &soundTaskBuffer[0],
  .stack_size = sizeof(soundTaskBuffer),
  .cb_mem = &soundTaskControlBlock,
  .cb_size = sizeof(soundTaskControlBlock),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for tofRangingTask */
osThreadId_t tofRangingTaskHandle;
uint32_t tofRangingTaskBuffer[ 128 ];
osStaticThreadDef_t tofRangingTaskControlBlock;
const osThreadAttr_t tofRangingTask_attributes = {
  .name = "tofRangingTask",
  .stack_mem = &tofRangingTaskBuffer[0],
  .stack_size = sizeof(tofRangingTaskBuffer),
  .cb_mem = &tofRangingTaskControlBlock,
  .cb_size = sizeof(tofRangingTaskControlBlock),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for logTask */
osThreadId_t logTaskHandle;
uint32_t logTaskBuffer[ 128 ];
osStaticThreadDef_t logTaskControlBlock;
const osThreadAttr_t logTask_attributes = {
  .name = "logTask",
  .stack_mem = &logTaskBuffer[0],
  .stack_size = sizeof(logTaskBuffer),
  .cb_mem = &logTaskControlBlock,
  .cb_size = sizeof(logTaskControlBlock),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for uartRecvTask */
osThreadId_t uartRecvTaskHandle;
uint32_t uartRecvTaskBuffer[ 128 ];
osStaticThreadDef_t uartRecvTaskControlBlock;
const osThreadAttr_t uartRecvTask_attributes = {
  .name = "uartRecvTask",
  .stack_mem = &uartRecvTaskBuffer[0],
  .stack_size = sizeof(uartRecvTaskBuffer),
  .cb_mem = &uartRecvTaskControlBlock,
  .cb_size = sizeof(uartRecvTaskControlBlock),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for uartSendTask */
osThreadId_t uartSendTaskHandle;
uint32_t uartSendTaskBuffer[ 128 ];
osStaticThreadDef_t uartSendTaskControlBlock;
const osThreadAttr_t uartSendTask_attributes = {
  .name = "uartSendTask",
  .stack_mem = &uartSendTaskBuffer[0],
  .stack_size = sizeof(uartSendTaskBuffer),
  .cb_mem = &uartSendTaskControlBlock,
  .cb_size = sizeof(uartSendTaskControlBlock),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for uartCommandTask */
osThreadId_t uartCommandTaskHandle;
uint32_t uartCommandTaskBuffer[ 128 ];
osStaticThreadDef_t uartCommandTaskControlBlock;
const osThreadAttr_t uartCommandTask_attributes = {
  .name = "uartCommandTask",
  .stack_mem = &uartCommandTaskBuffer[0],
  .stack_size = sizeof(uartCommandTaskBuffer),
  .cb_mem = &uartCommandTaskControlBlock,
  .cb_size = sizeof(uartCommandTaskControlBlock),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for soundQueue */
osMessageQueueId_t soundQueueHandle;
uint8_t soundQueueBuffer[ 2 * sizeof( scoreIndex ) ];
osStaticMessageQDef_t soundQueueControlBlock;
const osMessageQueueAttr_t soundQueue_attributes = {
  .name = "soundQueue",
  .cb_mem = &soundQueueControlBlock,
  .cb_size = sizeof(soundQueueControlBlock),
  .mq_mem = &soundQueueBuffer,
  .mq_size = sizeof(soundQueueBuffer)
};
/* Definitions for logQueue */
osMessageQueueId_t logQueueHandle;
uint8_t logQueueBuffer[ 2 * sizeof( uint32_t* ) ];
osStaticMessageQDef_t logQueueControlBlock;
const osMessageQueueAttr_t logQueue_attributes = {
  .name = "logQueue",
  .cb_mem = &logQueueControlBlock,
  .cb_size = sizeof(logQueueControlBlock),
  .mq_mem = &logQueueBuffer,
  .mq_size = sizeof(logQueueBuffer)
};
/* Definitions for Timer1kHz */
osTimerId_t Timer1kHzHandle;
osStaticTimerDef_t Timer_1kHzControlBlock;
const osTimerAttr_t Timer1kHz_attributes = {
  .name = "Timer1kHz",
  .cb_mem = &Timer_1kHzControlBlock,
  .cb_size = sizeof(Timer_1kHzControlBlock),
};
/* Definitions for measurementTriggerSemaphore */
osSemaphoreId_t measurementTriggerSemaphoreHandle;
osStaticSemaphoreDef_t measurementTriggerSemaphoreControlBlock;
const osSemaphoreAttr_t measurementTriggerSemaphore_attributes = {
  .name = "measurementTriggerSemaphore",
  .cb_mem = &measurementTriggerSemaphoreControlBlock,
  .cb_size = sizeof(measurementTriggerSemaphoreControlBlock),
};
/* Definitions for i2cTxCompleteSemaphore */
osSemaphoreId_t i2cTxCompleteSemaphoreHandle;
osStaticSemaphoreDef_t i2cTxCompleteControlBlock;
const osSemaphoreAttr_t i2cTxCompleteSemaphore_attributes = {
  .name = "i2cTxCompleteSemaphore",
  .cb_mem = &i2cTxCompleteControlBlock,
  .cb_size = sizeof(i2cTxCompleteControlBlock),
};
/* Definitions for i2cRxCompleteSemaphore */
osSemaphoreId_t i2cRxCompleteSemaphoreHandle;
osStaticSemaphoreDef_t i2cRxCompleteSemaphoreControlBlock;
const osSemaphoreAttr_t i2cRxCompleteSemaphore_attributes = {
  .name = "i2cRxCompleteSemaphore",
  .cb_mem = &i2cRxCompleteSemaphoreControlBlock,
  .cb_size = sizeof(i2cRxCompleteSemaphoreControlBlock),
};
/* Definitions for logTriggerSemaphore */
osSemaphoreId_t logTriggerSemaphoreHandle;
osStaticSemaphoreDef_t logTriggerSemaphoreControlBlock;
const osSemaphoreAttr_t logTriggerSemaphore_attributes = {
  .name = "logTriggerSemaphore",
  .cb_mem = &logTriggerSemaphoreControlBlock,
  .cb_size = sizeof(logTriggerSemaphoreControlBlock),
};
/* Definitions for uartRecvCompleteSemaphore */
osSemaphoreId_t uartRecvCompleteSemaphoreHandle;
osStaticSemaphoreDef_t uartRecvCompleteSemaphoreControlBlock;
const osSemaphoreAttr_t uartRecvCompleteSemaphore_attributes = {
  .name = "uartRecvCompleteSemaphore",
  .cb_mem = &uartRecvCompleteSemaphoreControlBlock,
  .cb_size = sizeof(uartRecvCompleteSemaphoreControlBlock),
};
/* Definitions for uartSendCompleteSemaphore */
osSemaphoreId_t uartSendCompleteSemaphoreHandle;
osStaticSemaphoreDef_t uartSendCompleteSemaphoreControlBlock;
const osSemaphoreAttr_t uartSendCompleteSemaphore_attributes = {
  .name = "uartSendCompleteSemaphore",
  .cb_mem = &uartSendCompleteSemaphoreControlBlock,
  .cb_size = sizeof(uartSendCompleteSemaphoreControlBlock),
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
extern void g_SoundTask(void *argument);
extern void g_TofRangingTask(void *argument);
extern void g_LogTask(void *argument);
extern void g_UartRecvTask(void *argument);
extern void g_UartSendTask(void *argument);
extern void g_UartCommandTask(void *argument);
void Timer1kHzCallback(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  uartRecvStreamBufferHandle = xStreamBufferCreateStatic(UART_RECV_STREAM_BUFFER_SIZE_BYTES, uartRecvTriggerLevel, ucUartRecvStreamBufferStorage, &uartRecvStreamBufferStruct);

  uartSendStreamBufferHandle = xStreamBufferCreateStatic(UART_SEND_STREAM_BUFFER_SIZE_BYTES, uartSendTriggerLevel, ucUartSendStreamBufferStorage, &uartSendStreamBufferStruct);
  
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of measurementTriggerSemaphore */
  measurementTriggerSemaphoreHandle = osSemaphoreNew(1, 0, &measurementTriggerSemaphore_attributes);

  /* creation of i2cTxCompleteSemaphore */
  i2cTxCompleteSemaphoreHandle = osSemaphoreNew(1, 0, &i2cTxCompleteSemaphore_attributes);

  /* creation of i2cRxCompleteSemaphore */
  i2cRxCompleteSemaphoreHandle = osSemaphoreNew(1, 0, &i2cRxCompleteSemaphore_attributes);

  /* creation of logTriggerSemaphore */
  logTriggerSemaphoreHandle = osSemaphoreNew(1, 0, &logTriggerSemaphore_attributes);

  /* creation of uartRecvCompleteSemaphore */
  uartRecvCompleteSemaphoreHandle = osSemaphoreNew(1, 0, &uartRecvCompleteSemaphore_attributes);

  /* creation of uartSendCompleteSemaphore */
  uartSendCompleteSemaphoreHandle = osSemaphoreNew(1, 0, &uartSendCompleteSemaphore_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of Timer1kHz */
  Timer1kHzHandle = osTimerNew(Timer1kHzCallback, osTimerPeriodic, NULL, &Timer1kHz_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of soundQueue */
  soundQueueHandle = osMessageQueueNew (2, sizeof(scoreIndex), &soundQueue_attributes);

  /* creation of logQueue */
  logQueueHandle = osMessageQueueNew (2, sizeof(uint32_t*), &logQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of soundTask */
  soundTaskHandle = osThreadNew(g_SoundTask, NULL, &soundTask_attributes);

  /* creation of tofRangingTask */
  tofRangingTaskHandle = osThreadNew(g_TofRangingTask, NULL, &tofRangingTask_attributes);

  /* creation of logTask */
  logTaskHandle = osThreadNew(g_LogTask, NULL, &logTask_attributes);

  /* creation of uartRecvTask */
  uartRecvTaskHandle = osThreadNew(g_UartRecvTask, NULL, &uartRecvTask_attributes);

  /* creation of uartSendTask */
  uartSendTaskHandle = osThreadNew(g_UartSendTask, NULL, &uartSendTask_attributes);

  /* creation of uartCommandTask */
  uartCommandTaskHandle = osThreadNew(g_UartCommandTask, NULL, &uartCommandTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */

  HAL_GPIO_WritePin(SPI1_CS_ENC_L_GPIO_Port, SPI1_CS_ENC_L_Pin, SET);
  HAL_GPIO_WritePin(SPI1_CS_ENC_R_GPIO_Port, SPI1_CS_ENC_R_Pin, SET);
  HAL_GPIO_WritePin(SPI1_CS_IMU_GPIO_Port, SPI1_CS_IMU_Pin, SET);

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  g_SoundInit();

  osTimerStart(Timer1kHzHandle, 1); // 1ms Timer start
  
  static uint8_t sendbuff[128] = {0};

  osDelay(1000);
  eraseFlash();
  osThreadResume(logTaskHandle);
  HAL_GPIO_WritePin(LED_0_GPIO_Port, LED_0_Pin, 1);


  /* Infinite loop */
  for(;;)
  {

    // sprintf(sendbuff, "ax:%d,ay:%d,az:%d,gx:%d,gy:%d,gz:%d,er:%d,el:%d\n",
    //   g_sensor.acc_x ,
    //   g_sensor.acc_y ,
    //   g_sensor.acc_z ,
    //   g_sensor.gyro_x,
    //   g_sensor.gyro_y,
    //   g_sensor.gyro_z,
    //   g_sensor.enc_r ,
    //   g_sensor.enc_l 
    // );
//    sprintf(sendbuff, "L:%04d,C:%04d,R:%04d\n",
//      g_sensor.range_l ,
//      g_sensor.range_f ,
//      g_sensor.range_r
//    );


    HAL_UART_Transmit_DMA(&huart1, sendbuff, 100);

    // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 500);
    // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 500);
    // HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);

    // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1300);
    osDelay(1000);
    // HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    // HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
  }
  /* USER CODE END StartDefaultTask */
}

/* Timer1kHzCallback function */
void Timer1kHzCallback(void *argument)
{
  /* USER CODE BEGIN Timer1kHzCallback */
  static int counter = 0;
  SPIReadDescriptor* device = &g_SPICommDevice[SPIORDER_FIRST];

  __HAL_SPI_DISABLE(&hspi1);
  LL_SPI_SetClockPolarity(hspi1.Instance, device->CLKPolarity);
  LL_SPI_SetClockPhase(hspi1.Instance, device->CLKPhase);
  __HAL_SPI_ENABLE(&hspi1);

  HAL_GPIO_WritePin(device->GPIOx, device->GPIOPin, RESET);
  HAL_SPI_TransmitReceive_DMA(&hspi1, device->TxData, device->RxData, device->TxRxBytes);
  if(READ_BIT(hspi1.ErrorCode, HAL_SPI_ERROR_DMA))
  {
    hspi1.hdmatx->Lock = HAL_UNLOCKED;
    hspi1.hdmatx->State = HAL_DMA_STATE_READY;
    hspi1.hdmarx->Lock = HAL_UNLOCKED;
    hspi1.hdmarx->State = HAL_DMA_STATE_READY;
    HAL_SPI_TransmitReceive_DMA(&hspi1, device->TxData, device->RxData, device->TxRxBytes);

  }

  if(counter == 0)
  { /* I2C */
    osSemaphoreRelease(measurementTriggerSemaphoreHandle);
  }
  counter++; counter %= 10;
  /* USER CODE END Timer1kHzCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

