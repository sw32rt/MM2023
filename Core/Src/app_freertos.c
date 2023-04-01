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
#include "spi.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticTimer_t osStaticTimerDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
    static volatile uint16_t g_getData = 0;

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
/* Definitions for Timer1kHz */
osTimerId_t Timer1kHzHandle;
osStaticTimerDef_t Timer_1kHzControlBlock;
const osTimerAttr_t Timer1kHz_attributes = {
  .name = "Timer1kHz",
  .cb_mem = &Timer_1kHzControlBlock,
  .cb_size = sizeof(Timer_1kHzControlBlock),
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void Timer1kHzCallback(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of Timer1kHz */
  Timer1kHzHandle = osTimerNew(Timer1kHzCallback, osTimerPeriodic, NULL, &Timer1kHz_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

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
  /* Infinite loop */
    HAL_GPIO_WritePin(SPI1_CS_ENC_L_GPIO_Port, SPI1_CS_ENC_L_Pin, SET);
    HAL_GPIO_WritePin(SPI1_CS_ENC_R_GPIO_Port, SPI1_CS_ENC_R_Pin, SET);
    HAL_GPIO_WritePin(SPI1_CS_IMU_GPIO_Port, SPI1_CS_IMU_Pin, SET);
    uint8_t tx_data[4] = {0};
    uint8_t rx_data[4] = {0};
  
  osTimerStart(Timer1kHzHandle, 1); // 1ms Timer start

  // tx_data[0] = 117 | 0x80;
  // tx_data[1] = 0x00;  // dummy
  // osDelay(100);
  // HAL_GPIO_WritePin(SPI1_CS_IMU_GPIO_Port, SPI1_CS_IMU_Pin, RESET);
  // HAL_SPI_TransmitReceive_DMA(&hspi1, tx_data, rx_data, 2);
  // osDelay(1);

  for(;;)
  {
    osDelay(100);
    // HAL_GPIO_WritePin(SPI1_CS_IMU_GPIO_Port, SPI1_CS_IMU_Pin, SET);
#if 0
    tx_data[0] = 0x3F | 0x40;
    tx_data[1] = 0xFF;
    osDelay(100);
    HAL_GPIO_WritePin(SPI1_CS_ENC_L_GPIO_Port, SPI1_CS_ENC_L_Pin, RESET);
    HAL_SPI_TransmitReceive_DMA(&hspi1, tx_data, rx_data, 2);
    osDelay(1);
    HAL_GPIO_WritePin(SPI1_CS_ENC_L_GPIO_Port, SPI1_CS_ENC_L_Pin, SET);
  g_getData = (rx_data[1] | rx_data[0] << 8) & 0b0011111111111111;
  (void)g_getData;
#endif
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
  __HAL_DMA_DISABLE_IT(hspi1.hdmarx, DMA_IT_HT);
  //g_getData = (g_rx_data[1] | g_rx_data[0] << 8) & 0b0011111111111111;
  (void)g_getData;


  if(counter == 0)
  { /* I2C */

  }
  counter++; counter %= 10;
  /* USER CODE END Timer1kHzCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

