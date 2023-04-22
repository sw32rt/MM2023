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
#include "app_freertos.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "custom_ranging_sensor.h"
#include "sensor.h"
#include "stdio.h"

#include "sound.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticQueue_t osStaticMessageQDef_t;
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
extern void g_SoundTask(void *argument);
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

  /* Create the queue(s) */
  /* creation of soundQueue */
  soundQueueHandle = osMessageQueueNew (2, sizeof(scoreIndex), &soundQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of soundTask */
  soundTaskHandle = osThreadNew(g_SoundTask, NULL, &soundTask_attributes);

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
  VL53L4CD_Result_t result = {0};
  VL53L4CD_ProfileConfig_t tofConfig = {0};
  
  tofConfig.RangingProfile = VL53L4CD_PROFILE_CONTINUOUS;
  tofConfig.TimingBudget   = 10;
  tofConfig.Frequency      = 0;
  tofConfig.EnableAmbient  = 0;
  tofConfig.EnableSignal   = 0;

  HAL_GPIO_WritePin(SPI1_CS_ENC_L_GPIO_Port, SPI1_CS_ENC_L_Pin, SET);
  HAL_GPIO_WritePin(SPI1_CS_ENC_R_GPIO_Port, SPI1_CS_ENC_R_Pin, SET);
  HAL_GPIO_WritePin(SPI1_CS_IMU_GPIO_Port, SPI1_CS_IMU_Pin, SET);

  CUSTOM_RANGING_SENSOR_Init(TOF_INSTANCE_L);
  CUSTOM_RANGING_SENSOR_Init(TOF_INSTANCE_C);
  CUSTOM_RANGING_SENSOR_Init(TOF_INSTANCE_R);
  CUSTOM_RANGING_SENSOR_ConfigProfile(TOF_INSTANCE_L, &tofConfig);
  CUSTOM_RANGING_SENSOR_ConfigProfile(TOF_INSTANCE_C, &tofConfig);
  CUSTOM_RANGING_SENSOR_ConfigProfile(TOF_INSTANCE_R, &tofConfig);
  CUSTOM_RANGING_SENSOR_Start(TOF_INSTANCE_L, VL53L4CD_MODE_ASYNC_CONTINUOUS);
  CUSTOM_RANGING_SENSOR_Start(TOF_INSTANCE_C, VL53L4CD_MODE_ASYNC_CONTINUOUS);
  CUSTOM_RANGING_SENSOR_Start(TOF_INSTANCE_R, VL53L4CD_MODE_ASYNC_CONTINUOUS);
  osTimerStart(Timer1kHzHandle, 1); // 1ms Timer start

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  g_SoundInit();
  
  static uint8_t sendbuff[128] = {0};
  /* Infinite loop */
  for(;;)
  {
    CUSTOM_RANGING_SENSOR_GetDistance(TOF_INSTANCE_L, &result);
    g_sensor.range_l = result.ZoneResult[0].Distance[0];
    CUSTOM_RANGING_SENSOR_GetDistance(TOF_INSTANCE_C, &result);
    g_sensor.range_f = result.ZoneResult[0].Distance[0];
    CUSTOM_RANGING_SENSOR_GetDistance(TOF_INSTANCE_R, &result);
    g_sensor.range_r = result.ZoneResult[0].Distance[0];

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


    // HAL_UART_Transmit_DMA(&huart1, sendbuff, 100);

    // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 500);
    // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 500);
    osDelay(10);
    // HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);

    // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1300);
    // osDelay(1000);
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

  if(counter == 0)
  { /* I2C */

  }
  counter++; counter %= 10;
  /* USER CODE END Timer1kHzCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

