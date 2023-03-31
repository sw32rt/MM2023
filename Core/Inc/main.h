/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_1_Pin GPIO_PIN_1
#define LED_1_GPIO_Port GPIOF
#define LED_2_Pin GPIO_PIN_10
#define LED_2_GPIO_Port GPIOG
#define TIM2_PWM_R_Pin GPIO_PIN_0
#define TIM2_PWM_R_GPIO_Port GPIOA
#define TIM2_PWM_L_Pin GPIO_PIN_1
#define TIM2_PWM_L_GPIO_Port GPIOA
#define PHASE_L_Pin GPIO_PIN_2
#define PHASE_L_GPIO_Port GPIOA
#define PHASE_R_Pin GPIO_PIN_3
#define PHASE_R_GPIO_Port GPIOA
#define LED_0_Pin GPIO_PIN_4
#define LED_0_GPIO_Port GPIOA
#define ADC1_IN15_VBATT_Pin GPIO_PIN_0
#define ADC1_IN15_VBATT_GPIO_Port GPIOB
#define TOF_R_XSHUT_Pin GPIO_PIN_10
#define TOF_R_XSHUT_GPIO_Port GPIOA
#define TOF_C_XSHUT_Pin GPIO_PIN_11
#define TOF_C_XSHUT_GPIO_Port GPIOA
#define TOF_L_XSHUT_Pin GPIO_PIN_12
#define TOF_L_XSHUT_GPIO_Port GPIOA
#define SPI1_CS_IMU_Pin GPIO_PIN_15
#define SPI1_CS_IMU_GPIO_Port GPIOA
#define SPI1_CS_ENC_L_Pin GPIO_PIN_4
#define SPI1_CS_ENC_L_GPIO_Port GPIOB
#define SPI1_CS_ENC_R_Pin GPIO_PIN_5
#define SPI1_CS_ENC_R_GPIO_Port GPIOB
#define USART1_TX_DEBUG_Pin GPIO_PIN_6
#define USART1_TX_DEBUG_GPIO_Port GPIOB
#define USART1_RX_DEBUG_Pin GPIO_PIN_7
#define USART1_RX_DEBUG_GPIO_Port GPIOB
#define TIM8_CH2_SPEAKER_Pin GPIO_PIN_8
#define TIM8_CH2_SPEAKER_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
typedef struct _GPIODescriptorTypeDef
{
  GPIO_TypeDef *GPIOx;
  uint16_t GPIO_Pin;
}GPIODescriptorTypeDef;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
