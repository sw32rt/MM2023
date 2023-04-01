/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.h
  * @brief   This file contains all the function prototypes for
  *          the spi.c file
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
#ifndef __SPI_H__
#define __SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN Private defines */

/** @defgroup SPI_LL_EC_PHASE Clock Phase
  * @{
  */
#define LL_SPI_PHASE_1EDGE                 0x00000000U               /*!< First clock transition is the first data capture edge  */
#define LL_SPI_PHASE_2EDGE                 (SPI_CR1_CPHA)            /*!< Second clock transition is the first data capture edge */
/**
  * @}
  */

/** @defgroup SPI_LL_EC_POLARITY Clock Polarity
  * @{
  */
#define LL_SPI_POLARITY_LOW                0x00000000U               /*!< Clock to 0 when idle */
#define LL_SPI_POLARITY_HIGH               (SPI_CR1_CPOL)            /*!< Clock to 1 when idle */
/**
  * @}
  */
typedef struct _SPIReadDescriptor
{
  GPIO_TypeDef *GPIOx;
  uint16_t GPIOPin;
  uint32_t CLKPolarity;
  uint32_t CLKPhase;
  uint8_t TxData[16];
  uint8_t RxData[16];
  uint8_t TxRxBytes;
}SPIReadDescriptor;


typedef enum _SPIReadOrder{
  SPIORDER_FIRST     = 0,
  SPIORDER_IMU_ACC_X = 0,
  SPIORDER_ENC_R,
  SPIORDER_ENC_L,
  SPIORDER_NUM,
}SPIReadOrder;

extern SPIReadDescriptor g_SPICommDevice[];

/* USER CODE END Private defines */

void MX_SPI1_Init(void);

/* USER CODE BEGIN Prototypes */

/**
  * @brief  Set clock phase
  * @note   This bit should not be changed when communication is ongoing.
  *         This bit is not used in SPI TI mode.
  * @rmtoll CR1          CPHA          LL_SPI_SetClockPhase
  * @param  SPIx SPI Instance
  * @param  ClockPhase This parameter can be one of the following values:
  *         @arg @ref LL_SPI_PHASE_1EDGE
  *         @arg @ref LL_SPI_PHASE_2EDGE
  * @retval None
  */
__STATIC_INLINE void LL_SPI_SetClockPhase(SPI_TypeDef *SPIx, uint32_t ClockPhase)
{
  MODIFY_REG(SPIx->CR1, SPI_CR1_CPHA, ClockPhase);
}

/**
  * @brief  Set clock polarity
  * @note   This bit should not be changed when communication is ongoing.
  *         This bit is not used in SPI TI mode.
  * @rmtoll CR1          CPOL          LL_SPI_SetClockPolarity
  * @param  SPIx SPI Instance
  * @param  ClockPolarity This parameter can be one of the following values:
  *         @arg @ref LL_SPI_POLARITY_LOW
  *         @arg @ref LL_SPI_POLARITY_HIGH
  * @retval None
  */
__STATIC_INLINE void LL_SPI_SetClockPolarity(SPI_TypeDef *SPIx, uint32_t ClockPolarity)
{
  MODIFY_REG(SPIx->CR1, SPI_CR1_CPOL, ClockPolarity);
}


/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H__ */

