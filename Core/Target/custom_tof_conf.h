/**
 ******************************************************************************
 * @file    custom_tof_conf.h
 * @author  IMG SW Application Team
 * @brief   This file contains definitions of the TOF components bus interfaces
 *          for custom boards
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CUSTOM_TOF_CONF_H__
#define __CUSTOM_TOF_CONF_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"
#include "custom_bus.h"
#include "custom_errno.h"

/* USER CODE BEGIN 1 */
#include "main.h"
/* USER CODE END 1 */

#define USE_CUSTOM_RANGING_VL53L4CD (3U)

#define CUSTOM_VL53L4CD_I2C_Init      BSP_I2C2_Init
#define CUSTOM_VL53L4CD_I2C_DeInit    BSP_I2C2_DeInit
#define CUSTOM_VL53L4CD_I2C_WriteReg  BSP_I2C2_Send_DMA
#define CUSTOM_VL53L4CD_I2C_ReadReg   BSP_I2C2_Recv_DMA


#ifdef __cplusplus
}
#endif

#endif /* __CUSTOM_TOF_CONF_H__*/
