/**
  ******************************************************************************
  * @file    custom_ranging_sensor.h
  * @author  IMG SW Application Team
  * @brief   This file contains definitions for the BSP Ranging Sensors
  *          interface for custom boards
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
#ifndef __CUSTOM_RANGING_SENSOR_H__
#define __CUSTOM_RANGING_SENSOR_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "custom_tof_conf.h"
#include "ranging_sensor.h"
#include "vl53l4cd.h"

#define RANGING_SENSOR_VL53L4CD_DEFAULT_ADDRESS     (VL53L4CD_DEVICE_ADDRESS)
#define RANGING_SENSOR_NB_TARGET_PER_ZONE   (VL53L4CD_NB_TARGET_PER_ZONE)
#define RANGING_SENSOR_MAX_NB_ZONES         (VL53L4CD_MAX_NB_ZONES)

#define RS_MODE_BLOCKING_CONTINUOUS     (VL53L4CD_MODE_BLOCKING_CONTINUOUS)
#define RS_MODE_BLOCKING_ONESHOT        (VL53L4CD_MODE_BLOCKING_ONESHOT)
#define RS_MODE_ASYNC_CONTINUOUS        (VL53L4CD_MODE_ASYNC_CONTINUOUS)
#define RS_MODE_ASYNC_ONESHOT           (VL53L4CD_MODE_ASYNC_ONESHOT)

#define CUSTOM_RANGING_INSTANCES_NBR    (USE_CUSTOM_RANGING_VL53L4CD)

#define CUSTOM_RANGING_POWERMODE_ON   	(0U)  /*!< run */
#define CUSTOM_RANGING_POWERMODE_OFF  	(1U)  /*!< standby */

#if (CUSTOM_RANGING_INSTANCES_NBR == 0U)
#error "No ranging sensor instance has been selected"
#endif

typedef struct
{
  uint32_t NumberOfZones;
  uint32_t MaxNumberOfTargetsPerZone;
  uint32_t CustomROI;          /*!< Not available: 0, Available: 1 */
  uint32_t ThresholdDetection; /*!< Not available: 0, Available: 1 */
} RANGING_SENSOR_Capabilities_t;

typedef struct
{
  uint32_t RangingProfile;
  uint32_t TimingBudget;   /*!< Expressed in milliseconds */
  uint32_t Frequency;      /*!< Expressed in Hz */
  uint32_t EnableAmbient;  /*<! Enable: 1, Disable: 0 */
  uint32_t EnableSignal;   /*<! Enable: 1, Disable: 0 */
} RANGING_SENSOR_ProfileConfig_t;

typedef struct
{
  uint8_t TopLeftX;   /*!< Top Left x coordinate */
  uint8_t TopLeftY;   /*!< Top Left y coordinate */
  uint8_t BotRightX;  /*!< Bot Right x coordinate */
  uint8_t BotRightY;  /*!< Bot Right y coordinate */
} RANGING_SENSOR_ROIConfig_t;

typedef struct
{
  uint32_t Criteria;
  uint32_t LowThreshold;
  uint32_t HighThreshold;
} RANGING_SENSOR_ITConfig_t;

typedef struct
{
  uint32_t NumberOfTargets;
  uint32_t Distance[RANGING_SENSOR_NB_TARGET_PER_ZONE];  /*!< millimeters */
  uint32_t Status[RANGING_SENSOR_NB_TARGET_PER_ZONE];    /*!< OK: 0, NOK: !0 */
  float_t Ambient[RANGING_SENSOR_NB_TARGET_PER_ZONE];    /*!< kcps / spad */
  float_t Signal[RANGING_SENSOR_NB_TARGET_PER_ZONE];     /*!< kcps / spad */
} RANGING_SENSOR_ZoneResult_t;

typedef struct
{
  uint32_t NumberOfZones;
  RANGING_SENSOR_ZoneResult_t ZoneResult[RANGING_SENSOR_MAX_NB_ZONES];
} RANGING_SENSOR_Result_t;

typedef struct
{
  GPIO_TypeDef *GPIOx;
  uint16_t GPIO_Pin;
  uint32_t I2CAddress;
} RANGING_SENSOR_DeviceDiscriptor_t;

typedef enum _TOFInstance{
  TOF_INSTANCE_L = 0,
  TOF_INSTANCE_C    ,
  TOF_INSTANCE_R    ,
}TOFInstance;

typedef enum _TOFI2CAddress{
  TOF_I2C_ADDRESS_L = RANGING_SENSOR_VL53L4CD_DEFAULT_ADDRESS + 2,
  TOF_I2C_ADDRESS_C = RANGING_SENSOR_VL53L4CD_DEFAULT_ADDRESS + 4,
  TOF_I2C_ADDRESS_R = RANGING_SENSOR_VL53L4CD_DEFAULT_ADDRESS + 8,
}TOFI2CAddress;

extern VL53L4CD_Object_t CUSTOM_RANGING_CompObj[CUSTOM_RANGING_INSTANCES_NBR];
extern const RANGING_SENSOR_DeviceDiscriptor_t g_ToFDevice[USE_CUSTOM_RANGING_VL53L4CD];

int32_t CUSTOM_RANGING_SENSOR_Init(uint32_t Instance);
int32_t CUSTOM_RANGING_SENSOR_DeInit(uint32_t Instance);
int32_t CUSTOM_RANGING_SENSOR_ReadID(uint32_t Instance, uint32_t *pId);
int32_t CUSTOM_RANGING_SENSOR_GetCapabilities(uint32_t Instance, VL53L4CD_Capabilities_t *pCapabilities);
int32_t CUSTOM_RANGING_SENSOR_ConfigProfile(uint32_t Instance, VL53L4CD_ProfileConfig_t *pConfig);
int32_t CUSTOM_RANGING_SENSOR_ConfigROI(uint32_t Instance, VL53L4CD_ROIConfig_t *pConfig);
int32_t CUSTOM_RANGING_SENSOR_ConfigIT(uint32_t Instance, VL53L4CD_ITConfig_t *pConfig);
int32_t CUSTOM_RANGING_SENSOR_GetDistance(uint32_t Instance, VL53L4CD_Result_t *pResult);
int32_t CUSTOM_RANGING_SENSOR_Start(uint32_t Instance, uint8_t Mode);
int32_t CUSTOM_RANGING_SENSOR_Stop(uint32_t Instance);
int32_t CUSTOM_RANGING_SENSOR_SetAddress(uint32_t Instance, uint32_t Address);
int32_t CUSTOM_RANGING_SENSOR_GetAddress(uint32_t Instance, uint32_t *pAddress);
int32_t CUSTOM_RANGING_SENSOR_SetPowerMode(uint32_t Instance, uint32_t PowerMode);
int32_t CUSTOM_RANGING_SENSOR_GetPowerMode(uint32_t Instance, uint32_t *pPowerMode);

#ifdef __cplusplus
}
#endif

#endif /* __CUSTOM_RANGING_SENSOR_H__ */
