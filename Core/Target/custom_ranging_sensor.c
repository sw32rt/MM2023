/**
  ******************************************************************************
  * @file    custom_ranging_sensor.c
  * @author  IMG SW Application Team
  * @brief   This file provides BSP Ranging Sensors interface for custom boards
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

/* Includes ------------------------------------------------------------------*/
#include "custom_ranging_sensor.h"

VL53L4CD_Object_t CUSTOM_RANGING_CompObj[CUSTOM_RANGING_INSTANCES_NBR] = {0};
static VL53L4CD_Capabilities_t RANGING_SENSOR_Cap[CUSTOM_RANGING_INSTANCES_NBR];
const RANGING_SENSOR_DeviceDiscriptor_t g_ToFDevice[USE_CUSTOM_RANGING_VL53L4CD] = 
{
  [TOF_INSTANCE_L] = {.GPIOx = TOF_L_XSHUT_GPIO_Port, .GPIO_Pin = TOF_L_XSHUT_Pin, .I2CAddress = TOF_I2C_ADDRESS_L},
  [TOF_INSTANCE_C] = {.GPIOx = TOF_C_XSHUT_GPIO_Port, .GPIO_Pin = TOF_C_XSHUT_Pin, .I2CAddress = TOF_I2C_ADDRESS_C},
  [TOF_INSTANCE_R] = {.GPIOx = TOF_R_XSHUT_GPIO_Port, .GPIO_Pin = TOF_R_XSHUT_Pin, .I2CAddress = TOF_I2C_ADDRESS_R},
};


static int32_t VL53L4CD_Probe(uint32_t Instance);

/**
  * @brief Initializes the ranging sensor.
  * @param Instance    Ranging sensor instance.
  * @retval BSP status
  */
int32_t CUSTOM_RANGING_SENSOR_Init(uint32_t Instance)
{
  int32_t ret;

  if (Instance >= CUSTOM_RANGING_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else
  {
    CUSTOM_RANGING_SENSOR_SetPowerMode(Instance, CUSTOM_RANGING_POWERMODE_OFF);
    CUSTOM_RANGING_SENSOR_SetPowerMode(Instance, CUSTOM_RANGING_POWERMODE_ON);

    VL53L4CD_Probe(Instance);
    CUSTOM_RANGING_SENSOR_SetAddress(Instance, g_ToFDevice[Instance].I2CAddress);

    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/**
  * @brief Deinitializes the ranging sensor.
  * @param Instance    Ranging sensor instance.
  * @retval BSP status
  */
int32_t CUSTOM_RANGING_SENSOR_DeInit(uint32_t Instance)
{
  int32_t ret;

  if(Instance >= CUSTOM_RANGING_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if (VL53L4CD_RANGING_SENSOR_Driver.DeInit(&CUSTOM_RANGING_CompObj[Instance]) < 0)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/**
  * @brief Read the ranging sensor device ID.
  * @param Instance    Ranging sensor instance.
  * @param pId    Pointer to the device ID.
  * @retval BSP status
  */
int32_t CUSTOM_RANGING_SENSOR_ReadID(uint32_t Instance, uint32_t *pId)
{
  int32_t ret;

  if (Instance >= CUSTOM_RANGING_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if (VL53L4CD_RANGING_SENSOR_Driver.ReadID(&CUSTOM_RANGING_CompObj[Instance], pId) < 0)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/**
  * @brief Get the ranging sensor capabilities.
  * @param Instance    Ranging sensor instance.
  * @param pCapabilities    Pointer to the ranging sensor capabilities.
  * @note This function should be called after the init.
  * @retval BSP status
  */
int32_t CUSTOM_RANGING_SENSOR_GetCapabilities(uint32_t Instance, VL53L4CD_Capabilities_t *pCapabilities)
{
    int32_t ret;

    if (Instance >= CUSTOM_RANGING_INSTANCES_NBR)
    {
      ret = BSP_ERROR_WRONG_PARAM;
    }
    else if (VL53L4CD_RANGING_SENSOR_Driver.GetCapabilities(&CUSTOM_RANGING_CompObj[Instance], pCapabilities) < 0)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    else
    {
      ret = BSP_ERROR_NONE;
    }

    return ret;
}

/**
  * @brief Set the ranging configuration profile.
  * @param Instance    Ranging sensor instance.
  * @param pConfig    Pointer to the new configuration profile to be applied.
  * @retval BSP status
  */
int32_t CUSTOM_RANGING_SENSOR_ConfigProfile(uint32_t Instance, VL53L4CD_ProfileConfig_t *pConfig)
{
  int32_t ret;

  if (Instance >= CUSTOM_RANGING_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if (VL53L4CD_RANGING_SENSOR_Driver.ConfigProfile(&CUSTOM_RANGING_CompObj[Instance], pConfig) < 0)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/**
  * @brief Configure the Region of Interest of the ranging sensor.
  * @param Instance    Ranging sensor instance.
  * @param pConfig    Pointer to the ROI configuration struct.
  * @note Should be called only if the device supports CustomROI.
  * @retval BSP status
  */
int32_t CUSTOM_RANGING_SENSOR_ConfigROI(uint32_t Instance, VL53L4CD_ROIConfig_t *pConfig)
{
  int32_t ret;

  if (Instance >= CUSTOM_RANGING_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if (RANGING_SENSOR_Cap[Instance].CustomROI == 0)
  {
    ret = BSP_ERROR_FEATURE_NOT_SUPPORTED;
  }
  else if (VL53L4CD_RANGING_SENSOR_Driver.ConfigROI(&CUSTOM_RANGING_CompObj[Instance], pConfig) < 0)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/**
  * @brief Configure the IT event generation parameters.
  * @param Instance    Ranging sensor instance.
  * @param pConfig    Pointer to the IT configuration struct.
  * @note The threshold modes can be used only if supported by the device (check the capabilities)
  * @retval BSP status
  */
int32_t CUSTOM_RANGING_SENSOR_ConfigIT(uint32_t Instance, VL53L4CD_ITConfig_t *pConfig)
{
  int32_t ret;

  if (Instance >= CUSTOM_RANGING_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if (VL53L4CD_RANGING_SENSOR_Driver.ConfigIT(&CUSTOM_RANGING_CompObj[Instance], pConfig) < 0)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/**
  * @brief Get the last distance measurement information.
  * @param Instance    Ranging sensor instance.
  * @param pResult    Pointer to the result struct.
  * @retval BSP status
  */
int32_t CUSTOM_RANGING_SENSOR_GetDistance(uint32_t Instance, VL53L4CD_Result_t *pResult)
{
  int32_t ret;

  if (Instance >= CUSTOM_RANGING_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if (VL53L4CD_RANGING_SENSOR_Driver.GetDistance(&CUSTOM_RANGING_CompObj[Instance], pResult) < 0)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/**
  * @brief Start ranging.
  * @param Instance    Ranging sensor instance.
  * @param Mode        The desired ranging mode.
  * @retval BSP status
  */
int32_t CUSTOM_RANGING_SENSOR_Start(uint32_t Instance, uint8_t Mode)
{
  int32_t ret;

  if (Instance >= CUSTOM_RANGING_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if (VL53L4CD_RANGING_SENSOR_Driver.Start(&CUSTOM_RANGING_CompObj[Instance], Mode) < 0)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/**
  * @brief Stop ranging.
  * @param Instance    Ranging sensor instance.
  * @retval BSP status
  */
int32_t CUSTOM_RANGING_SENSOR_Stop(uint32_t Instance)
{
  int32_t ret;

  if (Instance >= CUSTOM_RANGING_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if (VL53L4CD_RANGING_SENSOR_Driver.Stop(&CUSTOM_RANGING_CompObj[Instance]) < 0)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/**
  * @brief Set The I2C address of the device.
  * @param Instance    Ranging sensor instance.
  * @param Address     New I2C address.
  * @retval BSP status
  */
int32_t CUSTOM_RANGING_SENSOR_SetAddress(uint32_t Instance, uint32_t Address)
{
  int32_t ret;

  if (Instance >= CUSTOM_RANGING_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if (VL53L4CD_RANGING_SENSOR_Driver.SetAddress(&CUSTOM_RANGING_CompObj[Instance], Address) < 0)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/**
  * @brief Get the I2C address of the device.
  * @param Instance    Ranging sensor instance.
  * @param pAddress    Pointer to the current I2C address.
  * @retval BSP status
  */
int32_t CUSTOM_RANGING_SENSOR_GetAddress(uint32_t Instance, uint32_t *pAddress)
{
  int32_t ret;

  if (Instance >= CUSTOM_RANGING_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if (VL53L4CD_RANGING_SENSOR_Driver.GetAddress(&CUSTOM_RANGING_CompObj[Instance], pAddress) < 0)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/**
  * @brief Set the power mode.
  * @param Instance    Ranging sensor instance.
  * @param PowerMode    New power mode to be entered.
  * @retval BSP status
  */
int32_t CUSTOM_RANGING_SENSOR_SetPowerMode(uint32_t Instance, uint32_t PowerMode)
{
  GPIO_PinState state;

  if (PowerMode == CUSTOM_RANGING_POWERMODE_ON)
  {
    state = GPIO_PIN_SET;
  }
  else if (PowerMode == CUSTOM_RANGING_POWERMODE_OFF)
  {
    state = GPIO_PIN_RESET;
  }
  else
  {
    return BSP_ERROR_WRONG_PARAM;
  }

  HAL_GPIO_WritePin(g_ToFDevice[Instance].GPIOx, g_ToFDevice[Instance].GPIO_Pin, state);
  HAL_Delay(2);

  return BSP_ERROR_NONE;
}

/**
  * @brief Set the power mode.
  * @param Instance    Ranging sensor instance.
  * @param pPowerMode    Pointer to the current power mode.
  * @retval BSP status
  */
int32_t CUSTOM_RANGING_SENSOR_GetPowerMode(uint32_t Instance, uint32_t *pPowerMode)
{
  int32_t ret;

  if (Instance >= CUSTOM_RANGING_INSTANCES_NBR)
  {
    ret = BSP_ERROR_WRONG_PARAM;
  }
  else if (VL53L4CD_RANGING_SENSOR_Driver.GetPowerMode(&CUSTOM_RANGING_CompObj[Instance], pPowerMode) < 0)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    ret = BSP_ERROR_NONE;
  }

  return ret;
}

/**
  * @brief Register Bus IOs if component ID is OK.
  * @param Instance    Ranging sensor instance.
  * @retval BSP status
  */
static int32_t VL53L4CD_Probe(uint32_t Instance)
{
  int32_t ret;
  VL53L4CD_IO_t              IOCtx;

  /* Configure the ranging sensor driver */
  IOCtx.Address     = RANGING_SENSOR_VL53L4CD_DEFAULT_ADDRESS;
  IOCtx.Init        = CUSTOM_VL53L4CD_I2C_Init;
  IOCtx.DeInit      = CUSTOM_VL53L4CD_I2C_DeInit;
  IOCtx.WriteReg    = CUSTOM_VL53L4CD_I2C_WriteReg;
  IOCtx.ReadReg     = CUSTOM_VL53L4CD_I2C_ReadReg;
  IOCtx.GetTick     = BSP_GetTick;

  if (VL53L4CD_RegisterBusIO(&CUSTOM_RANGING_CompObj[Instance], &IOCtx) != VL53L4CD_OK)
  {
    ret = BSP_ERROR_COMPONENT_FAILURE;
  }
  else
  {
    if (VL53L4CD_RANGING_SENSOR_Driver.Init(&CUSTOM_RANGING_CompObj[Instance]) != VL53L4CD_OK)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    else if (VL53L4CD_RANGING_SENSOR_Driver.GetCapabilities(&CUSTOM_RANGING_CompObj[Instance], &RANGING_SENSOR_Cap[Instance]) != VL53L4CD_OK)
    {
      ret = BSP_ERROR_COMPONENT_FAILURE;
    }
    else
    {
      ret = BSP_ERROR_NONE;
    }
  }

  return ret;
}

