/* 
https://garberas.com/archives/362
より

 */
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "app_freertos.h"

#include "log.h"
#include "stm32g4xx_hal.h"
#include <string.h>
#include <stdint.h>
#include "sensor.h"
#include "main.h"

#define LOG_PAGE_ADDR_START 246
const uint32_t start_address = 0x08000000 + (0x800 * LOG_PAGE_ADDR_START); // start address
const uint32_t end_adress = 0x0807FFFF; // end address

uint8_t logData[8*32*2];

/*
 *@brief erase sector11
*/
void eraseFlash( void )
{
    uint32_t pageError = 0;
    FLASH_EraseInitTypeDef erase;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;	// page erase
    erase.Banks = FLASH_BANK_1;		       // set BANK1
    erase.Page = 246;		// set to erase start page
    erase.NbPages = 10;		// set to erase num of pages

    HAL_FLASH_Unlock();     // unlock flash
    HAL_FLASHEx_Erase(&erase, &pageError);	// erase sector
    HAL_FLASH_Lock(); // lock flash
}

/*
 * @brief write flash(sector11)
 * @param uint32_t address sector11 start address
 * @param uint8_t * data write data
 * @param uint32_t size write data size
*/
void writeFlash(uint32_t address, uint8_t *data, uint32_t size)
{
    HAL_FLASH_Unlock();     // unlock flash

    for ( uint32_t add = address; add < (address + size); add++ )
    {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_FAST_AND_LAST, add, *data); // write byte
        data++;  // add data pointer
    } 
    HAL_FLASH_Lock(); // lock flash
}

/*
 * @brief write flash(sector11)
 * @param uint32_t address sector11 start address
 * @param uint8_t * data read data
 * @param uint32_t size read data size
*/
void loadFlash(uint32_t address, uint8_t *data, uint32_t size)
{
    memcpy(data, (uint8_t*)address, size); // copy data
}

void g_LogTask(void *argument)
{
    uint32_t iterator = 0;
    uint32_t writeOffset = 0;
    uint16_t select = 0;

    osThreadSuspend(logTaskHandle);
    while(1)
    {
        for(int i = 0; i < 1000; i++)
        {
            osSemaphoreAcquire(logTriggerSemaphoreHandle, osWaitForever);
            *(uint16_t*)&logData[(iterator & 0x1FF)] = g_sensor.enc_l;
            iterator += 2;
            *(uint16_t*)&logData[(iterator & 0x1FF)] = g_sensor.enc_r;
            iterator += 2;

            if((select == 0)
            && ((iterator & 0x1FF) > 256))
            {
                HAL_FLASH_Unlock(); // unlock flash
                for(int i = 0; i < 256; i+=8)
                {
                    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, start_address+writeOffset+i, *(uint64_t*)&logData[i]); // write byte
                }
                HAL_FLASH_Lock(); // lock flash
                select = 1;
                writeOffset += 256;
            }
            if((select == 1)
            && ((iterator & 0x1FF) < 256))
            {
                HAL_FLASH_Unlock(); // unlock flash
                for(int i = 0; i < 256; i+=8)
                {
                    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, start_address+writeOffset+i, *(uint64_t*)&logData[256 + i]); // write byte
                }
                HAL_FLASH_Lock(); // lock flash
                select = 0;
                writeOffset += 256;
            }
        }
        HAL_GPIO_WritePin(LED_0_GPIO_Port, LED_0_Pin, 0);
        osThreadSuspend(logTaskHandle);
    }
}


