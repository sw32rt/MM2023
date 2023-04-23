/* 
https://garberas.com/archives/362
より

 */

#include "log.h"
#include "stm32g4xx_hal.h"
#include <string.h>
#include <stdint.h>

#define LOG_PAGE_ADDR_START 246
const uint32_t start_address = 0x08000000 + (0x800 * LOG_PAGE_ADDR_START); // start address
const uint32_t end_adress = 0x0807FFFF; // end address

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

    HAL_FLASHEx_Erase(&erase, &pageError);	// erase sector
}

/*
 * @brief write flash(sector11)
 * @param uint32_t address sector11 start address
 * @param uint8_t * data write data
 * @param uint32_t size write data size
*/
void writeFlash(uint32_t address, uint8_t *data, uint32_t size  )
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
void loadFlash(uint32_t address, uint8_t *data, uint32_t size )
{
    memcpy(data, (uint8_t*)address, size); // copy data
}