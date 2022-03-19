/*
 * flash_wrt.c
 *
 *  Created on: 13 авг. 2020 г.
 *      Author: dima
 */
#include "main.h"
#include "flash_wrt.h"

#include "stm32g0xx_hal.h"

#define COUNT     13

uint8_t my_erse_flash(uint32_t page, uint8_t numb)
{
	static FLASH_EraseInitTypeDef EraseInitStruct;
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Page = page;
	EraseInitStruct.NbPages = numb;
	uint32_t page_error = 0;

	uint8_t res = 0;

	HAL_FLASH_Unlock();

	if(HAL_FLASHEx_Erase(&EraseInitStruct, &page_error) != HAL_OK)
	{
		//trans_to_usart1("NOT ERASE");
		res = 0;
	}
	else
	{
		//trans_to_usart1("ERASE OK");
		res = 1;
	}

	HAL_FLASH_Lock();

	return res;
}


HAL_StatusTypeDef MY_FLASH_Program(uint32_t Address, uint32_t Data)
{
	HAL_StatusTypeDef status;

	__HAL_LOCK(&pFlash);

	status = FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);

	if(status == HAL_OK)
	{
		//SET_BIT(FLASH->CR, FLASH_CR_PG);

		SET_BIT(FLASH->CR, FLASH_CR_FSTPG);

		*(uint32_t*)Address = (uint32_t)Data;

		status = FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);

		CLEAR_BIT(FLASH->CR, FLASH_CR_FSTPG);
	}

	__HAL_UNLOCK(&pFlash);

	return status;
}



uint8_t Write_flash_data(uint32_t PAGE, uint32_t *data)
{
	uint32_t page = PAGE;
	uint8_t res = 0;
	uint32_t address = 0x0801F800;

	////////////////// ОЧИСТКА //////////////////
	res = my_erse_flash(page, 1);
	if(res == 0) return 0;

	//////////////////////// ЗАПИСЬ ////////////////////////////
	HAL_FLASH_Unlock();

	for(uint8_t i = 0; i < COUNT; i++)
	{
		/*if(MY_FLASH_Program(address, data[i]) != HAL_OK)
		{
			HAL_FLASH_Lock();
			return 0;
		}*/

		if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, (uint64_t)data[i]) != HAL_OK)
		{
			HAL_FLASH_Lock();
			return 0;
		}

		address = (address + 8);
	}

	HAL_FLASH_Lock();
	return 1;
}


void Read_flash_data(uint32_t PAGE, uint32_t *data)
{
	uint32_t address = 0x0801F800;

	for(uint8_t i = 0; i < COUNT; i++)
	{
		data[i] = *(uint64_t*)address;
		address = (address + 8);
	}
}

// 134217728
// 134346752




