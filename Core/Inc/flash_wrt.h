/*
 * flash_wrt.h
 *
 *  Created on: 13 авг. 2020 г.
 *      Author: dima
 */

#ifndef FLASH_WRT_H_
#define FLASH_WRT_H_

#define ADDR_FLASH_PAGE_63    63

uint8_t Write_flash_data(uint32_t PAGE, uint32_t *data);
void Read_flash_data(uint32_t PAGE, uint32_t *data);

#endif /* FLASH_WRT_H_ */
