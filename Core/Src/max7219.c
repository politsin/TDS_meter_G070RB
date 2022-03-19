/*
 * max7219.c
 *
 *  Created on: Feb 27, 2022
 *      Author: dima
 */
#include "main.h"
#include "max7219.h"

#define MAX7219_SPI_DR_8_BIT    *(__IO uint8_t*)&(SPI_MAX7219->DR)

void max7219_set_brightness(uint8_t br);
void max7219_clear();
void max7219_send_to_all(uint8_t reg, uint8_t data);
void max7219_test();

extern SPI_HandleTypeDef hspi1;


void max7219_init(uint8_t br)
{
	MAX7219M_CS_ON;

	max7219_send_to_all(REG_SHUTDOWN, 0x01); // restart
    max7219_send_to_all(REG_DECODE_MODE, 0x00); // use normal mode
    max7219_send_to_all(REG_SCAN_LIMIT, 0x07); // use all lines
    max7219_set_brightness(br);
    max7219_clear();

    max7219_send_to_all(REG_DIGIT_0, 0x01);
    max7219_send_to_all(REG_DIGIT_1, 0x02);
    max7219_send_to_all(REG_DIGIT_3, 0x02);
}


void max7219_set_brightness(uint8_t br)
{
	max7219_send_to_all(REG_INTENSITY, br);
}


void max7219_clear()
{
    for(uint32_t i = 0; i < 8; i++)
    {
    	max7219_send_to_all(REG_DIGIT_0 + i, 0x00);
    }
}


void send_data(uint8_t reg, uint8_t data)
{
    //SPI1->DR = (uint16_t)((reg << 8) | data );
    //while((SPI1->SR & SPI_SR_BSY) == SPI_SR_BSY);

	/*MAX7219_SPI_DR_8_BIT = reg;
    while(!(SPI_MAX7219->SR & SPI_SR_TXE));

    MAX7219_SPI_DR_8_BIT = data;
    while(!(SPI_MAX7219->SR & SPI_SR_TXE));*/

    HAL_SPI_Transmit(&hspi1, &reg, 1, 500);
    HAL_SPI_Transmit(&hspi1, &data, 1, 500);
}

void max7219_send_to_all(uint8_t reg, uint8_t data)
{
	//MAX7219M_CS_ON;

    //for(uint32_t i = 0; i < MAX7219_CHIPS; i++)
    //{
        send_data(reg, data);
    //}

        //MAX7219M_CS_OFF;
}



void max7219_test()
{
    max7219_send_to_all(REG_DIGIT_0, 0x01);
    max7219_send_to_all(REG_DIGIT_1, 0x02);
    max7219_send_to_all(REG_DIGIT_2, 0x04);
    max7219_send_to_all(REG_DIGIT_3, 0x08);
    max7219_send_to_all(REG_DIGIT_4, 0x0F);
    max7219_send_to_all(REG_DIGIT_5, 0x10);
    max7219_send_to_all(REG_DIGIT_6, 0x12);
    max7219_send_to_all(REG_DIGIT_7, 0x14);
}















