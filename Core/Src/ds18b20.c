/*
 * ds18b20.c
 *
 *  Created on: 5 мая 2019 г.
 *      Author: istarik.ru
 */


#include "ds18b20.h"
#include "delay_micros_tim.h"

#include "string.h"
#include "stdio.h"

uint32_t delay_wait_convert = DELAY_T_CONVERT;


uint8_t getDevider(DS18B20_Resolution resolution)
{
	uint8_t devider;

	switch(resolution)
	{
		case DS18B20_Resolution_9_bit:
			devider = 8;
			break;

		case DS18B20_Resolution_10_bit:
			devider = 4;
			break;

		case DS18B20_Resolution_11_bit:
			devider = 2;
			break;

		case DS18B20_Resolution_12_bit:
		default:
			devider = 1;
			break;
	}

	return devider;
}

void writeBit(uint8_t bit)
{
	DS18B20_LOW;
	delay_us(bit ? DELAY_WRITE_1 : DELAY_WRITE_0);
	DS18B20_HIGH;
	delay_us(bit ? DELAY_WRITE_1_PAUSE : DELAY_WRITE_0_PAUSE);
}

void writeByte(uint8_t data)
{
	for (uint8_t i = 0; i < 8; i++)
	{
		writeBit(data >> i & 1);
		delay_us(DELAT_PROTECTION);
	}
}

uint8_t readBit()
{
	uint8_t bit = 0;
	DS18B20_LOW;
	delay_us(DELAY_READ_SLOT);
	DS18B20_HIGH;
	delay_us(DELAY_BUS_RELAX);
	bit = (DS18B20_GPIO_Port->IDR & DS18B20_Pin) ? 1 : 0;
	delay_us(DELAY_READ_PAUSE);
	return bit;
}


void setResolution(DS18B20_Resolution resolution)
{
	DS18B20_LOW;
	delay_us(DELAY_RESET);
	DS18B20_HIGH;
	delay_us(DELAY_RESET);

	writeByte(SKIP_ROM);
	writeByte(WRITE_SCRATCHPAD);
	writeByte(TH_REGISTER);
	writeByte(TL_REGISTER);
	writeByte(resolution);
	delay_wait_convert = DELAY_T_CONVERT / getDevider(resolution);
}





