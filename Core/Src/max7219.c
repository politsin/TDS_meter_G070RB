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

// ---- NEW
static uint8_t decodeMode = 0x00;
#define NUMBER_OF_DIGITS	8
typedef enum {
	NUM_0 = 0b01111110,
	NUM_1 = 0b00110000,
	NUM_2 = 0b01101101,
	NUM_3 = 0b01111001,
	NUM_4 = 0b00110011,
	NUM_5 = 0b01011011,
	NUM_6 = 0b01011111,
	NUM_7 = 0b01110000,
	NUM_8 = 0b01111111,
	NUM_9 = 0b01111011,
	MINUS = 0b00000001,
	LETTER_E = 0b01001111,
	LETTER_C = 0b01001110,
	LETTER_H = 0b00110111,
	LETTER_L = 0b00001110,
	LETTER_P = 0b01100111,
	BLANK = 0b00000000
} MAX7219_Numeric;
void max7219_SendData(uint8_t addr, uint8_t data) {
	MAX7219M_CS_ON;
	HAL_SPI_Transmit(&hspi1, &addr, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi1, &data, 1, HAL_MAX_DELAY);
	MAX7219M_CS_OFF;
}
void max7219_Turn_On(void) {
	max7219_SendData(REG_SHUTDOWN, 0x01);
}
void max7219_SetIntensivity(uint8_t intensivity) {
	if (intensivity > 0x0F) {
		return;
	}
	max7219_SendData(REG_INTENSITY, intensivity);
}
void max7219_Clean() {
	uint8_t clear = 0x00;
	if (decodeMode == 0xFF) {
		clear = BLANK;
	}

	for (int i = 0; i < 8; ++i) {
		max7219_SendData(i + 1, clear);
	}
}
void max7219_setup() {
	uint8_t intensivity = 1;
	max7219_Turn_On();
	max7219_SendData(REG_SCAN_LIMIT, NUMBER_OF_DIGITS - 1);
	max7219_SetIntensivity(intensivity);
	// max7219_Clean();
}
void max7219_loop() {
	max7219_SendData(3, LETTER_E);
	max7219_SendData(2, LETTER_C);
	max7219_SendData(1, NUM_1);
	max7219_SendData(6, LETTER_L);
	max7219_SendData(5, LETTER_P);
	max7219_SendData(4, BLANK);
	HAL_Delay(100);
//	max7219_Clean();
// 	HAL_Delay(100);
}

void max7219_set_brightness(uint8_t br) {
	max7219_send_to_all(REG_INTENSITY, br);
}

void max7219_clear() {
	for (uint32_t i = 0; i < 8; i++) {
		max7219_send_to_all(REG_DIGIT_0 + i, 0x00);
	}
}

void send_data(uint8_t reg, uint8_t data) {
	//SPI1->DR = (uint16_t)((reg << 8) | data );
	//while((SPI1->SR & SPI_SR_BSY) == SPI_SR_BSY);

	/*MAX7219_SPI_DR_8_BIT = reg;
	 while(!(SPI_MAX7219->SR & SPI_SR_TXE));

	 MAX7219_SPI_DR_8_BIT = data;
	 while(!(SPI_MAX7219->SR & SPI_SR_TXE));*/

	HAL_SPI_Transmit(&hspi1, &reg, 1, 500);
	HAL_SPI_Transmit(&hspi1, &data, 1, 500);
}

void max7219_send_to_all(uint8_t reg, uint8_t data) {
	//MAX7219M_CS_ON;

	//for(uint32_t i = 0; i < MAX7219_CHIPS; i++)
	//{
	max7219_SendData(reg, data);
	//}

	//MAX7219M_CS_OFF;
}

void max7219_test() {
	max7219_send_to_all(REG_DIGIT_0, 0x01);
	max7219_send_to_all(REG_DIGIT_1, 0x02);
	max7219_send_to_all(REG_DIGIT_2, 0x04);
	max7219_send_to_all(REG_DIGIT_3, 0x08);
	max7219_send_to_all(REG_DIGIT_4, 0x0F);
	max7219_send_to_all(REG_DIGIT_5, 0x10);
	max7219_send_to_all(REG_DIGIT_6, 0x12);
	max7219_send_to_all(REG_DIGIT_7, 0x14);
}

