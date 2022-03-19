/*
 * max7219.h
 *
 *  Created on: Feb 27, 2022
 *      Author: dima
 */

#ifndef INC_MAX7219_H_
#define INC_MAX7219_H_


#define SPI_MAX7219          SPI1


#define REG_NO_OP            0x00
#define REG_DIGIT_0          0x01
#define REG_DIGIT_1          0x02
#define REG_DIGIT_2          0x03
#define REG_DIGIT_3          0x04
#define REG_DIGIT_4          0x05
#define REG_DIGIT_5          0x06
#define REG_DIGIT_6          0x07
#define REG_DIGIT_7          0x08
#define REG_DECODE_MODE      0x09
#define REG_INTENSITY        0x0A
#define REG_SCAN_LIMIT       0x0B
#define REG_SHUTDOWN         0x0C
#define REG_DISPLAY_TEST     0x0F



#define MAX7219M_CS_ON       (HAL_GPIO_WritePin(MAX7219M_CS_GPIO_Port, MAX7219M_CS_Pin, GPIO_PIN_RESET))
#define MAX7219M_CS_OFF      (HAL_GPIO_WritePin(MAX7219M_CS_GPIO_Port, MAX7219M_CS_Pin, GPIO_PIN_SET))





void max7219_init(uint8_t br);










#endif /* INC_MAX7219_H_ */
