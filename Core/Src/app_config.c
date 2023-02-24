#include "app.h"
#include "app_config.h"

#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "string.h"
#include "stdio.h"
#include "delay_micros_tim.h"
#include "flash_wrt.h"
#include "ds18b20.h"
#include "max7219.h"
#include "math.h"
#include "app.h"
#include "app_config.h"

#define BUF_UART 256


///////////////////// READ FLASH ////////////////////////
uint32_t data[13] = { 0, };
void readConfig() {

	Read_flash_data(ADDR_FLASH_PAGE_63, data);

#if DEBUG_USART1 // дефаин в файле main.h
	uint8_t len = snprintf(trans_str, BUF_UART,
			"R %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu", data[0],
			data[1], data[2], data[3], data[4], data[5], data[6], data[7],
			data[8], data[9], data[10], data[11], data[12]);
	trans_to_usart(trans_str, len);
#endif

	if (data[0] > 0xFFFFFFF1 || skip_settings == 1) // если флеш пустая, записываем туда дефолтные значения
			{
		data[0] = interval_ds18;    // 800
		data[1] = interval_ec;      // 500
		data[2] = referenceVoltage; // 2500
		data[3] = ntcR1;            // 9600
		data[4] = ntcRo;            // 10000
		data[5] = ntcTo;            // 25
		data[6] = ntcKoefB;         // 3950
		data[7] = ecRo;             // 1000
		data[8] = ecKoefA;          // 54790
		data[9] = ecKoefB;          // 90
		data[10] = ecKoefC;         // 34
		data[11] = ecKoefT;         // 0
		data[12] = ec_Hz;           // 9

		Write_flash_data(ADDR_FLASH_PAGE_63, data);
	} else {
		interval_ds18 = data[0];
		interval_ec = data[1];
		referenceVoltage = data[2];
		ntcR1 = data[3];
		ntcRo = data[4];
		ntcTo = data[5];
		ntcKoefB = data[6];
		ecRo = data[7];
		ecKoefA = data[8];
		ecKoefB = data[9];
		ecKoefC = data[10];
		ecKoefT = data[11];
		ec_Hz = data[12];
	}

#if DEBUG_USART1
	len = snprintf(trans_str, BUF_UART,
			"B %lu %lu %lu %lu %lu %lu %lu %ld %ld %ld %ld %ld %lu",
			interval_ds18, interval_ec, referenceVoltage, ntcR1, ntcRo, ntcTo,
			ntcKoefB, ecRo, ecKoefA, ecKoefB, ecKoefC, ecKoefT, ec_Hz);
	trans_to_usart(trans_str, len);
#endif

}

