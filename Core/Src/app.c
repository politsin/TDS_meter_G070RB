
#include "app.h"
#include "app_config.h"
#include "max7219.h"
#include "main.h"
#include "math.h"

char trans_str[128] = { 0, };

// размер буфера ацп для adc_33_volt, adc_2_volt, adc_ntc, adc_vrefint
// (должно быть чётное число).
#define COUNT_REQUEST     5
// количество каналов. Если тут не чётное число,
// то COUNT_REQUEST должен делится на него без остатка).
#define CH_ADC            5
#define DIV_ADC           10
// количество опросов EC (должно быть чётное число).
#define EC_COUNT_ADC      16


void app_display_top(int32_t val) {

}
void app_display_bottom(int32_t val) {

}

int32_t rawNtcToTemperature(uint16_t Vout) {
	Vout = 3300 * Vout / 4096;
	uint32_t Rt;
	double kelvin, celsius;

	uint32_t R1 = ntcR1; // 9600  | voltage divider resistor value
	uint32_t Ro = ntcRo; // 10000 | R of Thermistor at 25 degree
	uint32_t To = ntcTo; // 25 | Temperature in Kelvin for 25 degree
	uint32_t koefB = ntcKoefB; // 3950  | Beta value
	uint32_t mv = (referenceVoltage - Vout);

	Rt = R1 * mv / (referenceVoltage - mv);
	kelvin = (double) Ro / (double) Rt;              // R/Ro
	kelvin = log(kelvin);                          // ln(R/Ro)
	kelvin = (1 / (double) koefB) * kelvin;         // 1/B * ln(R/Ro)
	kelvin = (1 / ((double) To + 273.15)) + kelvin; // 1/To + 1/B * ln(R/Ro)
	kelvin = 1 / kelvin; // 1/( 1/To + 1/B * ln(R/Ro) )​

	celsius = kelvin - 273.15; // Convert Kelvin to Celsius.
	return round(celsius * 1000);
}
