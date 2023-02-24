/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "delay_micros_tim.h"
#include "flash_wrt.h"
#include "ds18b20.h"
#include "max7219.h"
#include "math.h"
#include "app.h"
#include "app_config.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// размер буфера ацп для adc_33_volt, adc_2_volt, adc_ntc, adc_vrefint
// (должно быть чётное число).
#define COUNT_REQUEST     5
// количество каналов. Если тут не чётное число,
// то COUNT_REQUEST должен делится на него без остатка).
#define CH_ADC            5
#define DIV_ADC           10
// количество опросов EC (должно быть чётное число).
#define EC_COUNT_ADC      16
//#define BUF_UART          256 (перенес)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// sb16 sb18 uart2 - del, sb17 mco - set, sb22 led - del
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

////////////// DS18B20 ///////////////
volatile uint8_t flag_ds18b20 = 1;

/////// Переменные АЦП1.
volatile uint8_t adc_flag_full = 0;
volatile uint8_t adc_flag_half = 0;
volatile uint8_t adc_count_full = 0;

//uint16_t adc_buf[COUNT_REQUEST] = {0,};
//uint32_t tmp_adc[CH_ADC] = {0,};
//uint16_t adc[CH_ADC] = {0,};

//////////////////////////////////////////////////
//uint32_t interval_uart = 500;

uint32_t interval_ds18 = 800; // миллисекунды, min 800                         [A800]
uint32_t interval_ec = 500; // миллисекунды, min 500                           [B

uint32_t referenceVoltage = 2500; // 2500mv (напряжение питания датчиков)      [C

uint32_t ntcR1 = 9600; // 10kΩ voltage divider resistor value                  [D
uint32_t ntcRo = 10000; // 10kΩ R of Thermistor at 25 degree                   [E
uint32_t ntcTo = 25; // 25 Temperature in Kelvin for 25 degree                 [F
uint32_t ntcKoefB = 3950; // 3950 Beta value                                   [G

int32_t ecRo = 1000; //  Ω Voltage divider resistor value 500Ω / 1000Ω         [K
int32_t ecKoefA = 54790; //  Alfa value                                        [L
int32_t ecKoefB = 90; //  Beta value                                           [M
int32_t ecKoefC = 34; //  С-value                                              [N
int32_t ecKoefT = 1; //  Ноль Koef Temperature                                 [P

uint32_t ec_Hz = 99; //  Частота ШиМа (в микросек, min 9, max 65535))          [Q
uint32_t skip_settings = 1;

typedef struct {
	double ec1;
	double ec2;
	double ec3;
	double adc_ec_raw1;
	double adc_ec_raw2;
	double adc_ec_raw3;
} calc_struct;

calc_struct my_struct = { 0, };

volatile uint32_t tim = 0;
volatile uint32_t tim17_count = 0;

volatile uint8_t flag_tim = 0;

uint16_t adc_buf[COUNT_REQUEST] = { 0, };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void trans_to_usart(char *buf, uint8_t len) {
	for (uint8_t i = 0; i < len; i++) {
		while ((USART1->ISR & USART_ISR_TC) == 0) {
		}
		USART1->TDR = buf[i];
	}

	//while((USART2->ISR & USART_ISR_TC) == 0){}
	//USART2->TDR = '\r';

	while ((USART1->ISR & USART_ISR_TC) == 0) {
	}
	USART1->TDR = '\n';
}

/////////// Таймер измерения времени для скорости & rpm ///////////
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	// ds18b20 tim17_count.
	if (htim == &htim6) {
		flag_ds18b20 = 2;
	}
	// ds18b20.
	if (htim->Instance == TIM17) {
		tim17_count++;
		//trans_to_usart("R", 1);
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	//if(hadc->Instance == ADC1)
	//{
	adc_flag_full = 1;

	/*flag_tim++;

	 if(flag_tim == 1)
	 {
	 TIM17->CNT = 0;
	 }
	 else if(flag_tim == 2)
	 {
	 tim = TIM17->CNT;
	 flag_tim = 0;
	 }*/

	//uint16_t len = sprintf(trans_str, "F %d %d %d %d %d  %lu", adc_buf[0], adc_buf[1], adc_buf[2], adc_buf[3], adc_buf[4], tim);
	//trans_to_usart(trans_str, len);
	//HAL_ADC_Stop(&hadc1);
	//trans_to_usart("F", 1);
	//}
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {
	//if(hadc->Instance == ADC1)
	//{
	adc_flag_half = 1;

	//uint16_t len = sprintf(trans_str, "H %d %d %d %d %d  %lu", adc_buf[0], adc_buf[1], adc_buf[2], adc_buf[3], adc_buf[4], tim);
	//trans_to_usart(trans_str, len);

	//HAL_ADC_Stop(&hadc1);
	//trans_to_usart("H", 1);
	//}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM7_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();
  MX_TIM17_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
	/*PWR-V25 - включает питание опорного напряжения 2.5V. Включается низким напряжением в начале работы

	 PWR-EC - включает питание драйвера шагового двигателя который щёлкает ногами EC (перед этим нужно включить питание PWR-V25).
	 Оно включается высоким уровнем в начале работы.

	 PWR-LED - включает питание на 7-сегментные индикаторы c линии батарейки.
	 Одна из возможных проблем. Без этой штуки питание 7-сегментного идёт через диод с линии 5 вольт

	 PWR-ESP - включает питание на esp32 от батарейки, вторая возможная проблема. ecp32 также будет запитана с линии 5V*/

	HAL_Delay(500);

	///////////////////// READ FLASH ////////////////////////
	readConfig();
	/////////////////////// DWT ///////////////////////////// 0x0801F800
	Delay_us_tim_init();

	///////////////////// DS18B20 ///////////////////////////
	setResolution(DS18B20_Resolution_12_bit);

	///////////////////// PWR_V25_ON ///////////////////////////
	PWR_V25_ON;

	////////////////////// PWR_EC_ON ///////////////////////////
	PWR_EC_ON;

	///////////////////// ADC1 /////////////////////////////
	HAL_ADCEx_Calibration_Start(&hadc1); // калибровка АЦП1
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_buf, COUNT_REQUEST);

	/////// переменные АЦП1 ///////
	uint16_t filter[CH_ADC] = { 0, };
	//uint16_t adc[CH_ADC] = {0,};

	uint16_t adc_ec = 0;
	uint16_t adc_ntc = 0;
	uint16_t adc_25 = 0;
	uint16_t adc_33 = 0;
	uint16_t adc_lipo = 0;

	uint16_t adc_ec_tmp = 0;
	uint16_t adc_ntc_tmp = 0;
	uint16_t adc_25_tmp = 0;
	uint16_t adc_33_tmp = 0;
	uint16_t adc_lipo_tmp = 0;

	uint32_t adc_ch_1 = 0;
	uint32_t adc_ch_2 = 0;
	uint32_t adc_ch_3 = 0;
	uint32_t adc_ch_4 = 0;
	uint32_t adc_ch_5 = 0;

	uint16_t *ptr_adc_buf = NULL;

	uint32_t tmp_adc_ec_positive = 0; // собирает пачку позитивных значений ЕС (по умолчанию 10 - #define EC_COUNT_ADC)
	uint16_t adc_ec_positive = 0; // среднее позитив = (tmp_adc_ec_positive / EC_COUNT_ADC)

	uint32_t tmp_adc_ec_negative = 0; // собирает пачку негативных значений ЕС (по умолчанию 10 - #define EC_COUNT_ADC)
	uint16_t adc_ec_negative = 0; // среднее негатив = (tmp_adc_ec_negative / EC_COUNT_ADC)

	///////////////// Готовые температуры ////////////////////
	int32_t temp_ntc = 0;
	int16_t temp_ds18 = 0;

	//MY_ADC1_Init(1);
	TIM1->PSC = 63;
	TIM1->ARR = ec_Hz;
	TIM1->CCR1 = TIM1->ARR;
	TIM1->CCR4 = TIM1->ARR;
	TIM1->EGR = TIM_EGR_UG;
	TIM1->SR = ~TIM_SR_UIF;

	HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_OCN_Start(&htim1, TIM_CHANNEL_1);

	//HAL_ADC_Start(&hadc1);
	//HAL_ADC_PollForConversion(&hadc1, 2000);
	//trans_to_usart("2", 1);
	//trans_to_usart("3", 1);
	//while(!(ADC1->ISR & ADC_ISR_EOC));
	//trans_to_usart("4", 1);
	//uint16_t len = sprintf(trans_str, "A1 %lu", ADC1->DR);
	//trans_to_usart(trans_str, len);
	//trans_to_usart("5", 1);
	//while(1){}

	///////////////////////// tick ///////////////////////////
	uint32_t tim_ds18 = 0;
	uint32_t tim_ec = 0;

	//////////////// Вкл прерывания UART /////////////////////
	//__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);

	//////////////////////////////////////////////////////////
	my_struct.ec1 = 22.56;

	HAL_TIM_Base_Start_IT(&htim17);

	max7219_setup();

	// 2507 = 3300/4095*3111
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		adc_ntc = adc_buf[1];
		temp_ntc = rawNtcToTemperature(adc_ntc);
		uint16_t len = sprintf(trans_str, "%d  %d  %d  %d  %d  %d %l", adc_buf[0],
				adc_buf[1], adc_buf[2], adc_buf[3], adc_buf[4], temp_ds18, temp_ntc);
		trans_to_usart(trans_str, len);

		//////////////////////////// ADC FULL ///////////////////////////////
		if (adc_flag_full == 1) {
			adc_flag_full = 0;

			adc_count_full++;

			adc_ch_1 += adc_buf[0];
			adc_ch_2 += adc_buf[1];
			adc_ch_3 += adc_buf[2];
			adc_ch_4 += adc_buf[3];
			adc_ch_5 += adc_buf[4];

			if (adc_count_full == 10) {

				TIM17->CNT = 0;
				//tim17_count = 0;

				adc_ec_tmp = adc_ch_1 / DIV_ADC;
				adc_ntc_tmp = adc_ch_2 / DIV_ADC;
				adc_25 = adc_ch_3 / DIV_ADC;
				adc_33 = adc_ch_4 / DIV_ADC;
				adc_lipo = adc_ch_5 / DIV_ADC;

				/*for(uint8_t i = 0; i < CH_ADC; i++)
				 {
				 filter[i] = filter[i] - ((filter[i]) >> 4) + adc_buf[i];
				 adc[i] = (filter[i]) >> 4;
				 }*/

				//filter[0] = filter[0] - ((filter[0]) >> 4) + adc_ntc_tmp;
				//adc_ntc = (filter[0]) >> 4;
				adc_ch_1 = 0;
				adc_ch_2 = 0;
				adc_ch_3 = 0;
				adc_ch_4 = 0;
				adc_ch_5 = 0;

				//tim = (TIM17->CNT + (tim17_count * 65535));
				tim = TIM17->CNT;

				adc_count_full = 0;

				//uint16_t len = sprintf(trans_str, "%d  %d  %d  %d  %d    %lu  t: %d", adc_buf[1], adc_ntc, adc_ntc_tmp, adc[3], adc[4], tim, temp_ds18);
				//trans_to_usart(trans_str, len);
			}

		}
		max7219_loop();

		LED2_ON;
		LED1_OFF;
		HAL_Delay(50);
		LED2_OFF;
		LED1_ON;
		HAL_Delay(50);

		//max7219_init(8);

		//////////////////////////// ADC HALF ///////////////////////////////
		/*if(adc_flag_half == 1)
		 {
		 adc_flag_half = 0;

		 adc_ch_1 = 0;
		 adc_ch_2 = 0;

		 ptr_adc_buf = &adc_buf[0];

		 for(uint16_t i = 0; i < COUNT_REQUEST / (2 * 2); i++)
		 {
		 adc_ch_1 += ptr_adc_buf[2 * i + 0];
		 adc_ch_2 += ptr_adc_buf[2 * i + 1];
		 }

		 pred_ch1_h += adc_ch_1 / (COUNT_REQUEST / (2 * 2));
		 pred_ch2_h += adc_ch_2 / (COUNT_REQUEST / (2 * 2));

		 }*/

		//////////////////////////////// DS18B20 ////////////////////////////////////
		if ((uwTick - tim_ds18) > interval_ds18) {
			if (flag_ds18b20 == 1) {
				flag_ds18b20 = 0;

				DS18B20_LOW;
				delay_us(DELAY_RESET);
				DS18B20_HIGH;
				delay_us(DELAY_RESET);

				writeByte(SKIP_ROM);
				writeByte(CONVERT_T);

				TIM6->DIER |= TIM_IT_UPDATE;
				TIM6->CR1 |= TIM_CR1_CEN;

				//adc_flag_full = 0;
				//adc_flag_half = 0;

				LED1_ON;
				LED2_OFF;

			} else if (flag_ds18b20 == 2) {
				// когда таймер отсчитает 750мс, он установит flag == 2.
				DS18B20_LOW;
				delay_us(DELAY_RESET);
				DS18B20_HIGH;
				delay_us(DELAY_RESET);

				writeByte(SKIP_ROM);
				writeByte(READ_SCRATCHPAD);

				temp_ds18 = 0;

				for (uint8_t i = 0; i < 16; i++)
					temp_ds18 += (int16_t) readBit() << i;

				temp_ds18 = (100 * temp_ds18 / 16);

				flag_ds18b20 = 1; // запускаем новое измерение.

				//adc_flag_full = 0;
				//adc_flag_half = 0;

				LED1_OFF;
				LED2_ON;
			}

			tim_ds18 = uwTick;
		}

		//////////////////////////// EC ///////////////////////////////
		if ((uwTick - tim_ec) > interval_ec) {
			/*HAL_ADC_Stop_DMA(&hadc1);
			 HAL_ADC_Stop(&hadc1);

			 tmp_adc_ec_positive = 0;
			 tmp_adc_ec_negative = 0;

			 MY_ADC1_Init(1);

			 HAL_ADC_Start(&hadc1);
			 HAL_ADC_PollForConversion(&hadc1, 100);

			 uint16_t len = sprintf(trans_str, "A1 %lu", ADC1->DR);
			 trans_to_usart(trans_str, len);


			 HAL_ADC_Start(&hadc1);
			 HAL_ADC_PollForConversion(&hadc1, 100);

			 len = sprintf(trans_str, "A2 %lu", ADC1->DR);
			 trans_to_usart(trans_str, len);

			 HAL_ADC_Start(&hadc1);
			 HAL_ADC_PollForConversion(&hadc1, 100);

			 len = sprintf(trans_str, "A3 %lu", ADC1->DR);
			 trans_to_usart(trans_str, len);

			 HAL_ADC_Stop(&hadc1);

			 HAL_Delay(200);*/

			// данные с АЦП2 (ЕС) читаются постоянно, преобразование запускается по триггеру от таймера №3.
			// блогодаря тому, что АЦП2 работает постоянно, нам не нужно тратить время на его запуск.
			/*ADC1->DR; // читаем регистр чтоб обнулить его

			 for(uint8_t i = 0; i < EC_COUNT_ADC; i++)
			 {
			 while(!(ADC1->ISR & ADC_ISR_EOC)); // ждём следуещего преобразования

			 if((GPIOA->IDR & GPIO_PIN_8)) // проверяем пин РА8 (в каком он состоянии - позитивном или негативном)
			 {
			 tmp_adc_ec_positive += ADC1->DR;
			 //trans_to_usart1("P");
			 }
			 else
			 {
			 tmp_adc_ec_negative += ADC1->DR;
			 //trans_to_usart1("N");
			 }
			 }*/

			adc_ec_positive = (tmp_adc_ec_positive / (EC_COUNT_ADC / 2));
			adc_ec_negative = (tmp_adc_ec_negative / (EC_COUNT_ADC / 2));

			//uint16_t vdd = 1210 * 4095 / adc_33; // напряжение питания на основе Vrefint (не точно)

			////////////////////////// Выводим все данные в УАРТ /////////////////////////////
			//uint16_t len = snprintf(trans_str, BUF_UART, "%d %d %d %d %ld %d %d %d %d\n", \
			//adc_33_volt, adc_2_volt, adc_ntc, adc_vrefint, temp_ntc, temp_ds18, adc_ec_positive, adc_ec_negative, vdd);

			//while((HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY));
			//HAL_UART_Transmit_DMA(&huart1, (uint8_t*)trans_str, len);

			//if(adc_ec_positive < 2900 || adc_ec_positive > 3100) while(1){}
			//if(adc_ec_negative < 2900 || adc_ec_negative > 3100) while(1){}

			//////////////// tm1637 ///////////////
			/*if(vdd >= 1000)
			 {
			 data_to_disp[0] = (vdd / 1000 % 10);
			 data_to_disp[1] = (vdd / 100 % 10);
			 data_to_disp[2] = (vdd / 10 % 10);
			 data_to_disp[3] = vdd % 10;
			 }
			 else if(vdd >= 100)
			 {
			 data_to_disp[0] = 0x7f; // ничего не выводит на сегмент
			 data_to_disp[1] = (vdd / 100 % 10);
			 data_to_disp[2] = (vdd / 10 % 10);
			 data_to_disp[3] = vdd % 10;
			 }
			 else if(vdd >= 10)
			 {
			 data_to_disp[0] = 0x7f;
			 data_to_disp[1] = 0x7f;
			 data_to_disp[2] = (vdd / 10 % 10);
			 data_to_disp[3] = vdd % 10;
			 }
			 else if(vdd >= 0)
			 {
			 data_to_disp[0] = 0x7f;
			 data_to_disp[1] = 0x7f;
			 data_to_disp[2] = 0x7f;
			 data_to_disp[3] = vdd % 10;
			 }
			 //// Для отрицательных значений -1 -99 ////
			 else if(vdd <= -10 && vdd >= -99)
			 {
			 int8_t tmp = vdd;
			 tmp *= -1;

			 data_to_disp[0] = 0x7f;
			 data_to_disp[1] = 18; // выводит на сегмент знак "минус"
			 data_to_disp[2] = (tmp / 10 % 10);
			 data_to_disp[3] = tmp % 10;
			 }
			 else if(vdd <= -1 && vdd >= -9)
			 {
			 int8_t tmp = vdd;
			 tmp *= -1;

			 data_to_disp[0] = 0x7f;
			 data_to_disp[1] = 0x7f;
			 data_to_disp[2] = 18;
			 data_to_disp[3] = tmp % 10;
			 }

			 display_mass(data_to_disp);*/

			//snprintf(trans_str, BUF_UART, "Vref: %d Vdd: %d mV\n", adc_vrefint, vdd);
			//trans_to_usart1(trans_str);
			//MY_ADC1_Init(6);
			//HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, COUNT_REQUEST);
			tim_ec = uwTick;
		}

		/*if(count == 5)
		 {
		 count = 6;

		 MY_ADC1_Init(1);
		 }
		 if(count == 11)
		 {
		 count = 0;

		 MY_ADC1_Init(4);
		 }*/

		//////////////////// READ BUTTON //////////////////////
		//read_but(); // 2.3 mks
		//TIM14->CNT = 0;
		/////////////////////////// DEBUG /////////////////////////////
		//uint8_t len = sprintf(trans_str, "%lu  %d %d %d %d %d %d %d  F: %lu %lu  T: %d %d", tim, adc[0], adc[1], adc[2], adc[3], adc[4], adc[5], adc[6], freq, (freq * 8), t0, t1);
		//uint8_t len = sprintf(trans_str, "%lu %lu %lu %d", TIM3->ARR, TIM3->CNT, TIM1->CNT, adc_buf[0]);
		//trans_to_usart(trans_str, len);
		//HAL_Delay(500);
		/*if(adc_flag_full == 1) // ждём флаг, который установится в прерывании АЦП1 по окончанию работы ДМА
		 {
		 adc_flag_full = 0; // обнуляем флаг

		 //TIM17->CNT = 0;
		 // обнуляем временные переменные
		 tmp_adc_ec = 0;
		 tmp_adc_ntc = 0;
		 tmp_adc_25 = 0;
		 tmp_adc_33 = 0;
		 tmp_adc_lipo = 0;
		 tmp_adc_in5 = 0;

		 ptr_adc_buf = &adc_buf[0];

		 // разбираем данные с АЦП1 полученые через ДМА
		 for(uint16_t i = 0; i < (COUNT_REQUEST / CH_ADC); i++)
		 {
		 tmp_adc_ec += ptr_adc_buf[CH_ADC * i + 0];
		 tmp_adc_ntc += ptr_adc_buf[CH_ADC * i + 1];
		 tmp_adc_25 += ptr_adc_buf[CH_ADC * i + 2];
		 tmp_adc_33 += ptr_adc_buf[CH_ADC * i + 3];
		 tmp_adc_lipo += ptr_adc_buf[CH_ADC * i + 4];
		 tmp_adc_in5 += ptr_adc_buf[CH_ADC * i + 5];
		 }

		 // усредняем
		 adc_ec = (tmp_adc_ec / DIV_ADC);
		 adc_ntc = (tmp_adc_ntc / DIV_ADC);
		 adc_25 = (tmp_adc_25 / DIV_ADC);
		 adc_33 = (tmp_adc_33 / DIV_ADC);
		 adc_lipo = (tmp_adc_lipo / DIV_ADC);
		 adc_in5 = (tmp_adc_in5 / DIV_ADC);

		 temp_ntc = rawNtcToTemperature(adc_ntc); // получаем температуру с NTC

		 //tim = TIM17->CNT;

		 uint16_t len = sprintf(trans_str, "n %d %d %d %d %d %d  %lu", adc_ec, adc_ntc, adc_25, adc_33, adc_lipo, adc_in5, tim);
		 trans_to_usart(trans_str, len);

		 HAL_Delay(200);

		 HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, COUNT_REQUEST);
		 }*/

		//uint16_t len = sprintf(trans_str, "T %lu", tim);
		//trans_to_usart(trans_str, len);
		/*if(adc_flag_full == 1) // ждём флаг, который установится в прерывании АЦП1 по окончанию работы ДМА
		 {
		 // обнуляем временные переменные
		 tmp_adc_ec = 0;
		 tmp_adc_ntc = 0;
		 tmp_adc_25 = 0;
		 tmp_adc_33 = 0;
		 tmp_adc_lipo = 0;
		 tmp_adc_in5 = 0;

		 // разбираем данные с АЦП1 полученые через ДМА
		 for(uint16_t i = 0; i < (COUNT_REQUEST / CH_ADC); i++)
		 {
		 tmp_adc_ec += adc_buf[CH_ADC * i + 0];
		 tmp_adc_ntc += adc_buf[CH_ADC * i + 1];
		 tmp_adc_25 += adc_buf[CH_ADC * i + 2];
		 tmp_adc_33 += adc_buf[CH_ADC * i + 3];
		 tmp_adc_lipo += adc_buf[CH_ADC * i + 4];
		 tmp_adc_in5 += adc_buf[CH_ADC * i + 5];
		 }

		 // усредняем
		 adc_ec = (tmp_adc_ec / DIV_ADC);
		 adc_ntc = (tmp_adc_ntc / DIV_ADC);
		 adc_25 = (tmp_adc_25 / DIV_ADC);
		 adc_33 = (tmp_adc_33 / DIV_ADC);
		 adc_lipo = (tmp_adc_lipo / DIV_ADC);
		 adc_in5 = (tmp_adc_in5 / DIV_ADC);

		 temp_ntc = rawNtcToTemperature(adc_ntc); // получаем температуру с NTC

		 uint16_t len = sprintf(trans_str, "A %d %d %d %d %d %d", adc_ec, adc_ntc, adc_25, adc_33, adc_lipo, adc_in5);
		 trans_to_usart(trans_str, len);

		 adc_flag_full = 0;
		 }*/

///////////////
		/*if(adc_flag_full == 1)
		 {
		 adc_flag_full = 0;

		 adc_ch_1 = 0;
		 adc_ch_2 = 0;
		 adc_ch_3 = 0;
		 adc_ch_4 = 0;
		 adc_ch_5 = 0;

		 ptr_adc_buf = &adc_buf[COUNT_REQUEST / 2];

		 for(uint16_t i = 0; i < DIV_ADC; i++)
		 {
		 adc_ch_1 += ptr_adc_buf[CH_ADC * i + 0];
		 adc_ch_2 += ptr_adc_buf[CH_ADC * i + 1];
		 adc_ch_3 += ptr_adc_buf[CH_ADC * i + 2];
		 adc_ch_4 += ptr_adc_buf[CH_ADC * i + 3];
		 adc_ch_5 += ptr_adc_buf[CH_ADC * i + 4];
		 }

		 TIM17->CNT = 0;
		 tim17_count = 0;

		 adc_ec = adc_ch_1 / DIV_ADC;
		 adc_ntc = adc_ch_2 / DIV_ADC;
		 adc_25 = adc_ch_3 / DIV_ADC;
		 adc_33 = adc_ch_4 / DIV_ADC;
		 adc_lipo = adc_ch_5 / DIV_ADC;

		 tim = (TIM17->CNT + (tim17_count * 65535));

		 uint16_t len = sprintf(trans_str, "%d  %d  %d  %d  %d  %lu %lu ms t: %d",\
					adc_ec, adc_ntc, adc_25, adc_33, adc_lipo, tim, (tim / 1000), temp_ds18);
		 trans_to_usart(trans_str, len);


		 LED2_ON;
		 //HAL_Delay(50);
		 LED2_OFF;
		 adc_flag_full = 0;
		 }*/

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//HAL_UART_Transmit(&huart2, (uint8_t*)trans_str, len, 1000);
/*void MY_ADC1_Init(uint8_t numb)
 {

 //HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_1); // запускает Ш�?М, и посылает триггер для запуска тим3
 //HAL_TIMEx_OCN_Stop(&htim1, TIM_CHANNEL_1);

 //HAL_TIM_Base_Stop(&htim3);
 //HAL_ADC_Stop(&hadc1);

 //HAL_ADC_Stop_DMA(&hadc1);

 //HAL_ADC_MspDeInit(&hadc1);

 hadc1.Instance = ADC1;
 hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
 hadc1.Init.Resolution = ADC_RESOLUTION_12B;
 hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
 hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
 hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV; // ADC_EOC_SEQ_CONV;
 hadc1.Init.LowPowerAutoWait = DISABLE;
 hadc1.Init.LowPowerAutoPowerOff = DISABLE;
 hadc1.Init.ContinuousConvMode = ENABLE;
 hadc1.Init.NbrOfConversion = numb;
 hadc1.Init.DiscontinuousConvMode = DISABLE;
 hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
 hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
 hadc1.Init.DMAContinuousRequests = ENABLE;
 hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
 hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_39CYCLES_5;
 hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_39CYCLES_5;
 hadc1.Init.OversamplingMode = DISABLE;
 hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;

 HAL_ADC_Init(&hadc1);

 ADC_ChannelConfTypeDef sConfig = {0};

 if(numb == 6)
 {
 sConfig.Channel = ADC_CHANNEL_0;
 sConfig.Rank = ADC_REGULAR_RANK_1;
 sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
 HAL_ADC_ConfigChannel(&hadc1, &sConfig);

 sConfig.Channel = ADC_CHANNEL_1;
 sConfig.Rank = ADC_REGULAR_RANK_2;
 HAL_ADC_ConfigChannel(&hadc1, &sConfig);

 sConfig.Channel = ADC_CHANNEL_2;
 sConfig.Rank = ADC_REGULAR_RANK_3;
 HAL_ADC_ConfigChannel(&hadc1, &sConfig);

 sConfig.Channel = ADC_CHANNEL_3;
 sConfig.Rank = ADC_REGULAR_RANK_4;
 HAL_ADC_ConfigChannel(&hadc1, &sConfig);

 sConfig.Channel = ADC_CHANNEL_4;
 sConfig.Rank = ADC_REGULAR_RANK_5;
 HAL_ADC_ConfigChannel(&hadc1, &sConfig);

 sConfig.Channel = ADC_CHANNEL_4;
 sConfig.Rank = ADC_REGULAR_RANK_6;
 HAL_ADC_ConfigChannel(&hadc1, &sConfig);
 }
 else if(numb == 1)
 {
 sConfig.Channel = ADC_CHANNEL_0;
 sConfig.Rank = ADC_REGULAR_RANK_1;
 sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
 HAL_ADC_ConfigChannel(&hadc1, &sConfig);
 }

 //HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, COUNT_REQUEST);
 //HAL_TIM_Base_Start(&htim3);
 //HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_1); // запускает Ш�?М, и посылает триггер для запуска тим3
 //HAL_TIMEx_OCN_Start(&htim1, TIM_CHANNEL_1);
 }*/

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
