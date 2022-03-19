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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "delay_micros_tim.h"
#include "flash_wrt.h"
#include "ds18b20.h"
#include "max7219.h"
#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define COUNT_REQUEST     5 // размер буфера ацп для adc_33_volt, adc_2_volt, adc_ntc, adc_vrefint (должно быть чётное число)
#define CH_ADC            5  // количество каналов (если тут не чётное число, то COUNT_REQUEST должен делится на него без остатка)
#define DIV_ADC           10

#define EC_COUNT_ADC      16 // количество опросов EC (должно быть чётное число)

#define BUF_UART          256
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// sb16 sb18 uart2 - del, sb17 mco - set, sb22 led - del
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char trans_str[128] = {0,};

////////////// DS18B20 ///////////////
volatile uint8_t flag_ds18b20 = 1;

/////// переменные АЦП1 ///////
volatile uint8_t adc_flag_full = 0;
volatile uint8_t adc_flag_half = 0;

volatile uint8_t adc_count_full = 0;

//uint16_t adc_buf[COUNT_REQUEST] = {0,};
//uint32_t tmp_adc[CH_ADC] = {0,};
//uint16_t adc[CH_ADC] = {0,};

//////////////////////////////////////////////////
//uint32_t interval_uart = 500;

uint32_t interval_ds18 = 800; // миллисекунды, min 800                             [A800]
uint32_t interval_ec = 500;   // миллисекунды, min 500                             [B

uint32_t referenceVoltage = 2500;  // 2500mv (напряжение питания датчиков)         [C

uint32_t ntcR1 = 9600;    // 10kΩ voltage divider resistor value                   [D
uint32_t ntcRo = 10000;   // 10kΩ R of Thermistor at 25 degree                     [E
uint32_t ntcTo = 25;      // 25 Temperature in Kelvin for 25 degree                [F
uint32_t ntcKoefB = 3950; // 3950 Beta value                                       [G

int32_t ecRo = 1000;      //  Ω Voltage divider resistor value 500Ω / 1000Ω        [K
int32_t ecKoefA = 54790;  //  Alfa value                                           [L
int32_t ecKoefB = 90;     //  Beta value                                           [M
int32_t ecKoefC = 34;     //  С-value                                              [N
int32_t ecKoefT = 1;      //  Ноль Koef Temperature                                [P

uint32_t ec_Hz = 20000;   //  Частота Ш�?Ма (в микросек, min 9, max 65535))         [Q


typedef struct
{
	double ec1;
	double ec2;
	double ec3;
	double adc_ec_raw1;
	double adc_ec_raw2;
	double adc_ec_raw3;
} calc_struct;

calc_struct my_struct = {0,};


volatile uint32_t tim = 0;
volatile uint32_t tim17_count = 0;

volatile uint8_t flag_tim = 0;

uint16_t adc_buf[COUNT_REQUEST] = {0,};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM17_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void trans_to_usart(char *buf, uint8_t len)
{
	for(uint8_t i = 0; i < len; i++)
	{
	  while((USART1->ISR & USART_ISR_TC) == 0){}
	  USART1->TDR = buf[i];
	}

	//while((USART2->ISR & USART_ISR_TC) == 0){}
	//USART2->TDR = '\r';

	while((USART1->ISR & USART_ISR_TC) == 0){}
	USART1->TDR = '\n';
}


/////////// Таймер измерения времени для скорости & rpm ///////////
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim6) // ds18b20 tim17_count
	{
		flag_ds18b20 = 2;
	}

	if(htim->Instance == TIM17) // ds18b20
	{
		tim17_count++;
		//trans_to_usart("R", 1);
	}
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
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

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
	//if(hadc->Instance == ADC1)
	//{
		adc_flag_half = 1;

		//uint16_t len = sprintf(trans_str, "H %d %d %d %d %d  %lu", adc_buf[0], adc_buf[1], adc_buf[2], adc_buf[3], adc_buf[4], tim);
		//trans_to_usart(trans_str, len);

		//HAL_ADC_Stop(&hadc1);
		//trans_to_usart("H", 1);
	//}
}


int32_t rawNtcToTemperature(uint16_t Vout)
{
	uint32_t Rt;
	double kelvin, celsius;

	uint32_t R1 = ntcR1; // 9600  | voltage divider resistor value
	uint32_t Ro = ntcRo; // 10000 | R of Thermistor at 25 degree
	uint32_t To = ntcTo; // 25 | Temperature in Kelvin for 25 degree
	uint32_t koefB = ntcKoefB; // 3950  | Beta value
	uint32_t mv = (referenceVoltage - Vout);

	Rt = R1 * mv / (referenceVoltage - mv);
	kelvin = (double)Ro / (double)Rt;              // R/Ro
	kelvin = log(kelvin);                          // ln(R/Ro)
	kelvin = (1 / (double)koefB) * kelvin;         // 1/B * ln(R/Ro)
	kelvin = (1 / ((double)To + 273.15)) + kelvin; // 1/To + 1/B * ln(R/Ro)
	kelvin = 1 / kelvin; // 1/( 1/To + 1/B * ln(R/Ro) )​

	celsius = kelvin - 273.15; // Convert Kelvin to Celsius.
	return round(celsius * 1000);
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
  uint32_t data[13] = {0,};

  Read_flash_data(ADDR_FLASH_PAGE_63, data);

  #if DEBUG_USART1 // дефаин в файле main.h
  uint8_t len = snprintf(trans_str, BUF_UART, "R %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu",\
		  data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9], data[10], data[11], data[12]);
  trans_to_usart(trans_str, len);
  #endif

  if(data[0] > 0xFFFFFFF1) // если флеш пустая, записываем туда дефолтные значения
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
  }
  else
  {
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
  len = snprintf(trans_str, BUF_UART, "B %lu %lu %lu %lu %lu %lu %lu %ld %ld %ld %ld %ld %lu",\
		  interval_ds18, interval_ec, referenceVoltage, ntcR1, ntcRo, ntcTo, ntcKoefB, ecRo, ecKoefA, ecKoefB, ecKoefC, ecKoefT, ec_Hz);
  trans_to_usart(trans_str, len);
  #endif


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
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, COUNT_REQUEST);

  /////// переменные АЦП1 ///////
  uint16_t filter[CH_ADC] = {0,};
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
// TODO: TIM1->ARR = 2999; //ec_Hz;
  TIM1->ARR = 9;
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

  max7219_init(10);

  // 2507 = 3300/4095*3111

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{

		/*TIM17->CNT = 0;
		tim17_count = 0;

		adc_flag_full = 0;

		while(!adc_flag_full);
		//while(!(ADC1->ISR & ADC_ISR_EOC));

		tim = (TIM17->CNT + (tim17_count * 65535));

		uint16_t len = sprintf(trans_str, "%d  %d  %d  %d  %d  %lu %lu ms t: %d",\
				adc_buf[0], adc_buf[1], adc_buf[2], adc_buf[3], adc_buf[4], tim, (tim / 1000), temp_ds18);
		trans_to_usart(trans_str, len);*/

		//LED2_ON;
		//HAL_Delay(50);
		//LED2_OFF;
		//HAL_Delay(50);  // hdma->Instance->CNDTR

		//uint16_t len = sprintf(trans_str, " %lu", DMA1_Channel1->CNDTR);
		//trans_to_usart(trans_str, len);

				//__HAL_DMA_GET_COUNTER(huart->hdmarx);

		//while(DMA1_Channel1->CNDTR > 0){}

		uint16_t len = sprintf(trans_str, "%d  %d  %d  %d  %d", adc_buf[0], adc_buf[1], adc_buf[2], adc_buf[3], adc_buf[4]);
		trans_to_usart(trans_str, len);


		//////////////////////////// ADC FULL ///////////////////////////////
		if(adc_flag_full == 1)
		{
			adc_flag_full = 0;

			adc_count_full++;

			adc_ch_1 += adc_buf[0];
			adc_ch_2 += adc_buf[1];
			adc_ch_3 += adc_buf[2];
			adc_ch_4 += adc_buf[3];
			adc_ch_5 += adc_buf[4];

			if(adc_count_full == 10)
			{

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

			max7219_init(8);
		}



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
		/*if((uwTick - tim_ds18) > interval_ds18)
		{
			if(flag_ds18b20 == 1)
			{
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


			}
			else if(flag_ds18b20 == 2) // когда таймер отсчитает 750мс, он установит flag == 2
			{
				DS18B20_LOW;
				delay_us(DELAY_RESET);
				DS18B20_HIGH;
				delay_us(DELAY_RESET);

				writeByte(SKIP_ROM);
				writeByte(READ_SCRATCHPAD);

				temp_ds18 = 0;

				for(uint8_t i = 0; i < 16; i++) temp_ds18 += (int16_t)readBit() << i;

				temp_ds18 = (temp_ds18 / 16);

				flag_ds18b20 = 1; // запускаем новое измерение.

				//adc_flag_full = 0;
				//adc_flag_half = 0;

				LED1_OFF;
				LED2_ON;
			}

			tim_ds18 = uwTick;
		}*/


		//////////////////////////// EC ///////////////////////////////
		if((uwTick - tim_ec) > interval_ec)
		{
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

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */
	/*tmp_adc[0] += adc_buf[CH_ADC * i + 0];
	tmp_adc[1] += adc_buf[CH_ADC * i + 1];
	tmp_adc[2] += adc_buf[CH_ADC * i + 2];
	tmp_adc[3] += adc_buf[CH_ADC * i + 3];
	tmp_adc[4] += adc_buf[CH_ADC * i + 4];
	tmp_adc[5] += adc_buf[CH_ADC * i + 5];
	tmp_adc[6] += adc_buf[CH_ADC * i + 6];*/
  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DiscontinuousConvMode = ENABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO2;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_12CYCLES_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_39CYCLES_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_LOW;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 63;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 100;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 1;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 6399;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 7600;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim6, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 63;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65535;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 63;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 65535;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DS18B20_Pin|PWR_EC_Pin|PWR_V25_Pin|MAX7219M_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PWR_BTN_GPIO_Port, PWR_BTN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CRISTALF1_Pin */
  GPIO_InitStruct.Pin = CRISTALF1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CRISTALF1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DS18B20_Pin */
  GPIO_InitStruct.Pin = DS18B20_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(DS18B20_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PWR_EC_Pin PWR_V25_Pin MAX7219M_CS_Pin */
  GPIO_InitStruct.Pin = PWR_EC_Pin|PWR_V25_Pin|MAX7219M_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED2_Pin PWR_BTN_Pin */
  GPIO_InitStruct.Pin = LED2_Pin|PWR_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
  while (1)
  {
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
