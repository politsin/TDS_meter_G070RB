/**
 * @brief  The application entry point.
 * @retval int
 */
int main_comments(void) {
	while (1) {
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
//		if ((uwTick - tim_ec) > interval_ec) {
		if (1) {
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
		}
	}
}

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
