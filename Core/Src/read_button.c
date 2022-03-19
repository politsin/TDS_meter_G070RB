
// read_button.h

#include "main.h"

#define BUTTON_TIME        500
#define BUTTON_TIME_2      150

//////////// PTT ////////////
uint8_t flag_ptt = 1;

//////////// PA ////////////
uint8_t flag_pa = 0;
uint8_t short_state1 = 0;
uint32_t time_key1 = 0;

//////////// LNA ////////////
uint8_t flag_lna = 0;
uint8_t short_state2 = 0;
uint32_t time_key2 = 0;

//////////// MODE ////////////
uint8_t flag_mode = 0;
uint8_t short_state3 = 0;
uint32_t time_key3 = 0;
uint8_t long_state = 0;

//////////// KBRES ////////////
uint8_t flag_kbres = 0;
uint8_t short_state4 = 0;
uint32_t time_key4 = 0;


/*
void read_but()
{
	//////////////// IN_PTT /////////////////
	if(!IN_PTT)
	{
		flag_ptt = 1;
		trans_to_usart2("PTT_VKL", 7);
	}
	else
	{
		flag_ptt = 0;
		//trans_to_usart2("PTT_OTKL", 8);
	}


	//////////////// IN_PA /////////////////
	if(!IN_PA && !short_state1 && (uwTick - time_key1) > BUTTON_TIME)
	{
		short_state1 = 1;
		time_key1 = uwTick;
	}
	else if(short_state1 && IN_PA && (uwTick - time_key1) > BUTTON_TIME_2) // действие на короткое нажатие
	{
		short_state1 = 0;

		if(!flag_pa) // VKL
		{
			trans_to_usart2("VKL1", 4);
		}
		else // OTKL
		{
			trans_to_usart2("OTKL1", 5);
		}

		flag_pa = !flag_pa;

		time_key1 = uwTick;
	}


	//////////////// IN_LNA /////////////////
	if(!IN_LNA && !short_state2 && (uwTick - time_key2) > BUTTON_TIME)
	{
		short_state2 = 1;
		time_key2 = uwTick;
	}
	else if(short_state2 && IN_LNA && (uwTick - time_key2) > BUTTON_TIME_2) // действие на короткое нажатие
	{
		short_state2 = 0;

		if(!flag_lna) // VKL
		{
			trans_to_usart2("VKL2", 4);
		}
		else // OTKL
		{
			trans_to_usart2("OTKL2", 5);
		}

		flag_lna = !flag_lna;

		time_key2 = uwTick;
	}


	//////////////// IN_MODE /////////////////
	if(!IN_MODE && !short_state3 && (uwTick - time_key3) > BUTTON_TIME)
	{
		short_state3 = 1;
		long_state = 0;
		time_key3 = uwTick;
	}
	else if(!long_state && !IN_MODE && (uwTick - time_key3) > 1200) // действие на длинное нажатие
	{
		long_state = 1;
		trans_to_usart2("LONG_PRESS", 10);
	}
	else if(short_state3 && IN_MODE && (uwTick - time_key3) > BUTTON_TIME_2)
	{
		short_state3 = 0;
		time_key3 = uwTick;

		if(!long_state) // действие на короткое нажатие
		{
			trans_to_usart2("SHORT_PRESS", 11);
		}
	}


	//////////////// IN_KBRES /////////////////
	if(!IN_KBRES && !short_state4 && (uwTick - time_key4) > BUTTON_TIME)
	{
		short_state4 = 1;
		time_key4 = uwTick;
	}
	else if(short_state4 && IN_KBRES && (uwTick - time_key4) > BUTTON_TIME_2) // действие на короткое нажатие
	{
		short_state4 = 0;

		if(!flag_kbres) // VKL
		{
			trans_to_usart2("VKL4", 4);
		}
		else // OTKL
		{
			trans_to_usart2("OTKL4", 5);
		}

		flag_kbres = !flag_kbres;

		time_key4 = uwTick;
	}


	//////////////// IN_TSW /////////////////
	if(IN_TSW)
	{
		trans_to_usart2("ALARM", 5);
	}

}*/













