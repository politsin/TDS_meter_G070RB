/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void trans_to_usart2(char *buf, uint8_t len);

void read_but();
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CRISTAL_Pin GPIO_PIN_0
#define CRISTAL_GPIO_Port GPIOF
#define CRISTALF1_Pin GPIO_PIN_1
#define CRISTALF1_GPIO_Port GPIOF
#define ADC_EC_Pin GPIO_PIN_0
#define ADC_EC_GPIO_Port GPIOA
#define ADC_NTC_Pin GPIO_PIN_1
#define ADC_NTC_GPIO_Port GPIOA
#define ADC_25_Pin GPIO_PIN_2
#define ADC_25_GPIO_Port GPIOA
#define ADC_33_Pin GPIO_PIN_3
#define ADC_33_GPIO_Port GPIOA
#define ADC_LIPo_Pin GPIO_PIN_4
#define ADC_LIPo_GPIO_Port GPIOA
#define DS18B20_Pin GPIO_PIN_2
#define DS18B20_GPIO_Port GPIOB
#define PWR_EC_Pin GPIO_PIN_12
#define PWR_EC_GPIO_Port GPIOB
#define PWR_V25_Pin GPIO_PIN_14
#define PWR_V25_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_9
#define LED1_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_6
#define LED2_GPIO_Port GPIOC
#define PWR_BTN_Pin GPIO_PIN_7
#define PWR_BTN_GPIO_Port GPIOC
#define MAX7219M_CS_Pin GPIO_PIN_6
#define MAX7219M_CS_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define LED1_ON     		 (LED1_GPIO_Port->BRR = (uint32_t)LED1_Pin)  // LOW
#define LED1_OFF      	     (LED1_GPIO_Port->BSRR = (uint32_t)LED1_Pin) // HIGH

#define LED2_ON     		 (LED2_GPIO_Port->BRR = (uint32_t)LED2_Pin)
#define LED2_OFF      	     (LED2_GPIO_Port->BSRR = (uint32_t)LED2_Pin)

#define PWR_V25_ON      	 (PWR_V25_GPIO_Port->BRR = (uint32_t)PWR_V25_Pin)
#define PWR_V25_OFF     	 (PWR_V25_GPIO_Port->BSRR = (uint32_t)PWR_V25_Pin)

#define PWR_EC_ON      	 	 (PWR_EC_GPIO_Port->BSRR = (uint32_t)PWR_EC_Pin) // HIGH
#define PWR_EC_OFF     	 	 (PWR_EC_GPIO_Port->BRR = (uint32_t)PWR_EC_Pin)  // LOW

/*#define LED1_ON     		 (HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET))
#define LED1_OFF      	     (HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET))

#define LED2_ON     		 (HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET))
#define LED2_OFF      	     (HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET))

#define PWR_V25_ON      	 (HAL_GPIO_WritePin(GPIOB, PWR_V25_Pin, GPIO_PIN_RESET))
#define PWR_V25_OFF     	 (HAL_GPIO_WritePin(GPIOB, PWR_V25_Pin, GPIO_PIN_SET))*/




/////////////// В�?Д�?МОСТЬ //////////////
#define DEBUG_USART1       1

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
