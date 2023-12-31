/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stm32f3xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOF
#define MOTOR_PWM_Pin GPIO_PIN_0
#define MOTOR_PWM_GPIO_Port GPIOA
#define FC_CURRENT_SENSOR_Pin GPIO_PIN_1
#define FC_CURRENT_SENSOR_GPIO_Port GPIOA
#define RS_TX_Pin GPIO_PIN_2
#define RS_TX_GPIO_Port GPIOA
#define RS_RX_Pin GPIO_PIN_3
#define RS_RX_GPIO_Port GPIOA
#define FC_TEMPERATURE_PIN_Pin GPIO_PIN_4
#define FC_TEMPERATURE_PIN_GPIO_Port GPIOA
#define SC_VOLTAGE_PIN_Pin GPIO_PIN_5
#define SC_VOLTAGE_PIN_GPIO_Port GPIOA
#define FC_VOLTAGE_PIN_Pin GPIO_PIN_6
#define FC_VOLTAGE_PIN_GPIO_Port GPIOA
#define SC_CURRENT_SENSOR_PIN_Pin GPIO_PIN_7
#define SC_CURRENT_SENSOR_PIN_GPIO_Port GPIOA
#define FC_MOSFET_PWM_TIM_Pin GPIO_PIN_0
#define FC_MOSFET_PWM_TIM_GPIO_Port GPIOB
#define FAN_2_GPIO_Pin GPIO_PIN_1
#define FAN_2_GPIO_GPIO_Port GPIOB
#define FAN_2_GPIO_EXTI_IRQn EXTI1_IRQn
#define FAN_PWM_TIM_Pin GPIO_PIN_8
#define FAN_PWM_TIM_GPIO_Port GPIOA
#define SC_ON_Pin GPIO_PIN_9
#define SC_ON_GPIO_Port GPIOA
#define PURGING_Pin GPIO_PIN_10
#define PURGING_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define FAN_1_GPIO_Pin GPIO_PIN_6
#define FAN_1_GPIO_GPIO_Port GPIOB
#define FAN_1_GPIO_EXTI_IRQn EXTI9_5_IRQn
#define FC_DECHARGING_Pin GPIO_PIN_7
#define FC_DECHARGING_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
