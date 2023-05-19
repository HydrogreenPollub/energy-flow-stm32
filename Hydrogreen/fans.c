/*
 * fans.c
 *
 *  Created on: 14 kwi 2021
 *      Author: User
 */
#include "fans.h"
#include "gpio.h"
#include "tim.h"
#include "rs485.h"

#define fan_1_PIN GPIO_PIN_6
#define fan_2_PIN GPIO_PIN_1
#define fan_1_PORT GPIOB
#define fan_2_PORT GPIOB

static void doCalculations(void);
static void fans_update_emergency(void);

static uint8_t sampleTime = 0;
static uint8_t pulses_1 = 0;
static uint8_t pulses_2 = 0;

uint8_t fans_error = 0;

FC_FANS FANS;

void fans_init()
{
  HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, &FANS.controlValue, 1);
  FANS.rpm_1 = 1;
  FANS.fanstors485.rpm_2 = 1;
}
void fansStep(void)
{
  doCalculations();
  fans_update_emergency();
}

void doCalculations(void)   //co 150 ms
{
  sampleTime++;
  if (sampleTime >= 150)	//oblicz predkosc obrotowa w rpm
    {
      FANS.rpm_1 = pulses_1 * 200;
      FANS.fanstors485.rpm_2 = pulses_2 * 200; //200 dla 150 ms okresu
      pulses_1 = 0;		//zeruj impulsy
      pulses_2 = 0;
      sampleTime = 0;	//zeruj czas pomiaru
    }
}

static void fans_update_emergency(void)
{
  if (FANS.rpm_1 == 0 || FANS.fanstors485.rpm_2 == 0)
    {
      fans_error = 1;
    }
  else
    {
      fans_error = 0;
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  //Zlicza impulsy poszczegolnych wentylatorow
  switch (GPIO_Pin)
    {
    case fan_1_PIN:
      pulses_1++;
    break;
    case fan_2_PIN:
      pulses_2++;
    break;
    default:
    break;
    }
}

