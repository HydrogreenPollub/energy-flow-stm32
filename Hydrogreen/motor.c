/*
 * motor.c
 *
 *  Created on: May 2, 2021
 *      Author: User
 */

#include "tim.h"
#include "rs485.h"

uint32_t motorPWM = 20;
uint32_t motorPWMconst = 7;

void motor_init(void)
{
  HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, &motorPWM, 1);
  //HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, &motorPWMconst, 1);
}

void motor_set_PWM(void)
{
  //if (!rs485_flt && !emergency)
    //{
      //if (RS485_RX_VERIFIED_DATA.motorPWM >= 100)
	//{
	  //motorPWM = 20;
	//}
      //else if (RS485_RX_VERIFIED_DATA.motorPWM < 0)
	//{
	  //motorPWM = 0;
	//}
      //else
	//{


  if (!rs485_flt && !emergency)
    {
	  motorPWM = RS485_RX_VERIFIED_DATA.motorPWM;
    }
  else
    {
      motorPWM = 0;
    }	//}
   // }
 // else
    //{
      //motorPWM = RS485_RX_VERIFIED_DATA.motorPWM;
    //}
}
