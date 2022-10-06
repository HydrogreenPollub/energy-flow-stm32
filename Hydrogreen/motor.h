/*
 * motor.h
 *
 *  Created on: May 2, 2021
 *      Author: User
 */

#pragma once

#include <stdint-gcc.h>

extern uint32_t motorPWM;
extern uint32_t motorPWMconst;

void motor_init(void);
void motor_set_PWM(void);
