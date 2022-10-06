/*
 * pidController.h
 *
 *  Created on: 22 kwi 2021
 *      Author: User
 */

#pragma once

#include <stdint-gcc.h>

void PID_init();
void PIDstep();

typedef struct
{
  float setValue;
  float measurement;
  float prevMeasurement;
  float error;
  float iError;
  float dError;
  float lastError;
  float Kp;
  float Ki;
  float Kd;
  float proportional;
  float integrator;
  float differentator;
  float integratorMax;
  float integratorMin;
  float controlMax;
  float controlMin;
  float PIDtime;
  float PIDtimeFactor;
  uint32_t controlValue;
} PID_struct;
extern PID_struct FC_T_PID;
