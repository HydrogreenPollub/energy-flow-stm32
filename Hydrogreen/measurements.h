/*
 * measurement.h
 *
 *  Created on: 11 kwi 2021
 *      Author: User
 */

#pragma once

#include <stdint-gcc.h>
#include "adc.h"

extern uint32_t sc_voltage_diff;
extern uint32_t sc_voltage_diff_value;
extern uint16_t adcDataToCalculate[4];
typedef struct
{
  union
  {
    float value;
    uint8_t array[4];
  } FC_TEMP;
  float prev_FC_Temp;
  float FC_Temp_to_average;
  uint8_t FC_Temp_Const;
  union
  {
    float value;
    uint8_t array[4];
  } FC_V;
  float prev_FC_V;
  float FC_V_to_average;
  uint8_t FC_V_Const;
  union
  {
    float value;
    uint8_t array[4];
  } SC_V;
  float prev_SC_V;
  float SC_V_to_average;
  uint8_t SC_V_Const;
  union
  {
    float value;
    uint8_t array[4];
  } SC_C;
  float prev_SC_C;
  float SC_C_to_average;
  uint8_t SC_C_Const;
} MEASUREMENTS;
extern MEASUREMENTS VALUES;
extern void adc_init(void);
extern void adc_step(void);

