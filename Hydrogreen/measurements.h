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
extern uint32_t rawFcCurrentSensorData; //Data in raw ADC

extern uint8_t fuel_cell_under_voltage;
extern uint8_t fuel_cell_over_temperature;
extern uint8_t fuel_cell_over_current;

extern uint8_t super_capacitors_over_current;
extern uint8_t super_capacitors_over_voltage;

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
  union {
	  float value;
	  uint8_t array[4];
  } SC_C_to_average;
  float prev_SC_C;

  uint8_t SC_C_Const;

  union
  {
    float value;
    uint8_t array[4];
  } FC_CURRENT;
  union
  {
	  float value;
	  uint8_t array[4];
  } fc_current_value_to_average;

} MEASUREMENTS;

extern MEASUREMENTS VALUES;
extern void adc_init(void);
extern void adc_step(void);

