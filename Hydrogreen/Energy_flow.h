/*
 * Energy_flow.h
 *
 *  Created on: 2 maj 2021
 *      Author: User
 */

#pragma once

#include <stdint-gcc.h>

extern uint8_t dupa;
extern uint32_t dupaPWM;
extern uint8_t dupaStanSC;
/*
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
} CurrentRegulator;
extern CurrentRegulator SC_C_regulator;
*/
typedef enum
{
  CHARGED, NOTFULLCHARGE, DECHARGED
} SuperCapacitorsStatus;

typedef struct
{
  float MaxSCVoltage;
  float MinSCVoltage;
  float MaxSCCurrent;
  SuperCapacitorsStatus SCstatus;
} SuperCapacitors;

typedef struct
{
  float MinFCVoltage;
} FuelCell;

typedef struct
{
  uint32_t charging;
  SuperCapacitors HydrosSC;
  FuelCell HydrosFC;
} EnergyFlow;
extern EnergyFlow hydros;

void energyflow_init(void);
void energyflow_step(void);

