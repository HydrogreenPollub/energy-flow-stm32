/**
 * @file measurements.c
 * @brief Sterownik przeplywu energii
 * @author Szymon Szewc
 * @date 22.10.2021
 * @todo
 * @bug
 * @copyright 2021 HYDROGREEN TEAM
 */

#include "Energy_flow.h"
#include "gpio.h"
#include "tim.h"
#include "rs485.h"
#include "fans.h"
#include "pidController.h"
#include "measurements.h"

uint8_t dupa = 2;
EnergyFlow hydros;
PID_struct SC_C_regulator;
//CurrentRegulator SC_C_regulator;
uint32_t dupaPWM = 0;
uint8_t dupaStanSC = 0;

static void energyFlow();
static void energyflow_mode0(void);
static void energyflow_preapre_to_race(void);
static void energyflow_race(void);
static void energyflow_end_race(void);
static void energyflow_emergency(void);
static void SC_State(uint8_t state);
static void SC_Set_charging();
static void FC_Decharching(uint8_t state);
static void FC_to_SC_Current_regulator(uint8_t current);

void energyflow_init(void)
{
  hydros.HydrosSC.MaxSCCurrent = 10;
  hydros.HydrosSC.MaxSCVoltage = 37;
  hydros.HydrosSC.MinSCVoltage = 30;
  if (VALUES.SC_V.value >= hydros.HydrosSC.MaxSCVoltage)
    {
      hydros.HydrosSC.SCstatus = CHARGED;
    }
  else if (VALUES.SC_V.value <= hydros.HydrosSC.MinSCVoltage)
    {
      hydros.HydrosSC.SCstatus = DECHARGED;
    }
  else if (VALUES.SC_V.value > hydros.HydrosSC.MinSCVoltage)
    {
      hydros.HydrosSC.SCstatus = NOTFULLCHARGE;
    }
  hydros.HydrosFC.MinFCVoltage = 33;
  FC_T_PID.setValue = 80;
  hydros.charging = 0;
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_GPIO_WritePin(SC_ON_GPIO_Port, SC_ON_Pin, RESET);
  HAL_GPIO_WritePin(FC_DECHARGING_GPIO_Port, FC_DECHARGING_Pin, RESET);
  SC_C_regulator.setValue = 8.6;
  SC_C_regulator.measurement = 0;
  SC_C_regulator.prevMeasurement = 0;
  SC_C_regulator.error = 0;
  SC_C_regulator.iError = 0;
  SC_C_regulator.dError = 0;
  SC_C_regulator.lastError = 0;
  SC_C_regulator.Kp = 7;
  SC_C_regulator.Ki = 3;
  SC_C_regulator.Kd = 0.02;
  SC_C_regulator.proportional = 0;
  SC_C_regulator.integrator = 1;
  SC_C_regulator.differentator = 0;
  SC_C_regulator.integratorMax = 20;
  SC_C_regulator.integratorMin = 0;
  SC_C_regulator.controlMax = 75;
  SC_C_regulator.controlMin = 0;
  SC_C_regulator.PIDtime = 0.001;
  SC_C_regulator.PIDtimeFactor = 1;
  SC_C_regulator.controlValue = 0;
}

void energyflow_step(void)
{
  if (!rs485_flt && !emergency)
    {
      dupa = 1;
      switch (RS485_RX_VERIFIED_DATA.emergencyScenario)
	{
	case 0:
	  energyFlow();
	break;
	case 1:
	  //energyflow_emergency();
	  energyFlow();

	break;
	default:
	  //energyflow_emergency();
	  energyFlow();

	break;
	}
    }
  else
    {
      energyflow_emergency();
    }
}

static void energyFlow()
{

  switch (RS485_RX_VERIFIED_DATA.mode)
    {
    case 0:
      energyflow_mode0();
      dupa = 2;
    break;

    case 1:
      energyflow_preapre_to_race();
    break;

    case 2:
      energyflow_race();
    break;

    case 3:
      energyflow_end_race();
    break;

    default:
      energyflow_emergency();
    break;
    }
}

static void energyflow_mode0()
{
  FC_T_PID.setValue = 50;

  switch (RS485_RX_VERIFIED_DATA.scOn)
    {
    case 0:
      SC_State(0);
      //hydros.charging = 0;
      FC_to_SC_Current_regulator(3);
      dupaPWM = hydros.charging;
      FC_Decharching(0);
    break;
    case 1:
      SC_State(1);
      hydros.charging = 0;
      FC_to_SC_Current_regulator(3);
      FC_Decharching(0);
    break;
    default:
      energyflow_emergency();
    break;
    }
}

static void energyflow_preapre_to_race()
{
  //FC_T_PID.setValue = 10;
  FC_T_PID.setValue = 50;
  hydros.charging = 0;
  SC_State(0);
  FC_to_SC_Current_regulator(0);
  //SC_Set_charging(hydros.charging);
  FC_Decharching(1);
}

static void energyflow_race()
{
  //FC_T_PID.setValue = 10;
  FC_T_PID.setValue = 50;
  FC_Decharching(0);
  //RS485_RX_VERIFIED_DATA.scOn = 0;
  switch (RS485_RX_VERIFIED_DATA.scOn)
    {
    case 0:
      SC_State(0);
      if (VALUES.SC_V.value <= 48.5)
	{
	  FC_to_SC_Current_regulator(5);
	}
      else
	{
	  FC_to_SC_Current_regulator(0);
	}
    break;
    case 1:
      SC_State(1);
      if (VALUES.SC_V.value <= 48.5)
	{
	  FC_to_SC_Current_regulator(5);
	}
      else
	{
	  FC_to_SC_Current_regulator(0);
	}
    break;
    default:
      energyflow_emergency();
    break;
    }
}

static void energyflow_end_race()
{
  hydros.charging = 0;
  SC_Set_charging(hydros.charging);
  SC_State(0);
  FC_Decharching(0);
}

static void SC_State(uint8_t state)
{
  switch (state)
    {
    case 0:
      HAL_GPIO_WritePin(SC_ON_GPIO_Port, SC_ON_Pin, RESET);
      dupaStanSC = 0;
    break;
    case 1:
      dupaStanSC = 1;
      HAL_GPIO_WritePin(SC_ON_GPIO_Port, SC_ON_Pin, SET);
    break;
    default:
      HAL_GPIO_WritePin(SC_ON_GPIO_Port, SC_ON_Pin, RESET);
      dupaStanSC = 0;
    break;
    }
}

static void SC_Set_charging(uint32_t charging)
{
  TIM3->CCR3 = hydros.charging;
  dupaPWM = hydros.charging;
}

static void FC_Decharching(uint8_t state)
{
  switch (state)
    {
    case 0:
      HAL_GPIO_WritePin(PURGING_GPIO_Port, PURGING_Pin, RESET);
    break;
    case 1:
      HAL_GPIO_WritePin(PURGING_GPIO_Port, PURGING_Pin, SET);
    break;
    default:
      energyflow_emergency();
    break;
    }
}

static void energyflow_emergency()
{
  hydros.charging = 0;
  SC_Set_charging(hydros.charging);
  SC_State(0);
  dupa = 0;
  switch (RS485_RX_VERIFIED_DATA.mode)
    {
    case 1:
      //FC_Decharching(1);
      FC_Decharching(0);
    break;
    default:
      FC_Decharching(0);
    break;
    }
}

static void FC_to_SC_Current_regulator(uint8_t current)
{
  static uint8_t time = 0;
  if (RS485_RX_VERIFIED_DATA.motorPWM == 0)
    {
      if (time >= SC_C_regulator.PIDtimeFactor)
	{
	  SC_C_regulator.setValue = current;
	  SC_C_regulator.measurement = VALUES.SC_C.value;

	  /*
	   * część proporcjonalna
	   */

	  SC_C_regulator.error = SC_C_regulator.setValue
	      - SC_C_regulator.measurement;
	  SC_C_regulator.proportional = SC_C_regulator.error
	      * SC_C_regulator.Kp;

	  /*
	   * część całkująca
	   */

	  SC_C_regulator.iError = SC_C_regulator.iError
	      + SC_C_regulator.PIDtime * SC_C_regulator.PIDtimeFactor * 0.5f
		  * (SC_C_regulator.error + SC_C_regulator.lastError); //Metoda trapezów suma dwóch następnych błędów podzielona na 2 pomnożona razy czas w [s] (wysokość trapezu)
	  if (SC_C_regulator.iError >= SC_C_regulator.integratorMax)
	    {
	      SC_C_regulator.iError = SC_C_regulator.integratorMax;
	    }
	  else if (SC_C_regulator.iError <= SC_C_regulator.integratorMin)
	    {
	      SC_C_regulator.iError = SC_C_regulator.integratorMin;
	    }
	  SC_C_regulator.integrator = SC_C_regulator.iError * SC_C_regulator.Ki;

	  /*
	   * część różniczkująca
	   */

	  SC_C_regulator.dError = (SC_C_regulator.measurement
	      - SC_C_regulator.prevMeasurement)
	      / (SC_C_regulator.PIDtime * SC_C_regulator.PIDtimeFactor);
	  SC_C_regulator.differentator = SC_C_regulator.dError
	      * SC_C_regulator.Kd;

	  /*
	   * wartość sterująca
	   */

	  SC_C_regulator.controlValue = SC_C_regulator.proportional
	      + SC_C_regulator.integrator + SC_C_regulator.differentator;
	  if (SC_C_regulator.controlValue >= SC_C_regulator.controlMax)
	    {
	      SC_C_regulator.controlValue = SC_C_regulator.controlMax;
	    }
	  else if (SC_C_regulator.controlValue <= SC_C_regulator.controlMin)
	    {
	      SC_C_regulator.controlValue = SC_C_regulator.controlMin;
	    }

	  /*
	   * Przepisanie
	   */

	  ;
	  SC_C_regulator.lastError = SC_C_regulator.error;
	  SC_C_regulator.prevMeasurement = SC_C_regulator.measurement;
	  if (!RS485_RX_VERIFIED_DATA.emergencyScenario)
	    {
	      hydros.charging = SC_C_regulator.controlValue;
	      dupaPWM = hydros.charging;
	      SC_Set_charging(hydros.charging);
	      if (VALUES.SC_C.value >= 40)
		{
		  SC_Set_charging(100);
		}
	    }
	  else
	    {
	      hydros.charging = 0;
	      SC_Set_charging(0);
	      dupaPWM = 0;
	    }
	  time = 0;
	}
      else
	{
	  time++;
	}
    }
  else
    {
      SC_Set_charging(100);
    }
}
