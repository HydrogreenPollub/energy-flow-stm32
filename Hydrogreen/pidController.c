/*
 * pidController.c
 *
 *  Created on: 22 kwi 2021
 *      Author: User
 */

#include "pidController.h"
#include "measurements.h"
#include "rs485.h"
#include "fans.h"

PID_struct FC_T_PID;

void PID_init()
{
  FC_T_PID.Kp = 3.9;
  FC_T_PID.Ki = 1.2;
  FC_T_PID.Kd = 0.0012;
  FC_T_PID.controlValue = 0;
  FC_T_PID.error = 0;
  FC_T_PID.iError = 0;
  FC_T_PID.dError = 0;
  FC_T_PID.lastError = 0;
  FC_T_PID.setValue = 50; //Temperatura w st Celsjusza
  FC_T_PID.measurement = 0;
  FC_T_PID.prevMeasurement = 0;
  FC_T_PID.proportional = 0;
  FC_T_PID.integrator = 0;
  FC_T_PID.differentator = 0;
  FC_T_PID.integratorMax = 150;
  FC_T_PID.integratorMin = -50;
  FC_T_PID.controlMax = 255;
  FC_T_PID.controlMin = 0;
  FC_T_PID.PIDtime = 0.001; //Podstawa czasu z jaką wykonuje się funkcja PIDstep() w [s]
  FC_T_PID.PIDtimeFactor = 1; //Mnożnik podstawy czasu
}
void PIDstep()
{
  static uint8_t time = 0;
  if (time >= FC_T_PID.PIDtimeFactor)
    {
      FC_T_PID.measurement = VALUES.FC_TEMP.value;

      /*
       * część proporcjonalna
       */

      FC_T_PID.error = FC_T_PID.measurement - FC_T_PID.setValue;
      FC_T_PID.proportional = FC_T_PID.error * FC_T_PID.Kp;

      /*
       * część całkująca
       */

      FC_T_PID.iError = FC_T_PID.iError
	  + FC_T_PID.PIDtime * FC_T_PID.PIDtimeFactor * 0.5f
	      * (FC_T_PID.error + FC_T_PID.lastError); //Metoda trapezów suma dwóch następnych błędów podzielona na 2 pomnożona razy czas w [s] (wysokość trapezu)
      if (FC_T_PID.iError >= FC_T_PID.integratorMax)
	{
	  FC_T_PID.iError = FC_T_PID.integratorMax;
	}
      else if (FC_T_PID.iError <= FC_T_PID.integratorMin)
	{
	  FC_T_PID.iError = FC_T_PID.integratorMin;
	}
      FC_T_PID.integrator = FC_T_PID.iError * FC_T_PID.Ki;

      /*
       * część różniczkująca
       */

      FC_T_PID.dError = (FC_T_PID.measurement - FC_T_PID.prevMeasurement)
	  / (FC_T_PID.PIDtime * FC_T_PID.PIDtimeFactor);
      FC_T_PID.differentator = FC_T_PID.dError * FC_T_PID.Kd;

      /*
       * wartość sterująca
       */

      FC_T_PID.controlValue = FC_T_PID.proportional + FC_T_PID.integrator
	  + FC_T_PID.differentator;
      if (FC_T_PID.controlValue >= FC_T_PID.controlMax)
	{
	  FC_T_PID.controlValue = FC_T_PID.controlMax;
	}
      else if (FC_T_PID.controlValue <= FC_T_PID.controlMin)
	{
	  FC_T_PID.controlValue = FC_T_PID.controlMin;
	}

      /*
       * Przepisanie
       */

      //FANS.controlValue = FC_T_PID.controlValue;
      FANS.controlValue = 150; //FIXED VALUE, IF YOU WANT REAL REGULATOR YOU NEED CHANGE HERE
      FC_T_PID.lastError = FC_T_PID.error;
      FC_T_PID.prevMeasurement = FC_T_PID.measurement;
      time = 0;
    }
  else
    {
      time++;
    }
}
