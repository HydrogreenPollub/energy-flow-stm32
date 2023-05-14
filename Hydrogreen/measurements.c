/**
 * @file measurements.c
 * @brief Obsluga pomiarow
 * @author Szymon Szewc
 * @date 22.10.2021
 * @todo
 * @bug
 * @copyright 2021 HYDROGREEN TEAM
 */

#include "adc.h"
#include "measurements.h"
#include "rs485.h"
#define adcChannel hadc2
#define NUM_OF_FC_CURRENT_SAMPLES 100


uint16_t adcDataToCalculate[4];
uint32_t sc_voltage_diff;
uint32_t sc_voltage_diff_value;


MEASUREMENTS VALUES;

uint32_t rawFcCurrentSensorData;

static uint32_t raw_fc_current_samples_to_average [NUM_OF_FC_CURRENT_SAMPLES];
static uint32_t raw_fc_current_average;

static void calculateValues(void);
static void averaging_Values(void);
static void get_raw_fc_current_value(void);
static void calculate_fc_current(void);
static uint8_t fill_fc_current_samples_table(void);
static void average_fc_current_measurement(void);

/*
 * Inicjalizacja adc z dma oraz stałych filtrujących dla filtru
 */
void adc_init(void)
{
  HAL_ADC_Start_DMA(&adcChannel, (uint32_t*) adcDataToCalculate, 4);
  VALUES.FC_Temp_Const = 255;
  VALUES.FC_V_Const = 99;
  VALUES.SC_C_Const = 5.76;
  VALUES.SC_V_Const = 99;


  HAL_ADC_Start(&hadc1);
  rawFcCurrentSensorData = 0;
}
/*
 * Funkcja przelicza wartosci z ADC na wartosci uzyteczne
 */
static void calculateValues(void)
{
  //Przelicza wartosci z ADC na temperature w stopniach Celsjusza, napiecie i prad
  VALUES.FC_Temp_to_average = (((4095.0f - (float) adcDataToCalculate[0]) * 20))
      / 300.0f; //zależność wyznaczona eksperymentalnie w sali
  VALUES.SC_V_to_average = (((float) adcDataToCalculate[1]) * 62.5f) / 4095.0f;
  // VALUES.SC_C_to_average = (((float) adcDataToCalculate[2]) / 4095.0f) * 5.0f;
  VALUES.SC_C_to_average = (((((float) adcDataToCalculate[3] * 5.178f) / 4095.0f))
      - 2.5f) * (25.0f / 0.625f);
  VALUES.FC_V_to_average = (((float) adcDataToCalculate[2]) * 65.2f) / 4095.0f;
}
/*
 * Funkcja filtrujaca pomiary
 */
static void averaging_Values(void)
{
  VALUES.FC_TEMP.value = (VALUES.prev_FC_Temp * (VALUES.FC_Temp_Const - 1) + VALUES.FC_Temp_to_average) / VALUES.FC_Temp_Const;

  VALUES.SC_V.value = (VALUES.prev_SC_V * (VALUES.SC_V_Const - 1) + VALUES.SC_V_to_average) / VALUES.SC_V_Const;

  VALUES.SC_C.value = (VALUES.prev_SC_C * (VALUES.SC_C_Const - 1) + VALUES.SC_C_to_average) / VALUES.SC_C_Const;

  VALUES.FC_V.value = (VALUES.prev_FC_V * (VALUES.FC_V_Const - 1) + VALUES.FC_V_to_average) / VALUES.FC_V_Const;

  VALUES.prev_FC_Temp = VALUES.FC_Temp_to_average;
  VALUES.prev_FC_V = VALUES.FC_V_to_average;
  VALUES.prev_SC_C = VALUES.SC_C_to_average;
  VALUES.prev_SC_V = VALUES.SC_V_to_average;
}

static void calculate_fc_current(void)
{
	float voltage_from_adc, temporary_fc_current;
	voltage_from_adc = (float)(raw_fc_current_average);
	voltage_from_adc = ((voltage_from_adc/4095)*5); //voltage from ADC

	temporary_fc_current = ((voltage_from_adc/5) * 96) - 48; //formula developed by Hydrogreen members

	VALUES.fc_current_value_to_average = temporary_fc_current;
}

static void get_raw_fc_current_value(void)
{
	if (HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK)
	{
			rawFcCurrentSensorData = HAL_ADC_GetValue(&hadc1);
			HAL_ADC_Start(&hadc1);
	}

}

static uint8_t fill_fc_current_samples_table(void)
{
	uint8_t cnt = 0;
	uint8_t status = 0;
	if(cnt < NUM_OF_FC_CURRENT_SAMPLES)
	{
		raw_fc_current_samples_to_average[cnt] = rawFcCurrentSensorData;
		cnt++;
		status = 0;
	}
	if(cnt >= NUM_OF_FC_CURRENT_SAMPLES)
	{
		cnt = 0;
		status = 1;
	}
	return status;
}

static void average_fc_current_measurement(void)
{
	uint32_t sum_of_samples;
	uint8_t i = 0;

	for(i = 0; i<NUM_OF_FC_CURRENT_SAMPLES; i++)
	{
		sum_of_samples += raw_fc_current_samples_to_average[i];

	}
	raw_fc_current_average = sum_of_samples/NUM_OF_FC_CURRENT_SAMPLES;
}

void adc_step()
{
  get_raw_fc_current_value();
   if (fill_fc_current_samples_table() == 1)
   {
	   average_fc_current_measurement();
	   calculate_fc_current();
   }
  calculateValues();
  averaging_Values();
  if(VALUES.SC_C.value >= 10)
    {
      emergency = 0;
    }
}

