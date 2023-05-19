/*
 * fans.h
 *
 *  Created on: 14 kwi 2021
 *      Author: User
 */

#pragma once

#include <stdint-gcc.h>

void fansStep(void);
void fans_init(void);

extern uint8_t fans_error;

typedef struct
{
  union
  {
    uint16_t rpm_2;
    uint8_t rpmtoRS[2];
  } fanstors485;
  uint16_t rpm_1;
  uint32_t controlValue;
} FC_FANS;

extern FC_FANS FANS;

