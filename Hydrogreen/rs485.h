/**
 * @file rs485.h
 * @brief Biblioteka do obslugi komunikacji UART <-> RS485 <-> UART
 * @author Piotr Durakiewicz
 * @date 08.12.2020
 * @todo
 * @bug
 * @copyright 2020 HYDROGREEN TEAM
 */
#pragma once

#include <stdint-gcc.h>

// ******************************************************************************************************************************************************** //

#define RS485_FLT_NONE 0x00					///< Brak bledu
#define RS485_NEW_DATA_TIMEOUT 0x11		//bylo 0x11		///< Nie otrzymano nowych dane (polaczenie zostalo zerwane)

extern uint8_t rs485_flt; ///< Zmienna przechowujaca aktualny kod bledu magistrali
extern uint8_t emergency; //Zmienna sygnalizujaca blad
extern uint32_t rejectedFramesInRow;
// ******************************************************************************************************************************************************** //

extern void rs485_init(void);///< Inicjalizacja magistrali RS-485, umiescic wewnatrz hydrogreen_init(void)
extern void rs485_step(void);///< Funkcja obslugujaca magistrale, umiescic wewnatrz hydrogreen_step(void)
uint8_t crc_calc(void);
uint8_t crc_calc_TX(void);
// ******************************************************************************************************************************************************** //

/**
 * @struct RS485_RECEIVED_VERIFIED_DATA
 * @brief Struktura zawierajaca sprawdzone otrzymane dane
 */
typedef struct
{
  ///< ELEMENTY W STRUKTURZE MUSZA BYC POSORTOWANE W PORZADKU MALEJACYM
  ///< https://www.geeksforgeeks.org/is-sizeof-for-a-struct-equal-to-the-sum-of-sizeof-of-each-member/

  uint8_t motorPWM;
  uint8_t mode;
  uint8_t scOn; //Mozliwe stany: 1- tranzystor moze byc otwarty otwarty/0 -tranzystor zamkniety
  uint8_t emergencyScenario;
} RS485_RECEIVED_VERIFIED_DATA;
extern RS485_RECEIVED_VERIFIED_DATA RS485_RX_VERIFIED_DATA;
extern uint8_t SumaKontrolnaBoKtosMimeczyDupe;
