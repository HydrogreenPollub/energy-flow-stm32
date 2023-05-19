/**
 * @file rs485.c
 * @brief Biblioteka do obslugi komunikacji UART <-> RS485 <-> UART
 * @author Piotr Durakiewicz
 * @date 08.12.2020
 * @todo
 * @bug
 * @copyright 2020 HYDROGREEN TEAM
 */

#include "rs485.h"
#include "measurements.h"
#include "fans.h"
#include "usart.h"
#include "crc.h"

// ******************************************************************************************************************************************************** //

#define UART_PORT_RS485 		huart2
#define TX_FRAME_LENGHT 		25		         	///< Dlugosc wysylanej ramki danych (z suma CRC)
#define RX_FRAME_LENGHT 		6				///< Dlugosc otrzymywanej ramki danych (z suma CRC)
#define EOT_BYTE			0x17				///< Bajt wskazujacy na koniec ramki

// ******************************************************************************************************************************************************** //

volatile static uint8_t dataFromRx[RX_FRAME_LENGHT]; ///< Tablica w ktorej zawarte sa nieprzetworzone przychodzace dane
volatile static uint16_t posInRxTab = 0; ///< Aktualna pozycja w tabeli wykorzystywanej do odbioru danych
volatile static uint8_t intRxCplt; ///< Flaga informujaca o otrzymaniu nowego bajtu (gdy 1 - otrzymanowy nowy bajt)
static uint8_t dataToTx[TX_FRAME_LENGHT]; ///< Tablica w ktorej zawarta jest ramka danych do wyslania
static uint16_t posInTxTab = 0;	///< Aktualna pozycja w tabeli wykorzystywanej do wysylania danych
uint8_t rs485_flt = RS485_NEW_DATA_TIMEOUT;///< Zmienna przechowujaca aktualny kod bledu magistrali
uint8_t emergency = 0;
uint8_t SumaKontrolnaBoKtosMimeczyDupe = 0;
uint32_t rejectedFramesInRow = 0;
// ******************************************************************************************************************************************************** //

/**
 * @struct RS485_BUFFER
 * @brief Struktura zawierajaca bufory wykorzystywane do transmisji danych
 */
typedef struct
{
  uint8_t tx;
  uint8_t rx;
} RS485_BUFFER;
static RS485_BUFFER RS485_BUFF;

RS485_RECEIVED_VERIFIED_DATA RS485_RX_VERIFIED_DATA; ///< Struktura w ktorej zawarte sa SPRAWDZONE przychodzace dane

// ******************************************************************************************************************************************************** //

static void sendData(void);
static void receiveData(void);
static void prepareNewDataToSend(void);
static void processReceivedData(void);
static void resetActData(void);

// ******************************************************************************************************************************************************** //

/**
 * @fn rs485_init(void)
 * @brief Inicjalizacja magistrali RS-485, umiescic wewnatrz hydrogreen_init(void)
 */
uint8_t crc_calc(void)
{
  uint8_t crcSumOnMCU = 0xFF;
  uint8_t xbit, data1 = 1;
#define polynomial 0x7;

  for (uint8_t l = 0; l < RX_FRAME_LENGHT - 1; l++)
    {
      uint8_t data = dataFromRx[l];
      xbit = data1 << 7;
      for (uint8_t k = sizeof(RX_FRAME_LENGHT - 1) * 8; k > 0; --k) // obliczanie wartosci najbardziej znaczacego bitu
	{
	  if (crcSumOnMCU & 0x80)    //jesli najbardziej znaczacy bit = 1
	    {
	      crcSumOnMCU = (crcSumOnMCU << 1) ^ polynomial
	      ; //XOR i leftshift
	    }
	  else
	    { //jesli = 0
	      crcSumOnMCU = (crcSumOnMCU << 1); //leftshift
	    }
	  if (data & xbit)
	    {
	      crcSumOnMCU = crcSumOnMCU ^ polynomial
	      ;
	    }
	  xbit >>= 1;
	}
    }
  return crcSumOnMCU;

}

uint8_t crc_calc_TX(void)
{
  uint8_t crcSumOnMCUTX = 0xFF;
  uint8_t xbit, data1 = 1;
#define polynomial 0x7;

  for (uint8_t l = 0; l < TX_FRAME_LENGHT - 1; l++)
    {
      uint8_t data = dataToTx[l];
      xbit = data1 << 7;
      for (uint8_t k = sizeof(TX_FRAME_LENGHT - 1) * 8; k > 0; --k) // obliczanie wartosci najbardziej znaczacego bitu
	{
	  if (crcSumOnMCUTX & 0x80)    //jesli najbardziej znaczacy bit = 1
	    {
	      crcSumOnMCUTX = (crcSumOnMCUTX << 1) ^ polynomial
	      ; //XOR i leftshift
	    }
	  else
	    { //jesli = 0
	      crcSumOnMCUTX = (crcSumOnMCUTX << 1); //leftshift
	    }
	  if (data & xbit)
	    {
	      crcSumOnMCUTX = crcSumOnMCUTX ^ polynomial
	      ;
	    }
	  xbit >>= 1;
	}
    }
  return crcSumOnMCUTX;

}
void rs485_init(void)
{
  HAL_UART_Receive_DMA(&UART_PORT_RS485, &RS485_BUFF.rx, 1); //Rozpocznij nasluchiwanie
  prepareNewDataToSend();			//Przygotuj nowy pakiet danych
}

/**
 * @fn rs485_step(void)
 * @brief Funkcja obslugujaca magistrale, umiescic wewnatrz hydrogreen_step(void)
 */
void rs485_step(void)
{
  receiveData();
  sendData();
}

/**
 * @fn sendData(void)
 * @brief Funkcja ktorej zadaniem jest obsluga linii TX, powinna zostac umieszczona w wewnatrz rs485_step()
 */
static void sendData(void)
{

  static uint16_t cntEndOfTxTick = 0;//Zmienna wykorzystywana do odliczenia czasu wskazujacego na koniec transmisji

  //Sprawdz czy wyslano cala ramke danych
  if (posInTxTab < TX_FRAME_LENGHT)
    {
      //Nie, wysylaj dalej
      RS485_BUFF.tx = dataToTx[posInTxTab];

      //Na czas wysylania danych wylacz przerwania
      __disable_irq();
      HAL_UART_Transmit(&UART_PORT_RS485, &RS485_BUFF.tx, 1, HAL_MAX_DELAY);
      __enable_irq();
      posInTxTab++;
    }
  else if (cntEndOfTxTick < TX_FRAME_LENGHT)
    {
      //Cala ramka danych zostala wyslana, zacznij odliczac "czas przerwy" pomiedzy przeslaniem kolejnej ramki
      cntEndOfTxTick++;
    }
  else
    {
      //Przygotuj nowe dane do wysylki
      cntEndOfTxTick = 0;
      posInTxTab = 0;

      prepareNewDataToSend();
    }
}

/**
 * @fn receiveData(void)
 * @brief Funkcja ktorej zadaniem jest obsluga linii RX, umiescic wewnatrz rs485_step()
 */
static void receiveData(void)
{

  //static uint32_t rejectedFramesInRow=0;	//Zmienna przechowujaca liczbe straconych ramek z rzedu
  static uint32_t cntEndOfRxTick = 0;//Zmienna wykorzystywana do odliczenia czasu wskazujacego na koniec transmisji

  //Sprawdz czy otrzymano nowe dane
  if (!intRxCplt)
    {
      //Nie otrzymano nowych danych, zacznij odliczac czas
      cntEndOfRxTick++;
    }
  else if (intRxCplt)
    {
      //Nowe dane zostaly otrzymane, zeruj flage informujaca o zakonczeniu transmisji
      intRxCplt = 0;
    }

  //Sprawdz czy minal juz czas wynoszacy RX_FRAME_LENGHT
  if (cntEndOfRxTick > RX_FRAME_LENGHT)
    {
      //Na czas przetwarzania danych wylacz przerwania
      __disable_irq();

      //Czas minal, oznacza to koniec ramki
      cntEndOfRxTick = 0;
      posInRxTab = 0;

      //OBLICZ SUME KONTROLNA
      //uint8_t crcSumOnMCU = HAL_CRC_Calculate(&hcrc, (uint32_t*)dataFromRx, (RX_FRAME_LENGHT - 2));
      //  uint8_t crcSumOnMCU = crc_calc();

      //Sprawdz czy sumy kontrolne oraz bajt EOT (End Of Tranmission) sie zgadzaja
      if ((dataFromRx[RX_FRAME_LENGHT - 2] == EOT_BYTE)
	  && (crc_calc() == dataFromRx[RX_FRAME_LENGHT - 1]))
	{
	  processReceivedData();
	  rs485_flt = RS485_FLT_NONE;
	  rejectedFramesInRow = 0;
	}
      else
	{
	  rejectedFramesInRow++;

	  //Jezeli odrzucono wiecej niz 50 ramek z rzedu uznaj ze tranmisja zostala zerwana
	  if (rejectedFramesInRow > 50)
	    {
	      resetActData();
	      rs485_flt = RS485_NEW_DATA_TIMEOUT;
	    }
	}
      //Wyczysc bufor odbiorczy
      for (uint8_t i = 0; i < RX_FRAME_LENGHT; i++)
	{
	  dataFromRx[i] = 0x00;
	}

      __enable_irq();
    }
  if (cntEndOfRxTick > 100 * RX_FRAME_LENGHT)
    {
      rs485_flt = RS485_NEW_DATA_TIMEOUT;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

  HAL_UART_Receive_DMA(&UART_PORT_RS485, &RS485_BUFF.rx, 1); //Ponownie rozpocznij nasluchiwanie nasluchiwanie

  intRxCplt = 1;	//Ustaw flage informujaca o otrzymaniu nowych danych

  if (posInRxTab > RX_FRAME_LENGHT)
    posInRxTab = 0;	//Zabezpieczenie przed wyjsciem poza zakres tablicy

  dataFromRx[posInRxTab] = RS485_BUFF.rx;//Przypisz otrzymany bajt do analizowanej tablicy
  posInRxTab++;
}

/**
 * @fn prepareNewDataToSend(void)
 * @brief Funkcja przygotowujaca dane do wysylki, wykorzystana wewnatrz sendData(void)
 */
static void prepareNewDataToSend(void)
{

  uint8_t j = 0;
  dataToTx[j] = emergency;
  dataToTx[++j] = VALUES.FC_V.array[0];
  dataToTx[++j] = VALUES.FC_V.array[1];
  dataToTx[++j] = VALUES.FC_V.array[2];
  dataToTx[++j] = VALUES.FC_V.array[3];
  dataToTx[++j] = VALUES.FC_TEMP.array[0];
  dataToTx[++j] = VALUES.FC_TEMP.array[1];
  dataToTx[++j] = VALUES.FC_TEMP.array[2];
  dataToTx[++j] = VALUES.FC_TEMP.array[3];
  dataToTx[++j] = VALUES.SC_C.array[0];
  dataToTx[++j] = VALUES.SC_C.array[1];
  dataToTx[++j] = VALUES.SC_C.array[2];
  dataToTx[++j] = VALUES.SC_C.array[3];
  dataToTx[++j] = VALUES.SC_V.array[0];
  dataToTx[++j] = VALUES.SC_V.array[1];
  dataToTx[++j] = VALUES.SC_V.array[2];
  dataToTx[++j] = VALUES.SC_V.array[3];
  dataToTx[++j] = FANS.fanstors485.rpmtoRS[0];
  dataToTx[++j] = FANS.fanstors485.rpmtoRS[1];
  dataToTx[++j] = VALUES.FC_CURRENT.array[0];
  dataToTx[++j] = VALUES.FC_CURRENT.array[1];
  dataToTx[++j] = VALUES.FC_CURRENT.array[2];
  dataToTx[++j] = VALUES.FC_CURRENT.array[3];
  dataToTx[++j] = EOT_BYTE;
  //OBLICZ SUME KONTROLNA
  uint8_t calculatedCrcSumOnMCU = crc_calc_TX();

  SumaKontrolnaBoKtosMimeczyDupe = crc_calc_TX();
  //Wrzuc obliczona sume kontrolna na koniec wysylanej tablicy
  dataToTx[TX_FRAME_LENGHT - 1] = calculatedCrcSumOnMCU;
}

/**
 * @fn processReveivedData()
 * @brief Funkcja przypisujaca odebrane dane do zmiennych docelowych
 */
static void processReceivedData(void)
{
  uint8_t i = 0;
  RS485_RX_VERIFIED_DATA.mode = dataFromRx[i];
  RS485_RX_VERIFIED_DATA.scOn = dataFromRx[++i];
  RS485_RX_VERIFIED_DATA.emergencyScenario = dataFromRx[++i];
  RS485_RX_VERIFIED_DATA.motorPWM = dataFromRx[++i];
}

/**
 * @fn resetActData
 * @brief Zerowanie zmiennych docelowych (odbywa sie m.in w przypadku zerwania transmisji)
 */
static void resetActData(void)
{
  RS485_RX_VERIFIED_DATA.motorPWM = 0;
  RS485_RX_VERIFIED_DATA.mode = 0;
  RS485_RX_VERIFIED_DATA.emergencyScenario = 0;
  RS485_RX_VERIFIED_DATA.scOn = 0;
}

