/**
 ******************************************************************************
 * @file    TCS3720.c 
 * @brief   Plik implementacyjny dla sterownika I2C czujnika światła TCS3720.
 *
 * @description
 * Ten plik implementuje funkcje zadeklarowane w TCS3720.h.
 *
 * Obsługuje on niskopoziomową komunikację (I2C) wymaganą do
 * konfiguracji i odczytu danych z czujnika TCS3720.
 ******************************************************************************
 */
#include "TCS3720.h"
#include "stm32l1xx_hal.h"
#include "stm32l1xx_hal_def.h"
#include "stm32l1xx_hal_i2c.h"
#include <stdint.h>

// Adresy rejestrów
#define TCS3720_I2C_ADDR    (0x39 << 1) // Adres I2C urządzenia
#define TCS3720_ID          0x92        // Adres rejestru przechowującego ID urządzenia (powinno wynosić 0x82)
#define TCS3720_ENABLE_REG  0x80        // Adres rejestru włączającego poszczególne funkcje urządzenia
                                        // [PVSYNC_EN AVSYNC_EN TEN PWEN AWEN PEN AEN PON]
#define TCS3720_IPTAT_REG   0x1A        // Adres rejestru IPTAT (Current Proportional To Absolute Power)
                                        // należy ustawić w nim wartość 00011 (0x03) podczas inicjalizacji
                                        // dla poprawnego działania urządzenia
#define TCS3720_CFG1_REG    0x94        // Adres drugiego rejestru konfiguracji urządzenia
                                        // służy do konfiguracji trybu kolorów (TWO_CHANN_MODE i COLOR_MODE) oraz
                                        // włączenia wewnetrznego czujnika temperatury
                                        // TWO_CHANN_MODE - Clear+Green, Red+Blue
                                        // COLOR_MODE - Clear, Green, Red, Blue
#define TCS3720_ATIME_REG   0xE6        // Adres rejestru odpowiadającego za ustalanie
                                        // czasu całkowania czujnika (min 2.779 [ms])
#define TCS3720_FIFO_ADATA_1  0xFE      // Adres pierwszego rejestru zawierającego informacje
#define TCS3720_AGAIN_1_0     0x95      // Adres rejestru wzmocnienia dla kanałów 1 i 0
                                        // kolejny rejestr ustawia wzmocnienie kanałów 3 i 2
#define TCS3720_FIFO_CONTROL_REG 0xF1   // Rejestr kontrolny FIFO

// Stałe
#define TCS3720_INIT_DELAY  5           // Producent zaleca conajmniej 1.6 [ms]
                                        // opóźnienia od włączenia zasilania do
                                        // rozpoczęcia operacji na czujniku
                                        // dla bezpieczeństwa ustawiamy je na wieksze

// Powiązanie z handlerem I2C znajdującym się w pliku main.c
// Oznajmia że handler komunikacji I2C istnieje ale w innym pliku - w tym przypadku w main.c
extern I2C_HandleTypeDef hi2c1;

/**
******************************************************************************
* @brief  Inicjalizuje czujnik TCS3720.
* @note   Ustawia kluczowe rejestry konfiguracyjne: IPTAT (0x03),
*         CFG1 (COLOR_MODE, 0x06), ATIME (5.558ms, 0x01)
*         oraz AGAIN (wzmocnienie 4096x, 0x0C).
* @retval HAL_StatusTypeDef: HAL_OK w przypadku sukcesu, lub kod błędu HAL.
******************************************************************************
*/
HAL_StatusTypeDef TCS3720_init(){
  uint8_t tx_buffer;

  // Opóźnienie potrzebne do rozpoczęcia pracy urządzenia po zasileniu
  HAL_Delay(TCS3720_INIT_DELAY);

  // Ustawienie wartości początkowej rejestru IPTAT
  tx_buffer = 0x03;
  HAL_StatusTypeDef status = HAL_I2C_Mem_Write(
    &hi2c1,
    TCS3720_I2C_ADDR,
    TCS3720_IPTAT_REG,
    I2C_MEMADD_SIZE_8BIT,
    &tx_buffer,
    1,
    HAL_MAX_DELAY
  );

  if (status != HAL_OK) {
    return status;
  }

  // Ustawienie wartości początkowej rejestru CFG1
  // Włączenie trybu COLOR_MODE (4 channel) oraz czujnika temperatury
  tx_buffer = 0x06; // Ustawia bity 0110b odpowiedzialne za PD_MUX_SEL
  status = HAL_I2C_Mem_Write(
    &hi2c1,
    TCS3720_I2C_ADDR,
    TCS3720_CFG1_REG,
    I2C_MEMADD_SIZE_8BIT,
    &tx_buffer,
    1,
    HAL_MAX_DELAY
  );

  // Ustawienie czasu całkowania - rejestr
  tx_buffer = 0x01; // Czas całkowania x2 - 11.116 [ms]
  status = HAL_I2C_Mem_Write(
    &hi2c1,
    TCS3720_I2C_ADDR,
    TCS3720_ATIME_REG,
    I2C_MEMADD_SIZE_8BIT,
    &tx_buffer,
    1,
    HAL_MAX_DELAY
  );

  if (status != HAL_OK) {
    return status;
  }

  // Ustawienie wzmocnienia dla wszystkich kanałów
  // Przy wartości domyślnej czujnik jest przesycony
  // Ustawiamy 0x00 dla rejestru AGAIN_1_0 oraz kolejnego po nim
  // czyli rejestru AGAIN_3_2
  // PS: obecnie jest na 0x0C bo założyłem na czujnik kawał kartonu. hehe..
  uint8_t gain_tx_buffer[2] = {0x0C, 0x0C};
  status = HAL_I2C_Mem_Write(
      &hi2c1,
      TCS3720_I2C_ADDR,
      TCS3720_AGAIN_1_0,
      I2C_MEMADD_SIZE_8BIT,
      gain_tx_buffer,
      2,
      HAL_MAX_DELAY
  );

  if (status != HAL_OK) {
      return status;
  }

  return status;
}

/**
 ******************************************************************************
 * @brief  TODO: Odczytuje rejestry konfiguracyjne.
 * @note   Ta funkcja jest obecnie nie zaimplementowana i nic nie robi.
 * @retval Zawsze zwraca HAL_OK.
 ******************************************************************************
 */
HAL_StatusTypeDef TCS3720_read_cfg(){
  return HAL_ERROR;
}

/**
 ******************************************************************************
 * @brief  Odczytuje 8-bitowy identyfikator urządzenia (Device ID).
 * @note   Odczytuje 1 bajt z rejestru ID (0x92). Oczekiwana wartość to 0x82.
 * @param  device_id Wskaźnik do zmiennej (uint8_t), w której zostanie zapisane ID.
 * @retval HAL_StatusTypeDef: HAL_OK w przypadku sukcesu, lub kod błędu HAL.
 ******************************************************************************
 */
HAL_StatusTypeDef TCS3720_read_device_id(uint8_t* device_id){
    uint8_t i2c_rx_buffer;

    // Odczytaj bajt rejestru ID
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
		&hi2c1,
		TCS3720_I2C_ADDR,
		TCS3720_ID,
		I2C_MEMADD_SIZE_8BIT,
		&i2c_rx_buffer,
		1,
		HAL_MAX_DELAY
    );

    // Jeżeli udało się odczytać dane to zapisz zawartość buforu do pointera
    if (status == HAL_OK){
        *device_id = i2c_rx_buffer;
    }

    return status;
}
/**
 ******************************************************************************
 * @brief  Wykonuje pojedynczy pomiar natężenia światła (kanał Clear) na żądanie.
 * @note   Realizuje pełny cykl: wybudzenie (PON), czyszczenie FIFO,
 *         aktywacja pomiaru (AEN), odczekanie, odczyt 16-bitowej
 *         wartości z FIFO (0xFE, 0xFF), a następnie uśpienie czujnika (PON=0).
 * @param  light_intensity Wskaźnik do zmiennej (uint16_t), w której zostanie
 *         zapisana surowa 16-bitowa wartość natężenia światła.
 * @retval HAL_StatusTypeDef: HAL_OK w przypadku sukcesu, lub kod błędu HAL.
 ******************************************************************************
 */
HAL_StatusTypeDef TCS3720_read_light_intensity(uint16_t* light_intensity){
  HAL_StatusTypeDef status;
  uint8_t tx_buffer;
  uint8_t rx_buffer[2];

  // Opóźnienie potrzebne do rozpoczęcia pracy urządzenia po zasileniu
  // HAL_Delay(TCS3720_INIT_DELAY);

  // Ustawienie wartości początkowej bitu PON (ENABLE) - wybudzenie czujnika
  tx_buffer = 0x01; // Ustawia 1 bit na 1 - [PON]
  status = HAL_I2C_Mem_Write(
    &hi2c1,
    TCS3720_I2C_ADDR,
    TCS3720_ENABLE_REG,
    I2C_MEMADD_SIZE_8BIT,
    &tx_buffer,
    1,
    HAL_MAX_DELAY
  );

  if (status != HAL_OK) {
    return status;
  }

  // Opóźnienie wymagane do ustabilizowania się sensora przed pomiarem - conajmniej 100 [us]
  HAL_Delay(1);

  // Czyszczenie kolejki FIFO
  // Ustawia bity FIFO_CLR na 1
  tx_buffer = 0x04; // 100b
  status = HAL_I2C_Mem_Write(
     &hi2c1,
     TCS3720_I2C_ADDR,
     TCS3720_FIFO_CONTROL_REG, // Zapis do rejestru 0xF1
     I2C_MEMADD_SIZE_8BIT,
     &tx_buffer,
     1,
    HAL_MAX_DELAY
  );

  if (status != HAL_OK) {
    return status;
  }

  // Ustawienie wartości bitu AEN (ENABLE) - aktywuje pomiar
  // Zmiana stanu czujnika z IDLE do ALS_ACTIVE
  tx_buffer = 0x03; // Ustawia 1 i 2 bit na 1 - [PON AEN]
  status = HAL_I2C_Mem_Write(
    &hi2c1,
    TCS3720_I2C_ADDR,
    TCS3720_ENABLE_REG,
    I2C_MEMADD_SIZE_8BIT,
    &tx_buffer,
    1,
    HAL_MAX_DELAY
  );

  if (status != HAL_OK) {
    return status;
  }

  // Opóźnienie potrzebne aby czujnik mógł wykonać pomiar
  HAL_Delay(5); // Niższa nie działa

  // Odczyt danych z rejestrów FIFO_ADATA_1 i FIFO_ADATA_0
  status = HAL_I2C_Mem_Read(
		&hi2c1,
		TCS3720_I2C_ADDR,
		TCS3720_FIFO_ADATA_1, // Zacznij od (LSB)
		I2C_MEMADD_SIZE_8BIT,
		rx_buffer,
		2,
		HAL_MAX_DELAY
  );

  // Uśpienie czujnika - rejestr ENABLE ustawiony na same 0
  tx_buffer = 0x00;
  status = HAL_I2C_Mem_Write(
    &hi2c1,
    TCS3720_I2C_ADDR,
    TCS3720_ENABLE_REG,
    I2C_MEMADD_SIZE_8BIT,
    &tx_buffer,
    1,
    HAL_MAX_DELAY
  );

  if (status != HAL_OK) {
    return status;
  }

  *light_intensity = (rx_buffer[1] << 8) | rx_buffer[0];

  return HAL_OK;
}