/**
 ******************************************************************************
 * @file    TCS3720.h
 * @brief   Plik nagłówkowy dla sterownika czujnika światła TCS3720 I2C.
 *
 * @description
 * Ten sterownik dostarcza interfejsu I2C dla cyfrowego czujnika światła
 * i koloru TCS3720 firmy ams OSRAM.
 *
 * Wspiera on:
 * - Inicjalizację czujnika (ustawienie IPTAT, trybu pracy, wzmocnienia).
 * - Odczyt 8-bitowego ID urządzenia.
 * - Wykonywanie pomiaru natężenia światła na żądanie (w trybie "single-shot").
 ******************************************************************************
 */
#ifndef TCS3720_H_
#define TCS3720_H_

#include "stm32l1xx_hal.h"
#include "stm32l1xx_hal_def.h"
#include <stdint.h>

// Deklaracje funkcji
/**
 ******************************************************************************
 * @brief  Inicjalizuje czujnik TCS3720.
 * @note   Ustawia kluczowe rejestry konfiguracyjne: IPTAT,
 * CFG1 (dla trybu 4-kanałowego COLOR_MODE), ATIME (czas integracji)
 * oraz AGAIN (wzmocnienie) na 0x01, aby uniknąć saturacji.
 * @retval HAL_StatusTypeDef: HAL_OK w przypadku sukcesu, lub kod błędu HAL.
 ******************************************************************************
 */
HAL_StatusTypeDef TCS3720_init();

/**
 ******************************************************************************
 * @brief  Odczytuje 8-bitowy identyfikator urządzenia (Device ID).
 * @note   Odczytuje rejestr ID (0x92). Oczekiwana wartość to 0x82.
 * @param  device_id Wskaźnik do zmiennej (uint8_t), w której zostanie zapisane ID.
 * @retval HAL_StatusTypeDef: HAL_OK w przypadku sukcesu, lub kod błędu HAL.
 ******************************************************************************
 */
HAL_StatusTypeDef TCS3720_read_device_id(uint8_t* device_id);

/**
 ******************************************************************************
 * @brief  Wykonuje pojedynczy pomiar natężenia światła (z kanału Clear) na żądanie.
 * @note   Funkcja realizuje pełny cykl: wybudzenie (PON=1), czyszczenie FIFO,
 * aktywacja pomiaru (AEN=1), odczekanie na wynik, odczyt 16-bitowej
 * wartości z FIFO (0xFE, 0xFF), a następnie uśpienie czujnika (PON=0).
 * @param  light_intensity Wskaźnik do zmiennej (uint16_t), w której zostanie
 * zapisana surowa 16-bitowa wartość natężenia światła.
 * @retval HAL_StatusTypeDef: HAL_OK w przypadku sukcesu, lub kod błędu HAL.
 ******************************************************************************
 */
HAL_StatusTypeDef TCS3720_read_light_intensity(uint16_t* light_intensity);

#endif /* ILPS28QSW_H_ */
