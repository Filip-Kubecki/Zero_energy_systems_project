/**
 ******************************************************************************
 * @file    TMP119.h
 * @brief   Plik nagłówkowy dla sterownika czujnika światła TMP119 I2C.
 * @description
 * Ten sterownik dostarcza interfejsu dla cyfrowego czujnika temperatury
 * TMP119 firmy Texas Instruments.
 *
 * Wspiera on:
 * - Odczyt 16-bitowego identyfikatora urządzenia (Device ID).
 * - Odczyt temperatury i konwersję do wartości zmiennoprzecinkowej (float) w °C.
 ******************************************************************************
 */
#ifndef SRC_TMP119_H_
#define SRC_TMP119_H_

#include "stm32l1xx_hal.h"
#include "stm32l1xx_hal_def.h"
#include <sys/types.h>

// DEKLARACJE FUNKCJI

/**
 ******************************************************************************
 * @brief  Inicjalizuje czujnik TMP119, ustawiając go w tryb uśpienia (Shutdown).
 * @note   Zapisuje do rejestru konfiguracyjnego (0x01) wartość `0x0400`,
 * ustawiając bity MOD[1:0] na `01` (Shutdown Mode).
 * Jest to wymagane przed użyciem pomiarów "one-shot".
 * @retval HAL_StatusTypeDef: HAL_OK (sukces) lub kod błędu HAL.
 ******************************************************************************
 */
HAL_StatusTypeDef TMP119_init();

/**
 ******************************************************************************
 * @brief  Odczytuje 16-bitowy rejestr ID i rozdziela go na ID urządzenia oraz numer rewizji.
 * @note   Odczytuje 2 bajty z rejestru o adresie 0x0F (Device ID).
 * @param  device_id Wskaźnik do zmiennej (uint16_t), w której zostanie zapisane 12-bitowe ID urządzenia.
 * @param  rev       Wskaźnik do zmiennej (uint8_t), w której zostanie zapisany 3-bitowy numer rewizji.
 * @retval HAL_StatusTypeDef: HAL_OK w przypadku sukcesu, lub kod błędu HAL.
 ******************************************************************************
 */
HAL_StatusTypeDef TMP119_read_device_id_and_rev(uint16_t* device_id, u_int8_t*);

/**
 ******************************************************************************
 * @brief  Wykonuje pojedynczy pomiar temperatury w trybie "One-Shot".
 * @note   1. Budzi czujnik, wysyłając komendę "One-Shot" (0x0C00) do rejestru konfiguracyjnego.
 * 2. Czeka 16ms (blokująco) na zakończenie konwersji (czas konwersji to 15.5ms).
 * 3. Odczytuje 16-bitową wartość z rejestru temperatury (0x00).
 * 4. Czujnik automatycznie wraca do trybu Shutdown po pomiarze.
 * @param  temperature_c Wskaźnik do zmiennej (float), w której zostanie zapisana temperatura w °C.
 * @retval HAL_StatusTypeDef: HAL_OK (sukces) lub kod błędu HAL.
 ******************************************************************************
 */
HAL_StatusTypeDef TMP119_read_temperature(float* temperature_c);

#endif /* SRC_TMP119_H_ */
