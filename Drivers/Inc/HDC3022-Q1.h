/**
 ******************************************************************************
 * @file    HDC3022-Q1.h
 * @brief   Plik nagłówkowy dla cyfrowego czujnika wilgotności i temperatury TI HDC3022-Q1 I2C.
 *
 * @description
 * Ten sterownik dostarcza interfejsu dla czujnika cyfrowego
 * HDC3022-Q1 firmy Texas Instruments, przeznaczonego do pomiaru
 * wilgotności względnej (RH) i temperatury, z kwalifikacją do
 * zastosowań w motoryzacji.
 *
 * Wspiera on:
 * - Odczyt 16-bitowego ID producenta
 * - Konwersję surowych danych z czujnika do wartości zmiennoprzecinkowych (Wilgotność w %RH)
 ******************************************************************************
*/

#ifndef INC_HDC3022_Q1_H_
#define INC_HDC3022_Q1_H_

#include "stm32l1xx_hal.h"
#include "stm32l1xx_hal_def.h"

// DEKLARACJE FUNKCJI

/**
 ******************************************************************************
 * @brief  Odczytuje 16-bitowy identyfikator producenta (Manufacturer ID).
 * @note   Poprawna wartość zwrotna dla czujnika TI to 0x3000.
 * @param  device_id Wskaźnik do zmiennej (uint16_t), w której zostanie zapisane ID.
 * @retval HAL_StatusTypeDef: HAL_OK w przypadku sukcesu, lub kod błędu HAL.
 ******************************************************************************
 */
HAL_StatusTypeDef HDC3022_read_device_id(uint16_t* device_id);

/**
 ******************************************************************************
 * @brief  Uruchamia pojedynczy pomiar temperatury i wilgotności w trybie niskiej mocy (LPM3).
 * @note   Odczytane wartości są konwertowane na jednostki fizyczne (float).
 * @param  humidity Wskaźnik do zmiennej (float), w której zostanie zapisana wilgotność w %RH.
 * @param  temp     Wskaźnik do zmiennej (float), w której zostanie zapisana temperatura w °C.
 * @retval HAL_StatusTypeDef: HAL_OK w przypadku sukcesu, lub kod błędu HAL.
 ******************************************************************************
 */
HAL_StatusTypeDef HDC3022_read_humidity_and_temperature(float* humidity, float* temp);

/**
 ******************************************************************************
 * @brief  Wysyła komendę miękkiego resetu do czujnika.
 * @note   Powoduje zresetowanie logiki czujnika i powrót do stanu domyślnego.
 * @retval HAL_StatusTypeDef: HAL_OK w przypadku sukcesu, lub kod błędu HAL.
 ******************************************************************************
 */
HAL_StatusTypeDef HDC3022_soft_reset();

/**
 ******************************************************************************
 * @brief  Wysyła komendę I2C General Call Reset (0x06) na adres ogólny (0x00).
 * @warning Ta komenda zresetuje WSZYSTKIE urządzenia na magistrali I2C,
 * które wspierają tę funkcję, nie tylko czujnik HDC302x.
 * @retval HAL_StatusTypeDef: HAL_OK w przypadku sukcesu, lub kod błędu HAL.
 ******************************************************************************
 */
HAL_StatusTypeDef HDC3022_general_call_reset();

#endif /* INC_HDC3022_Q1_H_ */
