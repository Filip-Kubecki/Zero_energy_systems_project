/*
 * HDC3022-Q1.h
 *
 * @brief   Plik nagłówkowy dla cyfrowego czujnika wilgotności i temperatury TI HDC302x-Q1 I2C.
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
*/

#ifndef INC_HDC3022_Q1_H_
#define INC_HDC3022_Q1_H_

#include "stm32l1xx_hal.h"
#include "stm32l1xx_hal_def.h"

// DEKLARACJE FUNKCJI

// Odczytaj identyfikator producenta - powinien wynosić 0x3000
HAL_StatusTypeDef HDC3022_read_device_id(uint16_t* device_id);

// Odczytaj wilgotność w trybie on demand LSB3 (najniższy pobór energii)
HAL_StatusTypeDef HDC3022_read_humidity_and_temperature(float* humidity, float* temp);

// Soft reset
HAL_StatusTypeDef HDC3022_soft_reset();

#endif /* INC_HDC3022_Q1_H_ */
