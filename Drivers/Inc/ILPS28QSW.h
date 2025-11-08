/*
 * ILPS28QSW.h
 * @brief   Plik nagłówkowy dla cyfrowego czujnika ciśnienia (barometru) ILPS28QSW.
 *
 * @description
 * Ten sterownik dostarcza interfejsu dla cyfrowego czujnika ciśnienia
 * ILPS28QSW firmy STMicroelectronics.
 *
 * Wspiera on:
 * - Inicjalizację czujnika (tryb pracy 1 Hz).
 * - Odczyt identyfikatora urządzenia (WHO_AM_I).
 * - Odczyt rejestrów konfiguracyjnych (CTRL_REG 1-3).
 * - Odczyt danych ciśnienia i konwersję do wartości zmiennoprzecinkowej (float).
*/

#ifndef ILPS28QSW_H_
#define ILPS28QSW_H_

#include "stm32l1xx_hal.h"

// Deklaracje funkcji

// Inicjuje czujnik w trybie pracy 1 [Hz]
HAL_StatusTypeDef ILPS28QSW_init();

// Odczyt zawartości rejestru Device_ID - zawiera rewizję oraz ID urządzenia
HAL_StatusTypeDef ILPS28QSW_read_who_am_i(uint8_t* device_id);

// Odczyt zawartości rejestrów CTRL_REG 1-3 - zawiera dane na temat działania urządzenia
HAL_StatusTypeDef ILPS28QSW_read_ctrl_regs(uint8_t* ctrl_reg_data);

// Odczyt mierzonego ciśnienia z 3 rejestrów - PRESS_OUT_H, PRESS_OUT_H, PRESS_OUT_XL
HAL_StatusTypeDef ILPS28QSW_read_pressure(float* pressure);


#endif /* ILPS28QSW_H_ */
