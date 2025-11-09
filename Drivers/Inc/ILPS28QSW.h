/**
 ******************************************************************************
 * @file    ILPS28QSW.h
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
 ******************************************************************************
*/

#ifndef ILPS28QSW_H_
#define ILPS28QSW_H_

#include "stm32l1xx_hal.h"
#include "stm32l1xx_hal_def.h"

// Deklaracje funkcji

/**
 ******************************************************************************
 * @brief  Inicjuje czujnik i ustawia go w tryb pomiaru ciągłego 1 Hz.
 * @note   Zapisuje domyślną konfigurację (tryb 1 Hz) do rejestru CTRL_REG1.
 * @retval HAL_StatusTypeDef: HAL_OK w przypadku sukcesu, lub kod błędu HAL.
 ******************************************************************************
 */
HAL_StatusTypeDef ILPS28QSW_init();

/**
 ******************************************************************************
 * @brief  Odczytuje 8-bitowy identyfikator urządzenia (WHO_AM_I).
 * @note   Odczytuje rejestr o adresie 0x0F.
 * @param  device_id Wskaźnik do zmiennej (uint8_t), w której zostanie zapisane ID.
 * @retval HAL_StatusTypeDef: HAL_OK w przypadku sukcesu, lub kod błędu HAL.
 ******************************************************************************
 */
HAL_StatusTypeDef ILPS28QSW_read_who_am_i(uint8_t* device_id);

/**
 ******************************************************************************
 * @brief  Odczytuje zawartość 3 rejestrów kontrolnych (CTRL_REG1, CTRL_REG2, CTRL_REG3).
 * @note   Rozpoczyna odczyt od adresu 0x10 i czyta dane z niego oraz 2 kolejnych rejestrów.
 * @param  ctrl_reg_data Wskaźnik do 3-elementowej tablicy (uint8_t), w której zostaną zapisane dane.
 * @retval HAL_StatusTypeDef: HAL_OK w przypadku sukcesu, lub kod błędu HAL.
 ******************************************************************************
 */
HAL_StatusTypeDef ILPS28QSW_read_ctrl_regs(uint8_t* ctrl_reg_data);

/**
 ******************************************************************************
 * @brief  Odczytuje surowe ciśnienie (3 bajty) i konwertuje je na hPa (float).
 * @note   Odczytuje rejestry PRESS_OUT (od 0x28) i przelicza na wartość fizyczną.
 * @param  pressure Wskaźnik do zmiennej (float), w której zostanie zapisane ciśnienie w hPa.
 * @retval HAL_StatusTypeDef: HAL_OK w przypadku sukcesu, lub kod błędu HAL.
 ******************************************************************************
 */
HAL_StatusTypeDef ILPS28QSW_read_pressure(float* pressure);

/**
 ******************************************************************************
 * @brief  Odczytuje surową temperaturę (2 bajty) i konwertuje ją na deg C (float).
 * @note   Odczytuje rejestry TEMP_OUT (od 0x2B) i przelicza na wartość fizyczną.
 * @param  temp Wskaźnik do zmiennej (float), w której zostanie zapisana temperatura w deg C.
 * @retval HAL_StatusTypeDef: HAL_OK w przypadku sukcesu, lub kod błędu HAL.
 ******************************************************************************
 */
HAL_StatusTypeDef ILPS28QSW_read_temp(float* temp);


#endif /* ILPS28QSW_H_ */
