
/**
 ******************************************************************************
 * @file    SEN54.h
 * @brief   Plik nagłówkowy dla sterownika czujnika środowiskowego Sensirion SEN54.
 *
 * @description
 * Ten plik zawiera publiczne deklaracje funkcji dla modułu SEN54.
 * Sterownik obsługuje komunikację I2C, włączanie/wyłączanie pomiarów
 * oraz odczyt wszystkich wartości (PM1.0, PM2.5, PM4, PM10, VOC, Temp, Wilgotność).
 ******************************************************************************
*/
#ifndef INC_SEN54_H_
#define INC_SEN54_H_

#include "stm32l1xx_hal.h"
#include "stm32l1xx_hal_def.h"

// DEKLARACJE FUNKCJI

/**
 ******************************************************************************
 * @brief  Uruchamia tryb pomiaru ciągłego w czujniku SEN54.
 * @note   Wysyła komendę I2C 0x0021. Po tej komendzie czujnik przechodzi
 * ze stanu Idle w stan Measurement. Włącza to całą wewnętrzną elektronikę
 * i rozpoczyna pomiar. Wymaga 1 minuty na rozgrzanie się przed przystąpieniem do odczytu.
 * @retval HAL_StatusTypeDef: HAL_OK (sukces) lub kod błędu HAL.
 ******************************************************************************
 */
HAL_StatusTypeDef SEN54_start_measurement();

/**
 ******************************************************************************
 * @brief  Zatrzymuje tryb pomiaru ciągłego i przełącza czujnik w tryb Idle.
 * @note   Wysyła komendę I2C 0x0104. Zatrzymuje wentylator i laser, oszczędzając energię.
 * @retval HAL_StatusTypeDef: HAL_OK (sukces) lub kod błędu HAL.
 ******************************************************************************
 */
HAL_StatusTypeDef SEN54_stop_measurement();

/**
 ******************************************************************************
 * @brief  Odczytuje najnowszy zestaw danych pomiarowych z czujnika (w trybie ciągłym).
 * @note   Zakłada, że czujnik jest już w trybie pomiaru. Wysyła komendę 0x03C4,
 * czeka 30ms, odczytuje 24 bajty danych i konwertuje je na wartości float.
 * @param  pm1      Wskaźnik (float) do zapisu wartości PM1.0 (μg/m³).
 * @param  pm2_5    Wskaźnik (float) do zapisu wartości PM2.5 (μg/m³).
 * @param  pm4      Wskaźnik (float) do zapisu wartości PM4.0 (μg/m³).
 * @param  pm10     Wskaźnik (float) do zapisu wartości PM10 (μg/m³).
 * @param  voc      Wskaźnik (float) do zapisu wartości VOC Index.
 * @param  temp     Wskaźnik (float) do zapisu wartości Temperatury (°C).
 * @param  humidity Wskaźnik (float) do zapisu wartości Wilgotności (%RH).
 * @retval HAL_StatusTypeDef: HAL_OK (sukces) lub kod błędu HAL.
 ******************************************************************************
 */
HAL_StatusTypeDef SEN54_read_measurement_values(float* pm1,float* pm2_5,float* pm4, float* pm10, float* voc, float* temp, float* humidity);

// TODO: Add a function to establish the VOC algorithm baseline.
// This function should run the sensor in a controlled, clean-air environment
// for a set duration (e.g., 2h) to allow the algorithm to stabilize.
//
// After the time elapses, it should read and return the 8-byte
// algorithm state using the Read/Write VOC Algorithm State (0x6181) command.

// TODO: Add a function to restore the VOC algorithm baseline state.
//
// This function should accept the 8-byte state (previously saved in non-volatile memory)
// and write it back to the sensor using the Read/Write VOC Algorithm State (0x6181) command.
//
// It must be called *after* power-on but *before* the "Start Measurement" command
// to skip the initial learning (warmup) phase.

#endif /* INC_SEN54_H_ */