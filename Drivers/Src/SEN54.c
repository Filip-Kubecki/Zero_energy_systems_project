/**
 ******************************************************************************
 * @file	 SEN54.c
 * @brief   Plik z implementacją funkcji dla czujnika SEN54
 *
 * @description
 * Ten plik implementuje funkcje zadeklarowane w pliku SEN54.h
 *
 * Obsługuje komunikację poprzez magistralę I2C potrzebną
 * do obsługi sensora. Przekazuje już przetworzone dane.
 ******************************************************************************
 */
#include "SEN54.h"
#include "stm32_hal_legacy.h"
#include "stm32l1xx_hal.h"
#include "stm32l1xx_hal_def.h"
#include "stm32l1xx_hal_i2c.h"
#include <stdint.h>
#include <string.h>

// Powiązanie z handlerem I2C znajdującym się w pliku main.c
// Oznajmia że handler komunikacji I2C istnieje ale w innym pliku - w tym przypadku w main.c
extern I2C_HandleTypeDef hi2c2;

// Definicje adresów
#define SEN54_I2C_ADDR          (0x69 << 1)		    // Adres I2C urządzenia
#define SEN54_START_MEAS        {0x00, 0x21}        // Komenda służąca do uruchomienia pomiaru
#define SEN54_STOP_MEAS         {0x01, 0x04}        // Komenda służąca do zatrzymania pomiaru
#define SEN54_READ_MEAS_VALUES  {0x03, 0xC4}        // Komenda służąca do zgłoszenia chęci odczytania wyników

/**
 ******************************************************************************
 * @brief  Uruchamia tryb pomiaru ciągłego w czujniku SEN54.
 * @note   Wysyła komendę I2C 0x0021. Czujnik przechodzi ze stanu Idle w stan Measurement.
 * @retval HAL_StatusTypeDef: HAL_OK (sukces) lub kod błędu HAL.
 ******************************************************************************
 */
HAL_StatusTypeDef SEN54_start_measurement(){
    return HAL_I2C_Master_Transmit(
        &hi2c2,
        SEN54_I2C_ADDR,
        (uint8_t[])SEN54_START_MEAS,
        2,
        HAL_MAX_DELAY
    );
}

/**
 ******************************************************************************
 * @brief  Zatrzymuje tryb pomiaru ciągłego i przełącza czujnik w tryb Idle.
 * @note   Wysyła komendę I2C 0x0104. Zatrzymuje wentylator i laser.
 * @retval HAL_StatusTypeDef: HAL_OK (sukces) lub kod błędu HAL.
 ******************************************************************************
 */
HAL_StatusTypeDef SEN54_stop_measurement(){
    // Wybudź czujnik i zacznij pomiar
    return HAL_I2C_Master_Transmit(
        &hi2c2,
        SEN54_I2C_ADDR,
        (uint8_t[])SEN54_STOP_MEAS,
        2,
        HAL_MAX_DELAY
    );
}

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
HAL_StatusTypeDef SEN54_read_measurement_values(float* pm1,float* pm2_5,float* pm4, float* pm10, float* voc, float* temp, float* humidity){
    uint8_t rx_buffer[24];      // Buffor wejściowy I2C do odczytu danych pomiarowych
    HAL_StatusTypeDef status;


    // Komenda przygotowania danych do odczytu
    status = HAL_I2C_Master_Transmit(
        &hi2c2,
        SEN54_I2C_ADDR,
        (uint8_t[])SEN54_READ_MEAS_VALUES,
        2,
        HAL_MAX_DELAY
    );

    if (status != HAL_OK) {
        return status;
    }

    // TODO: można zastąpić komendą Read Data-Ready Flag (0x0202) w pętli
    // Oczekiwanie na przygotowanie danych - conajmniej 20 [ms]
    HAL_Delay(30);

    // Odczyt 24 bajtów danych z czujnika
    status = HAL_I2C_Master_Receive(
        &hi2c2, 
        SEN54_I2C_ADDR,
        rx_buffer,
        24,
        HAL_MAX_DELAY
    );

    if (status != HAL_OK) {
        return status;
    }

    // --------------------------------------------------------------------
    // TODO: dodać sprawdzanie tego przeklętego CRC
    // Konwersja odebranych danych (bez sprawdzania CRC):
    *pm1 = (float)((uint16_t)(rx_buffer[0] << 8) | rx_buffer[1])/10.0f;

    *pm2_5 = (float)((uint16_t)(rx_buffer[3] << 8) | rx_buffer[4])/10.0f;

    *pm4 = (float)((uint16_t)(rx_buffer[6] << 8) | rx_buffer[7])/10.0f;

    *pm10 = (float)((uint16_t)(rx_buffer[9] << 8) | rx_buffer[10])/10.0f;

    *voc = (float)((int16_t)(rx_buffer[18] << 8) | rx_buffer[19])/10.0f;

    *temp = (float)((int16_t)(rx_buffer[15] << 8) | rx_buffer[16])/200.0f;

    *humidity = (float)((int16_t)(rx_buffer[12] << 8) | rx_buffer[13])/100.0f;

    return HAL_OK;
}

/**
 ******************************************************************************
 * @brief  Oblicza 8-bitową sumę kontrolną (CRC) dla 2 bajtów danych.
 * @note   Używa wielomianu 0x31 z wartością początkową 0xFF, zgodnie
 * z dokumentacją Sensirion. Jest to funkcja prywatna modułu.
 * @param  data Tablica 2-bajtowa (uint8_t) zawierająca dane (MSB, LSB).
 * @retval 8-bitowa obliczona suma kontrolna.
 ******************************************************************************
 */
static uint8_t CalcCrc(uint8_t data[2]) {
    uint8_t crc = 0xFF;
        for(int i = 0; i < 2; i++) {
            crc ^= data[i];
            for(uint8_t bit = 8; bit > 0; --bit) {
                if(crc & 0x80) {
                    crc = (crc << 1) ^ 0x31u;
                } else {
                    crc = (crc << 1);
                }
            }
        }
    return crc;
}