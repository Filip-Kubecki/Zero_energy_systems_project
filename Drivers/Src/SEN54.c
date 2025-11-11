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

// 
HAL_StatusTypeDef SEN54_read_measurement_values(float* pm1,float* pm2_5,float* pm4, float* pm10, float* voc, float* temp){
    // Start meas -> wait for read meas flag -> read values -> stop meas
    uint8_t rx_buffer[24];
    HAL_StatusTypeDef status;

    // Wybudź czujnik i zacznij pomiar
    status = HAL_I2C_Master_Transmit(
        &hi2c2,
        SEN54_I2C_ADDR,
        (uint8_t[])SEN54_START_MEAS,
        2,
        HAL_MAX_DELAY
    );

    if (status != HAL_OK) {
        return status;
    }

    HAL_Delay(60000); // Rozgrzewanie

    HAL_Delay(1000); // Oczekiwanie na pomiar

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

    // Oczekiwanie na przygotowanie danych - conajmniej 20 [ms]
    HAL_Delay(30);

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

    // -----------------------------
    // Konwersja odebranych danych (bez sprawdzania CRC):
    *pm1 = (float)((uint16_t)(rx_buffer[0] << 8) | rx_buffer[1])/10.0f;

    *pm2_5 = (float)((uint16_t)(rx_buffer[3] << 8) | rx_buffer[4])/10.0f;

    *pm4 = (float)((uint16_t)(rx_buffer[6] << 8) | rx_buffer[7])/10.0f;

    *pm10 = (float)((uint16_t)(rx_buffer[9] << 8) | rx_buffer[10])/10.0f;

    *temp = (float)((int16_t)(rx_buffer[15] << 8) | rx_buffer[16])/200.0f;

    *voc = (float)((int16_t)(rx_buffer[18] << 8) | rx_buffer[19])/10.0f;

    // TODO: If function fail before this part the sensor will never stop measurement - redo that
    // Uśpij czujnik
    status = HAL_I2C_Master_Transmit(
        &hi2c2,
        SEN54_I2C_ADDR,
        (uint8_t[])SEN54_STOP_MEAS,
        2,
        HAL_MAX_DELAY
    );

    if (status != HAL_OK) {
        return status;
    }

    return HAL_OK;
}




// Oblicza CRC
static uint8_t calcCrc(uint8_t data[2]) {
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