/**
 ******************************************************************************
 * @file    TMP119.c
 * @brief   Plik implementacyjny dla sterownika I2C czujnika temperatury TMP119.
 *
 * @description
 * Ten plik implementuje funkcje zadeklarowane w tmp119.h.
 *
 * Obsługuje on niskopoziomową komunikację (I2C) wymaganą do
 * odczytu danych z czujnika TMP119, w tym odczyt ID urządzenia
 * oraz odczyt temperatury. Zawiera również formułę konwersji
 * surowych danych na stopnie Celsjusza.
 ******************************************************************************
 */
#include "TMP119.h"
#include "stm32l1xx_hal.h"
#include "stm32l1xx_hal_def.h"
#include "stm32l1xx_hal_i2c.h"
#include <stdint.h>

// Definicje adresów
#define TMP119_I2C_ADDR     (0x48 << 1)	// Adres I2C urządzenia
#define TMP119_REG_TEMP     0x00		// Adres rejestru przechowującym wartość temperatury
#define TMP119_CONF_REG     0x01		// Adres rejestru zawierającego konfigurację urządzenia
#define TMP119_DEV_ID	    0x0F		// Adres rejestru zawierającego


// Powiązanie z handlerem I2C znajdującym się w pliku main.c
// Żeby nie zgłupiał bo nie wie gdzie co jest
extern I2C_HandleTypeDef hi2c1;

/**
 ******************************************************************************
 * @brief  Inicjalizuje czujnik TMP119, ustawiając go w tryb uśpienia (Shutdown).
 * @note   Zapisuje do rejestru konfiguracyjnego (0x01) wartość 0x0400,
 * ustawiając bity MOD[1:0] na 01 (Shutdown Mode).
 * Jest to wymagane przed użyciem pomiarów "one-shot".
 * @retval HAL_StatusTypeDef: HAL_OK (sukces) lub kod błędu HAL.
 ******************************************************************************
 */
HAL_StatusTypeDef TMP119_init(){
    uint8_t tx_buffer[2] = {0x04, 0x00}; // Bufor wyjściowy
    HAL_StatusTypeDef status;

    // Ustawia tryb pracy sensora jako shutdown
    status = HAL_I2C_Mem_Write(
        &hi2c1,
         TMP119_I2C_ADDR, 
         TMP119_CONF_REG, 
         I2C_MEMADD_SIZE_8BIT, 
         tx_buffer, 
         2,
        HAL_MAX_DELAY
    );

    return status;
}

/**
 ******************************************************************************
 * @brief  Odczytuje 16-bitowy rejestr ID i rozdziela go na ID urządzenia oraz numer rewizji.
 * @note   Odczytuje 2 bajty z rejestru o adresie 0x0F (Device ID).
 * @param  device_id Wskaźnik do zmiennej (uint16_t), w której zostanie zapisane 12-bitowe ID urządzenia.
 * @param  rev       Wskaźnik do zmiennej (uint8_t), w której zostanie zapisany 3-bitowy numer rewizji.
 * @retval HAL_StatusTypeDef: HAL_OK w przypadku sukcesu, lub kod błędu HAL.
 ******************************************************************************
 */
HAL_StatusTypeDef TMP119_read_device_id_and_rev(uint16_t* device_id, uint8_t* rev){
    uint8_t i2c_rx_buffer[2]; // Bufor wejściowy dla komunikacji I2C
    uint16_t device_id_data;

    // Odczytanie 2 bitów z rejestru Device_ID poprzez I2C - odczyt zapisany w buforze
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
		&hi2c1,
		TMP119_I2C_ADDR,
		TMP119_DEV_ID,
		I2C_MEMADD_SIZE_8BIT,
		i2c_rx_buffer,
		2,
		HAL_MAX_DELAY
    );

    // Jeżeli udało się odczytać dane to zapisz zawartość buforu do pointera
    if (status != HAL_OK){
        return status;
    }

    // Połączenie dwóch odczytanych bajtów w 16 bitową zmienną
    device_id_data = (uint16_t)(i2c_rx_buffer[0] << 8) | i2c_rx_buffer[1];

    // Konwersja czystych danych na DID i REV
	// Usunięcie 4 pierwszych bitów - pozostawienie ID urządzenia
    *device_id  = device_id_data & 0x0FFF;

    // Usunięcie pierwszych 12 bitów - pozostawienie numeru rewizji
    // I przesunięcie ich na początek zmiennej
    *rev  = (device_id_data & 0xF000) >> 12;

    return status;
}

/**
 ******************************************************************************
 * @brief  Wykonuje pojedynczy pomiar temperatury w trybie "One-Shot".
 * @note   1. Budzi czujnik, wysyłając komendę "One-Shot" (0x0C00) do rejestru konfiguracyjnego.
 * 2. Czeka 16ms (blokująco) na zakończenie konwersji (czas konwersji to 15.5ms).
 * 3. Odczytuje 16-bitową wartość z rejestru temperatury (0x00).
 * 4. Czujnik automatycznie wraca do trybu Shutdown po pomiarze.
 * @param  temperature_c Wskaźnik do zmiennej (float), w której zostanie zapisana temperatura w °C.
 * @retval HAL_StatusTypeDef: HAL_OK (sukces) lub kod błędu HAL.
 ******************************************************************************
 */
HAL_StatusTypeDef TMP119_read_temperature(float* temperature_c){
    uint8_t rx_buffer[2];
    uint8_t tx_buffer[2] = {0x00, 0x00};
    HAL_StatusTypeDef status;

    
    tx_buffer[0] = 0x0C; // Bufor wyjściowy - komenda one shot
    // Zapisz wartość 11 do rejestru MOD - wywołuje pojedyńczy pomiar
    status = HAL_I2C_Mem_Write(
        &hi2c1,
         TMP119_I2C_ADDR, 
         TMP119_CONF_REG, 
         I2C_MEMADD_SIZE_8BIT, 
         tx_buffer, 
         2,
        HAL_MAX_DELAY
    );

    if (status != HAL_OK) {
        return status;
    }

    // TODO: make it non-blocking in the future
    // Odczekaj na konwersję
    HAL_Delay(16);

    // Odczytaj temperaturę z rejestru Temp_Result
	status = HAL_I2C_Mem_Read(
		&hi2c1,
		TMP119_I2C_ADDR,
		TMP119_REG_TEMP,
		I2C_MEMADD_SIZE_8BIT,
		rx_buffer,
		2,
		HAL_MAX_DELAY
	);

    if (status != HAL_OK) {
        return status;
    }

    if (status == HAL_OK){
        int16_t raw_temp = (int16_t)(rx_buffer[0] << 8) | rx_buffer[1];

        // Konwersja do stopni celciusza - zaciągnięta z noty katalogowej TMP119
        *temperature_c = raw_temp * 0.0078125f;
    }

    return status;
}

