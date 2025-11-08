#include "tmp119.h"

// Definicje adresów
#define TMP119_I2C_ADDR	(0x48 << 1)	// Adres I2C czujnika temepratury
#define TMP119_REG_TEMP	0x00		// Adres rejestru przechowującym wartość temperatury
#define TMP119_DEV_ID	0x0F		// Adres rejestru zawierającego


// Powiązanie z handlerem I2C znajdującym się w pliku main.c
// Żeby nie zgłupiał bo nie wie gdzie co jest
extern I2C_HandleTypeDef hi2c1;

HAL_StatusTypeDef tmp119_read_device_id(uint16_t* device_id){
    uint8_t i2c_rx_buffer[2]; // Bufor wejściowy dla komunikacji I2C

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
    if (status == HAL_OK){
        *device_id = (uint16_t)(i2c_rx_buffer[0] << 8) | i2c_rx_buffer[1];
    }

    return status;
}

HAL_StatusTypeDef tmp119_read_temperature(float* temperature_c){
    uint8_t i2c_rx_buffer[2];

    // Odczytanie 2 bitów z rejestru Device_ID poprzez I2C - odczyt zapisany w buforze
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
		&hi2c1,
		TMP119_I2C_ADDR,
		TMP119_REG_TEMP,
		I2C_MEMADD_SIZE_8BIT,
		i2c_rx_buffer,
		2,
		HAL_MAX_DELAY
	);

    if (status == HAL_OK){
        int16_t raw_temp = (int16_t)(i2c_rx_buffer[0] << 8) | i2c_rx_buffer[1];

        // Konwersja do stopni celciusza - zaciągnięta z noty katalogowej TMP119
        *temperature_c = raw_temp * 0.0078125f;
    }

    return status;
}

