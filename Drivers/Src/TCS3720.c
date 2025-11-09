/**
 * TCS3720.c
 */
#include "TCS3720.h"
#include <stdint.h>

#define TCS3720_I2C_ADDR    (0x39 << 1) // Adres I2C urządzenia
#define TCS3720_ID          0x92         // Adres I2C urządzenia

// Powiązanie z handlerem I2C znajdującym się w pliku main.c
// Oznajmia że handler komunikacji I2C istnieje ale w innym pliku - w tym przypadku w main.c
extern I2C_HandleTypeDef hi2c1;


HAL_StatusTypeDef TCS3720_read_device_id(uint8_t* device_id){
    uint8_t i2c_rx_buffer;

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
		&hi2c1,
		TCS3720_I2C_ADDR,
		TCS3720_ID,
		I2C_MEMADD_SIZE_8BIT,
		&i2c_rx_buffer,
		1,
		HAL_MAX_DELAY
    );

    // Jeżeli udało się odczytać dane to zapisz zawartość buforu do pointera
    if (status == HAL_OK){
        *device_id = i2c_rx_buffer;
    }

    return status;
}