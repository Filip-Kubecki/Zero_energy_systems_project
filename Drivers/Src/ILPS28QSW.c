/**
 ******************************************************************************
 * @file	ILPS28QSW.c
 * @brief   Plik implementacyjny dla sterownika I2C czujnika ciśnienia ILPS28QSW.
 *
 * @description
 * Ten plik implementuje funkcje zadeklarowane w ILPS28QSW.h.
 *
 * Obsługuje on niskopoziomową komunikację (I2C) wymaganą do
 * konfiguracji i odczytu danych z czujnika ILPS28QSW.
 *
 * Plik ten zawiera logikę do odczytu wielobajtowych rejestrów ciśnienia
 * oraz formuły do konwersji surowych danych (LSB) na fizyczną
 * wartość ciśnienia wyrażoną w hPa (jako float).
 ******************************************************************************
 */
#include "ILPS28QSW.h"
#include <stdint.h>

#define ILPS28QSW_I2C_ADDR				(0x5C << 1)	// Adres I2C urządzenia
#define ILPS28QSW_
#define ILPS28QSW_WHO_AM_I				0x0F		// Adres rejestru zawierającego numer ID urządzenia (powinno wynosić)
#define ILPS28QSW_CTRL_REG_START		0x10		// Adres rejestru CTRL_REG1 - kolejne 3 rejestry to rejestry CTRL1-3
#define ILPS28QSW_PRESS_OUT_REG_START	0x28		// Adres pierwszego rejestru zawierającego odczyt ciśnienia
													// kolejne 2 rejestry zawierają kolejne części odczytu (wyższe bity)
													// [PRESS_OUT_H, PRESS_OUT_L, PRESS_OUT_XL]
#define ILPS28QSW_TEMP_OUT_REGS			0x2B		// Adres pierwszego rejestru zawierającego odczyt temperatury
													// kolejny rejestr zawiera kolejną część odczytu
													// [TEMP_OUT_L, TEMP_OUT_H]

// Powiązanie z handlerem I2C znajdującym się w pliku main.c
// Oznajmia że handler komunikacji I2C istnieje ale w innym pliku - w tym przypadku w main.c
extern I2C_HandleTypeDef hi2c1;

/**
 ******************************************************************************
 * @brief  Inicjalizuje czujnik i ustawia go w tryb pomiaru ciągłego z częstotliwością 1 Hz.
 * @note   Zapisuje wartość 0x10 (tryb 1 Hz) do rejestru CTRL_REG1 (0x10).
 * @retval HAL_StatusTypeDef: HAL_OK w przypadku sukcesu, lub kod błędu HAL.
 ******************************************************************************
 */
HAL_StatusTypeDef ILPS28QSW_init(){
    uint8_t tx_buffer = 0x10;

	// Zapis danych do rejestru - ustalenie trybu pracy
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(
		&hi2c1,
		ILPS28QSW_I2C_ADDR,
		ILPS28QSW_CTRL_REG_START, // Addres rejestru CTRL_REG1
		I2C_MEMADD_SIZE_8BIT,
		&tx_buffer,
		1,
		HAL_MAX_DELAY
	);

    return status;
}

/**
 ******************************************************************************
 * @brief  Odczytuje 8-bitowy identyfikator urządzenia (WHO_AM_I).
 * @note   Odczytuje rejestr o adresie 0x0F.
 * @param  device_id Wskaźnik do zmiennej (uint8_t), w której zostanie zapisane ID.
 * @retval HAL_StatusTypeDef: HAL_OK w przypadku sukcesu, lub kod błędu HAL.
 ******************************************************************************
 */
HAL_StatusTypeDef ILPS28QSW_read_who_am_i(uint8_t* device_id){
    uint8_t i2c_rx_buffer; // Bufor wejściowy dla komunikacji I2C - tu zapisane są dane

    // Odczytanie 1 bitu z rejestru WHO_AM_I poprzez I2C - odczyt zapisany w buforze
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
		&hi2c1,
		ILPS28QSW_I2C_ADDR,
		ILPS28QSW_WHO_AM_I,
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

/**
 ******************************************************************************
 * @brief  Odczytuje zawartość 3 rejestrów kontrolnych (CTRL_REG1, 2, 3).
 * @note   Rozpoczyna odczyt od adresu CTRL_REG_START (0x10) i czyta 3 kolejne bajty.
 * @param  ctrl_reg_data Wskaźnik do 3-elementowej tablicy (uint8_t), w której zostaną zapisane dane.
 * @retval HAL_StatusTypeDef: HAL_OK w przypadku sukcesu, lub kod błędu HAL.
 ******************************************************************************
 */
HAL_StatusTypeDef ILPS28QSW_read_ctrl_regs(uint8_t* ctrl_reg_data){
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
	        &hi2c1,
	        ILPS28QSW_I2C_ADDR,
			ILPS28QSW_CTRL_REG_START,
	        I2C_MEMADD_SIZE_8BIT,
			ctrl_reg_data,
	        3,
	        HAL_MAX_DELAY
	    );

	    return status;
};

/**
 ******************************************************************************
 * @brief  Odczytuje 3 bajty surowego ciśnienia i konwertuje je na hPa (float).
 * @note   Dane są odczytywane z rejestrów PRESS_OUT (0x28, 0x29, 0x2A)
 * i konwertowane na 24-bitową wartość ze znakiem. Później przelicza się
 * wartość na hPa dzieląc wartość 24-bitową przez 4096.
 * @param  pressure Wskaźnik do zmiennej (float), w której zostanie zapisane ciśnienie w hPa.
 * @retval HAL_StatusTypeDef: HAL_OK w przypadku sukcesu, lub kod błędu HAL.
 ******************************************************************************
 */
HAL_StatusTypeDef ILPS28QSW_read_pressure(float* pressure){
	uint8_t pressure_reading[3];

	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
		&hi2c1,
		ILPS28QSW_I2C_ADDR,
		ILPS28QSW_PRESS_OUT_REG_START, // Zacznij od (LSB)
		I2C_MEMADD_SIZE_8BIT,
		pressure_reading,
		3,
		HAL_MAX_DELAY
	);

	// Jeżeli udało się odczytać wartość to przejdź do jej konwersji
	if (status == HAL_OK) {
		// Poprawna kolejność bajtów
		// pressure_reading[0] = XL (Least Significant Byte)
		// pressure_reading[1] = L
		// pressure_reading[2] = H (Most Significant Byte)
		int32_t pressure_value = (int32_t)(pressure_reading[2] << 16) |
										  (pressure_reading[1] << 8)  |
										   pressure_reading[0];

		// Korekacja zapisu U2
		// Gdyż zapisujemy 24 bitową liczbę na 32 bitach
		// To należy wypełnić nieużywane bity jedynkami
		if (pressure_value & 0x00800000) {
			pressure_value |= 0xFF000000;
		}
		*pressure = (float)pressure_value / 4096.0f;
	} else {
		*pressure = 0.0f;
	}

	return status;
};

/**
 ******************************************************************************
 * @brief  Odczytuje 2 bajty surowej temperatury i konwertuje je na °C (float).
 * @note   Dane są odczytywane z rejestrów TEMP_OUT (0x2B, 0x2C)
 * i konwertowane na 16-bitową wartość ze znakiem, a następnie dzielone przez 100.
 * @param  temp Wskaźnik do zmiennej (float), w której zostanie zapisana temperatura w °C.
 * @retval HAL_StatusTypeDef: HAL_OK w przypadku sukcesu, lub kod błędu HAL.
 ******************************************************************************
 */
HAL_StatusTypeDef ILPS28QSW_read_temp(float* temp){
	uint8_t temp_reading[2];

	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
		&hi2c1,
		ILPS28QSW_I2C_ADDR,
		ILPS28QSW_TEMP_OUT_REGS,
		I2C_MEMADD_SIZE_8BIT,
		temp_reading,
		2,
		HAL_MAX_DELAY
	);

	// Jeżeli udało się odczytać wartość to przejdź do jej konwersji
	if (status == HAL_OK) {
		// Połącz dane z 2 bajtów do 16 bitowego odczytu
		int16_t temp_value = (int16_t)(temp_reading[1] << 8)  | temp_reading[0];

		*temp = (float)temp_value / 100.0f;
	} else {
		*temp = 0.0f;
	}

	return status;
}
