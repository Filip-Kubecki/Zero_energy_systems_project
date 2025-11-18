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
#include "stm32l1xx_hal_def.h"
#include <stdint.h>

#define ILPS28QSW_I2C_ADDR				(0x5C << 1)	// Adres I2C urządzenia
#define ILPS28QSW_STATUS_REG			0x27		// Rejestr statusu gotowości
#define ILPS28QSW_WHO_AM_I				0x0F		// Adres rejestru zawierającego numer ID urządzenia (powinno wynosić)
#define ILPS28QSW_CTRL_REG_START		0x10		// Adres rejestru CTRL_REG1 - kolejne 3 rejestry to rejestry CTRL1-3
#define ILPS28QSW_STATUS				0x11		// Rejestr zawierający status pomiaru ciśnienia i temperatury
#define ILPS28QSW_PRESS_OUT_REG_START	0x28		// Adres pierwszego rejestru zawierającego odczyt ciśnienia
													// kolejne 2 rejestry zawierają kolejne części odczytu (wyższe bity)
													// [PRESS_OUT_H, PRESS_OUT_L, PRESS_OUT_XL]
#define ILPS28QSW_TEMP_OUT_REGS			0x2B		// Adres pierwszego rejestru zawierającego odczyt temperatury
													// kolejny rejestr zawiera kolejną część odczytu
													// [TEMP_OUT_L, TEMP_OUT_H]

#define ILPS28QSW_TIMEOUT				20			// Maksymalny czas oczekiwania na zmianę wartości w rejestrze STATUS

// Powiązanie z handlerem I2C znajdującym się w pliku main.c
// Oznajmia że handler komunikacji I2C istnieje ale w innym pliku - w tym przypadku w main.c
extern I2C_HandleTypeDef hi2c1;


/**
 ******************************************************************************
 * @brief  Inicjuje czujnik i ustawia go w tryb uśpienia.
 * @note   Zapisuje wartość 0x00 (tryb Power-Down) do rejestru CTRL_REG1 (0x10).
 * @retval HAL_StatusTypeDef: HAL_OK w przypadku sukcesu, lub kod błędu HAL.
 ******************************************************************************
 */
HAL_StatusTypeDef ILPS28QSW_init(){
    uint8_t tx_buffer = 0x00;

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
 * @brief  Odczytuje surowe ciśnienie w trybie one-shot i konwertuje je na hPa (float).
 * @note   Odczytuje rejestry PRESS_OUT (od 0x28) i przelicza na wartość fizyczną.
 * @param  pressure Wskaźnik do zmiennej (float), w której zostanie zapisane ciśnienie w hPa.
 * @retval HAL_StatusTypeDef: HAL_OK w przypadku sukcesu, lub kod błędu HAL.
 ******************************************************************************
 */
HAL_StatusTypeDef ILPS28QSW_read_pressure(float* pressure){
	uint8_t pressure_reading[3];
	HAL_StatusTypeDef status;
	uint8_t ONE_SHOT_COMMAND = 0x01;
	uint32_t start_tick;
	uint8_t status_buffer;

	// Wywołaj pojedyńczy pomiar ustawiając bit ONE_SHOT na 1
	status = HAL_I2C_Mem_Write(
		&hi2c1,
		ILPS28QSW_I2C_ADDR, 
		ILPS28QSW_CTRL_REG_START+0x01, // CRE_REG_2
		I2C_MEMADD_SIZE_8BIT,
		&ONE_SHOT_COMMAND, 
		1, 
		HAL_MAX_DELAY
		);

    if (status != HAL_OK) return status;

	// Odczytywanie rejestru STATUS - oczekiwanie na wyniki pomiaru
    start_tick = HAL_GetTick();
    while (1){
        status = HAL_I2C_Mem_Read(&hi2c1,
			ILPS28QSW_I2C_ADDR,
			ILPS28QSW_STATUS_REG, 
			I2C_MEMADD_SIZE_8BIT,
			&status_buffer,
			1,
			HAL_MAX_DELAY
			);
        if (status != HAL_OK) return status;

        // Sprawdź bit P_DA - pressure data
		// Jeżeli jest równy 1 to znaczy że dane sa gotowe do odczytu
        if (status_buffer & 0x02) {
            break; // Dane są gotowe, wyjdź z pętli czekania
        }

        // Sprawdź Timeout (zabezpieczenie przed zawieszeniem)
        if ((HAL_GetTick() - start_tick) > ILPS28QSW_TIMEOUT) {
            return HAL_TIMEOUT;
        }
    }

	status = HAL_I2C_Mem_Read(
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
 * @brief  Odczytuje surową temperaturę w trybie one-shot i konwertuje ją na deg C (float).
 * @note   Odczytuje rejestry TEMP_OUT (od 0x2B) i przelicza na wartość fizyczną.
 * @param  temp Wskaźnik do zmiennej (float), w której zostanie zapisana temperatura w deg C.
 * @retval HAL_StatusTypeDef: HAL_OK w przypadku sukcesu, lub kod błędu HAL.
 ******************************************************************************
 */
HAL_StatusTypeDef ILPS28QSW_read_temp(float* temp){
	uint8_t temp_reading[2];
	HAL_StatusTypeDef status;
	uint8_t ONE_SHOT_COMMAND = 0x01;
	uint32_t start_tick;
	uint8_t status_buffer;

	// Wywołaj pojedyńczy pomiar ustawiając bit ONE_SHOT na 1
	status = HAL_I2C_Mem_Write(
		&hi2c1,
		ILPS28QSW_I2C_ADDR, 
		ILPS28QSW_CTRL_REG_START+0x01, // CRE_REG_2
		I2C_MEMADD_SIZE_8BIT,
		&ONE_SHOT_COMMAND, 
		1, 
		HAL_MAX_DELAY
		);

    if (status != HAL_OK) return status;

	// Odczytywanie rejestru STATUS - oczekiwanie na wyniki pomiaru
    start_tick = HAL_GetTick();
    while (1){
        status = HAL_I2C_Mem_Read(&hi2c1,
			ILPS28QSW_I2C_ADDR,
			ILPS28QSW_STATUS_REG, 
			I2C_MEMADD_SIZE_8BIT,
			&status_buffer,
			1,
			HAL_MAX_DELAY
			);
        if (status != HAL_OK) return status;

        // Sprawdź bit T_DA - dane temperatury
		// Jeżeli jest równy 1 to znaczy że dane sa gotowe do odczytu
        if (status_buffer & 0x01) {
            break; // Dane są gotowe, wyjdź z pętli czekania
        }

        // Sprawdź Timeout (zabezpieczenie przed zawieszeniem)
        if ((HAL_GetTick() - start_tick) > ILPS28QSW_TIMEOUT) {
            return HAL_TIMEOUT;
        }
    }

	status = HAL_I2C_Mem_Read(
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
