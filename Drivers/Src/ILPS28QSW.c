/**
 * ILPS28QSW.c
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
 *
 */
#include "ILPS28QSW.h"

#define ILPS28QSW_I2C_ADDR				(0x5C << 1)	// Adres I2C czujnika 
#define ILPS28QSW_
#define ILPS28QSW_WHO_AM_I				0x0F		// Adres rejestru zawierającego numer ID urządzenia (powinno wynosić)
#define ILPS28QSW_CTRL_REG_START		0x10		// Adres rejestru CTRL_REG1 - kolejne 3 rejestry to rejestry CTRL1-3
#define ILPS28QSW_PRESS_OUT_REG_START	0x28		// Adres pierwszego rejestru zawierającego odczyt ciśnienia
													// kolejne 2 rejestry zawierają kolejne części odczytu (wyższe bity)
													// [PRESS_OUT_H, PRESS_OUT_L, PRESS_OUT_XL]

// Powiązanie z handlerem I2C znajdującym się w pliku main.c
// Oznajmia że handler komunikacji I2C istnieje ale w innym pliku - w tym przypadku w main.c
extern I2C_HandleTypeDef hi2c1;

// Inicjalizacja pracy sensora w trybie pomiaru z częstotliwością 1 [Hz]
HAL_StatusTypeDef ILPS28QSW_init(){
    uint8_t tx_buffer = 0x10; // Powinno być chyba 0x10

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

// Odczyt rejestru WHO_AM_I zawierającego identyfikator urzadzenia
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

// Odczyt rejestrów CTRL_REG1, CTRL_REG2, CTRL_REG3
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

HAL_StatusTypeDef ILPS28QSW_read_pressure(float* pressure){
	uint8_t pressure_reading[3];

	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
		&hi2c1,
		ILPS28QSW_I2C_ADDR,
		ILPS28QSW_PRESS_OUT_REG_START, // Zacznij od 0x28 (LSB)
		I2C_MEMADD_SIZE_8BIT,
		pressure_reading, // Nazwa tablicy jest już wskaźnikiem
		3,                // Odczytaj 3 bajty na raz
		HAL_MAX_DELAY
	);

	if (status == HAL_OK) {
		// Poprawka 3: Poprawna kolejność bajtów
		// pressure_reading[0] = XL (Least Significant Byte)
		// pressure_reading[1] = L
		// pressure_reading[2] = H (Most Significant Byte)
		int32_t pressure_value = (int32_t)(pressure_reading[2] << 16) |
										  (pressure_reading[1] << 8)  |
										   pressure_reading[0];

		if (pressure_value & 0x00800000) {
			pressure_value |= 0xFF000000;
		}
		*pressure = (float)pressure_value / 4096.0f;
	} else {
		*pressure = 0.0f;
	}

	return status;
};

