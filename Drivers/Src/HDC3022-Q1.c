/**
 ******************************************************************************
 * @file	 HDC3022-Q1.c
 * @brief   Plik z implementacją funkcji dla czujnika HDC3022-Q1.
 *
 * @description
 * Ten plik implementuje funkcje zadeklarowane w pliku HDC3022-Q1.h
 *
 * Obsługuje komunikację poprzez magistralę I2C potrzebną
 * do obsługi sensora. Przekazuje już przetworzone dane.
 ******************************************************************************
 */
#include "HDC3022-Q1.h"
#include "stm32l1xx_hal_def.h"
#include <stdint.h>

// Definicje adresów
#define HDC3022_Q1_I2C_ADDR	(0x44 << 1)				// Adres I2C urządzenia

// Definicje komend
#define HDC3022_REG_MANUFACTURER_ID	{0x37, 0x81}	// Zczytaj identyfikator producenta (powinno wynosić 0x3000)
#define HDC3022_CMD_TRIG_MEAS_LPM3	{0x24, 0xFF}	// Zczytaj temperature i wilgotność w trybie
													// on demand LPM3 (najniższy pobór energii)
#define HDC3022_CMD_SOFT_RESET		{0x30, 0xA2}

#define HDC3022_MEAS_DELAY_LPM3 5

// Powiązanie z handlerem I2C znajdującym się w pliku main.c
// Oznajmia że handler komunikacji I2C istnieje ale w innym pliku - w tym przypadku w main.c
extern I2C_HandleTypeDef hi2c1;

/**
 ******************************************************************************
 * @brief  Odczytuje 16-bitowy identyfikator producenta (Manufacturer ID).
 * @note   Poprawna wartość zwrotna dla czujnika TI to 0x3000.
 * @param  device_id Wskaźnik do zmiennej (uint16_t), w której zostanie zapisane ID.
 * @retval HAL_StatusTypeDef: HAL_OK w przypadku sukcesu, lub kod błędu HAL (np. HAL_ERROR).
 ******************************************************************************
 */
HAL_StatusTypeDef HDC3022_read_device_id(uint16_t* device_id){
	HAL_StatusTypeDef status;
	uint8_t read_buf[3];

	// Wysłanie komendy do czujnika - odczytaj id producenta
	status = HAL_I2C_Master_Transmit(
		&hi2c1,
		HDC3022_Q1_I2C_ADDR,
		(uint8_t[]) HDC3022_REG_MANUFACTURER_ID,
		2,
		HAL_MAX_DELAY
	);

	// W przypadku błędu komunikacji I2C przerwij ze statusem HAL
	if (status != HAL_OK){
		return status;
	}


	// Odbiór danych - Temperatura (MSB, LSB) + CRC, Wilgotność (MSB, LSB) + CRC 
	status = HAL_I2C_Master_Receive(
		&hi2c1,
		HDC3022_Q1_I2C_ADDR,
		read_buf,
		3,
		HAL_MAX_DELAY
	);

	// W przypadku błędu komunikacji I2C przerwij ze statusem HAL
	if (status != HAL_OK){
		return status;
	}

	// Łączymy odczytane 2 bity zawierające ID producenta - pomijamy CRC
	*device_id = (read_buf[0] << 8) | read_buf[1];

	return HAL_OK;
}

/**
 ******************************************************************************
 * @brief  Uruchamia pojedynczy pomiar temperatury i wilgotności w trybie niskiej mocy (LPM3).
 * @note   Odczytane wartości są konwertowane na jednostki fizyczne (float).
 * @param  humidity Wskaźnik do zmiennej (float), w której zostanie zapisana wilgotność w %RH.
 * @param  temp     Wskaźnik do zmiennej (float), w której zostanie zapisana temperatura w °C.
 * @retval HAL_StatusTypeDef: HAL_OK w przypadku sukcesu, lub kod błędu HAL (np. HAL_ERROR).
 ******************************************************************************
 */
HAL_StatusTypeDef HDC3022_read_humidity_and_temperature(float* humidity, float* temp){
	HAL_StatusTypeDef status; 	// Status komunikacji I2C
	uint8_t read_buf[6];		// [Temp_MSB, Temp_LSB, Temp_CRC, RH_MSB, RH_LSB, RH_CRC]

	// Wysłanie komendy do czujnika - odczytaj temperaturę i wilgotność
	status = HAL_I2C_Master_Transmit(
		&hi2c1,
		HDC3022_Q1_I2C_ADDR,
		(uint8_t[]) HDC3022_CMD_TRIG_MEAS_LPM3,
		2,
		HAL_MAX_DELAY
	);

	// W przypadku błędu komunikacji I2C przerwij ze statusem HAL
	if (status != HAL_OK){
		return status;
	}

	// Dla tego trybu producent podaje czas konwersji na 3.7 [ms] - czekamy 5 [ms] (nadmiar dla pewności)
	HAL_Delay(HDC3022_MEAS_DELAY_LPM3);

	// Odbiór danych - Temperatura (MSB, LSB) + CRC, Wilgotność (MSB, LSB) + CRC 
	status = HAL_I2C_Master_Receive(&hi2c1, HDC3022_Q1_I2C_ADDR, read_buf, 6, HAL_MAX_DELAY);


	// W przypadku błędu komunikacji I2C przerwij ze statusem HAL
	if (status != HAL_OK){
		return status;
	}

	// Z otrzymanych danych wyciagamy tylko bity odpowiedzialne za temperaturę
	uint16_t raw_temp = (read_buf[0] << 8) | read_buf[1];

	// Odczytaną wartość konwertujemy zgodnie z zaleceniem producenta
	*temp = -45.0f + 175.0f * ((float)raw_temp / 65535.0f);

	// Z otrzymanych danych wyciagamy tylko bity odpowiedzialne za wilgotność
	uint16_t raw_rh = (read_buf[3] << 8) | read_buf[4];

	// Odczytaną wartość konwertujemy zgodnie z zaleceniem producenta
	*humidity = 100.0f * ((float)raw_rh / 65535.0f);

	return HAL_OK;
}

/**
 ******************************************************************************
 * @brief  Wysyła komendę miękkiego resetu do czujnika.
 * @note   Powoduje zresetowanie logiki czujnika i powrót do stanu domyślnego.
 * @retval HAL_StatusTypeDef: HAL_OK w przypadku sukcesu, lub kod błędu HAL (np. HAL_ERROR).
 ******************************************************************************
 */
HAL_StatusTypeDef HDC3022_soft_reset(){
	HAL_StatusTypeDef status;

	status = HAL_I2C_Master_Transmit(
		&hi2c1,
		HDC3022_Q1_I2C_ADDR,
		(uint8_t[])HDC3022_CMD_SOFT_RESET,
		2,
		HAL_MAX_DELAY
	);

	// W przypadku błędu komunikacji I2C przerwij ze statusem HAL
	if (status != HAL_OK){
		return status;
	}

	// Poczekaj na gotowość czujnika po resecie
	HAL_Delay(HDC3022_MEAS_DELAY_LPM3);

	return HAL_OK;
}

/**
 ******************************************************************************
 * @brief  Wysyła komendę I2C General Call Reset.
 * @warning Resetuje wszystkie urządzenia na magistrali wspierające tę komendę.
 ******************************************************************************
 */
#define I2C_GENERAL_CALL_ADDR (0x00 << 1) // Adres ogólny I2C (0x00)
#define I2C_GENERAL_CALL_RESET_CMD 0x06   // Bajt komendy resetu

HAL_StatusTypeDef HDC3022_general_call_reset(){
    HAL_StatusTypeDef status;
    uint8_t cmd_buf = I2C_GENERAL_CALL_RESET_CMD; // Komenda to 0x06

    // Wysyłamy 1 bajt (0x06) na adres 0x00 (General Call)
    status = HAL_I2C_Master_Transmit(
        &hi2c1,
        I2C_GENERAL_CALL_ADDR, // Użyj adresu 0x00
        &cmd_buf,              // Wskaźnik na komendę
        1,                     // Rozmiar danych to 1 bajt
        HAL_MAX_DELAY
    );

    // Po resecie warto krótko zaczekać, podobnie jak przy soft resecie
    if (status == HAL_OK)
    {
        HAL_Delay(5); // Użyj opóźnienia zdefiniowanego dla soft resetu
    }
    
    return status;
}