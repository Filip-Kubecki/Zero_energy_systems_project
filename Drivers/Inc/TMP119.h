#ifndef SRC_TMP119_H_
#define SRC_TMP119_H_

#include "stm32l1xx_hal.h"
// Deklaracje funkcji

// Odczyt zawartości rejestru Device_ID - zawiera rewizję oraz ID urządzenia
HAL_StatusTypeDef TMP119_read_device_id(uint16_t* device_id);

// Odczyt zawartości rejestru Temp_Result  - zawiera zmierzoną temperaturę
HAL_StatusTypeDef TMP119_read_temperature(float* temperature_c);

#endif /* SRC_TMP119_H_ */
