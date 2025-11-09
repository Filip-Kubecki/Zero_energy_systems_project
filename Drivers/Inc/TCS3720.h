/*
 * TCS3720.h
*/

#ifndef TCS3720_H_
#define TCS3720_H_

#include "stm32l1xx_hal.h"
#include "stm32l1xx_hal_def.h"

// Deklaracje funkcji

HAL_StatusTypeDef TCS3720_read_device_id(uint8_t* device_id);


#endif /* ILPS28QSW_H_ */

