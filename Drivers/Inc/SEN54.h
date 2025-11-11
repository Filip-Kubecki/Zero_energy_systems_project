
/**
 ******************************************************************************
 * @file    SEN54.h
 * @brief
 *
 * @description
 *
 ******************************************************************************
*/

#ifndef INC_SEN54_H_
#define INC_SEN54_H_

#include "stm32l1xx_hal.h"
#include "stm32l1xx_hal_def.h"

// DEKLARACJE FUNKCJI


HAL_StatusTypeDef SEN54_read_measurement_values(float* pm1,float* pm2_5,float* pm4, float* pm10, float* voc, float* temp);


#endif /* INC_SEN54_H_ */