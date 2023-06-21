/*
 * main.hpp
 *
 *  Created on: May 14, 2023
 *      Author: HuczAs
 */

#include "stm32l0xx_hal.h"
#include "stm32l0xx_hal_i2c.h"
#include "main.h"
#include "stdio.h"
#include "stdint.h"


#ifndef INC_MAINCPP_HPP_
#define INC_MAINCPP_HPP_
#ifdef __cplusplus
extern "C" {
#endif





void Serwo_Init(I2C_HandleTypeDef *hi2c, uint8_t driver_address);

void pwm_Signal(I2C_HandleTypeDef *hi2c, uint8_t driver_address, uint8_t number, uint16_t on, uint16_t off);

float map(float angle, float x, float y, float inputMIN, float inputMAX);




#ifdef __cplusplus
}
#endif
#endif /* INC_MAINCPP_HPP_ */
