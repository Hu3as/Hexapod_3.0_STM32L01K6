/*
 * PWM_Driver.hpp
 *
 *  Created on: May 10, 2023
 *      Author: HuczAs
 */

#ifndef INC_HEXAPOD_PWM_DRIVER_HPP_
#define INC_HEXAPOD_PWM_DRIVER_HPP_

#include "stm32l0xx_hal.h"
#include "stm32l0xx_hal_i2c.h"
#include "main.h"
#include "stdio.h"
#include "stdint.h"

#define PCA9685_MODE1 0x00

class Hexapod_PWM_Driver{
private:

	I2C_HandleTypeDef *_i2c;

	uint8_t driver_address;

public:

	//constructor
	Hexapod_PWM_Driver(I2C_HandleTypeDef *i2c, uint8_t driver_address);

	//methods
	void driver_Init(uint8_t driver_address);

	void generate_PWM_Signal(uint8_t id, uint16_t on, uint16_t off);

	float map(float angle, float x, float y, float inputMIN, float inputMAX);

	void MoveServo( int id, float angle);

	void MoveHorizonal(int id, float angle);

	void MoveVertical(int id, float angle);
};

#endif /* INC_HEXAPOD_PWM_DRIVER_HPP_ */
