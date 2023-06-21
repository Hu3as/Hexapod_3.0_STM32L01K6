/*
 * Hexapod_PWM_Driver.cpp
 *
 *  Created on: May 15, 2023
 *      Author: HuczAs
 */
#include "Hexapod_PWM_Driver.hpp"

Hexapod_PWM_Driver::Hexapod_PWM_Driver(I2C_HandleTypeDef *i2c, uint8_t addr):_i2c(i2c),driver_address(addr){
	driver_Init(driver_address);
}

void Hexapod_PWM_Driver::driver_Init(uint8_t driver_address){

	uint8_t oldMode = 0;
	uint8_t NewMode = ((oldMode & 0x7F) | 0x10);
	uint8_t buffor[2] = {PCA9685_MODE1, NewMode};

	HAL_I2C_Master_Transmit(_i2c, driver_address, PCA9685_MODE1, 1, 1);

	HAL_I2C_Master_Transmit(_i2c, driver_address, buffor, 2, 1);
	buffor[1] = 3; // Hard - coded prescaler
	HAL_I2C_Master_Transmit(_i2c, driver_address, buffor, 2, 1);
	buffor[1] = oldMode;
	HAL_I2C_Master_Transmit(_i2c, driver_address, buffor, 2, 1);
	HAL_Delay(10);
	buffor[1] = (oldMode | 0xA1 );
	HAL_I2C_Master_Transmit(_i2c, driver_address, buffor, 2, 1);
}


void Hexapod_PWM_Driver::generate_PWM_Signal( uint8_t id, uint16_t on, uint16_t off){

	uint8_t buffor[5] = {
			static_cast<uint8_t>(0x06 + 4*id),
			static_cast<uint8_t>(on),
			static_cast<uint8_t>(on  >> 8),
			static_cast<uint8_t>(off),
			static_cast<uint8_t>(off >> 8)
	};

	HAL_I2C_Master_Transmit(_i2c, driver_address, buffor, 5, 1);
}

float Hexapod_PWM_Driver::map(float angle, float x, float y, float inputMIN, float inputMAX){
	return (angle - x)*(inputMAX - inputMIN) / (y - x) + inputMIN;
}


void Hexapod_PWM_Driver::MoveServo( int id, float angle){
	float i = map(angle, 0, 180, 130, 230);		// 	< 130 - 230 > 	Hard coded range for MG90S servo
	generate_PWM_Signal(id, 0, 4095-(16*i));	// 4095 - ( 2080 , 3680 ) range
}

void Hexapod_PWM_Driver::MoveHorizonal(int id, float angle){

		if (angle <=120 && angle >=60 ){

			generate_PWM_Signal( id, 0, 4095 - ( 16*map(angle, 0, 180, 130, 230) ) );

		}
}

void Hexapod_PWM_Driver::MoveVertical(int id, float angle){

	if(angle <= 140 && angle >= 45 ){

		generate_PWM_Signal( id, 0, 4095 - ( 16*map(angle, 0, 180, 130, 230) ) );

	}

}
