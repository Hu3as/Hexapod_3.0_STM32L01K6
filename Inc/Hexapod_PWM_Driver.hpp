///*
// * PWM_Driver.hpp
// *
// *  Created on: May 10, 2023
// *      Author: HuczAs
// */
//
//#ifndef INC_HEXAPOD_PWM_DRIVER_HPP_
//#define INC_HEXAPOD_PWM_DRIVER_HPP_
//
//#include "stm32l0xx_hal.h"
//#include "stm32l0xx_hal_i2c.h"
//#include "main.h"
//#include "stdio.h"
//#include "stdint.h"
//#include "math.h"
//#include <vector>
//
//#define PCA9685_MODE1 0x00
//
//
//
//class Hexapod_Leg{
//private:
//	int leg_id;		//leg id
//
//	float position[2] = {0.0f, -43.0f};
//
//public:
//	// Constructor
//	Hexapod_Leg(int id);
//
//	// get leg identyficator
//	int Get_Leg_Id();
//
//	// set leg position
//	void Set_position(float x, float z);
//
//	// get leg position
//	float Get_position();
//};
//
//
//
//class Hexapod_PWM_Driver{
//private:
//
//	I2C_HandleTypeDef *_i2c;
//
//	uint8_t driver_address;
//
//
//public:
//
//	//constructor
//	Hexapod_PWM_Driver(I2C_HandleTypeDef *i2c, uint8_t driver_address);
//
//	//methods
//	void driver_Init(uint8_t driver_address);
//
//	void generate_PWM_Signal(uint8_t id, uint16_t on, uint16_t off);
//
//	float map(float angle, float x, float y, float inputMIN, float inputMAX);
//
//	void MoveServo( int id, float angle);
//
//	void MoveHorizonal(int id, float angle);
//
//	void MoveVertical(int id, float angle);
//
//	/*___________________________Moving Sequences____________________________________*/
//
//	void BasePosition();
//
//	void moveFromTo(int led_id, double X_in, double X_out);
//
//	void pump_up();
//
//	void turnover( int direction, float speed );
//
//	void forward();
//
//	void backward();
//
//	void cartesianPosition(int leg_id, double X, double Z);
//
//	void updateSectionPosition(int leg, float startX, float startZ, float endX, float endZ, float stepX, float stepZ);
//
//
//};
//
//#endif /* INC_HEXAPOD_PWM_DRIVER_HPP_ */
