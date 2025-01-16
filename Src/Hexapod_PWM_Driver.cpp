/*
 * Hexapod_PWM_Driver.cpp
 *
 *  Created on: May 15, 2023
 *      Author: HuczAs
 */
//#include "Hexapod_PWM_Driver.hpp"
//
//Hexapod_Leg::Hexapod_Leg(int id) {
//	leg_id = id;
//}
//
//int Hexapod_Leg::Get_Leg_Id() const {
//	return leg_id;
//}
//
//void Hexapod_Leg::Set_position(float x, float z) {
//	position[0] = x;
//	position[1] = z;
//}
//
//float Hexapod_Leg::Get_position() const {
//	return position;
//}
//
///*___________PWM_Driver_____________________*/
//
//Hexapod_PWM_Driver::Hexapod_PWM_Driver(I2C_HandleTypeDef *i2c, uint8_t addr):_i2c(i2c),driver_address(addr){
//	driver_Init(driver_address);
//}
//
//void Hexapod_PWM_Driver::driver_Init(uint8_t driver_address){
//
//	uint8_t oldMode = 0;
//	uint8_t NewMode = ((oldMode & 0x7F) | 0x10);
//	uint8_t buffor[2] = {PCA9685_MODE1, NewMode};
//
//	HAL_I2C_Master_Transmit(_i2c, driver_address, PCA9685_MODE1, 1, 1);
//
//	HAL_I2C_Master_Transmit(_i2c, driver_address, buffor, 2, 1);
//	buffor[1] = 3; // Hard - coded prescaler
//	HAL_I2C_Master_Transmit(_i2c, driver_address, buffor, 2, 1);
//	buffor[1] = oldMode;
//	HAL_I2C_Master_Transmit(_i2c, driver_address, buffor, 2, 1);
//	HAL_Delay(10);
//	buffor[1] = (oldMode | 0xA1 );
//	HAL_I2C_Master_Transmit(_i2c, driver_address, buffor, 2, 1);
//}
//
//
//void Hexapod_PWM_Driver::generate_PWM_Signal( uint8_t id, uint16_t on, uint16_t off){
//
//	uint8_t buffor[5] = {
//			static_cast<uint8_t>(0x06 + 4*id),
//			static_cast<uint8_t>(on),
//			static_cast<uint8_t>(on  >> 8),
//			static_cast<uint8_t>(off),
//			static_cast<uint8_t>(off >> 8)
//	};
//
//	HAL_I2C_Master_Transmit(_i2c, driver_address, buffor, 5, 1);
//}
//
//float Hexapod_PWM_Driver::map(float angle, float x, float y, float inputMIN, float inputMAX){
//	return (angle - x)*(inputMAX - inputMIN) / (y - x) + inputMIN;
//}
//
//
//void Hexapod_PWM_Driver::MoveServo( int id, float angle){
//	float i = map(angle, 0, 180, 130, 230);		// 	< 130 - 230 > 	Hard coded range for MG90S servo
//	generate_PWM_Signal(id, 0, 4095-(16*i));	// 4095 - ( 2080 , 3680 ) range
//}
//
//void Hexapod_PWM_Driver::MoveHorizonal(int id, float angle){
//
//	if (angle <=120 && angle >=60 ){
//
//		generate_PWM_Signal( id, 0, 4095 - ( 16*map(angle, 0, 180, 130, 230) ) );
//
//	}
//}
//
//void Hexapod_PWM_Driver::MoveVertical(int id, float angle){
//
//	if(angle <= 140 && angle >= 45 ){
//
//		generate_PWM_Signal( id, 0, 4095 - ( 16*map(angle, 0, 180, 130, 230) ) );
//
//	}
//
//}
//
//
///*___________________INVERSE KINEMATIC_________________________*/
//
//void Hexapod_PWM_Driver::cartesianPosition(int leg_id, double X, double Z){
//
//	Z += 43.0;
//	double rad_Z = asin(Z/27.7);
//	double angle_OZ = rad_Z * (180/M_PI);
//	angle_OZ+=90.0;
//
//	double rad_X = asin(X/65.0);
//	double angle_OX = rad_X * (180/M_PI);
//	angle_OX+=90.0;
//
//
//
//	MoveVertical(leg_id+1, angle_OZ);
//
//	MoveHorizonal(leg_id, angle_OX);
//
//	/*_________Cartesian moving range___________*/
//
//	//			X-Axis:		< -32 ; 32 >
//	//			Z-Axis:		< -62 ; -22>
//	//			Y-Axis: 	The y-axis is based on the
//	//						remaining coordinates, and due to the
//	//						construction fact, it is not required
//	//						for correct movement.
//	/*__________________________________________*/
//
//}
//
///*_____________________________________________________________*/
//
//void moveFromTo(int led_id, double X_in, double X_out){
//
//
//}
//
//void Hexapod_PWM_Driver::BasePosition(){
//
//
//
//	for(int i = 0; i<12; i++){
//	cartesianPosition(i, 0, -43);
//	}
//}
//
//void Hexapod_PWM_Driver::pump_up(){
//
//
//}
//
//void Hexapod_PWM_Driver::updateSectionPosition(int leg, float startX, float startZ, float endX, float endZ, float stepX, float stepZ){
//
//	for (float z = startZ, x = startX; (stepZ > 0 ? z < endZ : z > endZ) && (stepX > 0 ? x < endX : x > endX); z += stepZ, x += stepX) {
//
//	        for (int offset = 0; offset <= 8; offset += 4) {
//
//	        	cartesianPosition(leg + offset, x, z);
//
//	        }
//	}
//
//}
//
//void Hexapod_PWM_Driver::turnover( int direction, float speed ){
//
//
//
//	/*
//
//	updateSectionPosition(0, 0.0f, -43.0f, 0.0f, -29.0f, speed, speed);
//
//	if(direction > 0){
//		updateSectionPosition(0, 0.0f, -29.0f, -20.0f, -29.0f, -speed, speed);
//		updateSectionPosition(0, -20.0f, -29.0f, -20.0f, -63.0f, speed, -speed);
//		updateSectionPosition(0, -20.0f, -63.0f, 20.0f, -63.0f, speed, speed);
//		updateSectionPosition(0, 20.0f, -63.0f, 20.0f, -29.0f, speed, speed);
//		updateSectionPosition(0, 20.0f, -29.0f, 0.0f, -29.0f, -speed, speed);
//	}
//
//
//	if(direction < 0){
//		updateSectionPosition(2, 0.0f, -29.0f, 20.0f, -29.0f, speed, speed);
//		updateSectionPosition(2, 20.0f, -29.0f, 20.0f, -63.0f, speed, speed);
//		updateSectionPosition(2, 20.0f, -63.0f, -20.0f, -63.0f, -speed, speed);
//		updateSectionPosition(2, -20.0f, -63.0f, -20.0f, -29.0f, speed, speed);
//		updateSectionPosition(2, -20.0f, -29.0f, 0.0f, -29.0f, speed, speed);
//	}
//
//	if(direction> 0){
//		for (int leg = 0; leg <= 2; leg += 2) {
//				// Section 1: Move from (-29, 15) to (-63, -15)
//			updateSectionPosition(leg, 20.0f, -63.0f, -20.0f, -63.0f, -speed, speed);
//
//				// Section 2: Move from (-63, -15) back to (-29, 15)
//			updateSectionPosition(leg, -20.0f, -63.0f, 20.0f, -29.0f, speed, speed);
//
//				// Back section to start position X axis: Reset to (0, -29)
//				//for (int offset = 0; offset <= 8; offset += 4) {
//				//	cartesianPosition(leg + offset, 0, -29);
//				//}
//			}
//	}
//
//	if(direction < 0){
//		for (int leg = 0; leg <= 2; leg += 2) {
//						// Section 1: Move from (-29, 15) to (-63, -15)
//					updateSectionPosition(leg, -20.0f, -63.0f, 20.0f, -63.0f, speed, speed);
//
//						// Section 2: Move from (-63, -15) back to (-29, 15)
//					updateSectionPosition(leg, 20.0f, -63.0f, -20.0f, -29.0f, -speed, speed);
//
//						// Back section to start position X axis: Reset to (0, -29)
//						//for (int offset = 0; offset <= 8; offset += 4) {
//						//	cartesianPosition(leg + offset, 0, -29);
//						//}
//					}
//	}
//	*/
//}
//
//void Hexapod_PWM_Driver::forward(){
//
//
//}
//
//void Hexapod_PWM_Driver::backward(){
//
//
//}
//
