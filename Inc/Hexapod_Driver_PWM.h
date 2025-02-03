#ifndef INC_HEXAPOD_PWM_DRIVER_H_
#define INC_HEXAPOD_PWM_DRIVER_H_

#include "stm32l0xx_hal.h"
#include "math.h"

#define PCA9685_MODE1 0x00

typedef struct{
	int leg_id;
	float dimension[2];
	float base_dim[2];		//saved base dimensions //Not enough FLASH memory
}Hexapod_Leg;

// Initialize a single leg with its ID and default position
void Hexapod_Leg_init(Hexapod_Leg *leg, int id);

typedef struct {
    I2C_HandleTypeDef *i2c;  // I2C Handle
    uint8_t driver_address; // driver address
    int leg_section[3][2];
    Hexapod_Leg legs[6]; //number of legs
    float speed; 	//movement speed
} Hexapod_PWM_Driver;

/* Initialize the PWM driver for the Hexapod */
void Hexapod_PWM_Driver_Init(Hexapod_PWM_Driver *driver, I2C_HandleTypeDef *i2c, uint8_t driver_address);

/* Generating a PWM signal */
void Hexapod_PWM_Driver_Generate_PWM_Signal(Hexapod_PWM_Driver *driver, uint8_t id, uint16_t on, uint16_t off);

/* Mapping angle value */
float Hexapod_PWM_Driver_Map(float angle, float x, float y, float inputMIN, float inputMAX);

/* Base position */
void Hexapod_PWM_Driver_BasePosition(Hexapod_PWM_Driver *driver);

/* Point-to-point movement */
void Hexapod_PWM_Driver_MoveFromTo(Hexapod_PWM_Driver *driver, int leg_id, double X_in, double X_out);

/* Point-to-point movement with three legs */
void Hexapod_PWM_Driver_MovetoThreeLegs(Hexapod_PWM_Driver *driver, int legs[3], float *endX, float *endZ);

/* Lifting legs */
void Hexapod_PWM_Driver_PumpUp(Hexapod_PWM_Driver *driver);

/* Rotation */
void Hexapod_PWM_Driver_Turnover(Hexapod_PWM_Driver *driver, int direction);

/* Forward movement */
void Hexapod_PWM_Driver_Forward(Hexapod_PWM_Driver *driver, int direction);

void Hexapod_PWM_Driver_MoveServo(Hexapod_PWM_Driver *driver, uint8_t id, float horizontal_ang, float vertical_ang);

/* Cartesian position */
void Hexapod_PWM_Driver_CartesianPosition(Hexapod_PWM_Driver *driver, int leg_id, float X, float Z);

/* Updating section position */
void Hexapod_PWM_Driver_MovetoPoint(Hexapod_PWM_Driver *driver, int leg, float endX, float endZ);

#endif /* INC_HEXAPOD_PWM_DRIVER_H_ */

