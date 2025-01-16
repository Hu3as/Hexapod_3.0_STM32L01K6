#ifndef INC_HEXAPOD_PWM_DRIVER_H_
#define INC_HEXAPOD_PWM_DRIVER_H_

#include "stm32l0xx_hal.h"
#include "stm32l0xx_hal_i2c.h"
#include "main.h"
#include "stdio.h"
#include "stdint.h"
#include "math.h"

#define PCA9685_MODE1 0x00
#define MAX_LEGS 6
typedef struct{
	int leg_id;
	float dimension[2];
	float base_dim[2];		//saved base dimensions
}Hexapod_Leg;

void Hexapod_Leg_init(Hexapod_Leg *leg, int id);

typedef struct {
    I2C_HandleTypeDef *i2c;  // uchwyt do I2C
    uint8_t driver_address; // adres sterownika
    int leg_section[3][2];
    Hexapod_Leg legs[6]; //number of legs
    float speed; 	//movement speed
} Hexapod_PWM_Driver;

/* Konstruktor dla Hexapod_PWM_Driver */
void Hexapod_PWM_Driver_Init(Hexapod_PWM_Driver *driver, I2C_HandleTypeDef *i2c, uint8_t driver_address);

/* Inicjalizacja sterownika */
void Hexapod_PWM_Driver_Driver_Init(Hexapod_PWM_Driver *driver, uint8_t driver_address);

/* Generowanie sygnału PWM */
void Hexapod_PWM_Driver_Generate_PWM_Signal(Hexapod_PWM_Driver *driver, uint8_t id, uint16_t on, uint16_t off);

/* Mapowanie wartości kąta */
float Hexapod_PWM_Driver_Map(float angle, float x, float y, float inputMIN, float inputMAX);

/* Ruch serwomechanizmu */
void Hexapod_PWM_Driver_MoveServo(Hexapod_PWM_Driver *driver, int id, float angle);

/* Ruch w poziomie */
void Hexapod_PWM_Driver_MoveHorizontal(Hexapod_PWM_Driver *driver, int id, float angle);

/* Ruch w pionie */
void Hexapod_PWM_Driver_MoveVertical(Hexapod_PWM_Driver *driver, int id, float angle);

/* Pozycja bazowa */
void Hexapod_PWM_Driver_BasePosition(Hexapod_PWM_Driver *driver);

/* Ruch od punktu do punktu */
void Hexapod_PWM_Driver_MoveFromTo(Hexapod_PWM_Driver *driver, int leg_id, double X_in, double X_out);

/* Ruch od punktu do punktu trzema nogami */
void Hexapod_PWM_Driver_MovetoThreeLegs(Hexapod_PWM_Driver *driver, int legs[3],float endX, float endZ);

/* Ruch od punktu do punktu trzema nogami (Przyjmuje tablice)*/
void Hexapod_PWM_Driver_MovetoThreeLegsTAB(Hexapod_PWM_Driver *driver, int legs[3], float endX[3], float endZ[3]);

/* Unoszenie nóg */
void Hexapod_PWM_Driver_PumpUp(Hexapod_PWM_Driver *driver);

/* Obrót */
void Hexapod_PWM_Driver_Turnover(Hexapod_PWM_Driver *driver, int direction);

/* Ruch do przodu */
void Hexapod_PWM_Driver_Forward(Hexapod_PWM_Driver *driver, int direction);

/* Pozycja kartezjańska */
void Hexapod_PWM_Driver_CartesianPosition(Hexapod_PWM_Driver *driver, int leg_id, double X, double Z);

/* Aktualizacja pozycji sekcji */
void Hexapod_PWM_Driver_MovetoPoint(Hexapod_PWM_Driver *driver,int leg, float endX, float endZ);

#endif /* INC_HEXAPOD_PWM_DRIVER_H_ */

