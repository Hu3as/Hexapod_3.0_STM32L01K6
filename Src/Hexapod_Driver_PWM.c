


#include <Hexapod_Driver_PWM.h>
#include <Robot_Base_Settings.h>
#include <stdbool.h>

// Initialize a single leg with its ID and default position
void Hexapod_Leg_init( Hexapod_Leg *leg, int id){

	// Leg base coordinates initialization (X and Z coordinates for each leg)
	// Leg IDs:   		  L0      L1      L2      L3      L4      L5
	float X_Legs[6] = { -10.0,  -5.0,   -10.0,  -10.0,   15.0,   10.0};
	float Z_Legs[6] = { -40.0,  -35.0,  -40.0,  -40.0,  -40.0,  -30.0};

	leg->leg_id = id;
	leg->dimension[0] = X_Legs[(int)(id/2)];
	leg->dimension[1] = Z_Legs[(int)(id/2)];
	 // Store the base position of the leg
	leg->base_dim[0] = X_Legs[(int)(id/2)];
	leg->base_dim[1] = Z_Legs[(int)(id/2)];

}

/* Initialize the PWM driver for the Hexapod */
void Hexapod_PWM_Driver_Init(Hexapod_PWM_Driver *driver, I2C_HandleTypeDef *i2c, uint8_t driver_address) {

    driver->i2c = i2c;
	driver->driver_address = driver_address;


	 // Initialize leg section mapping
	for (int i = 0; i < 3; i++) {
	    driver->leg_section[i][0] = 4 * i;
	    driver->leg_section[i][1] = 4 * i + 2;
	}

	int leg_index = 0;
	for(int i = 0; i< 2; i++){		// Initialize all legs with base ID values
		for(int j = 0; j<3; j++ ){
			Hexapod_Leg_init(&driver->legs[leg_index], driver->leg_section[j][i]);
			leg_index++;
		}
	}

	// Configure PWM driver using I2C
	uint8_t oldMode = 0;
	uint8_t NewMode = ((oldMode & 0x7F) | 0x10);
	uint8_t buffor[2] = {PCA9685_MODE1, NewMode};

	HAL_I2C_Master_Transmit(driver->i2c, driver_address, PCA9685_MODE1, 1, 1);
	HAL_I2C_Master_Transmit(driver->i2c, driver_address, buffor, 2, 1);
	buffor[1] = 3; // Hard-coded prescaler
	HAL_I2C_Master_Transmit(driver->i2c, driver_address, buffor, 2, 1);
	buffor[1] = oldMode;
	HAL_I2C_Master_Transmit(driver->i2c, driver_address, buffor, 2, 1);
	HAL_Delay(10);
	buffor[1] = (oldMode | 0xA1);
	HAL_I2C_Master_Transmit(driver->i2c, driver_address, buffor, 2, 1);

}
/* Generate a PWM signal for a given servo */
void Hexapod_PWM_Driver_Generate_PWM_Signal(Hexapod_PWM_Driver *driver, uint8_t id, uint16_t on, uint16_t off) {
    uint8_t buffor[5] = {
        (uint8_t)(0x06 + 4 * id),
        (uint8_t)(on),
        (uint8_t)(on >> 8),
        (uint8_t)(off),
        (uint8_t)(off >> 8)
    };

    HAL_I2C_Master_Transmit(driver->i2c, driver->driver_address, buffor, 5, 1);
}

/* Map an angle to PWM range */
float Hexapod_PWM_Driver_Map(float angle, float x, float y, float inputMIN, float inputMAX) {
    return (angle - x) * (inputMAX - inputMIN) / (y - x) + inputMIN;
}

/* Move a servo to a specific angle */
void Hexapod_PWM_Driver_MoveServo(Hexapod_PWM_Driver *driver, uint8_t id, float horizontal_ang, float vertical_ang){

	bool isVertical = id & 1;
	uint8_t angle = isVertical ? vertical_ang : horizontal_ang;
	uint8_t min_angle = isVertical ? VERT_MIN : HORIZ_MIN;
	uint8_t max_angle = isVertical ? VERT_MAX : HORIZ_MAX;

	if (angle != IGNORE_ANGLE && angle >= min_angle && angle <= max_angle) {
		Hexapod_PWM_Driver_Generate_PWM_Signal(driver, id, 0, 4095 - (16 * Hexapod_PWM_Driver_Map(angle, 0, 180, 130, 230)));
	}
}

/*___________________INVERSE KINEMATIC_________________________*/
void Hexapod_PWM_Driver_CartesianPosition(Hexapod_PWM_Driver *driver, int leg_id, float X, float Z) {
    Z += 43.0;
    float angle_OZ = ( (asin(Z / 27.7)) * (180 / M_PI) ) + 90.0;
    float angle_OX = ( (asin(X / 65.0)) * (180 / M_PI) ) + 90.0;

    Hexapod_PWM_Driver_MoveServo(driver, leg_id, angle_OX, 255);	//hor
    Hexapod_PWM_Driver_MoveServo(driver, leg_id+1, 255, angle_OZ);//ver
/*_________Cartesian moving range___________*/

//			X-Axis:		< -20 ; 20 >
//			Z-Axis:		< -63 ; -29>
//			Y-Axis: 	The y-axis is based on the
//						remaining coordinates, and due to the
//						construction fact, it is not required
//						for correct movement.
/*__________________________________________*/
}


/* Move the hexapod to its base position */
void Hexapod_PWM_Driver_BasePosition(Hexapod_PWM_Driver *driver) {
	for(int i = 0; i<6; i++){
		Hexapod_PWM_Driver_CartesianPosition(driver, driver->legs[i].leg_id, driver->legs[i].base_dim[0], driver->legs[i].base_dim[1]);
	}
}

/* Move three legs to the specified coordinates with damping and elasticity */
void Hexapod_PWM_Driver_MovetoThreeLegs(Hexapod_PWM_Driver *driver, int legs[3], float *endX, float *endZ) {
    float currentX[3], currentZ[3], velocityX[3] = {0}, velocityZ[3] = {0};
    float stiffness = STIFFNESS;  		//def 0.5	//slow 0.1
    float damping = DAMPING;    		//def 0.6	//slow 0.4
    int max_steps = ITERATION_STEPS;  	//def 30	//slow 150
    int completed[3] = {0, 0, 0};


    // Initialize positions
    for (int i = 0; i < 3; i++) {
        currentX[i] = driver->legs[legs[i]].dimension[0];
        currentZ[i] = driver->legs[legs[i]].dimension[1];
    }

    // Apply dynamic movement with damping
    for (int step = 0; step < max_steps; step++) {
        for (int i = 0; i < 3; i++) {
            if (!completed[i]) {
                // Force attracting towards the target position
                float forceX = (endX[i] - currentX[i]) * stiffness;
                float forceZ = (endZ[i] - currentZ[i]) * stiffness;

                // Velocity update considering damping
                velocityX[i] = (velocityX[i] + forceX) * damping;
                velocityZ[i] = (velocityZ[i] + forceZ) * damping;

                // Leg position update
                currentX[i] += velocityX[i];
                currentZ[i] += velocityZ[i];

                driver->legs[legs[i]].dimension[0] = currentX[i];
                driver->legs[legs[i]].dimension[1] = currentZ[i];

                Hexapod_PWM_Driver_CartesianPosition(driver, driver->legs[legs[i]].leg_id, currentX[i], currentZ[i]);

                // If the leg is close to the target and the velocity is very small → stop the movement
                if (fabs(currentX[i] - endX[i]) < 0.1 && fabs(currentZ[i] - endZ[i]) < 0.1 && fabs(velocityX[i]) < 0.01 && fabs(velocityZ[i]) < 0.01) {
                    completed[i] = 1;
                }
            }
        }
    }
}

/*Hexapod rotation*/
void Hexapod_PWM_Driver_Turnover(Hexapod_PWM_Driver *driver, int direction) {
    int groupA[3] = {0, 1, 2};
    int groupB[3] = {3, 4, 5};

    float turnover_steps[6][4] = {
    		TURNOVER_ROW_0,
			TURNOVER_ROW_1,
			TURNOVER_ROW_2,
			TURNOVER_ROW_3,
			TURNOVER_ROW_4,
			TURNOVER_ROW_5
    };

    for (int step = 0; step < 6; step++) {
        float endX_A[3] = {turnover_steps[step][0], turnover_steps[step][0], turnover_steps[step][0]};

        float endZ_A[3] = {turnover_steps[step][1], turnover_steps[step][1], turnover_steps[step][1]};
        float endX_B[3] = {turnover_steps[step][2], turnover_steps[step][2], turnover_steps[step][2]};

        float endZ_B[3] = {turnover_steps[step][3], turnover_steps[step][3], turnover_steps[step][3]};

        Hexapod_PWM_Driver_MovetoThreeLegs(driver, groupA, endX_A, endZ_A);
        Hexapod_PWM_Driver_MovetoThreeLegs(driver, groupB, endX_B, endZ_B);
    }
}

/*Hexapod movement forward or backward*/
void Hexapod_PWM_Driver_Forward(Hexapod_PWM_Driver *driver, int direction) {

    int groupA[3] = {0, 1, 2};
    int groupB[3] = {3, 4, 5};
    int move = 15;

    float forward_X[4][2][3] = {
    		// Faza 1:
        {
        		{-25, 				 10,  				 0},								//groupA
	            { 0,   				 -25,  				10}									//groupB
        },
			// Faza 2:
		{
				{-25-((direction)*move),  10-((direction)*move),  0+((direction)*move) },
				{ 0, 				 -25,  				 10}
		},
			// Faza 3:
		{
				{-25-((direction)*move),  10-((direction)*move),  0+((direction)*move) },
				{ 0-((direction)*move),  -25+((direction)*move),  10+((direction)*move)}
		},
			// Faza 4:
		{
        		{-25,  				 10,  				  0},
				{ 0-((direction)*move), -25+((direction)*move),  10+((direction)*move) }
		},
   };
	float forward_Z[4][2][3] = {
			//groupA			//groupB
		{ {-40, -40, -40},  {-40, -40, -40} },
		{ {-25, -25, -25},  {-40, -40, -40} },
		{ {-40, -40, -40},  {-25, -25, -25} },
		{ {-40, -40, -40},  {-40, -40, -40} }
	};

    for (int i = 0; i < 4; i++) {
        Hexapod_PWM_Driver_MovetoThreeLegs(driver, groupA, forward_X[i][0], forward_Z[i][0]);
        Hexapod_PWM_Driver_MovetoThreeLegs(driver, groupB, forward_X[i][1], forward_Z[i][1]);
    }
}

