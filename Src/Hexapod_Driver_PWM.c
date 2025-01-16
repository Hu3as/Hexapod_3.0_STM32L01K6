/*
 * Hexapod_Driver_PWM.c
 *
 *  Created on: Dec 15, 2024
 *      Author: HuczAs
 */


#include <Hexapod_Driver_PWM.h>
//#include <Hexapod_Leg.h>
#include <Robot_Base_Settings.h>
#include <string.h>

void Hexapod_Leg_init( Hexapod_Leg *leg, int id){

	memset(leg, 0, sizeof(Hexapod_Leg)); // Wyzerowanie pamięci
	leg->leg_id = id;
	leg->dimension[0] = X_Legs[(int)(id/2)];
	leg->dimension[1] = Z_Legs[(int)(id/2)];

	leg->base_dim[0] = X_Legs[(int)(id/2)];
	leg->base_dim[1] = Z_Legs[(int)(id/2)];

}

/* Konstruktor dla Hexapod_PWM_Driver */
void Hexapod_PWM_Driver_Init(Hexapod_PWM_Driver *driver, I2C_HandleTypeDef *i2c, uint8_t addr) {

    driver->i2c = i2c;
	driver->driver_address = addr;
	driver->speed = MOVEMENT_SPEED;

	// Inicjalizacja tablicy leg_section
	for (int i = 0; i < 3; i++) {
	    driver->leg_section[i][0] = 4 * i;
	    driver->leg_section[i][1] = 4 * i + 2;
	}

	int leg_index = 0;
	for(int i = 0; i< 2; i++){
		for(int j = 0; j<3; j++ ){
			Hexapod_Leg_init(&driver->legs[leg_index], driver->leg_section[j][i]);
			leg_index++;
		}
	}


	// Inicjalizacja sterownika
	Hexapod_PWM_Driver_Driver_Init(driver, addr);
}

/* Inicjalizacja sterownika */
void Hexapod_PWM_Driver_Driver_Init(Hexapod_PWM_Driver *driver, uint8_t driver_address) {
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

/* Generowanie sygnału PWM */
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

/* Mapowanie wartości kąta */
float Hexapod_PWM_Driver_Map(float angle, float x, float y, float inputMIN, float inputMAX) {
    return (angle - x) * (inputMAX - inputMIN) / (y - x) + inputMIN;
}

/* Ruch serwomechanizmu */
void Hexapod_PWM_Driver_MoveServo(Hexapod_PWM_Driver *driver, int id, float angle) {
    float i = Hexapod_PWM_Driver_Map(angle, 0, 180, 130, 230); // Hard-coded range for MG90S servo
    Hexapod_PWM_Driver_Generate_PWM_Signal(driver, id, 0, 4095 - (16 * i));
}

/* Ruch w poziomie */
void Hexapod_PWM_Driver_MoveHorizontal(Hexapod_PWM_Driver *driver, int id, float angle) {
    if (angle <= 120 && angle >= 60) {
        Hexapod_PWM_Driver_Generate_PWM_Signal(driver, id, 0, 4095 - (16 * Hexapod_PWM_Driver_Map(angle, 0, 180, 130, 230)));
    }
}

/* Ruch w pionie */
void Hexapod_PWM_Driver_MoveVertical(Hexapod_PWM_Driver *driver, int id, float angle) {
    if (angle <= 140 && angle >= 45) {
        Hexapod_PWM_Driver_Generate_PWM_Signal(driver, id, 0, 4095 - (16 * Hexapod_PWM_Driver_Map(angle, 0, 180, 130, 230)));
    }
}

/*___________________INVERSE KINEMATIC_________________________*/
void Hexapod_PWM_Driver_CartesianPosition(Hexapod_PWM_Driver *driver, int leg_id, double X, double Z) {
    Z += 43.0;
    double rad_Z = asin(Z / 27.7);
    double angle_OZ = rad_Z * (180 / M_PI);
    angle_OZ += 90.0;

    double rad_X = asin(X / 65.0);
    double angle_OX = rad_X * (180 / M_PI);
    angle_OX += 90.0;

    Hexapod_PWM_Driver_MoveVertical(driver, leg_id + 1, angle_OZ);
    Hexapod_PWM_Driver_MoveHorizontal(driver, leg_id, angle_OX);
/*_________Cartesian moving range___________*/

//			X-Axis:		< -20 ; 20 >
//			Z-Axis:		< -63 ; -29>
//			Y-Axis: 	The y-axis is based on the
//						remaining coordinates, and due to the
//						construction fact, it is not required
//						for correct movement.
/*__________________________________________*/
}


/* Pozycja bazowa */
void Hexapod_PWM_Driver_BasePosition(Hexapod_PWM_Driver *driver) {
	for(int i = 0; i<6; i++){
		Hexapod_PWM_Driver_CartesianPosition(driver, driver->legs[i].leg_id, driver->legs[i].dimension[0], driver->legs[i].dimension[1]);
	}
}

/* Aktualizacja pozycji sekcji */
void Hexapod_PWM_Driver_MovetoPoint(Hexapod_PWM_Driver *driver, int leg, float endX, float endZ){


	 // Pobranie aktualnej pozycji nogi
	    float currentX = driver->legs[leg].dimension[0];
	    float currentZ = driver->legs[leg].dimension[1];

	    // Wyliczenie różnic w pozycjach (wektor ruchu)
	    float deltaX = endX - currentX;
	    float deltaZ = endZ - currentZ;

	    // Przybliżona długość wektora (odległość) - unikanie `sqrt`
	    float distanceSquared = deltaX * deltaX + deltaZ * deltaZ;
	    float speedSquared = driver->speed * driver->speed;

	    // Normalizacja wektora ruchu (przybliżona)
	    float normX = deltaX;
	    float normZ = deltaZ;
	    float magnitude = fabs(deltaX) > fabs(deltaZ) ? fabs(deltaX) : fabs(deltaZ); // Maksymalna wartość (przybliżona normalizacja)
	    if (magnitude > 0) {
	        normX /= magnitude;
	        normZ /= magnitude;
	    }

	    // Iteracyjne przemieszczanie w kierunku celu
	    while (distanceSquared > speedSquared) {
	        // Przemieszczenie o krok równy driver->speed
	        currentX += normX * driver->speed;
	        currentZ += normZ * driver->speed;

	        // Aktualizacja pozycji w strukturze
	        driver->legs[leg].dimension[0] = currentX;
	        driver->legs[leg].dimension[1] = currentZ;

	        // Wywołanie funkcji obsługującej ruch
	        Hexapod_PWM_Driver_CartesianPosition(driver, driver->legs[leg].leg_id, currentX, currentZ);

	        // Ponowne obliczenie różnic i przybliżonej odległości (bez sqrt)
	        deltaX = endX - currentX;
	        deltaZ = endZ - currentZ;
	        distanceSquared = deltaX * deltaX + deltaZ * deltaZ;
	    }

	    // Ostateczne przemieszczenie, jeśli odległość jest mniejsza niż driver->speed
	    driver->legs[leg].dimension[0] = endX;
	    driver->legs[leg].dimension[1] = endZ;
	    Hexapod_PWM_Driver_CartesianPosition(driver, driver->legs[leg].leg_id, endX, endZ);


}

void Hexapod_PWM_Driver_MovetoThreeLegsTAB(Hexapod_PWM_Driver *driver, int legs[3], float endX[3], float endZ[3]) {
    float currentX[3], currentZ[3], deltaX[3], deltaZ[3], normX[3], normZ[3];
    int completed[3] = {0, 0, 0}; // Flagi zakończenia ruchu dla każdej nogi

    // Pobranie aktualnych pozycji i wyliczenie kierunków ruchu
    for (int i = 0; i < 3; i++) {
        currentX[i] = driver->legs[legs[i]].dimension[0];
        currentZ[i] = driver->legs[legs[i]].dimension[1];
        deltaX[i] = endX[i] - currentX[i];
        deltaZ[i] = endZ[i] - currentZ[i];

        float magnitude = fabs(deltaX[i]) > fabs(deltaZ[i]) ? fabs(deltaX[i]) : fabs(deltaZ[i]);
        normX[i] = (magnitude > 0) ? deltaX[i] / magnitude : 0;
        normZ[i] = (magnitude > 0) ? deltaZ[i] / magnitude : 0;
    }

    // Ruch krokowy
    while (!completed[0] || !completed[1] || !completed[2]) {
        for (int i = 0; i < 3; i++) {
            if (!completed[i]) {
                // Wykonanie kroku
                currentX[i] += normX[i] * driver->speed;
                currentZ[i] += normZ[i] * driver->speed;

                // Sprawdzenie, czy osiągnięto cel
                if ((endX[i] - currentX[i]) * normX[i] <= 0 && (endZ[i] - currentZ[i]) * normZ[i] <= 0) {
                    currentX[i] = endX[i];
                    currentZ[i] = endZ[i];
                    completed[i] = 1;
                }

                // Aktualizacja pozycji w strukturze
                driver->legs[legs[i]].dimension[0] = currentX[i];
                driver->legs[legs[i]].dimension[1] = currentZ[i];

                // Wywołanie funkcji obsługującej ruch
                Hexapod_PWM_Driver_CartesianPosition(driver, driver->legs[legs[i]].leg_id, currentX[i], currentZ[i]);
            }
        }
    }

    // Ostateczne ustawienie pozycji dla każdej nogi
    for (int i = 0; i < 3; i++) {
        driver->legs[legs[i]].dimension[0] = endX[i];
        driver->legs[legs[i]].dimension[1] = endZ[i];
        Hexapod_PWM_Driver_CartesianPosition(driver, driver->legs[legs[i]].leg_id, endX[i], endZ[i]);
    }
}

void Hexapod_PWM_Driver_MovetoThreeLegs(Hexapod_PWM_Driver *driver, int legs[3], float endX, float endZ) {
    float currentX[3], currentZ[3], deltaX[3], deltaZ[3], normX[3], normZ[3];
    int completed[3] = {0, 0, 0}; // Flagi zakończenia ruchu dla każdej nogi

    // Pobranie aktualnych pozycji i wyliczenie kierunków ruchu
    for (int i = 0; i < 3; i++) {
        currentX[i] = driver->legs[legs[i]].dimension[0];
        currentZ[i] = driver->legs[legs[i]].dimension[1];
        deltaX[i] = endX - currentX[i];
        deltaZ[i] = endZ - currentZ[i];

        float magnitude = fabs(deltaX[i]) > fabs(deltaZ[i]) ? fabs(deltaX[i]) : fabs(deltaZ[i]);
        normX[i] = (magnitude > 0) ? deltaX[i] / magnitude : 0;
        normZ[i] = (magnitude > 0) ? deltaZ[i] / magnitude : 0;
    }

    // Ruch krokowy
    while (!completed[0] || !completed[1] || !completed[2]) {
        for (int i = 0; i < 3; i++) {
            if (!completed[i]) {
                // Wykonanie kroku
                currentX[i] += normX[i] * driver->speed;
                currentZ[i] += normZ[i] * driver->speed;

                // Sprawdzenie, czy osiągnięto cel
                if ((endX - currentX[i]) * normX[i] <= 0 && (endZ - currentZ[i]) * normZ[i] <= 0) {
                    currentX[i] = endX;
                    currentZ[i] = endZ;
                    completed[i] = 1;
                }

                // Aktualizacja pozycji w strukturze
                driver->legs[legs[i]].dimension[0] = currentX[i];
                driver->legs[legs[i]].dimension[1] = currentZ[i];

                // Wywołanie funkcji obsługującej ruch
                Hexapod_PWM_Driver_CartesianPosition(driver, driver->legs[legs[i]].leg_id, currentX[i], currentZ[i]);
            }
        }
    }

    // Ostateczne ustawienie pozycji dla każdej nogi
    for (int i = 0; i < 3; i++) {
        driver->legs[legs[i]].dimension[0] = endX;
        driver->legs[legs[i]].dimension[1] = endZ;
        Hexapod_PWM_Driver_CartesianPosition(driver, driver->legs[legs[i]].leg_id, endX, endZ);
    }
}


/* Obrót */
void Hexapod_PWM_Driver_Turnover(Hexapod_PWM_Driver *driver, int direction) {
    int groupA[3] = {0, 1, 2}; // Nogi grupy A
    int groupB[3] = {3, 4, 5}; // Nogi grupy B
    float turnover[6][4] = {
    		TURNOVER_ROW_0,
			TURNOVER_ROW_1,
			TURNOVER_ROW_2,
			TURNOVER_ROW_3,
			TURNOVER_ROW_4,
			TURNOVER_ROW_5
    };

    // Iteracja przez kroki obrotu
    for (int step = 0; step < 6; step++) {
        // Ruch dla grupy A
        Hexapod_PWM_Driver_MovetoThreeLegs(driver, groupA, turnover[step][0], turnover[step][1]);

        // Po zakończeniu ruchu grupy A, ruch grupy B
        Hexapod_PWM_Driver_MovetoThreeLegs(driver, groupB, turnover[step][2], turnover[step][3]);
    }
}

/* Ruch do przodu */
void Hexapod_PWM_Driver_Forward(Hexapod_PWM_Driver *driver, int direction) {
    // Indeksy grup nóg
    int groupA[3] = {0, 1, 2}; // Nogi grupy A
    int groupB[3] = {3, 4, 5}; // Nogi grupy B
    float help_X[3], help_Z[3];

    float forward_X[4][2][3] = {
		{X_ROW_0_A, X_ROW_0_B},
		{X_ROW_1_A, X_ROW_1_B},
		{X_ROW_2_A, X_ROW_2_B},
		{X_ROW_3_A, X_ROW_3_B}
	};

    float forward_Z[4][2][3] = {
        {Z_ROW_0_A, Z_ROW_0_B},
        {Z_ROW_1_A, Z_ROW_1_B},
        {Z_ROW_2_A, Z_ROW_2_B},
        {Z_ROW_3_A, Z_ROW_3_B}
    };

    // Iteracja przez sekwencje ruchu
    for (int i = 0; i < 4; i++) {
        // Ruch grupy A
        for (int j = 0; j < 3; j++) {
            help_X[j] = forward_X[i][0][j]; // Lewa kolumna
            help_Z[j] = forward_Z[i][0][j]; // Lewa kolumna
        }
        Hexapod_PWM_Driver_MovetoThreeLegsTAB(driver, groupA, help_X, help_Z);

        // Zatrzymanie grupy A przed ruchem grupy B
        for (int j = 0; j < 3; j++) {
            driver->legs[groupA[j]].dimension[0] = help_X[j];
            driver->legs[groupA[j]].dimension[1] = help_Z[j];
        }

        // Ruch grupy B
        for (int j = 0; j < 3; j++) {
            help_X[j] = forward_X[i][1][j]; // Prawa kolumna
            help_Z[j] = forward_Z[i][1][j]; // Prawa kolumna
        }
        Hexapod_PWM_Driver_MovetoThreeLegsTAB(driver, groupB, help_X, help_Z);

        // Zatrzymanie grupy B przed kolejnym ruchem grupy A
        for (int j = 0; j < 3; j++) {
            driver->legs[groupB[j]].dimension[0] = help_X[j];
            driver->legs[groupB[j]].dimension[1] = help_Z[j];
        }
    }
}


