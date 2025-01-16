/*
 * Robot_Base_Settings.h
 *
 *  Created on: Jan 15, 2025
 *      Author: HuczAs
 */

#ifndef INC_ROBOT_BASE_SETTINGS_H_
#define INC_ROBOT_BASE_SETTINGS_H_

#define MOVEMENT_SPEED 0.30


#define BASE_LEGS_
/*--------------------------------------*/
//							L0		L1		L2		L3		L4		L5
const float X_Legs[6] = {   0.0,    0.0,    0.0,    0.0,    0.0,    0.0};
const float Z_Legs[6] = { -40.0,  -40.0,  -40.0,  -40.0,  -40.0,  -40.0};
/*--------------------------------------*/



/*---- TURNOVER_MOVEMENT_TAB_CONFIG ----*/
					   //X - Grupa A          Z        X - Grupa B       Z
#define TURNOVER_ROW_0	{0, 				-43,	   0, 				-22}
#define TURNOVER_ROW_1	{direction* 30,		-43, 	   0, 				-43}
#define TURNOVER_ROW_2	{0,    				-22,       0,               -43}
#define TURNOVER_ROW_3	{0,                 -22,       0,               -43}
#define TURNOVER_ROW_4	{0,                 -22,       direction * 30,  -43}
#define TURNOVER_ROW_5	{0,                 -43,       0,  				-22}

/*---- FORWARD / BACKWARD_MOVEMENT_TAB_CONFIG ----*/
//			   A/B - group A or Group B
#define X_ROW_0_A {-20, 20, 0}
#define Z_ROW_0_A {-43, -43, -43}
#define X_ROW_0_B {0, -20, 20}
#define Z_ROW_0_B {-43, -43, -43}

#define X_ROW_1_A {-20 - (20 * direction), 20 - (20 * direction), 0 + (20 * direction)}
#define Z_ROW_1_A {-25, -25, -25}
#define X_ROW_1_B {0, -20, 20}
#define Z_ROW_1_B {-43, -43, -43}

#define X_ROW_2_A {-20 - (20 * direction), 20 - (20 * direction), 0 + (20 * direction)}
#define Z_ROW_2_A {-55, -55, -55}
#define X_ROW_2_B {0 - (20 * direction), -20 + (20 * direction), 20 + (20 * direction)}
#define Z_ROW_2_B {-25, -25, -25}

#define X_ROW_3_A {-20, 20, 0}
#define Z_ROW_3_A {-43, -43, -43}
#define X_ROW_3_B {0 - (20 * direction), -20 + (20 * direction), 20 + (20 * direction)}
#define Z_ROW_3_B {-55, -55, -55}

#endif /* INC_ROBOT_BASE_SETTINGS_H_ */
