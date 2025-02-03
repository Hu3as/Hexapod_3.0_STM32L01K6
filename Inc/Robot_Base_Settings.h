
#ifndef INC_ROBOT_BASE_SETTINGS_H_
#define INC_ROBOT_BASE_SETTINGS_H_

#define STATES_AMOUNT 132	//amount of potential recieved marks
							//leg angle range
#define HORIZ_MIN 60		//horizontal movement
#define HORIZ_MAX 120
#define VERT_MIN 45			//vertical movement
#define VERT_MAX 140

#define IGNORE_ANGLE 255  	//DO NOT CHANGE //ignore angle

/*_______MOVEMENT_PARAMETERS_______*/

#define STIFFNESS 0.3f	   //stiffness effect during movement
#define DAMPING 0.3f	   //damping effect during movement
#define ITERATION_STEPS 100//whole movement steps


/*---- TURNOVER_MOVEMENT_TAB_CONFIG ----*/
						   //X - Grupa A          Z        X - Grupa B       Z
	#define TURNOVER_ROW_0	{0, 				-43,	   0, 				-22}
	#define TURNOVER_ROW_1	{direction* 30,		-43, 	   0, 				-43}
	#define TURNOVER_ROW_2	{0,    				-22,       0,               -43}
	#define TURNOVER_ROW_3	{0,                 -22,       0,               -43}
	#define TURNOVER_ROW_4	{0,                 -22,       direction * 30,  -43}
	#define TURNOVER_ROW_5	{0,                 -43,       0,  				-22}
#endif /* INC_ROBOT_BASE_SETTINGS_H_ */
