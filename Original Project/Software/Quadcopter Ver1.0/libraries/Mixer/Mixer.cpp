/*
 * Source File Rules:
 * ------------------
 * - Constants MUST be defined with capitals. E.g #define VALUE 10
 * - Functions MUST follow the following standard. E.g. adc_init(), adc_write()
 * - Variables Must follow the following standard. E.g. adcValueX, adcValueY
 * - Call by reference for variables. E.g.
 *
 * int a = 100;
 * int b = 200;
 * 
 * swap(&a, &b); //pass variable address in
 * 
 * Where
 * void swap(int *x, int *y){ // using the address will point to value (*x)
 * 	int temp;
 * 	temp = *x;
 * 	*x = *y;
 * 	*y = temp;
 * }
 * }
 * 
 * Source File Comments:
 * ---------------------
 **                 __
*                 /    \
*                |  M1  | RED
*      	          \ __ /
*           __      ||      __
*         /    \    ||    /    \
* YELLOW |  M2  |========|  M3  | BLUE
*         \ __ /    ||    \ __ /
*   	           _||_
*                 /    \
*                |  M4  | GREEN
* 	              \ __ /
*
* RED Motor      = 3   -   Motor 1
* YELLOW Motor   = 5   -   Motor 2
* BLUE Motor     = 6   -   Motor 3
* GREEN Motor    = 11  -   Motor 4
 *
 *
 */

/*----- #include -----*/
 #include "Arduino.h" /* Required to use the arduino libary */
 #include "Mixer.h"
 #include "Motors.h"


/*----- Constants (#define) -----*/
#define THRO_THRESH 10 //amount the motors need to be on before corrective action


/*----- (Module) Local Variables (Static) -----*/
static volatile int qConfig; // Hold the configuration of the quadcopter either CROSS (x):2 or PLUS (+):1

/*----- Local Function -----*/


/*----- Global Functions -----*/
/* Can change the motor mix configuration this is set up for +(1) not X(2) */
void config_mixer(int config){
	qConfig = config;
	
}
void pitch_mixer(float throVal, float mixVal){
	
	/* PLUS configuration */
	if (qConfig == 1){
		if (throVal > THRO_THRESH){
			write_motor_1(throVal + mixVal);
			write_motor_4(throVal - mixVal);
		}
		else {
			write_motor_1(throVal);
			write_motor_4(throVal);
		}
	}
	/* CROSS Confiuration */
	else if (qConfig == 2){
		if (throVal > THRO_THRESH){
			write_motor_1(throVal + mixVal);
			write_motor_2(throVal + mixVal);
			write_motor_3(throVal - mixVal);
			write_motor_4(throVal - mixVal);
		}
		else {
			write_motor_1(throVal);
			write_motor_2(throVal);
			write_motor_3(throVal);
			write_motor_4(throVal);
		}
	}
	else{
		write_motor_1(throVal);
		write_motor_2(throVal);
		write_motor_3(throVal);
		write_motor_4(throVal);
	}
}

void roll_mixer(float throVal, float mixVal){
	/* PLUS configuration */
	if (qConfig == 1){
		if (throVal > THRO_THRESH){
			write_motor_2(throVal + mixVal);
			write_motor_3(throVal - mixVal);
		}
		else {
			write_motor_2(throVal);
			write_motor_3(throVal);
		}
	}
	/* CROSS Configuration */
	else if (qConfig == 2){
		if (throVal > THRO_THRESH){
			write_motor_1(throVal - mixVal);
			write_motor_2(throVal + mixVal);
			write_motor_3(throVal - mixVal);
			write_motor_4(throVal + mixVal);
		}
		else {
			write_motor_1(throVal);
			write_motor_2(throVal);
			write_motor_3(throVal);
			write_motor_4(throVal);
		}
	}
	else{
		write_motor_1(throVal);
		write_motor_2(throVal);
		write_motor_3(throVal);
		write_motor_4(throVal);
	}
}
//void yaw_mixer();



/* END OF FILE */