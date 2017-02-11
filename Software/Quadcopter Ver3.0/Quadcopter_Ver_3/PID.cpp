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
 *
 *
 *
 */

/*----- #include -----*/
 #include "Arduino.h" /* Required to use the arduino libary */
 #include "PID.h"
 #include "T1.h"

/*----- Constants (#define) -----*/
#define KP 10
#define KI 0
#define KD 0


/*----- (Module) Local Variables (Static) -----*/
/*
static float previousTimeI = 0;
static float sumError = 0;
static float previousTimeD = 0;
static float previousErrorD = 0;
*/

/*----- Local Function -----*/


/*----- Global Functions -----*/
void init_pid(){
	init_t1();
	}

// Constructor, function called as soon at the object is created
// All variables are initialized by the constructor to 0
// All variables are also private to the class
PID::PID(){
	volatile float error = 0;
	volatile float previousTimeI = 0;
	volatile float sumError = 0;
	volatile float previousTimeD = 0;
	volatile float previousErrorD = 0;
	volatile float currentTime = 0;
	volatile float dt = 0;
	volatile float d = 0;

}

float PID::p_pid(float setPoint, float processVariable) {
		error = setPoint - processVariable;
		return error;
	}

float PID::i_pid(float setPoint, float processVariable) {
	error = setPoint - processVariable;
	currentTime = get_micros_t1();
	dt = currentTime - previousTimeI;
	sumError =+ (error / dt);
	previousTimeI = currentTime;
	return sumError;
}

float PID::d_pid(float setPoint, float processVariable){
	error = setPoint - processVariable;
	currentTime = get_micros_t1();
	dt = currentTime - previousTimeD;
	d = (previousErrorD - error)/dt;
	previousErrorD = error;
	previousTimeD = currentTime;
	return d;
}


/* END OF FILE */
