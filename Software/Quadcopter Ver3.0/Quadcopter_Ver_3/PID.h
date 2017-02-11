/*
 * Header File Rules
 * -----------------
 * - DO included all function prototyes that are public. E.g. adc_init(), adc_config(), adc_read()
 * - DON'T  include private functions. E.g. adc_scale()
 * - DON'T include executable lines of code or variable declarations
 * - DON'T expose any variables in a header - pass variables by reference E.g. void swap(int *x, int *y);
 * - Comments in here are for the user of the libary
 * 
 * Header File Comments:
 * ---------------------
 *
 *
 *
 */

/* Protect the header file from multiple inclusions */
 #ifndef PID_H_
 #define PID_H_

/*----- #include ... -----*/


		

/*----- Global Function Prototypes -----*/

void init_pid();

class PID {


public:
	PID(); // Constructor initialises all variables to 0
	float p_pid(float setPoint, float processVariable);
	float i_pid(float setPoint, float processVariable);
	float d_pid(float setPoint, float processVariable);

private:
	volatile float error;
	volatile float previousTimeI;
	volatile float sumError;
	volatile float previousTimeD;
	volatile float previousErrorD;
	volatile float currentTime;
	volatile float dt;
	volatile float d;
};


/* Header protection close */
 #endif

 /* END OF FILE */
