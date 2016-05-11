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
 #include "Complementary_Filter.h" 
 #include "ADXL345.h"
 #include "L3G4200D.h"
 #include "Pt_By_Pt_Mean.h"


/*----- Constants (#define) -----*/
#define ALPHA 0.75	// The LOWER the ALPHA - more Accelerometer to listen too
					// The HIGHER the ALPHA - more Gyroscope to listen too


/*----- (Module) Local Variables (Static) -----*/
volatile static float newAngleX;
volatile static float newAngleY;
volatile static float newAngleZ;

/*----- Local Function -----*/
float comp_fil(float gyro, float accel){
	float angle = (ALPHA * gyro) + ((1.0 - ALPHA) * accel);
	return angle;
}
/*----- Global Functions -----*/

float x_angle_comp_fil(){
	read_angle_x_L3G4200D();
	//read_angle_x_ADXL345();
	float currentGyroAngle = read_fil_angle_x_L3G4200D(newAngleX);
	//float currentGyroAngle = read_angle_x_L3G4200D();
	float currentAccAngle = read_angle_x_ADXL345();
	
	newAngleX = comp_fil(currentGyroAngle, currentAccAngle);
	
	return newAngleX;
}

float y_angle_comp_fil(){
	read_angle_y_L3G4200D();
	//read_angle_y_ADXL345();
	float currentGyroAngle = read_fil_angle_y_L3G4200D(newAngleY);
	//float currentGyroAngle = read_angle_y_L3G4200D();
	float currentAccAngle = read_angle_y_ADXL345();
	
	newAngleY = comp_fil(currentGyroAngle, currentAccAngle);
	
	return newAngleY;
}

float z_angle_comp_fil(){
	read_angle_z_L3G4200D();
	read_angle_z_ADXL345();
	float currentGyroAngle = read_fil_angle_z_L3G4200D(newAngleZ);
	float currentAccAngle = read_angle_z_ADXL345();
	
	newAngleZ = comp_fil(currentGyroAngle, currentAccAngle);
	
	return newAngleZ;
}

/* END OF FILE */
