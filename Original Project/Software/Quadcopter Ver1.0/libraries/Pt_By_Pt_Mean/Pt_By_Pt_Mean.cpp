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
 #include "Pt_By_Pt_Mean.h"


/*----- Constants (#define) -----*/
/*----- Gyrp Rate sample size -----*/
#define RATE_X_SAMPLE_SIZE 10
#define RATE_Y_SAMPLE_SIZE 10
#define RATE_Z_SAMPLE_SIZE 10

/*--- Radio Sample size -----*/
#define RATE_THRO_SAMPLE_SIZE 10
#define RATE_RUDD_SAMPLE_SIZE 10
#define RATE_ALIE_SAMPLE_SIZE 10
#define RATE_ELEV_SAMPLE_SIZE 10

#define ACC_X_SAMPLE_SIZE 20
#define ACC_Y_SAMPLE_SIZE 20
#define ACC_Z_SAMPLE_SIZE 20


//NOTE: These buffers should be in the corresponding modules. Just here for now



/*----- (Module) Local Variables (Static) -----*/
volatile static float bufferRateX[RATE_X_SAMPLE_SIZE]; // This is a memory bufferRateX
volatile static int iRateX = 0; // This is a memory bufferRateX

volatile static float bufferRateY[RATE_Y_SAMPLE_SIZE]; // This is a memory bufferRateX
volatile static int iRateY = 0; // This is a memory bufferRateX

volatile static float bufferRateZ[RATE_Z_SAMPLE_SIZE]; // This is a memory bufferRateX
volatile static int iRateZ = 0; // This is a memory bufferRateX


/*----- Buffers for Radio -----*/ 
volatile static float bufferThro[RATE_THRO_SAMPLE_SIZE]; // This is a memory bufferRateX
volatile static int iThro = 0; // This is a memory bufferRateX

volatile static float bufferRudd[RATE_RUDD_SAMPLE_SIZE]; // This is a memory bufferRateX
volatile static int iRudd = 0; // This is a memory bufferRateX

volatile static float bufferAlie[RATE_ALIE_SAMPLE_SIZE]; // This is a memory bufferRateX
volatile static int iAlie = 0; // This is a memory bufferRateX

volatile static float bufferElev[RATE_ELEV_SAMPLE_SIZE]; // This is a memory bufferRateX
volatile static int iElev = 0; // This is a memory bufferRateX


/*----- Buffers for ADXL345 -----*/
volatile static float bufferAccX[ACC_X_SAMPLE_SIZE]; // This is a memory bufferRateX
volatile static int iAccX = 0; // This is a memory bufferRateX

volatile static float bufferAccY[ACC_Y_SAMPLE_SIZE]; // This is a memory bufferRateX
volatile static int iAccY = 0; // This is a memory bufferRateX

volatile static float bufferAccZ[ACC_Z_SAMPLE_SIZE]; // This is a memory bufferRateX
volatile static int iAccZ = 0; // This is a memory bufferRateX

/*----- Local Function -----*/


/*----- Global Functions -----*/


float average_rate_x (float data){
	bufferRateX[iRateX] = data;	//Add the new data element to the array
	iRateX++;	//Increment iRateX ready for next iteration
	
	if (iRateX == RATE_X_SAMPLE_SIZE){
		iRateX = 0; 	// Reset iRateX as reached the end of the array and start again to replace the oldest data
	}
	
	float total = 0;	//holds the total sum of the element in the array
	for (int j=0; j < RATE_X_SAMPLE_SIZE; j++){
		total += bufferRateX[j];	//Sum all elements in the array
	}
	return total/RATE_X_SAMPLE_SIZE; // Calculate and return the mean. 
	}
	

	
float average_rate_y (float data){
	bufferRateY[iRateY] = data;	//Add the new data element to the array
	iRateY++;	//Increment iRateY ready for next iteration
	
	if (iRateY == RATE_Y_SAMPLE_SIZE){
		iRateY = 0; 	// Reset iRateY as reached the end of the array and start again to replace the oldest data
	}
	
	float total = 0;	//holds the total sum of the element in the array
	for (int j=0; j < RATE_Y_SAMPLE_SIZE; j++){
		total += bufferRateY[j];	//Sum all elements in the array
	}
	return total/RATE_Y_SAMPLE_SIZE; // Calculate and return the mean. 
	}
	
	
float average_rate_z (float data){
	bufferRateZ[iRateZ] = data;	//Add the new data element to the array
	iRateZ++;	//Increment iRateZ ready for next iteration
	
	if (iRateZ == RATE_Z_SAMPLE_SIZE){
		iRateZ = 0; 	// Reset iRateZ as reached the end of the array and start again to replace the oldest data
	}
	
	float total = 0;	//holds the total sum of the element in the array
	for (int j=0; j < RATE_Z_SAMPLE_SIZE; j++){
		total += bufferRateZ[j];	//Sum all elements in the array
	}
	return total/RATE_Z_SAMPLE_SIZE; // Calculate and return the mean. 
	}
	
/*----- RADIO AVERAGING -----*/	
float average_thro (float data){
	bufferThro[iThro] = data;	//Add the new data element to the array
	iThro++;	//Increment iRateZ ready for next iteration
	
	if (iThro == RATE_THRO_SAMPLE_SIZE){
		iThro = 0; 	// Reset iRateZ as reached the end of the array and start again to replace the oldest data
	}
	
	float total = 0;	//holds the total sum of the element in the array
	for (int j=0; j < RATE_THRO_SAMPLE_SIZE; j++){
		total += bufferThro[j];	//Sum all elements in the array
	}
	return total/RATE_THRO_SAMPLE_SIZE; // Calculate and return the mean. 
	}
	
	
	
float average_rudd (float data){
	bufferRudd[iRudd] = data;	//Add the new data element to the array
	iRudd++;	//Increment iRateZ ready for next iteration
	
	if (iRudd == RATE_RUDD_SAMPLE_SIZE){
		iRudd = 0; 	// Reset iRateZ as reached the end of the array and start again to replace the oldest data
	}
	
	float total = 0;	//holds the total sum of the element in the array
	for (int j=0; j < RATE_RUDD_SAMPLE_SIZE; j++){
		total += bufferRudd[j];	//Sum all elements in the array
	}
	return total/RATE_RUDD_SAMPLE_SIZE; // Calculate and return the mean. 
	}
	
	
	
float average_alie (float data){
	bufferAlie[iAlie] = data;	//Add the new data element to the array
	iAlie++;	//Increment iRateZ ready for next iteration
	
	if (iAlie == RATE_ALIE_SAMPLE_SIZE){
		iAlie = 0; 	// Reset iRateZ as reached the end of the array and start again to replace the oldest data
	}
	
	float total = 0;	//holds the total sum of the element in the array
	for (int j=0; j < RATE_ALIE_SAMPLE_SIZE; j++){
		total += bufferAlie[j];	//Sum all elements in the array
	}
	return total/RATE_ALIE_SAMPLE_SIZE; // Calculate and return the mean. 
	}
	
	
	
float average_elev (float data){
	bufferElev[iElev] = data;	//Add the new data element to the array
	iElev++;	//Increment iRateZ ready for next iteration
	
	if (iElev == RATE_ELEV_SAMPLE_SIZE){
		iElev = 0; 	// Reset iRateZ as reached the end of the array and start again to replace the oldest data
	}
	
	float total = 0;	//holds the total sum of the element in the array
	for (int j=0; j < RATE_ELEV_SAMPLE_SIZE; j++){
		total += bufferElev[j];	//Sum all elements in the array
	}
	return total/RATE_ELEV_SAMPLE_SIZE; // Calculate and return the mean. 
	}
	
	

/*----- Avearging for ACC -----*/
float average_acc_x (float data){
	bufferAccX[iAccX] = data;	//Add the new data element to the array
	iAccX++;	//Increment iRateX ready for next iteration
	
	if (iAccX == ACC_X_SAMPLE_SIZE){
		iAccX = 0; 	// Reset iRateX as reached the end of the array and start again to replace the oldest data
	}
	
	float total = 0;	//holds the total sum of the element in the array
	for (int j=0; j < ACC_X_SAMPLE_SIZE; j++){
		total += bufferAccX[j];	//Sum all elements in the array
	}
	return total/ACC_X_SAMPLE_SIZE; // Calculate and return the mean. 
	}
	

	
float average_acc_y (float data){
	bufferAccY[iAccY] = data;	//Add the new data element to the array
	iAccY++;	//Increment iRateX ready for next iteration
	
	if (iAccY == ACC_Y_SAMPLE_SIZE){
		iAccY = 0; 	// Reset iRateX as reached the end of the array and start again to replace the oldest data
	}
	
	float total = 0;	//holds the total sum of the element in the array
	for (int j=0; j < ACC_Y_SAMPLE_SIZE; j++){
		total += bufferAccY[j];	//Sum all elements in the array
	}
	return total/ACC_Y_SAMPLE_SIZE; // Calculate and return the mean. 
	}
	
	
float average_acc_z (float data){
	bufferAccZ[iAccZ] = data;	//Add the new data element to the array
	iAccZ++;	//Increment iRateX ready for next iteration
	
	if (iAccZ == ACC_Z_SAMPLE_SIZE){
		iAccZ = 0; 	// Reset iRateX as reached the end of the array and start again to replace the oldest data
	}
	
	float total = 0;	//holds the total sum of the element in the array
	for (int j=0; j < ACC_Z_SAMPLE_SIZE; j++){
		total += bufferAccZ[j];	//Sum all elements in the array
	}
	return total/ACC_Z_SAMPLE_SIZE; // Calculate and return the mean. 
	}

/* END OF FILE */