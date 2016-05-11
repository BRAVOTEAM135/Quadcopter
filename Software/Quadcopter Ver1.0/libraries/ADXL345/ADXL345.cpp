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
 * https://www.sparkfun.com/products/9836
 *
 *
 */

/*----- #include -----*/
 #include "Arduino.h" /* Required to use the arduino libary */
 #include "Register_Access.h"
 #include "ADXL345.h"
 #include "T1.h"
 #include "Pt_By_Pt_Mean.h"

 



/*----- Constants (#define) -----*/
// Register Definition
// Reserved 0x00 to 0x1C

#define ADXL345_DEVID             0x00  // R
#define ADXL345_THRESH_TAP        0x1D  // R/W
#define ADXL345_OFSX              0x1E  // R/W
#define ADXL345_OFSY              0x1F  // R/W
#define ADXL345_OFSZ              0x20  // R/W
#define ADXL345_DUR               0x21  // R/W
#define ADXL345_LATENT            0x22  // R/W
#define ADXL345_WINDOW            0x23  // R/W
#define ADXL345_THRESH_ACT        0x24  // R/W
#define ADXL345_THRESH_INACT      0x25  // R/W
#define ADXL345_TIME_INACT        0x26  // R/W
#define ADXL345_ACT_INACT_CTL     0x27  // R/W
#define ADXL345_THRESH_FF         0x28  // R/W
#define ADXL345_TIME_FF           0x29  // R/W
#define ADXL345_TAP_AXES          0x2A  // R/W
#define ADXL345_ACT_TAP_STATUS    0x2B  // R
#define ADXL345_BW_RATE           0x2C  // R/W
#define ADXL345_POWER_CTL         0x2D  // R/W
#define ADXL345_INT_ENABLE        0x2E  // R/W
#define ADXL345_INT_MAP           0x2F  // R/W
#define ADXL345_INT_SOURCE        0x30  // R
#define ADXL345_DATA_FORMAT       0x31  // R/W
#define ADXL345_DATAX0            0x32  // R
#define ADXL345_DATAX1            0x33  // R
#define ADXL345_DATAY0            0x34  // R
#define ADXL345_DATAY1            0x35  // R
#define ADXL345_DATAZ0            0x36  // R
#define ADXL345_DATAZ1            0x37  // R
#define ADXL345_FIFO_CTL          0x38  // R/W
#define ADXL345_FIFO_STATUS       0x39  // R

// ADXL345 I2C Address
#define ADXL345_I2C_ADDRESS 0x53

#define ADXL345_SCALE 0.00390625	// 2g range scale 256 1/256 = scale multiplyer see data sheet
#define RANGE 2						// Range is defined as 2g

#define ALPHA	0.95 	// Low pass filter constant

/*----- (Module) Local Variables (Static) -----*/
static int calX;
static int calY;
static int calZ;

/* Stored values for low pass filter - Not ADXL345 has built in filter */
volatile static float lastAngleX;
volatile static float lastAngleY;
volatile static float lastAngleZ;

/*----- Local Function -----*/

/* Read the regisiters and combine the 2 8 bit values */
int read_raw_x_ADXL345(){
	byte L = read_reg(ADXL345_I2C_ADDRESS, ADXL345_DATAX0);
	byte H = read_reg(ADXL345_I2C_ADDRESS, ADXL345_DATAX1);
	int rawX = ((H << 8) | L);
	return rawX;
	}

int read_raw_y_ADXL345(){
	byte L = read_reg(ADXL345_I2C_ADDRESS, ADXL345_DATAY0);
	byte H = read_reg(ADXL345_I2C_ADDRESS, ADXL345_DATAY1);
	int rawY = ((H << 8) | L);
	return rawY;
	}

int read_raw_z_ADXL345(){
	byte L = read_reg(ADXL345_I2C_ADDRESS, ADXL345_DATAZ0);
	byte H = read_reg(ADXL345_I2C_ADDRESS, ADXL345_DATAZ1);
	int rawZ = ((H << 8) | L);
	return rawZ;
	}

/* Perfomrs a calibration of the unit to find the datum */
void cal_ADXL345(){
	byte numReading = 15;
	int sumRawX = 0;
	int sumRawY = 0;
	int sumRawZ = 0;
	byte i = 0;

	for (i = 0; i < numReading; i++){
		sumRawX += read_raw_x_ADXL345();
		sumRawY += read_raw_y_ADXL345();
		sumRawZ += read_raw_z_ADXL345();
		delay_t1(100);
	}
		
	calX = sumRawX / numReading;
	calY = sumRawY / numReading;
	calZ = sumRawZ / numReading;
	
}


/*----- Global Functions -----*/
void init_ADXL345(){
	init_reg();
	init_t1();
}
/* Configure sets the range and then runs through a calibration */
void config_ADXL345() {
	write_reg(ADXL345_I2C_ADDRESS,ADXL345_POWER_CTL, 0b00000000);  // Reset POWER_CTL register.
	write_reg(ADXL345_I2C_ADDRESS,ADXL345_POWER_CTL, 0b00010000);  // Standby Mode
	write_reg(ADXL345_I2C_ADDRESS,ADXL345_POWER_CTL, 0b00001000);  // Measure mode

	if (RANGE == 2){
		write_reg(ADXL345_I2C_ADDRESS, ADXL345_DATA_FORMAT, 0b00100000); // Put the ADXL345 into +/- 2G range by writing the value 0x01 to the DATA_FORMAT register
		} 
	else if(RANGE == 4) {
		write_reg(ADXL345_I2C_ADDRESS, ADXL345_DATA_FORMAT, 0b00100001); // 4g
		}
	else if (RANGE == 8){
		write_reg(ADXL345_I2C_ADDRESS, ADXL345_DATA_FORMAT, 0b00100010); // 8g 
		}
	else{
		write_reg(ADXL345_I2C_ADDRESS, ADXL345_DATA_FORMAT, 0b00100011); // 16g
		}	
	
	cal_ADXL345();	
	}

/* Scales the raw value and provide angular postion */	
float read_angle_x_ADXL345(){
	float angle = ((read_raw_x_ADXL345() - calX) * ADXL345_SCALE) * 90; 	/* Takes the raw value mulitplies by scale factor  */
	angle = (angle * ALPHA) + ((ALPHA - 1.0) * lastAngleX);
	angle = average_acc_x(angle);
	lastAngleX = angle;														/* and then by 90 to give degrees */
	
	return angle;												/* returns the data */																				
	}

	/* Scales the raw value and provide angular postion */	
float read_angle_y_ADXL345(){
	float angle = ((read_raw_y_ADXL345() - calY) * ADXL345_SCALE) * 90; 	/* Takes the raw value mulitplies by scale factor  */
	angle = (angle * ALPHA) + ((ALPHA - 1.0) * lastAngleY); /* and then by 90 to give degrees */
	angle = average_acc_y(angle);
	lastAngleY = angle;
	
	return angle;												/* returns the data */
	}
	
/* Scales the raw value and provide angular postion - for z we are not really interested as no g in z axis */	
float read_angle_z_ADXL345(){
	float angle = ((read_raw_z_ADXL345() - calZ) * ADXL345_SCALE) * 90; 	/* Takes the raw value mulitplies by scale factor  */
	angle = (angle * ALPHA) + ((ALPHA - 1.0) * lastAngleZ);														/* and then by 90 to give degrees */
	angle = average_acc_z(angle);
	lastAngleZ = angle;
	
	return angle;												/* returns the data */																				
	}
	
	



