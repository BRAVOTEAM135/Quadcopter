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
 * https://www.sparkfun.com/products/10612
 * L3G4200D MEMS motion sensor: ultra-stable three-axis digital output gyroscope
 *
 *
 */

/*----- #include -----*/
 #include "Arduino.h" /* Required to use the arduino libary */
 #include "Register_Access.h"
 #include "L3G4200D.h"
 #include "T1.h"
 #include "Pt_By_Pt_Mean.h"


/*----- Constants (#define) -----*/
// 0x00 to 0x0E Reserved and 0x10 to 0x1F

#define L3G4200D_WHO_AM_I         0x0F  // R
#define L3G4200D_CTRL_REG1        0x20  // R/W
#define L3G4200D_CTRL_REG2        0x21  // R/W
#define L3G4200D_CTRL_REG3        0x22  // R/W
#define L3G4200D_CTRL_REG4        0x23  // R/W
#define L3G4200D_CTRL_REG5        0x24  // R/W
#define L3G4200D_REFERENCE        0x25  // R/W
#define L3G4200D_OUT_TEMP         Ox26  // R
#define L3G4200D_STATUS_REG       0x27  // R
#define L3G4200D_OUT_X_L          0x28  // R
#define L3G4200D_OUT_X_H          0x29  // R
#define L3G4200D_OUT_Y_L          0x2A  // R
#define L3G4200D_OUT_Y_H          0x2B  // R
#define L3G4200D_OUT_Z_L          0x2C  // R
#define L3G4200D_OUT_Z_H          0x2D  // R
#define L3G4200D_FIFO_CTRL_REG    0x2E  // R/W
#define L3G4200D_FIFO_SRC_REG     0x2F  // R
#define L3G4200D_INT1_CFG         0x30  // RW
#define L3G4200D_INT1_SRC         0x31  // R
#define L3G4200D_INT1_TSH_XH      0x32  // R/W
#define L3G4200D_INT1_TSH_XL      0x33  // R/W
#define L3G4200D_INT1_TSH_YH      0x34  // R/W
#define L3G4200D_INT1_TSH_YL      0x35  // R/W
#define L3G4200D_INT1_TSH_ZH      0x36  // R/W
#define L3G4200D_INT1_TSH_ZL      0x37  // R/W
#define L3G4200D_INT1_DURATION    0x38  // R/W

/* Default I2C address for the MPU-6050 is 0x68.
 * But only if the AD0 pin is low.
 * Some sensor boards have AD0 high, and the
 * I2C address thus becomes 0x69.
 */
#define L3G4200D_I2C_ADDRESS 0x69

#define L3G4200D_FS_SCALE 250      	// Configure L3G4200  - 250, 500 or 2000 deg/sec
#define L3G4200D_GY_SCALE 0.00875  	/*  FS = 250 	dps	  	8.75 	mdps/digit Most acurate
										FS = 500 	dps	  	17.50	mdps/digit	
										FS = 2000 	dps*	70		mdps/digit 
									*/
#define ALPHA 0.98			// Low pass filter on scaled gyro values
#define RATE_ALPHA 0.6

/*----- (Module) Local Variables (Static) -----*/
/* Holds the calibration values */
static int calX;
static int calY;
static int calZ;

/* Holds the current time for use on the next run as the previous time  */
volatile static unsigned long lastTimeX;
volatile static unsigned long lastTimeY;
volatile static unsigned long lastTimeZ;

/* Hold the tiime for the filter version */
volatile static unsigned long lastTimeXF;
volatile static unsigned long lastTimeYF;
volatile static unsigned long lastTimeZF;



/* Holds the current gyro angle for use on the next run as previous angle */
volatile static float lastGyroXAngle;  
volatile static float lastGyroYAngle;
volatile static float lastGyroZAngle; 



/* Use for low pass fileter */
volatile static float lastRateX;
volatile static float lastRateY;
volatile static float lastRateZ;

/* Stores the value for the rate in the filter read function */
volatile static float lastRateXF;
volatile static float lastRateYF;
volatile static float lastRateZF;



 
/*----- Local Function -----*/

/* Raw read directly from the registers giving measured angular rate deg/sec degrees per second */
/* NOTE: Gyro is mounted incorrectly on the board - X and Y axis are reversed GY80*/
int read_raw_x_L3G4200D(){
	byte H = read_reg(L3G4200D_I2C_ADDRESS, L3G4200D_OUT_Y_H);	// Read Most Significant Byte MSB
	byte L = read_reg(L3G4200D_I2C_ADDRESS, L3G4200D_OUT_Y_L);	// Read Least Significant Byte LSB
	int raw = ((H << 8) | L) * -1;			// NOTE: Value to negated to be same range as accelerometer on GY80
	return raw;																// Combine MSB and LSB
}
int read_raw_y_L3G4200D(){
	byte H = read_reg(L3G4200D_I2C_ADDRESS, L3G4200D_OUT_X_H);	// Read Most Significant Byte MSB
	byte L = read_reg(L3G4200D_I2C_ADDRESS, L3G4200D_OUT_X_L);	// Read Least Significant Byte LSB
	int raw = ((H << 8) | L);
	return raw;
}
int read_raw_z_L3G4200D(){
	byte H = read_reg(L3G4200D_I2C_ADDRESS, L3G4200D_OUT_Z_H);	// Read Most Significant Byte MSB
	byte L = read_reg(L3G4200D_I2C_ADDRESS, L3G4200D_OUT_Z_L);	// Read Least Significant Byte LSB
	int raw = ((H << 8) | L);
	return raw;
}

/* Calibration of all thress axis - device must be at rest */
void cal_L3G4200D(){
	byte num_reading = 20;
	int sum_raw_x = 0;
	int sum_raw_y = 0;
	int sum_raw_z = 0;
	byte i = 0;
	
	for (i = 0; i < num_reading; i++){
		sum_raw_x += read_raw_x_L3G4200D();
		sum_raw_y += read_raw_y_L3G4200D();
		sum_raw_z += read_raw_z_L3G4200D();
		delay_t1(100);
	}
	
	calX = sum_raw_x / num_reading;
	calY = sum_raw_y / num_reading;
	calZ = sum_raw_z / num_reading;
	
}

/*----- Global Functions -----*/
/* initialise the Gyroscope - init I2C and Timer1 */
void init_L3G4200D(){
	init_reg(); 	// From regisiterAcess Library 
	init_t1();	// From timer 1 Library
	}

/* Configures the Gyroscope with a basic setup - see data sheet */
void config_L3G4200D(){
	// Enable x, y, z and turn off power down:
  write_reg(L3G4200D_I2C_ADDRESS, L3G4200D_CTRL_REG1, 0b01001111);

  // If you'd like to adjust/use the HPF, you can edit the line below to configure CTRL_REG2:
  write_reg(L3G4200D_I2C_ADDRESS, L3G4200D_CTRL_REG2, 0b00000000);

  // Configure CTRL_REG3 to generate data ready interrupt on INT2
  // No interrupts used on INT1, if you'd like to configure INT1
  // or INT2 otherwise, consult the datasheet:
  write_reg(L3G4200D_I2C_ADDRESS, L3G4200D_CTRL_REG3, 0b00001000);
  
  // enable block data update
  write_reg(L3G4200D_I2C_ADDRESS, L3G4200D_CTRL_REG4, 0b10000000);

  // CTRL_REG4 controls the full-scale range, among other things:

  if(L3G4200D_FS_SCALE == 250){
    write_reg(L3G4200D_I2C_ADDRESS, L3G4200D_CTRL_REG4, 0b00000000);
	}
	
  else if(L3G4200D_FS_SCALE == 500){
    write_reg(L3G4200D_I2C_ADDRESS, L3G4200D_CTRL_REG4, 0b00010000);
	}
	
  else{
    write_reg(L3G4200D_I2C_ADDRESS, L3G4200D_CTRL_REG4, 0b00110000);
	}
cal_L3G4200D();
	}

	
/* Get the degrees per second - rate of rotation */
float read_rate_x_L3G4200D(){
	float rate = ((read_raw_x_L3G4200D() - calX) * L3G4200D_GY_SCALE);
	return average_rate_x(rate);
	}

float read_rate_y_L3G4200D(){
	float rate = ((read_raw_y_L3G4200D() - calY) * L3G4200D_GY_SCALE);
	return average_rate_y(rate);
	}

float read_rate_z_L3G4200D(){
	float rate = ((read_raw_y_L3G4200D() - calY) * L3G4200D_GY_SCALE);
	return average_rate_z(rate);
	}


	
/* Read the angle from the gyro  */	
float read_angle_x_L3G4200D(){
	float rate = ((read_raw_x_L3G4200D() - calX) * L3G4200D_GY_SCALE); 					/* Remove the calibration offset and multiply by scale factor raw to dps */
	rate = (rate * ALPHA) + ((ALPHA - 1.0) * lastRateX);
	unsigned long currentTime = get_micros_t1();

	float angle = ((rate * ((currentTime - lastTimeX) * 0.000001)) + lastGyroXAngle); 	/* Compute angle - /10000 scale micros to milli */
	
	lastGyroXAngle = angle; 															/* Set the current angle in memory for next call */
	lastTimeX = currentTime; 
	lastRateX = rate;														/* Set the current time in memory for next call */
	
	return angle;
	}
	
float read_angle_y_L3G4200D(){
	
	float rate = ((read_raw_y_L3G4200D() - calY) * L3G4200D_GY_SCALE); 					/* Remove the calibration offset and multiply by scale factor raw to dps */
	rate = (rate * ALPHA) + ((ALPHA - 1.0) * lastRateY);
	unsigned long currentTime = get_micros_t1();

	float angle = ((rate * ((currentTime - lastTimeY) * 0.000001)) + lastGyroYAngle); 	/* Compute angle - /10000 scale micros to milli */
	
	lastGyroYAngle = angle; 															/* Set the current angle in memory for next call */
	lastTimeY = currentTime; 															/* Set the current time in memory for next call */
	lastRateY = rate;
		
	return angle;
}
float read_angle_z_L3G4200D(){
	float rate = ((read_raw_z_L3G4200D() - calZ) * L3G4200D_GY_SCALE); 					/* Remove the calibration offset and multiply by scale factor raw to dps */
	 rate = (rate * ALPHA) + ((ALPHA - 1.0) * lastRateZ);
	unsigned long currentTime = get_micros_t1();

	float angle = ((rate * ((currentTime - lastTimeZ) * 0.000001)) + lastGyroZAngle); 	/* Compute angle - /10000 scale micros to milli */

	lastGyroZAngle = angle; 															/* Set the current angle in memory for next call */
	lastTimeZ = currentTime; 															/* Set the current time in memory for next call */
	lastRateZ = rate;
	
	return angle;
}

/* read filter angle X - incorparate feedback of the angle. used in the complementry fiiter to pass
 * the angle from the filter back in to the function.  */
float read_fil_angle_x_L3G4200D(float filAngle){
	
	float rate = ((read_raw_x_L3G4200D() + calX) * L3G4200D_GY_SCALE); 					/* Remove the calibration offset and multiply by scale factor raw to dps */
	
	unsigned long currentTime = get_micros_t1();

	float angle = ((rate * ((currentTime - lastTimeX) * 0.000001)) + filAngle);	/* Compute angle - /10000 scale micros to milli */
	
	lastTimeXF = currentTime; 													/* Set the current time in memory for next call */
	lastRateXF = rate;
		
	return angle;
}

float read_fil_angle_y_L3G4200D(float filAngle){
	
	float rate = ((read_raw_y_L3G4200D() + calY) * L3G4200D_GY_SCALE); 					/* Remove the calibration offset and multiply by scale factor raw to dps */
	
	unsigned long currentTime = get_micros_t1();

	float angle = ((rate * ((currentTime - lastTimeY) * 0.000001)) + filAngle);	/* Compute angle - /10000 scale micros to milli */
	
	lastTimeYF = currentTime; 													/* Set the current time in memory for next call */
	lastRateYF = rate;
		
	return angle;
}


float read_fil_angle_z_L3G4200D(float filAngle){
	
	float rate = ((read_raw_z_L3G4200D() + calZ) * L3G4200D_GY_SCALE); 					/* Remove the calibration offset and multiply by scale factor raw to dps */
	
	unsigned long currentTime = get_micros_t1();

	float angle = ((rate * ((currentTime - lastTimeZ) * 0.000001)) + filAngle);	/* Compute angle - /10000 scale micros to milli */
	
	lastTimeZF = currentTime; 													/* Set the current time in memory for next call */
	lastRateZF = rate;
		
	return angle;
}

/* END OF FILE */