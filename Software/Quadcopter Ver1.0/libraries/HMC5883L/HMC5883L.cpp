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
 * Documentation
 * https://www.sparkfun.com/products/10530
 * http://www.loveelectronics.co.uk/Tutorials/8/hmc5883l-tutorial-and-arduino-library
 *
 *
 */

/*----- #include -----*/
 #include "Arduino.h" /* Required to use the arduino libary */
 #include "Register_Access.h"
 #include "HMC5883L.h"


/*----- Constants (#define) -----*/
// Register Map
// Register names follow the datasheet loosley
// I have shortened the names
#define HMC5883L_CONFIG_A     0x00  // R/W
#define HMC5883L_CONFIG_B     0x01  // R/W
#define HMC5883L_MODE_REG     0x02  // R/W
#define HMC5883L_DATA_X_H     0x03  // R
#define HMC5883L_DATA_X_L     0x04  // R
#define HMC5883L_DATA_Z_H     0x05  // R
#define HMC5883L_DATA_Z_L     0x06  // R
#define HMC5883L_DATA_Y_H     0x07  // R
#define HMC5883L_DATA_Y_L     0x08  // R
#define HMC5883L_STATUS       0x09  // R
#define HMC5883L_IDENT_A      0x10  // R
#define HMC5883L_IDENT_B      0x11  // R
#define HMC5883L_IDENT_C      0x12  // R

// HMC5883L I2C address
#define HMC5883L_I2C_ADDRESS  	0x1E

#define HMC5883L_GAIN 			1.3 // 0.88, 1.3, 1.9, 2.5, 4.0, 4.7, 5.6, 8.1 [Ga] 
#define HMC5883L_DATA_RATE 		15 	// 0.75, 1.5, 3, 7.5, 15 (default), 30, 75 [Hz] 
#define HMC5883L_SAMPLES 		4 	// 1 (default), 2, 4, 8 samples averaged per measurement output
#define HMC5883L_MEASURE_MODE 	0 	// 0 = Normal, 1 = positive bias, 2 = Negative Bias

/*----- (Module) Local Variables (Static) -----*/
static volatile float digRes = 0.00; /* Digital Resolution (mG/LSb) - Value cmes out of set_gain()*/

/*----- Local Function -----*/
/* Configures the gain for the device */
float set_gain(float gain){
	if (gain == 0.88){
		write_reg(HMC5883L_I2C_ADDRESS, HMC5883L_CONFIG_B, 0b00000000);
		digRes = 0.73;
	}
	else if(gain == 1.3){
		write_reg(HMC5883L_I2C_ADDRESS, HMC5883L_CONFIG_B, 0b00100000);
		digRes = 0.92;
	}
	else if (gain == 1.9){
		write_reg(HMC5883L_I2C_ADDRESS, HMC5883L_CONFIG_B, 0b01000000);
		digRes = 1.22;
	}
	else if (gain == 2.5){
		write_reg(HMC5883L_I2C_ADDRESS, HMC5883L_CONFIG_B, 0b01100000);
		digRes = 1.52;
	}
	else if (gain == 4.0) {
		write_reg(HMC5883L_I2C_ADDRESS, HMC5883L_CONFIG_B, 0b10000000);
		digRes = 2.27;
	}
	else if (gain == 4.7) {
		write_reg(HMC5883L_I2C_ADDRESS, HMC5883L_CONFIG_B, 0b10100000);
		digRes = 2.56;
	}
	else if (gain == 5.6) {
		write_reg(HMC5883L_I2C_ADDRESS, HMC5883L_CONFIG_B, 0b11000000);
		digRes = 3.03;
	}
	else if (gain == 8.1) {
		write_reg(HMC5883L_I2C_ADDRESS, HMC5883L_CONFIG_B, 0b11100000);
		digRes = 4.35;
	}
	else {
	// Default value
		write_reg(HMC5883L_I2C_ADDRESS, HMC5883L_CONFIG_B, 0b00100000);
		digRes = 0.92;
	}
	return digRes;
	}

/* These bits set the rate at which data is written to all three data output registers */
void set_data_rate(float rate) {
	// 0.75Hz
	if (rate == 0.75) {
		write_reg(HMC5883L_I2C_ADDRESS, HMC5883L_CONFIG_A, read_reg(HMC5883L_I2C_ADDRESS, HMC5883L_CONFIG_A) | 0b00000000);
	}
	// 1.5Hz
	else if (rate == 1.5) {
		write_reg(HMC5883L_I2C_ADDRESS, HMC5883L_CONFIG_A, read_reg(HMC5883L_I2C_ADDRESS, HMC5883L_CONFIG_A) | 0b00000100);
	}
	// 3Hz
	else if (rate == 3) {
		write_reg(HMC5883L_I2C_ADDRESS, HMC5883L_CONFIG_A, read_reg(HMC5883L_I2C_ADDRESS, HMC5883L_CONFIG_A) | 0b00001000);
	}
	// 7.5Hz
	else if (rate == 7.5) {
		write_reg(HMC5883L_I2C_ADDRESS, HMC5883L_CONFIG_A, read_reg(HMC5883L_I2C_ADDRESS, HMC5883L_CONFIG_A) | 0b00001100);
	}
	// 15Hz
	else if (rate == 15) {
		write_reg(HMC5883L_I2C_ADDRESS, HMC5883L_CONFIG_A, read_reg(HMC5883L_I2C_ADDRESS, HMC5883L_CONFIG_A) | 0b00010000);
	}
	// 30Hz
	else if (rate == 30) {
		write_reg(HMC5883L_I2C_ADDRESS, HMC5883L_CONFIG_A, read_reg(HMC5883L_I2C_ADDRESS, HMC5883L_CONFIG_A) | 0b00010100);
	}
	// 75Hz
	else if (rate == 75) {
		write_reg(HMC5883L_I2C_ADDRESS, HMC5883L_CONFIG_A, read_reg(HMC5883L_I2C_ADDRESS, HMC5883L_CONFIG_A) | 0b00011000);
	}
	// Default
	else  
		write_reg(HMC5883L_I2C_ADDRESS, HMC5883L_CONFIG_A, read_reg(HMC5883L_I2C_ADDRESS, HMC5883L_CONFIG_A) | 0b00010000);
}	

/* Select number of samples averaged (1 to 8) per measurement output */
void samples_averaged(int samples) {
	// 1 samples
	if (samples == 1) {
		write_reg(HMC5883L_I2C_ADDRESS, HMC5883L_CONFIG_A, read_reg(HMC5883L_I2C_ADDRESS, HMC5883L_CONFIG_A) | 0b00000000);
	}
	// 2 SAmples
	else if (samples == 2) {
		write_reg(HMC5883L_I2C_ADDRESS, HMC5883L_CONFIG_A, read_reg(HMC5883L_I2C_ADDRESS, HMC5883L_CONFIG_A) | 0b00100000);
	}
	// 4 Samples
	else if (samples == 4) {
		write_reg(HMC5883L_I2C_ADDRESS, HMC5883L_CONFIG_A, read_reg(HMC5883L_I2C_ADDRESS, HMC5883L_CONFIG_A) | 0b01000000);
	}
	// 8 Samples
	else if (samples == 8) {
		write_reg(HMC5883L_I2C_ADDRESS, HMC5883L_CONFIG_A, read_reg(HMC5883L_I2C_ADDRESS, HMC5883L_CONFIG_A) | 0b01100000);
	}
	// Default
	else
		write_reg(HMC5883L_I2C_ADDRESS, HMC5883L_CONFIG_A, read_reg(HMC5883L_I2C_ADDRESS, HMC5883L_CONFIG_A) | 0b00000000);
	}

/* These bits define the measurement flow of the device, specifically whether or not to incorporate an applied bias into the measurement */	
void measurment_mode(int mode){
	// Continuous-measurement mode
	if (mode == 0){
		write_reg(HMC5883L_I2C_ADDRESS, HMC5883L_MODE_REG, 0b00000000);
	}
	// Single-measurement mode
	else if (mode == 1){
		write_reg(HMC5883L_I2C_ADDRESS, HMC5883L_MODE_REG, 0b00000001);
	}
	// Idle mode
	else if (mode == 2|3){
		write_reg(HMC5883L_I2C_ADDRESS, HMC5883L_MODE_REG, 0b00000011);
	}
	// Default
	else { 
		write_reg(HMC5883L_I2C_ADDRESS, HMC5883L_MODE_REG, 0b00000001);
	}
}


int read_raw_x_HMC5883L(){
	byte H = read_reg(HMC5883L_I2C_ADDRESS, HMC5883L_DATA_X_H);
	byte L = read_reg(HMC5883L_I2C_ADDRESS, HMC5883L_DATA_X_L);
	int raw = ((H << 8) | L);
	return raw;													// Combine MSB and LSB
}

int read_raw_y_HMC5883L(){
	byte H = read_reg(HMC5883L_I2C_ADDRESS, HMC5883L_DATA_Y_H);
	byte L = read_reg(HMC5883L_I2C_ADDRESS, HMC5883L_DATA_Y_L);
	int raw = ((H << 8) | L);
	return raw;													// Combine MSB and LSB
}

int read_raw_z_HMC5883L(){
	byte H = read_reg(HMC5883L_I2C_ADDRESS, HMC5883L_DATA_Z_H);
	byte L = read_reg(HMC5883L_I2C_ADDRESS, HMC5883L_DATA_Z_L);
	int raw = ((H << 8) | L);
	return raw;													// Combine MSB and LSB
}


/*----- Global Functions -----*/
void init_HMC5883L(){
	init_reg();
	}

/* Possibly could expose these as arguments in the future.  */
void config_HMC5883L(){
	set_gain(HMC5883L_GAIN);
	set_data_rate(HMC5883L_DATA_RATE);
	samples_averaged(HMC5883L_SAMPLES);
	measurment_mode(HMC5883L_MEASURE_MODE);
	}

	/* Scale the raw values */
float scale_x_HMC5883L(){
	float scaled = ((read_raw_x_HMC5883L()) * digRes);
	return scaled;
	}

float scale_y_HMC5883L(){
	float scaled = ((read_raw_y_HMC5883L()) * digRes);
	return scaled;
	}

float scale_z_HMC5883L(){
	float scaled = ((read_raw_z_HMC5883L()) * digRes);
	return scaled;
	}

/* Calculates the heading in radians */
float rad_heading_HMC5883L(){
	/* NOTE: there is strange issue here
	even though read raw is called in scaled, when scaled is called it does not seem to
	execute the read raw function	
	*/
	read_raw_x_HMC5883L();
	read_raw_y_HMC5883L();
	read_raw_z_HMC5883L();
	
	float radHeading = atan2((scale_x_HMC5883L()), (scale_y_HMC5883L()) );
	
	/* Correct for when signs are reversed */
	if(radHeading < 0){
		radHeading += (2 * 3.142);
		}
    
	/* Check for wrap due to addition of declination */
	if(radHeading > (2 * 3.142)){
		radHeading -= (2 * 3.142);
		}
  	return radHeading;
	}

/* Calculates the heading in degrees */
float deg_heading_HMC5883L(){
	return (rad_heading_HMC5883L() * 180.0 / 3.142); 
}	

/* END OF FILE */