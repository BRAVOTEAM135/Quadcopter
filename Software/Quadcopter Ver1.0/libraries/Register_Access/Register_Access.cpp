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
 #include "Wire.h"
 //#include "Register_Access.h"



/*----- Constants (#define) -----*/



/*----- (Module) Local Variables (Static) -----*/


/*----- Local Function -----*/


/*----- Global Functions -----*/
void init_reg(){
	Wire.begin();
}
/* Read I2C Register */
int read_reg(int deviceAddress, byte address) {

	int data;

	Wire.beginTransmission(deviceAddress);  // begin transmission to slave device with the deviceaddress
	Wire.write(address);                    // queuse data for transmission, writes register address
	Wire.endTransmission();                 // end transmission to the slave device 

	Wire.requestFrom(deviceAddress, 1);    	// Request 1 byte from slave (gy80) wire.requestFrom(address, quantity)

	while(!Wire.available()) {              // reads the numbers of byte avaialable
											//waiting - if 1 byte available will not run
											// if 0 bytes available will run until 1 byts available
		}
	data = Wire.read();                  	// Reads the byte that was sent from slave to master

	return data;
	}
	

/* Write to the I2C Register */
void write_reg(int deviceAddress, byte address, byte val){
	Wire.beginTransmission(deviceAddress);	// Begin I2C transmission
	Wire.write(address);					// Write the register address
	Wire.write(val);						// Write the value to the register
	Wire.endTransmission();					// End transmission
	}


