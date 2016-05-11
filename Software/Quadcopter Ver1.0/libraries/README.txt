Function Readme

LIBARY
	|__FUNCTIONS


Register_Access (Requires:- wire.h)
	|__	void init_reg()												Establish I2C communication using 
																	the Arduino wire library. 
																	MUST be called to use other functions		
	|__	int read_reg(int deviceAddress, byte address)				read the I2C registers
	|__	void write_reg(int deviceAddress, byte address, byte val)	write to the I2C register

Timer_1 - (Requires:- )
	|__ void init_timer()						Configure the Timer 1 on Arduino - 
												MUST be called to use library
	|__	unsigned long get_micros()				returns the micro second count
	|__void delay_1(unsigned long millisDelay)	Time delay for milli second value
	
	
ADXL345 (Requires:- Register_Access.h)
	|__	void init_ADXL345()				Establish I2C communication	- MUST be called to use library	
	|__	void config_ADXL345()			Configures the ADXL345 - see function definition for details
	|__	float read_angle_x_ADXL345()	Read the calculated positional angle
	|__	float read_angle_y_ADXL345()
	|__	float read_angle_z_ADXL345()
	
L34200DH (Requires:- Register_Access.h, Timer_1.h)
	|__	void init_L3G4200D()			Establish I2C communication	 - MUST be called to use library
	|__	void config_L3G4200D()			Configures the L34200DH - see function definition for details
	|__	float read_angle_x_L3G4200D()	Reads the calculated angle
	|__	float read_angle_y_L3G4200D()
	|__	float read_angle_z_L3G4200D()
	
HMC5883L (Requires:- Register_Access.h)
	|__	void init_HMC5883L()			Establish I2C communication - - MUST be called to use library
	|__	void config_HMC5883L()			Configures the HMC5883L - see function definition for details
	|__	float scale_x_HMC5883L()		Reads the scaled ma field
	|__	float scale_y_HMC5883L()
	|__	float scale_z_HMC5883L()
	|__	float rad_heading_HMC5883L()	Read heading in radians
	|__	float deg_heading_HMC5883L()	Read heading in degrees
	
	
Complementary_Filter (Requires:- )
	|__	float x_angle_comp_fil();
	|__ float y_angle_comp_fil();
	|__ float z_angle_comp_fil();
													
Motors (Requires:- )
	|__	void init_motors()				Sets up Timer 0 and timer 2 to generate a PWM signal - 
										MUST be called to use library
	|__	void config_motors()			Arms the Electronic Speed Controllers - MUST be called
	|__	void write_motor_1(int M1)		Writes a percentage value too the motor 0 - 100%
	|__	void write_motor_2(int M2)
	|__	void write_motor_3(int M3)
	|__	void write_motor_4(int M4)
	
Radio (Requires:- PinChangeInt, Timer_1)
	|__ void init_radio()			MUST be called to use the libary. 
	|__ void read_thro()
	|__ void read_rudd()
	|__ void read_elev()
	|__ void read_alie()