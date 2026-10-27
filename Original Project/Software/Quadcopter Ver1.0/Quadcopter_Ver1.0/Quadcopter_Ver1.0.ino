/* APPLICATION NOTES
* 
*
*                   __
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
* 	          \ __ /
*
* RED Motor      = D3   -   Motor 1
* YELLOW Motor   = D5   -   Motor 2
* BLUE Motor     = D6   -   Motor 3
* GREEN Motor    = D11  -   Motor 4
*
*
* RX Colour Code
* THRO - Red - Arduino Pin D2
* ALIE - Blue - Arduino Pin D8
* ELEV - Green - Arduino Pin D7
* RUDD - Yellow - Arduino Pin D4

Gyr, Acc and Comp
SCL A5
SDA A4

DEBUG - Use scope on pin 13 to bench mark code
Debug LED = D13
// DEBUG FUNCTIONS
PORTB |= B00100000; // Set
PORTB &= ~B00100000; // Clear
PORTB ^= B00100000; // Toggle
*/



#include <Wire.h>
#include <T1.h>
#include <Register_Access.h>
#include <Radio.h>
#include <PID.h>
#include <Motors.h>
#include <L3G4200D.h>
#include <HMC5883L.h>
#include <ADXL345.h>
#include <Complementary_Filter.h>
#include <Watchdog.h>
#include <Mixer.h>
#include <Pt_By_Pt_Mean.h>
#include <String_To_Float.h>

#define PLUS 1
#define CROSS 2


#define P_RATE_GAIN 0.077000
#define I_RATE_GAIN 0.000
#define D_RATE_GAIN 0.000

#define P_STAB_GAIN 0
#define I_STAB_GAIN 0
#define D_STAB_GAIN 0

void setup() {
Serial.begin(115200);
Serial.println("STARTING...");

init_watchdog();
//GTCCR = (1<<TSM)|(1<<PSRASY)|(1<<PSRSYNC); // Pause the timers, this needs to be done before we onfigure the timers
Serial.println("Initialising Timer...");
init_t1();
Serial.println("Complete");
reset_watchdog();

Serial.println("Initialising Motors...");
init_motors();
config_motors();
Serial.println("Complete");
reset_watchdog();

/*
// set all timers to the same value
TCNT0 = 0; // set timer0 to 0
TCNT1H = 0; // set timer1 high byte to 0
TCNT1L = 0; // set timer1 low byte to 0
TCNT2 = 0; // set timer2 to 0

GTCCR = 0; // release all timers

config_motors();
*/

Serial.println("Initialising ADXL345...");
init_ADXL345();
config_ADXL345();
Serial.println("Complete");
reset_watchdog();

Serial.println("Initialising L3G4200D...");
init_L3G4200D();
config_L3G4200D();
Serial.println("Complete");
reset_watchdog();

Serial.println("Initialising HMC5883L...");
init_HMC5883L();
config_HMC5883L();
Serial.println("Complete");
reset_watchdog();

Serial.println("Initialising Radio...");
init_radio();
Serial.println("Complete");
reset_watchdog();

Serial.println("Initialising Mixer...");
config_mixer(PLUS);
Serial.println("Complete");
reset_watchdog();

Serial.println("Initialising PID...");
init_pid();
Serial.println("Complete");
reset_watchdog();

Serial.println("Initialising Debug LED...");
DDRB = DDRB | B00100000; //Set Pi n 13 as an Output - Will enable onboard LED
Serial.println("Complete");
reset_watchdog();

Serial.println("READY");


/* Radio saftey code to ensure the throttle stick is 0 before the application is allowed to start*/
Serial.println(read_thro());
while(read_thro() > 0){
// For safety loop around until thro stick moved to 0
/* 0 all motors */
  write_motor_1(0);
  write_motor_2(0);
  write_motor_3(0);
  write_motor_4(0);
  reset_watchdog();
  }

}



/* Main Loop of the appplication */

void loop() {
  while(1){


/* READ RADIO VALUES */
  float throValue = read_thro();
  float elevValue = read_elev();
  float alieValue = read_alie();
  float ruddValue = read_rudd();
 
/* READ GYRO RATE */  
   float xRate = read_rate_x_L3G4200D(); // Read rate for the rate controlller
   float yRate = read_rate_y_L3G4200D();
   float zRate = read_rate_z_L3G4200D();
   
/* READ ANGLE */ 
   float xAngle = x_angle_comp_fil();
   float yAngle = y_angle_comp_fil();
   float zAngle = z_angle_comp_fil();
    
    

   /*MOTORS - DEBUG*/
   
   /*----- DEBUG: READ FROM SERIAL -----*/
  /*
   volatile float prevValue;
  float value = read_serial();
  if (value >= 0){
    Serial.println(value,6);
    prevValue = value;
  }
  else {
  value = prevValue;
  }
  */
  /*------------------------------------*/
   
   
   /*
   DEBUG FOR MOTORS
   */
   /*
   write_motor_1(throValue);
   write_motor_2(throValue);
   write_motor_3(throValue);
   write_motor_4(throValue);
*/



  /* Stabalise PID */
  /*
  // ROLL - Elevator
  float P_STAB = (p_pid(elevValue, Xangle))*P_STAB_GAIN;
  float I_STAB = (i_pid(elevValue, Yangle))*I_STAB_GAIN;
  float D_STAB = (p_pid(elevValue, zAngle))*D_STAB_GAIN;
  
  float PID_STAB = P_STAB + I_STAB + D_STAB;
  */
  
  
  
  /* Rate PID */
  
  //--------------------------------------------------
  // Used to read a value from the serial port
  volatile float prevGain;
  float Gain = read_serial();
  if (Gain >= 0){
    Serial.println(Gain,6);
    prevGain = Gain;
  }
  else {
  Gain = prevGain;
  }
  //------------------------------------------------



  // Yellow and blue axis - Counter clock wise
  float pRateY = (p_pid(elevValue, xRate)) * Gain; //P_RATE_GAIN;
  float iRateY = (i_pid(elevValue, xRate)) * I_RATE_GAIN;
  float dRateY = (p_pid(elevValue, xRate)) * D_RATE_GAIN;
 
  /*
  // Red and Green axis - clock wise
  float pRateX = (p_pid(alieValue, yRate)) * P_RATE_GAIN;
  float iRateX = (i_pid(alieValue, yRate)) * I_RATE_GAIN;
  float dRateX = (p_pid(alieValue, yRate)) * D_RATE_GAIN;
  */
  
  float pidRateY = pRateY + iRateY + dRateY;
  //float pidRateX = pRateX + iRateX + dRateX;
  
  
  
  
  /*Mixer Outputs to motors*/
  roll_mixer(throValue, pidRateY);
  //pitch_mixer(throValue, pidRateX);
  
  
  //DEBUG - To Serial Port
  //Serial.print("#");
  
  Serial.print("SP = ");
  Serial.print(elevValue);
  Serial.print("  PV = ");
  Serial.print(xRate);
  Serial.print("  E = ");
  Serial.println(pRateY);
  
/*
  Serial.print("   THRO: ");
  Serial.print(throValue);
  Serial.println(",");
*/
/*  
  Serial.print("    RUDD: ");
  Serial.print(ruddValue);
  Serial.print(",");
  
  Serial.print("    ALIE: ");
  Serial.print(alieValue);
  Serial.print(",");
  */
  /*
  Serial.print("    ELEV: ");
  Serial.print(elevValue);
  Serial.print(",");
  //Serial.println("");
  */
  /*
  Serial.print("    X: ");
  Serial.print(xAngle);
  Serial.print(",");
  
  
  Serial.print("    Y: ");
  Serial.print(yAngle);
  Serial.print(",");
  
  
  Serial.print("    Z: ");
  Serial.print(zAngle);
  Serial.print(",");
  */
  
  /*
  Serial.print("    X: ");
  Serial.print(xRate);
  Serial.print(",");
  //Serial.println();
  */
  /*
  Serial.print("    Y: ");
  Serial.print(yRate);
  Serial.println();
  */
  /*
  Serial.print("    PID: ");
  Serial.print(pidRateY);
  Serial.print(",");
  Serial.println();
  */
  /*
  Serial.print("    THRO: ");
  Serial.print(throValue);
  Serial.print(",");
  */
  
  /*
  Serial.print("OCR0A = ");
  Serial.print(OCR0A);
  Serial.print("  OCR0B = ");
  Serial.print(OCR0B);
  Serial.print("  OCR2A = ");
  Serial.print(OCR2A);
  Serial.print("  OCR2B = ");
  Serial.print(OCR2B); 
  Serial.println("");
  */
  //Serial.println();
  delay_t1(1);
  reset_watchdog();
  
  
  }
}

