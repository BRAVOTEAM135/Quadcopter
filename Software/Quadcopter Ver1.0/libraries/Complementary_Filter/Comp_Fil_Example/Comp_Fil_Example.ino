#include <Timer_1.h>
#include <Wire.h>
#include <Register_Access.h>
#include <L3G4200D.h>
#include <ADXL345.h>
#include <Complementary_Filter.h>

float lastAngle;


void setup() {
  Serial.begin(9600);
  init_timer();
  init_ADXL345();
  config_ADXL345();
  init_L3G4200D();
  config_L3G4200D();
}

void loop() {
 /*
Serial.print("GYR:  "); 
Serial.print(read_angle_x_L3G4200D());  
Serial.print(" , ");
Serial.print(read_angle_y_L3G4200D());  
Serial.print(" ,  ");
Serial.print(read_angle_z_L3G4200D());  
Serial.print("      ");

Serial.print("ACC:  ");  
Serial.print(read_angle_x_ADXL345());
Serial.print(" , ");
Serial.print(read_angle_y_ADXL345());
Serial.print(" ,  ");
Serial.print(read_angle_z_ADXL345());
Serial.print("      ");
*/
Serial.print("FIL:  ");  
Serial.print(x_angle_comp_fil());
Serial.print(" , ");
Serial.print(y_angle_comp_fil());
Serial.print(" , ");
Serial.print(z_angle_comp_fil());
Serial.print("");

Serial.println("");
delay_1(10);
}


