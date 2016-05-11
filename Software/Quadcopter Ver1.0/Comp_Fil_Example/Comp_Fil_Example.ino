#include <T1.h>
#include <Wire.h>
#include <Register_Access.h>
#include <L3G4200D.h>
#include <ADXL345.h>
#include <Complementary_Filter.h>
#include <Pt_By_Pt_Mean.h>

float lastAngle;


void setup() {
  Serial.begin(115200);
  init_t1();
  init_ADXL345();
  config_ADXL345();
  init_L3G4200D();
  config_L3G4200D();
  
}

void loop() {
  

  // have to call these first before comp filter

read_angle_x_L3G4200D();
read_angle_x_ADXL345();

read_angle_y_L3G4200D();
read_angle_y_ADXL345();

read_angle_z_L3G4200D();
read_angle_z_ADXL345();


Serial.print("#");
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


Serial.print("FILX:  ");  
Serial.print(x_angle_comp_fil());
Serial.print(" , ");

Serial.print("FILY:  "); 
Serial.print(y_angle_comp_fil());
Serial.print(" , ");

Serial.print("FILZ:  "); 
Serial.print(z_angle_comp_fil());
Serial.print(",");

Serial.println("");
delay_t1(5);
}


