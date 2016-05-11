#include <Wire.h>
#include <L3G4200D.h>
#include <Register_Access.h>
#include <T1.h>
#include <Pt_By_Pt_Mean.h>

float oldAngleX = 0;

void setup() {
  init_L3G4200D();
  config_L3G4200D();
  Serial.begin(9600);
}

void loop() {
/*
Serial.println(read_fil_angle_y_L3G4200D(oldAngleX));
oldAngleX = read_fil_angle_y_L3G4200D(oldAngleX);
*/
Serial.print("X: ");  
Serial.print(read_rate_x_L3G4200D()); 

Serial.print("  Y: ");  
Serial.print(read_rate_y_L3G4200D());

Serial.print("  Z: ");  
Serial.print(read_rate_z_L3G4200D());

Serial.print("  X: ");  
Serial.print(read_angle_x_L3G4200D()); 

Serial.print("  Y: ");  
Serial.print(read_angle_y_L3G4200D());

Serial.print("  Z: ");  
Serial.print(read_angle_z_L3G4200D());

Serial.println("");

delay_t1(100);

}
