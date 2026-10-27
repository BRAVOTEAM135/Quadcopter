#include <Wire.h>

#include <Register_Access.h>

#include <ADXL345.h>
#include <T1.h>
#include <Pt_By_Pt_Mean.h>

void setup() {
Serial.begin(9600);
init_ADXL345();  
config_ADXL345();
}


void loop() {
Serial.print("#");  

Serial.print("X: ");  
Serial.print(read_angle_x_ADXL345());
Serial.print(",");

Serial.print("    Y: ");  
Serial.print(read_angle_y_ADXL345());
Serial.print(",");

Serial.print("    Z: ");  
Serial.print(read_angle_z_ADXL345());
Serial.print(",");

Serial.println("");
delay_t1(1);
}
