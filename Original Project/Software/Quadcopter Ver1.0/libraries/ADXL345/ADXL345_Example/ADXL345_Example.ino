#include <Wire.h>

#include <Register_Access.h>

#include <ADXL345.h>
#include <Timer_1.h>

void setup() {
init_ADXL345();  
config_ADXL345();
Serial.begin(9600);
}


void loop() {
Serial.print("X: ");  
Serial.print(read_angle_x_ADXL345());

Serial.print("    Y: ");  
Serial.print(read_angle_y_ADXL345());

Serial.print("    Z: ");  
Serial.println(read_angle_z_ADXL345());
//Serial.println("");
delay(10);
}
