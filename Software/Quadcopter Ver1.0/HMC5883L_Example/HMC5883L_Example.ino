#include <Register_Access.h>
#include <HMC5883L.h>
#include <Wire.h>

void setup() {
  Serial.begin(9600);
  init_HMC5883L();
  config_HMC5883L();
}

void loop() {
 Serial.print("RAD: "); 
 Serial.print(rad_heading_HMC5883L());
 Serial.print("  DEG: ");
 Serial.print(deg_heading_HMC5883L()); 
 
 /*
 Serial.print(" X: ");
 Serial.print(scale_x_HMC5883L()); 
 Serial.print(" Y: ");
 Serial.print(scale_y_HMC5883L()); 
 Serial.print(" Z: ");
 Serial.println(scale_z_HMC5883L()); 
 */
 
 Serial.println(""); 
  delay(100);
}
