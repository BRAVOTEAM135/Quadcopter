#include <T1.h>
#include <Motors.h>

void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600);
  init_t1();
  init_motors();
  config_motors();

}

void loop() {
  // put your main code here, to run repeatedly:
      write_motor_1(238);   
      write_motor_2(238);  
      write_motor_3(238);  
      write_motor_4(238);  
}
