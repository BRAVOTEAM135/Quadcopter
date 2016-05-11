#include <T1.h>

#include <Motors.h>


void setup() {
  Serial.begin(9600);
  init_t1();
  init_motors();
  config_motors();

   
  Serial.println("Please Select one of the following options:");
  Serial.println("a: Arms the ESC (125)");
  Serial.println("b: Throttle Down");
  Serial.println("c: Throttle up");
  Serial.println("d: E-Stop");
  
 }

void loop() {
  while (Serial.available() > 0) {
    char val = Serial.read();
    //Serial.println(val);
    
    // If the letter 'a' is input
    if (val == 'a'){
      Serial.println("a: Arm ESC");
      
      write_motor_1(100);   
      write_motor_2(100);  
      write_motor_3(100);  
      write_motor_4(100);  
     }
     
    // If the letter 'b' is input
    else if (val == 'b'){
      Serial.println("Throttle Down");
      
      for(int i = 100; i > 0; i--){
        Serial.println(i);
        write_motor_1(i);   
        write_motor_2(i);  
        write_motor_3(i);  
        write_motor_4(i);
        delay_t1(100);
        //delay(10000); //Delay is large because the timer that deal with this function has been modified to generate pwm
       }
     }
     
    else if (val == 'c'){
      Serial.println("Throttle Up");
      
      for(int i = 0; i <= 255; i++){
        Serial.println(i);
        write_motor_1(i);   
        write_motor_2(i);  
        write_motor_3(i);  
        write_motor_4(i);
        delay_t1(100);
        //delay(10000);
        }
      }
     
    if (val == 'd'){
      Serial.println("a: Arm ESC");
      write_motor_1(135);   
      write_motor_2(135);  
      write_motor_3(135);  
      write_motor_4(135);  
    }
  }
  
}
