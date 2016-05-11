#include <T1.h>
#include <Motors.h>
#include <String_To_Float.h>

void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
init_t1();
init_motors();
config_motors();
/*
Serial.print("TCCR0A = ");
Serial.print (TCCR0A, BIN);
Serial.println("");

Serial.print("TCCR0B = ");
Serial.print (TCCR0B,BIN);
Serial.println("");

Serial.print("TCCR2A = ");
Serial.print (TCCR2A,BIN);
Serial.println("");

Serial.print("TCCR2B = ");
Serial.print (TCCR2B,BIN);
Serial.println("");
*/
/*
write_motor_1(0);
write_motor_2(0);
write_motor_3(0);
write_motor_4(0);
*/

  
Serial.println("READY");
Serial.println();
}

void loop() {
  // put your main code here, to run repeatedly:
   /*----- DEBUG: READ FROM SERIAL -----*/
   volatile float prevValue;
  float value = read_serial();
  if (value >= 0){
    Serial.print("Input = ");
    Serial.println(value,6);
    

    
   write_motor_1(value);
   write_motor_2(value);
   write_motor_3(value);
   write_motor_4(value);
   
   /*
   Serial.print("OCR0A = ");
    Serial.println(OCR0A);
    
    Serial.print("OCR0B = ");
    Serial.println(OCR0B);
    
    Serial.print("OCR2A = ");
    Serial.println(OCR2A);
    
    Serial.print("OCR2B = ");
    Serial.println(OCR2B);
    
    Serial.println();
    */
    prevValue = value;
  }
  else {
  value = prevValue;
  }
  /*------------------------------------*/
   
   write_motor_1(value);
   write_motor_2(value);
   write_motor_3(value);
   write_motor_4(value);
   
   
   delay_t1(5);
}
