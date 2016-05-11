#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "String_To_Float.h"

void setup(){
Serial.begin(9600);
Serial.println("CONNECTED");
}


void loop(){
  float val = read_serial(); // read_serial will read the incoming number from the command line
  
  if (val >= 0){
    Serial.println(val,6);
  }
  
  delay(10);
}
