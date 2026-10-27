#include <Timer_1.h>

void setup() {
  init_timer();
  Serial.begin(115200);
  }

void loop() {

unsigned long startTime = get_micros();
delay_1(500);
unsigned long finishTime = get_micros();
  Serial.println("original");

Serial.println((finishTime - startTime)/1000.00);
 Serial.println("");
 
 /*
unsigned long startTime2 = get_micros2();
delay_2(500);
unsigned long finishTime2 = get_micros2();
 Serial.println("new");

Serial.println(finishTime2 - startTime2); 
 Serial.println("");
*/
}
