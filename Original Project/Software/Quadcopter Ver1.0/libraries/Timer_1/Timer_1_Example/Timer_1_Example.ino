#include <Timer_1.h>

void setup() {
  init_timer();
  Serial.begin(9600);
  }

void loop() {
unsigned long startTime = get_micros();
delay_1(1000);
unsigned long finishTime = get_micros();

Serial.println((finishTime - startTime)/1000.00);
 
}
