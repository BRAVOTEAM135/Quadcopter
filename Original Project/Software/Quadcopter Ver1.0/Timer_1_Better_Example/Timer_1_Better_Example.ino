#include "T1.h"

void setup() {
  // put your setup code here, to run once:
  config_t1();
  
  Serial.begin(115200);
Serial.println("START");
  //Output a header of info:
  Serial.println(F("Notes:"));
  Serial.println(F("micros() has a precision of 4us"));
  Serial.println(F("get_count() with unsigned long final data type has a final precision of 1us, and is fast"));
  Serial.println(F("get_count() with float final data type has a final precision of 0.5us, and is not quite as fast"));
  Serial.println(F("get_micros() has a precision of 0.5us, and is slower than the above 2 methods, so one of the above 2 methods is preferred"));
  Serial.println(F("=============================================="));

}

void loop() {
  // put your main code here, to run repeatedly:
static unsigned long start_time = get_count_t1();
  
  unsigned long t_micros = micros();
  unsigned long end_time = get_count_t1();
  float mymicros = get_micros_t1();

  if((end_time - start_time)/2 >= 1000003)  {
    start_time = end_time;
    
    Serial.print("micros() = "); Serial.print(t_micros); Serial.println(" us");
    Serial.print("get_count() = "); Serial.print(end_time); Serial.println(" (units of 0.5 us per count)");
    Serial.print("get_count()/2 = "); Serial.print(end_time/2); Serial.println(" us");
    Serial.print("get_count()/2.0 = "); Serial.print(end_time/2.0); Serial.println(" us");
    Serial.print("get_micros() = "); Serial.print(mymicros); Serial.println(" us");
    
    Serial.println(""); //add an extra line space
}
}
