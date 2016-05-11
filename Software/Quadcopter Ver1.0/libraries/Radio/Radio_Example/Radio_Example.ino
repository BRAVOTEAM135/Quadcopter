#include <Radio.h>
#include <Timer_1.h>
/* NOTE: PinChangeInt.h is included as part of the radio libary and does not need to be included in this level,
this is okk because radio is the only libary that requires the PinChangeInt */

void setup() {
init_radio();
Serial.begin(9600);
}

void loop() {
Serial.print("THRO: ");  
Serial.print(read_thro());

Serial.print("  RUDD: ");  
Serial.print(read_rudd());

Serial.print("  ELEV: ");  
Serial.print(read_elev());

Serial.print("  ALIE: ");  
Serial.print(read_alie());

Serial.println("");

delay_1(100);
}

