/* WARNING!!!! Ensure saftey fail safe iis anabled on the reciver
in case of signal loss or corruption */

#include <Radio.h>
#include <T1.h>
#include <Pt_By_Pt_Mean.h>
/* NOTE: PinChangeInt.h is included as part of the radio libary and does not need to be included in this level,
this is okk because radio is the only libary that requires the PinChangeInt */
float prev;
float curr;

void setup() {
init_radio();
Serial.begin(115200);
}

void loop() {
read_radio();
  
curr = get_count_t1()*0.5;  
Serial.print("DT: ");
Serial.print(curr - prev);
prev = curr;
Serial.print("  THRO: ");  
Serial.print(read_thro());

Serial.print("  RUDD: ");  
Serial.print(read_rudd());

Serial.print("  ELEV: ");  
Serial.print(read_elev());

Serial.print("  ALIE: ");  
Serial.print(read_alie());
Serial.println("");

delay_t1(100);
}

