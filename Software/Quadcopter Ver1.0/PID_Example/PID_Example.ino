#include <Timer_1.h>

#include <PID.h>


#define SET_POINT 5
#define PROCCESS_VARIABLE 2



void setup() {
init_pid();
Serial.begin(9600);
}

void loop() {
Serial.print(p_pid(SET_POINT, PROCCESS_VARIABLE));
//i_pid(SET_POINT,  PROCCESS_VARIABLE);
//d_pid(SET_POINT,  PROCCESS_VARIABLE);

Serial.println("");
}
