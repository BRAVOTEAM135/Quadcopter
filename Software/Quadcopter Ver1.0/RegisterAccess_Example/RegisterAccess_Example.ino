#include <Register_Access.h>
#include <Wire.h>

#define DEVICE_ADDRESS  0x53
#define REG_ADDRESS     0x00


void setup() {
  Serial.begin(9600);
  Serial.println("Alive");
  init_reg();
  }

void loop() {
  Serial.println("Alive again");
  int temp = read_reg(DEVICE_ADDRESS,REG_ADDRESS);
  Serial.println(temp);
  delay(500);
  }
