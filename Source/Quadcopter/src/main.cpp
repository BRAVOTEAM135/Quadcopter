#include <Arduino.h>
#include "myMath.h"
#include "ledBlink.h"
#include "serialPort.h"
#include "T1.h"
#include "scheduler.h"

ledBlink led;
SerialPort serialPort;
T1 t1;

void setup() {
  // put your setup code here, to run once:
  serialPort.init(115200);

  t1.init();

  led.init();
  led.ledOff();
  
  // Startup message
  serialPort.println("Quadcopter MKII Ready");

  // Test mathematical function
  int result = add(10, 21);

  serialPort.print("Math Result: ");
  serialPort.println(result);

  // Floating-point test
  serialPort.print("Altitude: ");
  serialPort.println(125.567f, 2);
}

void loop() {
  // put your main code here, to run repeatedly:
  scheduler_run();
  /*
  led.ledToggle();
  t1.delay(100);
  led.ledToggle();
  t1.delay(100);
  led.ledToggle();
  t1.delay(100);
  led.ledToggle();
  t1.delay(500);
  */

}

