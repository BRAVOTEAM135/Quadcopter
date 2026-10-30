#define IMU_SENSOR GY80 // Select GY80 or MPU6050 (GY-521).

#include <Arduino.h>
#include "myMath.h"
#include "ledBlink.h"
#include "serialPort.h"
#include "T1.h"
#include "scheduler.h"
#include "i2cBus.h"
#include "imu.h"
#include "GY80.h"
#include "MPU6050.h"
#include "imuFilter.h"


ledBlink led;
SerialPort serialPort;
T1 t1;
i2cBus bus;
IMU_SENSOR imuSensor(bus);
IMU imu(imuSensor);
ImuFilter imuFilter;


void setup() {
  serialPort.init(115200);

  t1.init();

  led.init();
  led.ledOff();

  bus.init();
  imu.begin(t1.get_micros()); // Initialize motion sensors and any available compass.
  imuFilter.begin(imu.hasMagnetometer()); // Average a stationary startup zero.

}

void loop() {

    // Keep scheduled tasks running while serial output is pending.
    scheduler_run();
}
