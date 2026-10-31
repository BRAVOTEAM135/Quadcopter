# BMP085 component driver

BMP085 uses init(), read(), getStatus() and close(). GY80 owns one and reads it
at up to 10 Hz when enabled. The main IMU path disables pressure. Use GY80's
read result for board-level pressure; do not construct another driver for a chip
already owned by GY80.

For independent use:

```cpp
BMP085 barometer(bus);
PressureSample sample = {};
// After bus.init():
barometer.init(nowUs);
// Every loop pass, including while initializing or busy:
if (barometer.read(sample, nowUs)) {
    // sample.pressurePa, sample.temperatureC, sample.timestampUs
}
DriverStatus status = barometer.getStatus();
```

read() services the shared bus internally and advances a temperature/pressure
cycle without waiting. True copies one new result; false leaves sample unchanged.
Timestamp is the cycle request time. OSS=0 conversions each wait at least 4.5 ms
while releasing the bus for other chips. getStatus() is passive; busy remains
true during those conversion waits.

close() cancels this driver's transfer and clears stored results, leaving the
chip powered and the shared bus open. Keep calling read/bus service to finish a
cancelled STOP or timeout. Repeated close calls are safe; init() reopens.

Bosch's integer compensation and calibration validation are retained. Simulator
checks cover the independent 69964 Pa / 15 degrees C reference, conversion waits,
invalid calibration and board-level motion/pressure coordination. Hardware
checks are still required for electrical and execution timing.
