# GY80 board

GY80 owns ADXL345 acceleration, L3G4200D gyro, HMC5883L compass and BMP085 pressure
component drivers. Each chip handles its own registers. The board schedules and
combines their measurements; the application uses init/read/getStatus/close.

```cpp
i2cBus bus;
GY80 board(bus);
Gy80Sample sample = {};

// Once during setup:
bus.init();
board.init(nowUs); // All four chips by default.

// On EVERY loop pass, including while initializing or busy:
if (board.read(sample, nowUs)) {
    if (sample.motionFresh) { /* sample.motion: acceleration + gyro */ }
    if (sample.magneticFresh) { /* sample.magnetic: compass */ }
    if (sample.pressureFresh) { /* sample.pressure: pressure + temperature */ }
}
DriverStatus status = board.getStatus();
```

read() services the bus internally. It never waits for a byte or conversion.
True copies a snapshot with fresh flags identifying channels completed during
that call. False leaves sample unchanged; ignore its old fresh flags on false.
The latest complete motion pair is retained during partial acceleration/gyro
reads, including when another channel publishes a result in between.

Motion runs at up to 200 Hz, compass at up to 25 Hz and pressure at up to 10 Hz.
Periods are #define constants at the top of GY80.cpp. Compass reads run between
motion acquisitions. Pressure conversion waits release the bus so motion can
continue. A stale accelerometer or gyro never produces a partial motion pair.
The motion timestamp is the acceleration data read start time.

`init(nowUs, true, false)` enables motion/compass only; `init(nowUs, false, false)`
enables motion only. Disabled chips are neither initialized nor sampled.
getStatus().ready means all enabled hardware initialized; busy includes active
reads and conversion waits. getStatus() does not progress work or consume data.
Initialization errors require init() again; read errors can retry on later polls.

All component drivers can also be used independently with the same four methods:

| Driver | read() output |
| --- | --- |
| ADXL345 | AccelerationSample: m/s^2, including gravity |
| L3G4200D | GyroscopeSample: rad/s |
| HMC5883L | MagneticSample: microtesla, X/Y/Z |
| BMP085 | PressureSample: pascals, degrees C |

Default addresses are 0x53, 0x69, 0x1E and 0x77 respectively. GY80 constructor
address arguments select ADXL345 0x53/0x1D and L3G4200D 0x69/0x68. Use one driver
per physical chip, including those already owned by a GY80 object.

close() closes all four chips, clears stored readings and leaves the bus open.
Repeated close calls are safe; call init() to reopen. Chips stay powered. Continue
read/bus service to finish a cancelled STOP or timeout. Stored application copies
are not erased by close(). Axis alignment, calibration and timing need hardware
verification; orientation calculations remain in imuFilter.
