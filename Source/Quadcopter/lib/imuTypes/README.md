# Sensor drivers

`GY80` and `MPU6050` use the existing `i2cBus` hardware driver. Sensor drivers
contain register addresses, identity checks, initialization and unit conversion;
only `i2cBus.cpp` accesses the ATmega328P TWI peripheral.

The current application uses the common `IMU` wrapper in `lib/imu`; see its
[usage guide](../imu/README.md) for hardware selection and scheduler calls.
A separate direct-driver readout example is in `examples/imuReadout`.

## Build and run the example

From the repository's PlatformIO project directory:

```powershell
platformio run --project-dir examples/imuReadout -e gy80
platformio run --project-dir examples/imuReadout -e mpu6050
```

Upload the matching environment with `--target upload`, then open its serial
monitor at 115200 baud. Its upload port defaults to COM3. The example prints
initialization status and motion CSV at 10 Hz while requesting samples at 200 Hz.
CSV acceleration is in mm/s² and angular velocity is in mrad/s.

## Common motion API

```cpp
#include "GY80.h"       // Or "MPU6050.h".

i2cBus i2cBus;
GY80 imu(i2cBus);       // Or MPU6050 imu(i2cBus).

// During setup, once:
i2cBus.init();
imu.begin(micros());

// During every loop iteration, using the same microsecond clock throughout:
uint32_t nowUs = micros();
i2cBus.service(nowUs);
imu.service(nowUs);

// In the scheduled 5 ms acquisition task, after initialization:
if (imu.ready()) {
    imu.requestSample(nowUs);
}

// Once acquisition completes, pass the new measurement to the IMU/fusion layer:
ImuSample sample;
if (imu.takeSample(sample)) {
    // sample.accelerationMps2: m/s², including gravity.
    // sample.angularVelocityRadS: radians/s.
    // sample.timestampUs: host read start time.
}
```

The snippets show where each call belongs, rather than a complete `main.cpp`.
The readout example is a complete program. Initialize the shared bus once, call
its `service()` frequently, and service every driver whose `begin()` you called.
No driver uses `delay()`, busy-waits for data, or allocates heap memory.

`begin()` starts initialization; its return value means the request was accepted,
not that the sensor is ready. `ready()` means initialization succeeded.
`requestSample()` queues an acquisition, returning false if that driver's motion
job is busy or a previous motion sample is unread. A busy shared bus is retried
by `service()` automatically. `takeSample()` returns each completed sample once.
Polling before hardware data is ready completes without publishing a duplicate.

`error()` exposes `Nack`, `BusError`, `Timeout`, `WrongDevice` or `InvalidData`.
An initialization failure requires another `begin()` to retry. An acquisition
failure drops that acquisition; a subsequent accepted request clears the error
and retries. `busy()` includes initialization, queued jobs and conversion waits;
GY80 motion requests are still allowed during pressure conversion waits.

Use one stable instance of each selected driver: copying is disabled because the
instance owns its receive buffer and uses its address to identify bus results.
Do not destroy an instance or reinitialize the bus during an active transaction.

## Configuration

Both motion drivers default to ±2 g and ±250 degrees/s with 200 Hz sensor output.
GY80 uses the L3G4200D's 50 Hz low-pass setting; MPU6050 uses DLPF setting 3.
Startup, reset and settling waits are state-machine timers. GY80 waits 300 ms
after configuration for gyro settling; MPU6050 waits 100 ms at power-up, after
reset, and after waking.

GY80 uses ADXL345 at `0x53` and L3G4200D at `0x69`. Override strap-selected
addresses with `GY80 imu(bus, 0x1D, 0x68)`. MPU6050 defaults to `0x68`, with
`MPU6050 imu(bus, 0x69)` available for AD0 high. Devices sharing a bus must have
different addresses. Identity checks reject other chips, including substitutes
such as MPU6500 or QMC5883L; there is no automatic chip detection/fallback.

Measurements retain the individual chips' datasheet axis conventions. GY80
acceleration and gyro measurements are separate transactions, not simultaneous
samples. Mounting alignment and sensor scale calibration still need verification.
The separate `imuFilter` estimates stationary gyro bias internally and zeros
starting angles; these drivers keep their raw scaled readings unchanged.

## GY80 compass and barometer

By default GY80 also checks and initializes HMC5883L (`0x1E`) and BMP085 (`0x77`).
For a motion-only board or development stage, use `imu.begin(nowUs, false, false)`.
Either optional device can be enabled separately through those two booleans.

Use `requestMagneticField(nowUs)` and `takeMagneticField(MagneticSample&)` for
compass measurements. They return X/Y/Z in microtesla at the configured 75 Hz
output rate and ±1.3 gauss range. Overflow readings are rejected. A locked partial
reading is drained without publishing it, allowing the next measurement to run.

Use `requestPressure(nowUs)` and `takePressure(PressureSample&)` for compensated
pressure in pascals and temperature in °C. The BMP085 calibration coefficients
are read during initialization. OSS=0 temperature and pressure conversions each
wait at least 4.5 ms, releasing the bus between transfers. Motion requests can run
during these waits. The pressure timestamp is the request time for the combined
temperature/pressure cycle. Request auxiliary measurements at their own slower
cadence and retry rejected requests. Unread results are preserved.

## I2C register transfers

The original `startProbe()` / `takeProbeResult()` API remains available. Added:

```cpp
bus.readRegisters(address, registerAddress, buffer, length, nowUs, owner);
bus.writeRegister(address, registerAddress, value, nowUs, owner);
bus.takeResult(result, owner);
```

Reads use write-register / repeated-START / read, ACK all but the final byte, and
wait for STOP completion. Up to 32 bytes can be read per transaction. Writes
copy a single register value. Transactions time out after 5 ms. The read buffer
must remain valid and untouched until the matching result is consumed, including
after errors. The bus accepts one transaction at a time and remains reserved
until its owner consumes the result. A different owner or the probe API cannot
consume a register transaction's result. Drivers pass their `this` pointer as
the owner token. Direct callers may omit the token for ownerless transactions.

## Validation

```powershell
python test/sensor_drivers/run_tests.py
```

This compiles the actual drivers with the PlatformIO AVR toolchain and runs them
on AVR-GDB's CPU simulator. Fake TWI registers let tests inject hardware status
transitions. The suite covers byte transfers, repeated START, ACK/NACK sequencing,
result ownership, busy rejection, STOP completion, bus errors, timeouts across
clock wraparound, initialization, freshness, signed scaling, compass axis order
and Bosch's 69964 Pa / 15 °C reference example. This is a protocol/CPU test;
electrical timing and the connected boards require hardware validation.

## Register references

- [Analog Devices ADXL345 datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/ADXL345.pdf)
- [ST L3G4200D datasheet, manufacturer PDF mirror](https://www.pololu.com/file/0J491/L3G4200D.pdf)
- [ST AN3393, gyro operating modes and turn-on timing, manufacturer PDF mirror](https://vctec.co.kr/web/product/elcw/img/SMS34200D-8.pdf)
- [Honeywell HMC5883L datasheet, manufacturer PDF mirror](https://cdn.sparkfun.com/datasheets/Sensors/Magneto/HMC5883L-FDS.pdf)
- [Bosch BMP085 datasheet, manufacturer PDF mirror](https://cdn-shop.adafruit.com/datasheets/BST-BMP085-DS000-06.pdf)
- [InvenSense MPU6000/MPU6050 register map, manufacturer PDF mirror](https://cdn.sparkfun.com/datasheets/Sensors/Accelerometers/RM-MPU-6000A.pdf)
