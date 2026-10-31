# Sensor drivers and measurement types

Measurement drivers expose init(), read(), getStatus() and close(). Call read()
on every loop pass, including while busy or initializing. It advances hardware
without waiting; true copies one fresh result, false leaves the output unchanged.
getStatus() returns DriverStatus with ready, busy and DriverError error fields;
it does not service hardware, collect results or start requests.

| Driver | Output of read(sample, nowUs) |
| --- | --- |
| ADXL345 | AccelerationSample: accelerationMps2, timestampUs |
| L3G4200D | GyroscopeSample: angularVelocityRadS, timestampUs |
| HMC5883L | MagneticSample: fieldMicrotesla, timestampUs |
| BMP085 | PressureSample: pressurePa, temperatureC, timestampUs |
| MPU6050 | ImuSample: accelerationMps2, angularVelocityRadS, timestampUs, temperatureC |
| GY80 | Gy80Sample: motion, magnetic, pressure and per-call fresh flags |
| IMU | ImuReading: motion and orientation |

All timestamps use the caller's microsecond clock. Motion timestamps are host
read-start times; pressure timestamp is the combined conversion request time.
Axes follow chip datasheets before mounting alignment or calibration. Raw
acceleration includes gravity and raw gyro retains its bias. Temperature in
ImuSample is supplied by MPU6050; GY80 leaves it zero and supplies temperature
in its PressureSample instead.

GY80 owns the ADXL345, L3G4200D, HMC5883L and BMP085 drivers. One read call handles
initialization order, shared-bus access and the different acquisition rates.
See [GY80](../GY80/README.md), [IMU](../imu/README.md) and
[BMP085](../BMP085/README.md) for use. The filter stays hardware-independent.

## Configuration and examples

Motion remains +/-2 g, +/-250 degrees/s and 200 Hz. GY80 gyro uses the 50 Hz
low-pass setting; MPU6050 uses DLPF setting 3. Gyro settling is a nonblocking
300 ms wait; MPU6050 power-up/reset/wake waits are each 100 ms. Identity checks
reject unsupported substitutes rather than guessing the connected chip.

GY80 defaults to the whole board; init(nowUs, true, false) disables pressure,
and init(nowUs, false, false) selects motion only. Independent chip drivers can
be used on a shared initialized bus. Use one instance per physical device.

```powershell
platformio run --project-dir examples/imuReadout -e gy80
platformio run --project-dir examples/imuReadout -e mpu6050
```

The example polls read continuously and prints motion at 10 Hz in mm/s^2 and
mrad/s. Upload settings remain in its platformio.ini.

## Transport and lifecycle

Only i2cBus.cpp accesses TWI hardware. Chip read() calls its service internally;
the bus register read/write/probe functions remain transport primitives used by
the chip state machines. A transaction reserves the bus until its own driver
collects the result. Transfers time out after 5 ms, including across clock wrap.

close() cancels only a chip's own transfer and discards its readings. Cancellation
detaches the receive buffer immediately and finishes STOP or timeout on later
read/bus service calls. Close sensors before the shared bus, which disables TWI.
Repeated close is safe and init() permits reuse. Chips stay powered. Application
snapshots remain copies; inspect status and timestamps before control use.

## Validation

```powershell
python test/sensor_drivers/run_tests.py
```

Suites use actual AVR code and hardware-side fixtures to check bus protocol and
ownership, four-method chip APIs, stale data and errors, board rate coordination,
startup/filter mathematics, peripheral lifecycle and both IMU selections.
CPU simulation does not establish electrical timing or real-time CPU margins.

## Register references

- [Analog Devices ADXL345 datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/ADXL345.pdf)
- [ST L3G4200D datasheet, manufacturer PDF mirror](https://www.pololu.com/file/0J491/L3G4200D.pdf)
- [ST AN3393, gyro operating modes and turn-on timing, manufacturer PDF mirror](https://vctec.co.kr/web/product/elcw/img/SMS34200D-8.pdf)
- [Honeywell HMC5883L datasheet, manufacturer PDF mirror](https://cdn.sparkfun.com/datasheets/Sensors/Magneto/HMC5883L-FDS.pdf)
- [Bosch BMP085 datasheet, manufacturer PDF mirror](https://cdn-shop.adafruit.com/datasheets/BST-BMP085-DS000-06.pdf)
- [InvenSense MPU6000/MPU6050 register map, manufacturer PDF mirror](https://cdn.sparkfun.com/datasheets/Sensors/Accelerometers/RM-MPU-6000A.pdf)
