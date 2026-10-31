# IMU

IMU combines one selected sensor with the orientation filter. Its normal driver
interface is init(), read(), getStatus() and close(). The application supplies
the shared bus and a timestamp from the same microsecond clock on every call.

```cpp
i2cBus bus;
IMU imu(bus);
ImuReading reading = {};

// Once in setup:
bus.init();
imu.init(t1.get_micros());

// Every loop pass, even while initializing or busy:
uint32_t nowUs = t1.get_micros();
if (imu.read(reading, nowUs)) {
    // reading.motion: new raw scaled acceleration, gyro and timestamp.
    // reading.orientation: current angles; may be invalid during startup.
}
DriverStatus status = imu.getStatus();
```

True means new motion was returned, including during stationary startup
averaging. False leaves reading unchanged. Always call read frequently: it
services transfers, schedules acquisition and feeds fresh samples into the
filter once. getStatus() only inspects state and does not advance hardware.

status.ready means hardware and startup averaging are complete. busy includes
initialization, acquisition and startup averaging. error reports the sensor's
latest error. Check orientation.valid AND timestampUs before using angles for
control; an old caller-owned snapshot does not expire automatically.

GY80 is default. Enable IMU_USE_MPU6050 in platformio.ini for GY-521. GY80 motion
and compass are enabled; pressure is disabled in this orientation-only path.
A direct GY80 object provides the whole board through one read; see its guide.
Motion is requested at up to 200 Hz and GY80 compass at up to 25 Hz.

Raw motion retains bias and chip axes: acceleration in m/s^2 including gravity,
gyro in rad/s. motion.temperatureC is MPU6050 chip temperature; GY80 motion leaves
that field zero and exposes temperature through its pressure reading instead.
The filter subtracts estimated gyro bias internally and reports angles relative
to the averaged stationary starting pose. Keep the board still for about two
seconds after hardware initialization. MPU6050 has no compass and yaw can drift.

reset() explicitly restarts zeroing if open. setMagneticCalibration() validates
external compass correction and restarts zeroing if open. These configuration
operations are separate from the four normal driver calls.

close() stops the sensor/filter and leaves the bus open and chips powered. Call
init() to reuse. Caller-owned snapshots remain copies; inspect status to detect
closure. Continue read/bus service to complete cancellation STOP or timeout.
Repeated close calls are safe; reset() does not reopen a closed IMU.
