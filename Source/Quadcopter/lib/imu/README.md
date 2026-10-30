# IMU interface

`IMU` gives the application and scheduler one accelerometer/gyroscope API for
either GY80 or MPU6050. It forwards calls to one selected sensor driver and adds
no heap allocation, blocking waits, calibration or fusion. It also requests
GY80 compass measurements independently of motion acquisition.

The selector is at the top of `src/main.cpp`:

```cpp
#define IMU_SENSOR GY80
```

Change `GY80` to `MPU6050` for the GY-521 and rebuild. The scheduler uses the
same IMU calls for either selection. There is no build flag to set in the main
project. The selection uses each driver's default I2C addresses. GY80's compass
is enabled for yaw correction; its barometer remains disabled. Initialization
therefore also requires the HMC5883L compass to respond with the expected ID.

The application owns the bus and one selected sensor driver. `IMU` holds
pointers to that driver and forwards calls; it does not create a second driver
or allocate heap memory. Construct the bus, then the driver, then the IMU, and
keep them alive while the system runs. Copying the IMU instance is disabled.

A `#define` in `main.cpp` affects only that file. It replaces `IMU_SENSOR` with
the selected class name when declaring `imuSensor`. C++ then chooses the IMU
constructor accepting that driver type. `imu.cpp` and `scheduler.cpp` therefore
do not depend on seeing the macro, and the IMU class has the same layout in
every source file.

```cpp
i2cBus bus;
IMU_SENSOR imuSensor(bus);
IMU imu(imuSensor);

// Once in setup(), after Timer1 initialization:
bus.init();
imu.begin(t1.get_micros());

// Every scheduler pass, using the same Timer1 clock throughout:
uint32_t nowUs = t1.get_micros();
bus.service(nowUs);
imu.service(nowUs);

// In the existing 5 ms task:
if (imu.ready()) {
    imu.requestSample(nowUs);
}

// Collect finished acquisitions between task releases:
ImuSample sample;
if (imu.takeSample(sample)) {
    // Process the new sample here.
}
```

These snippets show where calls belong; the working integration is in
`src/main.cpp` and `lib/scheduler/scheduler.cpp`.

`begin()` returns whether initialization was accepted. `ready()` indicates
successful initialization; it does not mean a new sample is available.
`requestSample()` returns immediately, rejecting requests when the driver is
busy, uninitialized, or holding an unread sample. `takeSample()` returns true
once per completed sample. `error()` forwards the selected driver's error;
initialization errors require another `begin()` to retry.

`hasMagnetometer()` is true for GY80 and false for MPU6050 alone. `service()`
automatically requests compass data at up to 25 Hz when the GY80 driver is free.
Consume each result using `takeMagneticField(MagneticSample&)`; unread results
are preserved. Pass these samples to `imuFilter.updateMagneticField()` before
passing new motion samples to `imuFilter.update()`. The current scheduler
already does this. MPU6050's `takeMagneticField()` always returns false.

`ImuSample` is unchanged: acceleration in m/s^2 (including gravity), angular
velocity in rad/s, and a host read-start timestamp in microseconds. Axes remain
in each chip's datasheet convention. Mounting alignment and calibration are
separate concerns; swapping boards does not guarantee matching physical axes or
corrected raw offsets. This wrapper does not estimate roll, pitch or yaw angles.
The [imuFilter](../imuFilter/README.md) now averages stationary gyro bias and
zeros the starting angles internally; it does not modify these raw samples.
