# imuFilter

`ImuFilter` estimates roll, pitch and yaw relative to the stationary starting
pose. It combines accelerometer, gyroscope and optional magnetometer samples
without accessing hardware, timers or serial output.

## Startup zero

Call `imuFilter.begin(imu.hasMagnetometer())` once in setup. Keep supplying
samples while the board stays still for about two seconds after sensor
initialization. This averaging period never blocks the scheduler.

The filter averages 400 motion samples for stationary gyro bias and gravity.
With a compass enabled it also requires at least 20 fresh magnetic samples.
Movement or a data gap restarts averaging. These motion checks are heuristics:
slow steady rotation can resemble gyro bias, so the board must actually be still.

After averaging, `calibrated()` becomes true and the starting angles are `0,0,0`.
The filter subtracts estimated gyro bias internally. Raw IMU samples remain
unchanged, and acceleration still includes gravity. Zeroing does not remove
random noise or calibrate accelerometer scale factors.

## Scheduler use

The existing scheduler already performs these calls in this order:

```cpp
MagneticSample magnetic;
if (imu.takeMagneticField(magnetic)) {
    imuFilter.updateMagneticField(magnetic);
}
ImuSample sample;
if (imu.takeSample(sample)) {
    imuFilter.update(sample);
}
const ImuOrientation& angles = imuFilter.orientation();
if (angles.valid) {
    // angles.rollDeg(), angles.pitchDeg(), angles.yawDeg()
    // Or angles.rollRad, angles.pitchRad, angles.yawRad for control calculations.
}
```

Serial prints `roll,pitch,yaw` in degrees after zeroing completes. `orientation()`
returns a reference to the stored result; it does not read sensors or run fusion.
Call `reset()` explicitly to choose a new zero. It restarts averaging and retains
any supplied compass calibration.

## How it works

The gyro tracks fast rotation. Accelerometer feedback corrects tilt with an
approximately 0.5 second correction time. New compass samples correct heading
around the estimated vertical axis with an approximately 1 second correction
time. Projecting the magnetic field onto the horizontal plane compensates for
tilt. Each compass sample is used once.

A quaternion stores the internal 3D rotation. The output rotation is
`inverse(startingRotation) * currentRotation`, allowing inverted or tilted
starting poses to be zero. Subtracting three Euler angles would not implement
the same reference. Euler conversion happens when publishing the result.

These gains are initial bench settings. Inputs must share the same right-handed
body axes: acceleration in m/s^2, gyro in rad/s, magnetic field in microtesla,
and timestamps from the same clock. Drivers preserve chip axes; verify physical
chip/mounting alignment on the actual board. Swapping boards does not establish
that alignment automatically.

## Compass calibration and yaw

GY80 uses HMC5883L compass data acquired by `IMU` at up to 25 Hz. MPU6050 alone
has no magnetometer: its yaw uses bias-corrected gyro data and can still drift.
`yawReferenced` indicates that a recent usable compass reading is correcting
yaw. It does not mean that the compass is calibrated or that yaw is true north;
all displayed angles remain relative to the starting pose.

Startup averaging cannot measure hard-iron offsets or soft-iron distortion.
Those require rotating the board and deriving calibration from measurements.
Default compass correction is identity; `magneticCalibrated()` is false.
Supply externally measured `MagneticCalibration` values through
`setMagneticCalibration()` before startup averaging when available:

```text
correctedField = matrix * (rawField - offsetMicrotesla)
```

The offset corrects hard iron; the 3x3 matrix supports scale and cross-axis
correction. Supplying calibration restarts zeroing. The filter rejects
non-finite/singular calibration, magnetic overflow, very weak/strong fields and
field-strength changes beyond 30% of the startup value. These checks cannot
detect every magnetic disturbance. Nearby metal or electrical equipment can
still alter heading. Compass feedback limits gyro drift when compass data is
reliable; it does not guarantee perfectly stationary, drift-free output.

## Timing and validity

- `update()` returns true only for a usable new orientation, and false during
  startup averaging or for rejected inputs/timing.
- Duplicate/backwards timestamps are ignored. Unsigned timestamp differences
  handle normal clock wraparound.
- A gap over 50 ms or non-finite motion input invalidates tracking. With fresh
  compass and suitable gravity data the absolute rotation can be reconstructed
  while preserving the original zero. Without a compass, lost yaw motion cannot
  be recovered; explicitly restart zeroing with `reset()`.
- Compass data older than 200 ms stops yaw correction and clears `yawReferenced`.
  Normal gyro tracking continues while motion samples arrive.
- `valid` and `yawReferenced` describe the stored result and do not age by
  themselves. Compare `timestampUs` with current Timer1 time to detect old data.
- Tilt feedback is skipped outside 0.8-1.2 g. Acceleration inside this band can
  still contain motion, so sustained linear acceleration can corrupt tilt.
- Roll/yaw wrap at +/-180 degrees. Relative Euler pitch is limited to +/-90
  degrees; `valid` is false at its singularity while the quaternion keeps tracking.

No heap allocation, hardware waits or blocking calibration loops are used.
Updates have bounded work, including software floating-point/math-library calls.
Execution time at the requested 200 Hz must be measured on the Uno; compilation
and CPU simulation do not establish real-time timing margins.

## Validation

Run `python test/sensor_drivers/run_tests.py`. Separate AVR simulator suites
exercise the drivers and actual filter. Independent Euler rotation fixtures
cover stationary bias, movement rejection, inverted zeroing, angle/clock wrap,
tilted compass correction, changing gyro bias, magnetic calibration, compass
loss and recovery across data gaps. Hardware measurements are still required
to verify axes, magnetic calibration and timing.

## References

- [NXP AN4248: tilt-compensated electronic compass and magnetic calibration](https://www.nxp.com/docs/en/application-note/AN4248.pdf)
- [NXP AN3461: accelerometer tilt equations and axes](https://www.nxp.com/docs/en/application-note/AN3461.pdf)

This implementation uses its own quaternion and vector equations.
