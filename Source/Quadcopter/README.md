# Quadcopter

Start with `src/main.cpp`. Hardware setup, task periods and the control,
heartbeat and serial sections are visible together there.

## Driver interface

Measurement drivers use four main operations:

| Function | Purpose |
| --- | --- |
| `init(nowUs)` | Start initialization; true means accepted, not finished |
| `read(sample, nowUs)` | Advance work without waiting; true returns a fresh reading |
| `getStatus()` | Inspect `ready`, `busy` and `error` without changing state |
| `close()` | Stop the driver and discard internal readings |

Call `read()` on every loop pass, including while initializing or busy. It
services its shared bus and progresses any pending transfer internally. A false
return leaves the caller's sample unchanged. Process a sample only when read
returns true; check stored timestamps before using old readings for control.
Do not wait for `busy` to clear before calling read: polling read makes progress.

```cpp
AccelerationSample acceleration;
if (accelerometer.read(acceleration, currentTime)) {
    // Use the fresh acceleration in m/s^2.
}
DriverStatus status = accelerometer.getStatus();
// status.ready, status.busy, status.error
```

Constructors only connect software objects. Initialize the shared bus first.
Measurement drivers also expose `getStatus()`. UART and LED retain their output commands;
Timer1 retains its clock functions. The bus retains its register-transfer
primitives for the chip drivers. The filter is a separate calculation library.

## Follow one reading

1. `imu.read(imuReading, currentTime)` progresses the selected sensor on every pass.
2. GY80 coordinates its chips internally: motion at up to 200 Hz, compass at up
   to 25 Hz and pressure at up to 10 Hz when enabled. MPU6050 motion is scheduled
   at up to 200 Hz by IMU.
3. IMU feeds each new compass/motion reading into the filter once and returns
   `ImuReading`, containing `motion` and `orientation`.
4. The 5 ms control section reads that stored snapshot. Receiver demands, PID
   calculations and motor mixing are still placeholders.
5. Serial prints roll, pitch and yaw twice per second after zeroing.

`imu.getStatus().ready` means hardware initialization and stationary startup
averaging have finished. Keep the board still for about two seconds after the
hardware initializes. Angles also carry validity and timestamp information.
IMU retains `reset()` and `setMagneticCalibration()` for explicit configuration.

## Where to look

| Folder | Responsibility |
| --- | --- |
| `src/main.cpp` | Visible application and task bodies |
| `lib/imu` | Selected sensor, startup zero and orientation snapshots |
| `lib/GY80` | Whole board: owns four chips and schedules their readings |
| `lib/ADXL345`, `lib/L3G4200D`, `lib/HMC5883L`, `lib/BMP085` | Chip registers and conversion |
| `lib/MPU6050` | Alternative motion sensor |
| `lib/imuFilter` | Bias averaging and orientation calculations |
| `lib/i2cBus` | Interrupt-driven bytes, ownership and timeout handling |
| `lib/driverStatus` | Shared status and error types |
| `lib/escPwm` | Four ESC PWM outputs on D3, D5, D6 and D11 using Timer0/Timer2 |

## Motor outputs

`escPwm` exposes `init()`, `config()`, `write_motor_1()` through `write_motor_4()`,
an optional four-channel `write()`, and `close()`. Setup prepares PWM with outputs
low, then calls config to begin ESC arming with 1000 us pulses on all channels.
The control task has commented individual writes ready for the mixer. Config
does not wait for or confirm ESC readiness. Pulse widths are clamped to
1000..2000 us, with 8 us resolution and an approximately 490 Hz repetition rate.
Timer1 remains the project clock. Arduino's Timer0 clock functions are unavailable
while the motor driver owns Timer0. See [motor driver details](lib/escPwm/README.md).

## Select hardware

GY80 is the default. Uncomment the MPU6050 build flag in `platformio.ini` for
GY-521. The main IMU path enables GY80 motion/compass and leaves pressure disabled.
A direct GY80 read exposes all enabled board readings; see
[GY80 usage](lib/GY80/README.md). The raw readout example is in `examples/imuReadout`.

## Closing and reuse

Repeated close calls are safe. Close sensors before their shared bus, and the
timer last. Chips remain powered; close cancels software acquisition rather than
configuring sleep mode. A cancelled STOP finishes on later bus service/read calls
or its timeout. UART close waits for queued output before disabling hardware.
Caller-owned snapshots remain copies: close does not erase the caller's data.
The main application keeps drivers open and does not call close.

## Build and check

```powershell
platformio run -e uno
python test/sensor_drivers/run_tests.py
```

AVR simulator suites cover transport ownership/timeouts, polling sensor APIs,
board sampling and freshness, filter mathematics, peripheral lifecycle and both
IMU selections. They do not establish electrical timing or CPU timing margins
on the actual aircraft.
