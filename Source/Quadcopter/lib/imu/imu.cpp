#include "imu.h"

#define MOTION_INTERVAL_US 5000UL

/// Begin nonblocking sensor setup and stationary orientation zeroing.
bool IMU::init(uint32_t nowUs) {
#ifdef IMU_USE_MPU6050
    if (!sensor_.init(nowUs)) {
        return false;
    }
    filter_.init(false);
    motionReading_ = false;
    motionTimeUs_ = nowUs;
#else
    if (!sensor_.init(nowUs, true, false)) {
        return false;
    }
    filter_.init(true);
#endif
    return true;
}

/// Invalidate internal tracking and stop the selected sensor.
void IMU::close() {
    sensor_.close();
    filter_.close();
#ifdef IMU_USE_MPU6050
    motionReading_ = false;
#endif
}

/// Inspect sensor activity and include startup averaging in the readiness result.
DriverStatus IMU::getStatus() const {
    DriverStatus status = sensor_.getStatus();
    if (status.ready && !filter_.calibrated()) {
        status.ready = false;
        status.busy = true;
    }
    return status;
}

/// Advance hardware and fuse each fresh motion/compass sample exactly once.
bool IMU::read(ImuReading& reading, uint32_t nowUs) {
    ImuSample motion;
    bool freshMotion = false;
#ifdef IMU_USE_MPU6050
    DriverStatus status = sensor_.getStatus();
    bool due = static_cast<uint32_t>(nowUs - motionTimeUs_) >= MOTION_INTERVAL_US;
    if (!status.ready || motionReading_ || due) {
        freshMotion = sensor_.read(motion, nowUs);
        if (status.ready && !motionReading_) {
            motionTimeUs_ = nowUs;
        }
        DriverStatus next = sensor_.getStatus();
        motionReading_ = next.ready && next.busy;
    }
#else
    Gy80Sample board;
    if (sensor_.read(board, nowUs)) {
        if (board.magneticFresh) {
            filter_.updateMagneticField(board.magnetic);
        }
        if (board.motionFresh) {
            motion = board.motion;
            freshMotion = true;
        }
    }
#endif
    if (!freshMotion) {
        return false;
    }
    filter_.update(motion);
    reading.motion = motion;
    reading.orientation = filter_.orientation();
    return true;
}
