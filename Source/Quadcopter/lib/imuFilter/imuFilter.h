#ifndef IMU_FILTER_H
#define IMU_FILTER_H

#include "imuTypes.h"

struct ImuOrientation {
    float rollRad;
    float pitchRad;
    float yawRad;
    uint32_t timestampUs;
    bool valid;
    bool yawReferenced; // A recent compass reading is correcting gyro yaw.

    /// Return the stored roll angle converted from radians to degrees.
    float rollDeg() const { return rollRad * 57.2957795f; }
    /// Return the stored pitch angle converted from radians to degrees.
    float pitchDeg() const { return pitchRad * 57.2957795f; }
    /// Return the stored yaw angle converted from radians to degrees.
    float yawDeg() const { return yawRad * 57.2957795f; }
};

// Optional externally measured compass calibration. A stationary startup zero
// cannot determine these values: obtaining them requires rotating the board.
struct MagneticCalibration {
    Vector3 offsetMicrotesla = {};
    float matrix[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
};

// Internal rotation representation: four numbers avoid Euler-angle singularities
// during gyro integration and allow zeroing relative to any starting pose.
struct ImuQuaternion {
    float w, x, y, z;
};

class ImuFilter {
public:
    /// Start nonblocking stationary bias and pose averaging; keep the board still for about two
    /// seconds. Enable compass input with useMagnetometer.
    void init(bool useMagnetometer);
    /// Clear tracking and invalidate angles; reject updates until init(), retaining compass
    /// calibration.
    void close();
    /// Restart stationary zeroing if open, retaining supplied compass calibration.
    void reset(); // Explicitly choose a new startup zero; retains mag calibration.
    /// Return true after startup bias and pose averaging has completed; this does not guarantee
    /// the stored angles are current.
    bool calibrated() const { return calibrated_; }
    /// Return the stationary gyro bias estimated at startup, in radians per second.
    Vector3 gyroBiasRadS() const { return gyroBias_; }

    /// Process one motion sample; return true only for a new valid orientation, and false
    /// during zeroing or for rejected data.
    bool update(const ImuSample& sample);
    /// Validate, calibrate and store a newer compass sample; false means disabled, closed,
    /// invalid data or a duplicate/backward timestamp.
    bool updateMagneticField(const MagneticSample& sample);
    /// Validate and store external compass offset/matrix correction; restart zeroing if open
    /// and return false for invalid input.
    bool setMagneticCalibration(const MagneticCalibration& calibration);
    /// Return true if an external compass calibration has been supplied.
    bool magneticCalibrated() const { return magneticCalibrated_; }

    /// Return stored angles relative to the starting pose; validity does not age automatically,
    /// so also check timestampUs.
    const ImuOrientation& orientation() const { return orientation_; }

private:
    /// Discard accumulated startup averages so stationary zeroing can start a new window.
    void clearCalibrationWindow();
    /// Accumulate stationary motion/compass readings; return true when a valid startup zero has
    /// been established.
    bool collectCalibration(const ImuSample& sample);
    /// Return true if a stored compass reading is recent enough for the given motion timestamp.
    bool magneticFresh(uint32_t nowUs) const;
    /// Convert the current rotation relative to the startup pose into stored Euler angles and
    /// validity flags.
    void publish(uint32_t timestampUs);

    ImuQuaternion rotation_ = {1, 0, 0, 0};
    ImuQuaternion reference_ = {1, 0, 0, 0};
    ImuOrientation orientation_ = {};
    MagneticCalibration magneticCalibration_;
    MagneticSample magnetic_ = {};
    Vector3 gyroBias_ = {};
    Vector3 gyroSum_ = {};
    Vector3 accelerationSum_ = {};
    Vector3 magneticSum_ = {};
    float referenceMagneticStrength_ = 0;
    uint32_t calibrationStartUs_ = 0;
    uint32_t lastSampleUs_ = 0;
    uint32_t lastMagneticCorrectionUs_ = 0;
    uint16_t calibrationCount_ = 0;
    uint16_t magneticCount_ = 0;
    bool useMagnetometer_ = false;
    bool initialized_ = false;
    bool calibrated_ = false;
    bool tracking_ = false;
    bool magneticAvailable_ = false;
    bool magneticPending_ = false;
    bool magneticUsable_ = false;
    bool magneticCalibrated_ = false;
};

#endif
