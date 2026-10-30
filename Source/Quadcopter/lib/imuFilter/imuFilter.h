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

    float rollDeg() const { return rollRad * 57.2957795f; }
    float pitchDeg() const { return pitchRad * 57.2957795f; }
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
    // Begin a non-blocking startup zero. Keep the board stationary for about
    // two seconds after the sensors initialize. GY80 uses its compass; MPU6050
    // alone can only provide gyro-based relative yaw, which can still drift.
    void begin(bool useMagnetometer);
    void reset(); // Explicitly choose a new startup zero; retains mag calibration.
    bool calibrated() const { return calibrated_; }
    Vector3 gyroBiasRadS() const { return gyroBias_; }

    bool update(const ImuSample& sample);
    bool updateMagneticField(const MagneticSample& sample);
    bool setMagneticCalibration(const MagneticCalibration& calibration);
    bool magneticCalibrated() const { return magneticCalibrated_; }

    // Angles are relative to the averaged starting pose, not magnetic north.
    // valid does not age by itself: consumers must check timestampUs as well.
    const ImuOrientation& orientation() const { return orientation_; }

private:
    void clearCalibrationWindow();
    bool collectCalibration(const ImuSample& sample);
    bool magneticFresh(uint32_t nowUs) const;
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
    bool calibrated_ = false;
    bool tracking_ = false;
    bool magneticAvailable_ = false;
    bool magneticPending_ = false;
    bool magneticUsable_ = false;
    bool magneticCalibrated_ = false;
};

#endif
