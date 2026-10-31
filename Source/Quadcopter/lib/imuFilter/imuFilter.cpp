#include "imuFilter.h"
#include <math.h>

namespace {
    constexpr float G = 9.80665f;
    constexpr uint32_t MAX_INTERVAL_US = 50000UL;
    constexpr uint32_t MAGNETIC_MAX_AGE_US = 200000UL;
    constexpr uint32_t ZERO_DURATION_US = 2000000UL;
    constexpr uint16_t ZERO_SAMPLES = 400;
    constexpr float TILT_GAIN = 2.0f; // Approximately 0.5 s correction time.
    constexpr float HEADING_TIME_S = 1.0f;

    /// Return true when all three vector components are finite numbers.
    bool finite(const Vector3 &v) {
        return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
    }
    /// Return the scalar dot product of two 3D vectors.
    float dot(const Vector3 &a, const Vector3 &b) {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }
    /// Return a vector perpendicular to both input vectors using their cross product.
    Vector3 cross(const Vector3 &a, const Vector3 &b) {
        return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
    }
    /// Multiply every component of a vector by the supplied scale factor.
    Vector3 scale(Vector3 v, float k) {
        return {v.x * k, v.y * k, v.z * k};
    }
    /// Subtract the second vector from the first, component by component.
    Vector3 subtract(Vector3 a, Vector3 b) {
        return {a.x - b.x, a.y - b.y, a.z - b.z};
    }
    /// Add the supplied vector to an existing vector sum.
    void add(Vector3 &sum, const Vector3 &v) {
        sum.x += v.x;
        sum.y += v.y;
        sum.z += v.z;
    }
    /// Normalize the vector to unit length; return false for a non-finite or near-zero
    /// magnitude.
    bool normalize(Vector3 &v) {
        float n = dot(v, v);
        if (!isfinite(n) || n < 1.0e-10f) {
            return false;
        }
        v = scale(v, 1.0f / sqrtf(n));
        return true;
    }
    /// Return true for a finite acceleration magnitude within 0.8 to 1.2 g, suitable for tilt
    /// correction.
    bool gravityReading(const Vector3 &v) {
        const float n = dot(v, v);
        return finite(v) && n >= (0.8f * G) * (0.8f * G) && n <= (1.2f * G) * (1.2f * G);
    }
    /// Multiply two quaternions to compose their rotations in the specified order.
    ImuQuaternion multiply(const ImuQuaternion &a, const ImuQuaternion &b) {
        return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
                a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
    }
    /// Negate a quaternion's vector part; for a unit quaternion this reverses its rotation.
    ImuQuaternion conjugate(ImuQuaternion q) {
        return {q.w, -q.x, -q.y, -q.z};
    }
    /// Normalize the quaternion to unit length; return false for a non-finite or near-zero
    /// magnitude.
    bool normalize(ImuQuaternion &q) {
        float n = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
        if (!isfinite(n) || n < 1.0e-10f) {
            return false;
        }
        float k = 1.0f / sqrtf(n);
        q.w *= k;
        q.x *= k;
        q.y *= k;
        q.z *= k;
        return true;
    }
    // World Z and X expressed in body axes (rows of the rotation matrix).
    // Direct formulas avoid two quaternion products for each reference vector.
    /// Return world vertical expressed in the sensor's body axes for the given rotation.
    Vector3 bodyUp(const ImuQuaternion &q) {
        return {2 * (q.x * q.z - q.w * q.y), 2 * (q.y * q.z + q.w * q.x),
                1 - 2 * (q.x * q.x + q.y * q.y)};
    }
    /// Return world north expressed in the sensor's body axes for the given rotation.
    Vector3 bodyNorth(const ImuQuaternion &q) {
        return {1 - 2 * (q.y * q.y + q.z * q.z), 2 * (q.x * q.y - q.w * q.z),
                2 * (q.x * q.z + q.w * q.y)};
    }
    /// Project a vector onto the horizontal plane and normalize it; return false if degenerate.
    bool horizontal(Vector3 &v, const Vector3 &up) {
        v = subtract(v, scale(up, dot(v, up)));
        return normalize(v);
    }
    // Build body-to-world rotation from gravity and horizontal magnetic north.
    // The matrix rows are world X/Y/Z expressed in body axes. Trace branches
    // handle inverted starting poses, including the bench's negative-Z reading.
    /// Build a body-to-world quaternion from gravity and magnetic north; return false for
    /// unusable reference vectors.
    bool rotationFromReferences(Vector3 up, Vector3 north, ImuQuaternion &q) {
        if (!normalize(up) || !horizontal(north, up)) {
            return false;
        }
        Vector3 side = cross(up, north);
        const float trace = north.x + side.y + up.z;
        if (trace > 0) {
            float s = 2.0f * sqrtf(1 + trace);
            q = {s * 0.25f, (up.y - side.z) / s, (north.z - up.x) / s, (side.x - north.y) / s};
        } else if (north.x > side.y && north.x > up.z) {
            float s = 2.0f * sqrtf(1 + north.x - side.y - up.z);
            q = {(up.y - side.z) / s, s * 0.25f, (north.y + side.x) / s, (north.z + up.x) / s};
        } else if (side.y > up.z) {
            float s = 2.0f * sqrtf(1 + side.y - north.x - up.z);
            q = {(north.z - up.x) / s, (north.y + side.x) / s, s * 0.25f, (side.z + up.y) / s};
        } else {
            float s = 2.0f * sqrtf(1 + up.z - north.x - side.y);
            q = {(side.x - north.y) / s, (north.z + up.x) / s, (side.z + up.y) / s, s * 0.25f};
        }
        return normalize(q);
    }
    /// Choose a heading reference not parallel to gravity when no compass is available.
    Vector3 arbitraryNorth(Vector3 up) {
        // Without a compass, choose an arbitrary heading for the startup zero.
        return fabsf(up.x) < 0.9f ? Vector3{1, 0, 0} : Vector3{0, 1, 0};
    }
} // namespace

/// Discard accumulated startup averages so stationary zeroing can start a new window.
void ImuFilter::clearCalibrationWindow() {
    gyroSum_ = {};
    accelerationSum_ = {};
    magneticSum_ = {};
    calibrationCount_ = 0;
    magneticCount_ = 0;
    calibrationStartUs_ = 0;
}

/// Start nonblocking stationary bias and pose averaging; keep the board still for about two
/// seconds. Enable compass input with useMagnetometer.
void ImuFilter::init(bool useMagnetometer) {
    initialized_ = true;
    useMagnetometer_ = useMagnetometer;
    calibrated_ = tracking_ = magneticAvailable_ = magneticPending_ = magneticUsable_ = false;
    gyroBias_ = {};
    magnetic_ = {};
    orientation_ = {};
    rotation_ = reference_ = {1, 0, 0, 0};
    lastSampleUs_ = lastMagneticCorrectionUs_ = 0;
    referenceMagneticStrength_ = 0;
    clearCalibrationWindow();
}

/// Restart stationary zeroing if open, retaining supplied compass calibration.
void ImuFilter::reset() {
    if (initialized_) {
        init(useMagnetometer_);
    }
}

/// Clear tracking and invalidate angles; reject updates until init(), retaining compass
/// calibration.
void ImuFilter::close() {
    init(useMagnetometer_);
    initialized_ = false;
}

/// Return true if a stored compass reading is recent enough for the given motion timestamp.
bool ImuFilter::magneticFresh(uint32_t nowUs) const {
    // Allow the small timestamp skew of sequential motion/compass transfers.
    int32_t age = static_cast<int32_t>(nowUs - magnetic_.timestampUs);
    return magneticAvailable_ && age >= -5000 && age <= static_cast<int32_t>(MAGNETIC_MAX_AGE_US);
}

/// Validate and store external compass offset/matrix correction; restart zeroing if open and
/// return false for invalid input.
bool ImuFilter::setMagneticCalibration(const MagneticCalibration &calibration) {
    if (!finite(calibration.offsetMicrotesla)) {
        return false;
    }
    for (uint8_t row = 0; row < 3; ++row) {
        for (uint8_t col = 0; col < 3; ++col) {
            if (!isfinite(calibration.matrix[row][col])) {
                return false;
            }
        }
    }
    const float (*m)[3] = calibration.matrix;
    float determinant = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
                        m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
                        m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    if (!isfinite(determinant) || determinant <= 1.0e-6f) {
        return false;
    }
    magneticCalibration_ = calibration;
    magneticCalibrated_ = true;
    reset(); // Changing compass calibration changes the heading reference.
    return true;
}

/// Validate, calibrate and store a newer compass sample; false means disabled, closed,
/// invalid data or a duplicate/backward timestamp.
bool ImuFilter::updateMagneticField(const MagneticSample &sample) {
    if (!initialized_ || !useMagnetometer_ || !finite(sample.fieldMicrotesla)) {
        return false;
    }
    if (magneticAvailable_) {
        uint32_t elapsed = sample.timestampUs - magnetic_.timestampUs;
        if (elapsed == 0 || elapsed >= 0x80000000UL) {
            return false;
        }
    }
    Vector3 raw = subtract(sample.fieldMicrotesla, magneticCalibration_.offsetMicrotesla);
    const float (*m)[3] = magneticCalibration_.matrix;
    Vector3 field = {m[0][0] * raw.x + m[0][1] * raw.y + m[0][2] * raw.z,
                     m[1][0] * raw.x + m[1][1] * raw.y + m[1][2] * raw.z,
                     m[2][0] * raw.x + m[2][1] * raw.y + m[2][2] * raw.z};
    const float strengthSquared = dot(field, field);
    if (!finite(field) || strengthSquared < 25 || strengthSquared > 22500) {
        return false;
    }
    // Reject large field-strength changes after establishing the startup field.
    if (calibrated_ &&
        (strengthSquared < 0.49f * referenceMagneticStrength_ * referenceMagneticStrength_ ||
         strengthSquared > 1.69f * referenceMagneticStrength_ * referenceMagneticStrength_)) {
        return false;
    }
    magnetic_ = {field, sample.timestampUs};
    magneticAvailable_ = magneticPending_ = true;
    if (!calibrated_ && magneticCount_ < 256) {
        add(magneticSum_, field);
        ++magneticCount_;
    }
    return true;
}

/// Accumulate stationary motion/compass readings; return true when a valid startup zero has
/// been established.
bool ImuFilter::collectCalibration(const ImuSample &sample) {
    if (!gravityReading(sample.accelerationMps2) ||
        dot(sample.angularVelocityRadS, sample.angularVelocityRadS) > 0.01f) {
        clearCalibrationWindow();
        return false;
    }
    if (calibrationCount_ > 0) {
        uint32_t elapsed = sample.timestampUs - lastSampleUs_;
        if (elapsed == 0 || elapsed >= 0x80000000UL) {
            return false;
        }
        Vector3 accelMean = scale(accelerationSum_, 1.0f / calibrationCount_);
        Vector3 gyroMean = scale(gyroSum_, 1.0f / calibrationCount_);
        Vector3 accelerationChange = subtract(sample.accelerationMps2, accelMean);
        Vector3 gyroChange = subtract(sample.angularVelocityRadS, gyroMean);
        if (elapsed > MAX_INTERVAL_US || dot(accelerationChange, accelerationChange) > 0.5625f ||
            dot(gyroChange, gyroChange) > 0.0016f) {
            clearCalibrationWindow(); // Movement restarts the stationary window.
        }
    }
    if (calibrationCount_ == 0) {
        calibrationStartUs_ = sample.timestampUs;
    }
    lastSampleUs_ = sample.timestampUs;
    if (calibrationCount_ < ZERO_SAMPLES) {
        add(gyroSum_, sample.angularVelocityRadS);
        add(accelerationSum_, sample.accelerationMps2);
        ++calibrationCount_;
    }
    if (calibrationCount_ < ZERO_SAMPLES ||
        static_cast<uint32_t>(sample.timestampUs - calibrationStartUs_) < ZERO_DURATION_US) {
        return false;
    }
    if (useMagnetometer_ && (magneticCount_ < 20 || !magneticFresh(sample.timestampUs))) {
        return false;
    }

    Vector3 up = scale(accelerationSum_, 1.0f / calibrationCount_);
    if (!normalize(up)) {
        return false;
    }
    Vector3 north =
        useMagnetometer_ ? scale(magneticSum_, 1.0f / magneticCount_) : arbitraryNorth(up);
    if (!rotationFromReferences(up, north, rotation_)) {
        return false;
    }
    reference_ = rotation_;
    gyroBias_ = scale(gyroSum_, 1.0f / calibrationCount_);
    referenceMagneticStrength_ = sqrtf(dot(north, north));
    lastMagneticCorrectionUs_ = magnetic_.timestampUs;
    magneticPending_ = false;
    calibrated_ = tracking_ = true;
    magneticUsable_ = useMagnetometer_;
    publish(sample.timestampUs); // Identity relative rotation -> 0,0,0.
    return orientation_.valid;
}

/// Convert the current rotation relative to the startup pose into stored Euler angles and
/// validity flags.
void ImuFilter::publish(uint32_t timestampUs) {
    ImuQuaternion relative = multiply(conjugate(reference_), rotation_);
    if (!normalize(relative)) {
        orientation_.valid = false;
        return;
    }
    float sinPitch = 2 * (relative.w * relative.y - relative.z * relative.x);
    sinPitch = fmaxf(-1.0f, fminf(1.0f, sinPitch));
    orientation_.rollRad = atan2f(2 * (relative.w * relative.x + relative.y * relative.z),
                                  1 - 2 * (relative.x * relative.x + relative.y * relative.y));
    orientation_.pitchRad = asinf(sinPitch);
    orientation_.yawRad = atan2f(2 * (relative.w * relative.z + relative.x * relative.y),
                                 1 - 2 * (relative.y * relative.y + relative.z * relative.z));
    orientation_.timestampUs = timestampUs;
    orientation_.valid = fabsf(sinPitch) < 0.999999f; // Euler roll/yaw undefined at +/-90 pitch.
    orientation_.yawReferenced = useMagnetometer_ && magneticUsable_ && magneticFresh(timestampUs);
}

/// Process one motion sample; return true only for a new valid orientation, and false during
/// zeroing or for rejected data.
bool ImuFilter::update(const ImuSample &sample) {
    if (!initialized_) {
        return false;
    }
    if (!finite(sample.accelerationMps2) || !finite(sample.angularVelocityRadS)) {
        orientation_.valid = false;
        tracking_ = false;
        if (!calibrated_) {
            clearCalibrationWindow();
        }
        return false;
    }
    if (!calibrated_) {
        return collectCalibration(sample);
    }
    const uint32_t elapsed = sample.timestampUs - lastSampleUs_;
    if (elapsed == 0 || elapsed >= 0x80000000UL) {
        return false;
    }
    lastSampleUs_ = sample.timestampUs;
    if (elapsed > MAX_INTERVAL_US) {
        tracking_ = false;
    }
    if (!tracking_) {
        // Keep the original zero across missing data. A compass lets us recover
        // the current absolute rotation. Gyro-only yaw cannot recover lost motion.
        if (!useMagnetometer_ || !magneticFresh(sample.timestampUs) ||
            !gravityReading(sample.accelerationMps2) ||
            !rotationFromReferences(sample.accelerationMps2, magnetic_.fieldMicrotesla,
                                    rotation_)) {
            orientation_.valid = false;
            return false;
        }
        tracking_ = true;
        lastMagneticCorrectionUs_ = magnetic_.timestampUs;
        magneticPending_ = false;
        magneticUsable_ = true;
        publish(sample.timestampUs);
        return orientation_.valid;
    }

    // Integrate angular velocity to track fast motion between samples.
    const float intervalS = elapsed * 1.0e-6f;
    Vector3 correctedGyro = subtract(sample.angularVelocityRadS, gyroBias_);
    // Accelerometer feedback corrects tilt, while the gyro provides fast motion.
    // The cross-product sign makes a positive estimated tilt error correct back.
    if (gravityReading(sample.accelerationMps2)) {
        Vector3 measured = sample.accelerationMps2;
        normalize(measured);
        Vector3 expected = bodyUp(rotation_);
        add(correctedGyro, scale(cross(measured, expected), TILT_GAIN));
    }
    // q_dot = 0.5 * q * [0, gyro]. Normalize after the integration step.
    ImuQuaternion derivative =
        multiply(rotation_, {0, correctedGyro.x, correctedGyro.y, correctedGyro.z});
    rotation_.w += 0.5f * intervalS * derivative.w;
    rotation_.x += 0.5f * intervalS * derivative.x;
    rotation_.y += 0.5f * intervalS * derivative.y;
    rotation_.z += 0.5f * intervalS * derivative.z;
    if (!normalize(rotation_)) {
        tracking_ = false;
        orientation_.valid = false;
        return false;
    }

    if (useMagnetometer_ && magneticPending_ && magneticFresh(sample.timestampUs)) {
        Vector3 up = bodyUp(rotation_);
        Vector3 predictedNorth = bodyNorth(rotation_);
        Vector3 measuredNorth = magnetic_.fieldMicrotesla;
        magneticUsable_ = horizontal(predictedNorth, up) && horizontal(measuredNorth, up);
        if (magneticUsable_) {
            // Signed, tilt-compensated heading error. Correct only around world
            // vertical so magnetic feedback does not pull roll/pitch sideways.
            float headingErrorRad = atan2f(dot(up, cross(measuredNorth, predictedNorth)),
                                           dot(measuredNorth, predictedNorth));
            uint32_t interval = magnetic_.timestampUs - lastMagneticCorrectionUs_;
            float magneticIntervalS = fminf(interval * 1.0e-6f, 0.2f);
            float halfCorrectionRad =
                0.5f * headingErrorRad * magneticIntervalS / (HEADING_TIME_S + magneticIntervalS);
            float sinHalfCorrection = sinf(halfCorrectionRad);
            rotation_ = multiply(rotation_, {cosf(halfCorrectionRad), up.x * sinHalfCorrection,
                                             up.y * sinHalfCorrection, up.z * sinHalfCorrection});
            if (!normalize(rotation_)) {
                tracking_ = false;
                orientation_.valid = false;
                return false;
            }
            lastMagneticCorrectionUs_ = magnetic_.timestampUs;
        }
        magneticPending_ = false; // Each compass sample is used once.
    }
    publish(sample.timestampUs);
    return orientation_.valid;
}
