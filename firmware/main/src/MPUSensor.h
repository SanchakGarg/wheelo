#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>

class MPUSensor
{
public:
    float angle = 0.0f;
    float angleOffset = 0.0f;
    float rateDps = 0.0f;
    float accelAngle = 0.0f;
    float accelNormG = 1.0f;
    uint32_t droppedReads = 0;
    bool ok = false;
    bool calibrating = false;
    // Measured noise floor of the filtered gyro output (°/s).
    // Set by characterizeNoise(); used as the rate deadband in BalancePID
    // and as the bias-tracking gate in read(). Defaults to 1 °/s until cal.
    float gyroNoiseFloor    = 1.0f;   // 3σ of filtered rateDps — PID derivative deadband
    float accelInnovDeadband = 1.5f; // 3σ of filtered accel angle — telemetry only (Mahony has no hard deadband)

    MPUSensor(uint8_t addr, int sda, int scl);

    void begin(Preferences &prefs);
    bool read();
    void calibrate(Preferences &prefs, int samples = 1000);
    bool characterizeNoise(Preferences &prefs, int samples = 3000);
    void resetAngle(Preferences &prefs);
    void setAngle(float target, Preferences &prefs);

private:
    uint8_t _addr;
    int _sda, _scl;

    BiquadState _bqAx, _bqAy, _bqAz, _bqGx;

    float _biasAx = 0, _biasAy = 0, _biasAz = 0;
    float _biasGx = 0, _biasGy = 0, _biasGz = 0;

    // Smoothing state
    float _eAx = 0, _eAy = 0, _eAz = 0, _eGx = 0;
    float _cfAngle = 0.0f;
    uint32_t _lastUs = 0;
    bool _cfInited = false;

    // Median pre-filter — knocks out impulse spikes from BLDC EMF before LP.
    static constexpr size_t MEDIAN_N = 5;
    float    _gxMed[MEDIAN_N] = {0};
    float    _axMed[MEDIAN_N] = {0};
    float    _ayMed[MEDIAN_N] = {0};
    float    _azMed[MEDIAN_N] = {0};
    uint8_t  _medIdx = 0;
    uint8_t  _medCount = 0;

    // LP-filtered accel norm — used to detect real acceleration events
    // (impacts, falls) vs. BLDC vibration (which averages to 1 g after LP).
    float _accelNormFilt = 1.0f;

    // Mahony complementary filter: accumulated gyro bias correction (dps).
    // Converges to -(true_thermal_drift_dps) so the integrated gyro rate
    // stays anchored without a hard innovation deadband.
    float _mahonyInt = 0.0f;

    // Sensor is mounted upside-down on the underside of the chassis,
    // rotated 180° around its X axis. That flips accel Y/Z and gyro Y/Z
    // sign; accel X and gyro X (the pitch-rate axis we actually use)
    // stay the same. Applied at the raw-read boundary so the rest of
    // the pipeline (bias cal, median, LP, complementary filter) is
    // orientation-agnostic. If you remount right-side up, set to false.
    //
    // If after toggling this you find the angle changes the *correct*
    // amount but in the *wrong direction*, your chip was actually
    // flipped around the Y axis instead of X — change the axis flips
    // in rawRead() (negate out[0] and out[4] instead of out[1]/out[2]).
    static constexpr bool MPU_UPSIDE_DOWN = true;

    static constexpr uint8_t REG_SMPLRT_DIV = 0x19;
    static constexpr uint8_t REG_CONFIG     = 0x1A;
    static constexpr uint8_t REG_GYRO_CFG   = 0x1B;
    static constexpr uint8_t REG_ACCEL_CFG  = 0x1C;
    static constexpr uint8_t REG_ACCEL_XOUT = 0x3B;
    static constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
    static constexpr uint8_t REG_WHO_AM_I   = 0x75;

    // DLPF 5 = 10 Hz gyro / 10 Hz accel bandwidth. 
    // We use a higher hardware bandwidth so we can use a sharper software 
    // LP (which has better phase characteristics for control).
    static constexpr uint8_t MPU_DLPF_CFG   = 5;
    static constexpr uint8_t MPU_SMPLRT_DIV = 9; // 1 kHz / (1 + 9) = 100 Hz

    // Software gyro LP cutoff (Hz). 
    // 5 Hz is aggressive; it creates lag but is necessary for 3-degree noise floors.
    static constexpr float   GYRO_LP_HZ  = 5.0f;
    // Accel needs an extremely aggressive LP for scissored CMG vibration.
    // 2 Hz is the 'safe' limit for balancing without falling over from lag.
    static constexpr float   ACCEL_LP_HZ = 2.0f;

    // Gyro Soft-Deadband: if (rate < GYRO_DEADBAND) then rate = 0.
    // This stops the angle from crawling when the robot is sitting still 
    // despite massive flywheel vibration.
    static constexpr float   GYRO_SOFT_DEADBAND = 0.4f; 

    // Mahony complementary filter gains.
    // REDUCED Kp: We trust the gyro more and the vibrating accel less.
    // This stops the 3-degree vibration from "pulling" the angle around.
    static constexpr float   MAHONY_KP       = 0.4f; 
    static constexpr float   MAHONY_KI       = 0.20f;
    static constexpr float   MAX_MAHONY_INT  = 20.0f;  
    static constexpr float   ACCEL_DIVERGE_G = 0.12f;

    // Biquad Filter State for 2nd-order noise rejection
    struct BiquadState {
        float x1=0, x2=0, y1=0, y2=0;
    };

    MPUSensor(uint8_t addr, int sda, int scl);

    bool readReg(uint8_t reg, uint8_t &value);
    bool rawRead(int16_t out[7]);
    void resetFilters();
    void saveBias(Preferences &prefs);
    void saveOffset(Preferences &prefs);
};
