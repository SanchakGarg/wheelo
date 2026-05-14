#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>

class MPUSensor
{
public:
    // Biquad Filter State for 2nd-order noise rejection
    struct BiquadState {
        float x1=0, x2=0, y1=0, y2=0;
    };

    float angle = 0.0f;
    float angleOffset = 0.0f;
    float rateDps = 0.0f;
    float accelAngle = 0.0f;
    float accelNormG = 1.0f;
    uint32_t droppedReads = 0;
    bool ok = false;
    bool calibrating = false;
    
    // Measured noise floor of the filtered gyro output (°/s).
    float gyroNoiseFloor    = 1.0f;
    float accelInnovDeadband = 1.5f;

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

    float _eAx = 0, _eAy = 0, _eAz = 0, _eGx = 0;
    float _cfAngle = 0.0f;
    uint32_t _lastUs = 0;
    bool _cfInited = false;

    static constexpr size_t MEDIAN_N = 5;
    float    _gxMed[MEDIAN_N] = {0};
    float    _axMed[MEDIAN_N] = {0};
    float    _ayMed[MEDIAN_N] = {0};
    float    _azMed[MEDIAN_N] = {0};
    uint8_t  _medIdx = 0;
    uint8_t  _medCount = 0;

    float _accelNormFilt = 1.0f;
    float _mahonyInt = 0.0f;

    static constexpr bool MPU_UPSIDE_DOWN = true;

    static constexpr uint8_t REG_SMPLRT_DIV = 0x19;
    static constexpr uint8_t REG_CONFIG     = 0x1A;
    static constexpr uint8_t REG_GYRO_CFG   = 0x1B;
    static constexpr uint8_t REG_ACCEL_CFG  = 0x1C;
    static constexpr uint8_t REG_ACCEL_XOUT = 0x3B;
    static constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
    static constexpr uint8_t REG_WHO_AM_I   = 0x75;

    static constexpr uint8_t MPU_DLPF_CFG   = 5;
    static constexpr uint8_t MPU_SMPLRT_DIV = 9;

    static constexpr float   GYRO_LP_HZ  = 5.0f;
    static constexpr float   ACCEL_LP_HZ = 2.0f;
    static constexpr float   GYRO_SOFT_DEADBAND = 0.6f; // Increased for high-vibe CMG

    // Stillness Lock: if gyro is quiet for 20 samples (200ms), lock the angle.
    uint16_t _stillCount = 0;
    static constexpr uint16_t STILL_THRESHOLD_SAMPLES = 20;

    // Moving Average Buffer to kill the "oscillation pattern"
    float _gxMA[4] = {0};
    uint8_t _maIdx = 0;

    static constexpr float   MAHONY_KP       = 0.4f; 
    static constexpr float   MAHONY_KI       = 0.20f;
    static constexpr float   MAX_MAHONY_INT  = 20.0f;  
    static constexpr float   ACCEL_DIVERGE_G = 0.12f;

    bool writeReg(uint8_t reg, uint8_t value);
    bool readReg(uint8_t reg, uint8_t &value);
    bool rawRead(int16_t out[7]);
    void resetFilters();
    void saveBias(Preferences &prefs);
    void saveOffset(Preferences &prefs);
};
