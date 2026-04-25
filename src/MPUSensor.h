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

    MPUSensor(uint8_t addr, int sda, int scl);

    void begin(Preferences &prefs);
    bool read();
    void calibrate(Preferences &prefs, int samples = 1000);
    void resetAngle(Preferences &prefs);
    void setAngle(float target, Preferences &prefs);

private:
    uint8_t _addr;
    int _sda, _scl;

    float _biasAx = 0, _biasAy = 0, _biasAz = 0;
    float _biasGx = 0, _biasGy = 0, _biasGz = 0;

    // Smoothing state
    float _eAx = 0, _eAy = 0, _eAz = 0, _eGx = 0;
    float _cfAngle = 0.0f;
    uint32_t _lastUs = 0;
    bool _cfInited = false;

    static constexpr uint8_t REG_SMPLRT_DIV = 0x19;
    static constexpr uint8_t REG_CONFIG     = 0x1A;
    static constexpr uint8_t REG_GYRO_CFG   = 0x1B;
    static constexpr uint8_t REG_ACCEL_CFG  = 0x1C;
    static constexpr uint8_t REG_ACCEL_XOUT = 0x3B;
    static constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
    static constexpr uint8_t REG_WHO_AM_I   = 0x75;

    // DLPF 4 is ~20 Hz gyro bandwidth. If the flywheel still aliases into
    // the estimate on hardware, DLPF 5 (~10 Hz) is the next conservative step.
    static constexpr uint8_t MPU_DLPF_CFG   = 4;
    static constexpr uint8_t MPU_SMPLRT_DIV = 9; // 1 kHz / (1 + 9) = 100 Hz

    bool writeReg(uint8_t reg, uint8_t value);
    bool readReg(uint8_t reg, uint8_t &value);
    bool rawRead(int16_t out[7]);
    void resetFilters();
    void saveBias(Preferences &prefs);
    void saveOffset(Preferences &prefs);
};
