#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>

class MPUSensor
{
public:
    float angle = 0.0f;
    float angleOffset = 0.0f;
    bool ok = false;
    bool calibrating = false;

    MPUSensor(uint8_t addr, int sda, int scl);

    void begin(Preferences &prefs);
    void read();
    void calibrate(Preferences &prefs, int samples = 1000);
    void resetAngle(Preferences &prefs);
    void setAngle(float target, Preferences &prefs);

private:
    uint8_t _addr;
    int _sda, _scl;

    float _biasAx = 0, _biasAy = 0, _biasAz = 0;
    float _biasGx = 0, _biasGy = 0, _biasGz = 0;

    // Smoothing state
    float _eAx = 0, _eAy = 0, _eAz = 0, _eGx = 0; // stage 1: raw EMA
    float _cfAngle = 0.0f;                        // complementary filter output
    bool _emaInited = false;
    uint32_t _lastUs = 0;
    bool _cfInited = false;

    // Stage 2: moving median buffer
    static constexpr int MEDIAN_N = 7; // window size — odd number (5, 7, 9)
    float _medBuf[MEDIAN_N] = {};
    int _medHead = 0;
    int _medCount = 0; // samples collected so far (fills up to MEDIAN_N)

    static constexpr float RAW_ALPHA = 0.12f; // raw EMA alpha (lower = smoother)

    void rawRead(int16_t out[7]);
    void saveBias(Preferences &prefs);
    void saveOffset(Preferences &prefs);
};
