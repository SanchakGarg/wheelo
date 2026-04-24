#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>

class MPUSensor {
public:
    struct Vals { float ax, ay, az, gx, gy, gz; };

    Vals  filtered    = {};
    float cfRoll      = 0.0f;
    float cfPitch     = 0.0f;
    float rollOffset  = 0.0f;
    float pitchOffset = 0.0f;
    bool  ok          = false;
    bool  calibrating = false;

    MPUSensor(uint8_t addr, int sda, int scl);

    void begin(Preferences& prefs);
    void read();
    void calibrate(Preferences& prefs, int samples = 1000);
    void vibCalibrate(Preferences& prefs, int samples = 500);
    void resetAngles(Preferences& prefs);
    void setAngles(float roll, float pitch, Preferences& prefs);

private:
    uint8_t  _addr;
    int      _sda, _scl;
    Vals     _bias    = {};
    Vals     _ema     = {};
    float    _vibStd[6] = {};
    uint32_t _cfLastUs  = 0;
    bool     _emaInit   = false;

    static constexpr float EMA_A = 0.12f;

    void  rawRead(int16_t out[7]);
    float dezone(float v, float std);
    void  saveOffsets(Preferences& prefs);
};
