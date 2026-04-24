#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include "MPUSensor.h"
#include "ServoController.h"

class BalancePID {
public:
    bool  running     = false;
    float kp          = 0.20f;
    float ki          = 0.00f;
    float kd          = 0.04f;
    float setpoint    = 0.0f;
    float output      = 0.0f;
    float gimbalAccum = 0.0f;

    static constexpr float KP_MAX            = 50.0f;
    static constexpr float KI_MAX            = 10.0f;
    static constexpr float KD_MAX            = 50.0f;
    static constexpr float GIMBAL_ACCUM_RATE = 800.0f;
    static constexpr float OUTPUT_MAX_DEG    = 350 * 0.088f;  // ≈ 30.8°
    static constexpr float ILIMIT            = 500.0f;
    static constexpr float SAFETY_DEG        = 60.0f;

    BalancePID(ServoController& servo, MPUSensor& mpu);

    void loadGains(Preferences& prefs);
    void saveGains(Preferences& prefs);
    void start();
    void stop();
    void compute();
    void setGains(float kp, float ki, float kd, float sp, Preferences& prefs);

private:
    ServoController& _servo;
    MPUSensor&       _mpu;
    float    _integral = 0.0f;
    float    _prevMeas = 0.0f;
    uint32_t _lastUs   = 0;
};
