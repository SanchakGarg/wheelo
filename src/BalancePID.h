#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include "MPUSensor.h"
#include "ServoController.h"

class BalancePID {
public:
    bool  running       = false;
    float kp            = 3.5f;    // degrees servo per degree roll error
    float ki            = 0.0f;    // integral — keep near 0
    float kd            = 0.10f;   // degrees servo per (°/s) roll rate
    float setpoint      = 0.0f;    // target roll angle (degrees)
    float output        = 0.0f;    // PID output → direct servo deflection (degrees)
    float setpointAccum = 0.0f;    // XRobots gimbal-centering drift

    // Physical limits
    static constexpr float OUTPUT_MAX_DEG       = SERVO_SWING * 0.088f; // ≈ 61.6° with swing=700
    static constexpr float ILIMIT               = 50.0f;
    static constexpr float SAFETY_DEG           = 60.0f;
    static constexpr float SETPOINT_ACCUM_DIV   = 7500.0f; // higher = slower re-centering
    static constexpr float SETPOINT_ACCUM_LIMIT = 0.5f;    // max setpoint drift (degrees)

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
    float    _integral  = 0.0f;
    float    _prevMeas  = 0.0f;
    float    _prevOutput= 0.0f;
    uint32_t _lastUs    = 0;
};
