#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include "config.h"
#include "MPUSensor.h"
#include "ServoController.h"

class BalancePID {
public:
    bool  running       = false;
    float kp            = 1.8f;   // matches XRobots V2
    float ki            = 0.0f;   // start with integral off while tuning noise
    float kd            = 0.09f;  // matches XRobots V2
    float trim          = 0.0f;   // balance-point trim (V2's hardcoded +0.6)
    float output        = 0.0f;
    float setpointAccum = 0.0f;

    static constexpr float OUTPUT_LIMIT         = 35.0f;   // degrees, matches V2
    static constexpr float SAFETY_DEG           = 60.0f;
    static constexpr float ANGLE_DEADBAND_DEG   = 0.25f;
    static constexpr float RATE_DEADBAND_DPS    = 1.0f;
    static constexpr float MAX_OUTPUT_STEP      = 2.0f;
    static constexpr float SETPOINT_ACCUM_DIV   = 1500.0f; // matches V2
    static constexpr float SETPOINT_ACCUM_LIMIT = 0.5f;    // matches V2
    static constexpr float SAMPLE_TIME_S        = 0.01f;   // 10ms fixed, matches V2
    static constexpr int   SERVO_DIR            = 1;

    BalancePID(ServoController& servo, MPUSensor& mpu);

    void loadGains(Preferences& prefs);
    void saveGains(Preferences& prefs);
    void start();
    void stop();
    void compute();
    void setGains(float kp, float ki, float kd, float trim, Preferences& prefs);

private:
    ServoController& _servo;
    MPUSensor&       _mpu;

    float _outputSum  = 0.0f;
    float _prevOutput = 0.0f;
};
