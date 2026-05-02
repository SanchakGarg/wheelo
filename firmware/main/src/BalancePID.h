#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include "config.h"
#include "MPUSensor.h"
#include "ServoController.h"

class BalancePID {
public:
    bool  running       = false;
    bool  accumEnabled  = false;
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
    // CMG torque ∝ gimbal rate, not gimbal angle. 4°/cycle = 400°/s slew,
    // which lets a fast tilt error translate into a real precession kick
    // instead of a slow drift. ST3215 can sustain this comfortably.
    static constexpr float MAX_OUTPUT_STEP      = 4.0f;
    // Setpoint accumulator: slowly drifts the balance target toward the
    // chassis's natural lean so the gimbal can recenter between
    // disturbances. Without enough range here, the gimbal stays parked
    // at the edge of its travel and the CMG saturates the moment a
    // second disturbance arrives.
    //
    // DIV = 1000 → at output=1°, accum drifts ~0.1°/s (slow enough not to
    //              fight the P term, fast enough to clear bias in seconds).
    // LIMIT = 3.0 → absorbs up to 3° of mechanical bias before maxing out,
    //              vs 0.5° before which was too tight for any real chassis.
    static constexpr float SETPOINT_ACCUM_DIV   = 1000.0f;
    static constexpr float SETPOINT_ACCUM_LIMIT = 3.0f;
    static constexpr float SAMPLE_TIME_S        = 0.01f;   // 10ms fixed, matches V2
    static constexpr int   SERVO_DIR            = 1;

    BalancePID(ServoController& servo, MPUSensor& mpu);

    void loadGains(Preferences& prefs);
    void saveGains(Preferences& prefs);
    void start();
    void stop();
    void compute();
    void setGains(float kp, float ki, float kd, float trim, Preferences& prefs);
    void setAccumEnabled(bool enabled, Preferences& prefs);

private:
    ServoController& _servo;
    MPUSensor&       _mpu;

    float _outputSum  = 0.0f;
    float _prevOutput = 0.0f;
};
