#include "BalancePID.h"
#include "config.h"

BalancePID::BalancePID(ServoController& servo, MPUSensor& mpu)
    : _servo(servo), _mpu(mpu) {}

void BalancePID::loadGains(Preferences& prefs) {
    prefs.begin(NVS_NS, true);
    kp           = prefs.getFloat("pid_kp", 1.8f);
    ki           = prefs.getFloat("pid_ki", 0.0f);
    kd           = prefs.getFloat("pid_kd", 0.09f);
    trim         = prefs.getFloat("pid_tr", 0.0f);
    accumEnabled = prefs.getBool("pid_acc_en", false);
    prefs.end();
}

void BalancePID::saveGains(Preferences& prefs) {
    prefs.begin(NVS_NS, false);
    prefs.putFloat("pid_kp", kp);
    prefs.putFloat("pid_ki", ki);
    prefs.putFloat("pid_kd", kd);
    prefs.putFloat("pid_tr", trim);
    prefs.putBool("pid_acc_en", accumEnabled);
    prefs.end();
}

void BalancePID::start() {
    _outputSum    = 0.0f;
    _prevOutput   = 0.0f;
    setpointAccum = 0.0f;
    output        = 0.0f;
    running       = true;
}

void BalancePID::stop() {
    running       = false;
    _outputSum    = 0.0f;
    _prevOutput   = 0.0f;
    setpointAccum = 0.0f;
    output        = 0.0f;
    _servo.moveTo(_servo.defaultPos);
}

void BalancePID::compute() {
    if (!running) return;

    float angle = _mpu.angle;

    if (fabsf(angle) > SAFETY_DEG) {
        Serial.printf("[PID] Safety cut at %.1f deg\n", angle);
        stop();
        return;
    }

    if (accumEnabled) {
        // Drift setpoint toward centre to allow gimbal re-centering.
        setpointAccum += _prevOutput / SETPOINT_ACCUM_DIV;
        setpointAccum  = constrain(setpointAccum, -SETPOINT_ACCUM_LIMIT, SETPOINT_ACCUM_LIMIT);
    } else {
        setpointAccum = 0.0f;
    }

    float setpoint = setpointAccum + trim;
    float error    = setpoint - angle;
    float rate     = _mpu.rateDps;

    // PID_v1-equivalent integral with derivative damping from filtered gyro rate.
    const float ki_int = ki * SAMPLE_TIME_S;
    const float iError = fabsf(error) > ANGLE_DEADBAND_DEG ? error : 0.0f;
    const float dRate  = fabsf(rate)  > RATE_DEADBAND_DPS  ? rate  : 0.0f;

    _outputSum += ki_int * iError;
    _outputSum  = constrain(_outputSum, -OUTPUT_LIMIT, OUTPUT_LIMIT);

    float desired = kp * error + _outputSum - kd * dRate;
    if (fabsf(error) < ANGLE_DEADBAND_DEG && fabsf(rate) < RATE_DEADBAND_DPS) {
        desired = 0.0f;
        _outputSum *= 0.98f;
    }

    desired = constrain(desired, -OUTPUT_LIMIT, OUTPUT_LIMIT);
    output = _prevOutput + constrain(desired - _prevOutput,
                                     -MAX_OUTPUT_STEP,
                                      MAX_OUTPUT_STEP);
    _prevOutput = output;

    _servo.moveTo(_servo.defaultPos + SERVO_DIR * (int)(output / 0.088f));
}

void BalancePID::setGains(float newKp, float newKi, float newKd, float newTrim,
                          Preferences& prefs) {
    kp = newKp; ki = newKi; kd = newKd; trim = newTrim;
    _outputSum    = 0.0f;
    _prevOutput   = 0.0f;
    setpointAccum = 0.0f;
    _servo.moveTo(_servo.defaultPos);
    saveGains(prefs);
}

void BalancePID::setAccumEnabled(bool enabled, Preferences& prefs) {
    accumEnabled  = enabled;
    setpointAccum = 0.0f;
    _outputSum    = 0.0f;
    _prevOutput   = 0.0f;
    saveGains(prefs);
}
