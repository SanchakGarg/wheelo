#include "BalancePID.h"
#include "config.h"

BalancePID::BalancePID(ServoController& servo, MPUSensor& mpu)
    : _servo(servo), _mpu(mpu) {}

void BalancePID::loadGains(Preferences& prefs) {
    prefs.begin(NVS_NS, true);
    kp = prefs.getFloat("pid_kp", 0.20f);
    ki = prefs.getFloat("pid_ki", 0.00f);
    kd = prefs.getFloat("pid_kd", 0.04f);
    prefs.end();
}

void BalancePID::saveGains(Preferences& prefs) {
    prefs.begin(NVS_NS, false);
    prefs.putFloat("pid_kp", kp);
    prefs.putFloat("pid_ki", ki);
    prefs.putFloat("pid_kd", kd);
    prefs.end();
}

void BalancePID::start() {
    _integral   = 0.0f;
    gimbalAccum = 0.0f;
    _prevMeas   = _mpu.cfRoll + _mpu.rollOffset;
    _lastUs     = 0;
    running     = true;
}

void BalancePID::stop() {
    running     = false;
    _integral   = 0.0f;
    gimbalAccum = 0.0f;
    _servo.moveTo(_servo.defaultPos);
}

void BalancePID::compute() {
    if (!running) return;

    float dispRoll = _mpu.cfRoll + _mpu.rollOffset;

    if (fabsf(dispRoll) > SAFETY_DEG) {
        Serial.printf("[PID] Safety cut at %.1f deg\n", dispRoll);
        stop();
        return;
    }

    uint32_t now = micros();
    float dt = (_lastUs == 0) ? 0.002f
                              : constrain((now - _lastUs) / 1e6f, 0.0001f, 0.1f);
    _lastUs = now;

    float error = setpoint - dispRoll;
    float kp_a  = kp * KP_MAX;
    float ki_a  = ki * KI_MAX;
    float kd_a  = kd * KD_MAX;

    _integral += error * dt;
    _integral  = constrain(_integral, -ILIMIT, ILIMIT);

    float dMeas = (dispRoll - _prevMeas) / dt;
    _prevMeas   = dispRoll;

    output = kp_a * error + ki_a * _integral - kd_a * dMeas;
    output = constrain(output, -OUTPUT_MAX_DEG, OUTPUT_MAX_DEG);

    gimbalAccum += output / GIMBAL_ACCUM_RATE;
    gimbalAccum  = constrain(gimbalAccum, -OUTPUT_MAX_DEG, OUTPUT_MAX_DEG);

    _servo.moveTo(_servo.defaultPos - (int)(gimbalAccum / 0.088f));
}

void BalancePID::setGains(float newKp, float newKi, float newKd, float newSp,
                          Preferences& prefs) {
    kp = newKp; ki = newKi; kd = newKd; setpoint = newSp;
    _integral   = 0.0f;
    gimbalAccum = 0.0f;
    _servo.moveTo(_servo.defaultPos);
    saveGains(prefs);
}
