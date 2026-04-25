#include "BalancePID.h"
#include "config.h"

BalancePID::BalancePID(ServoController& servo, MPUSensor& mpu)
    : _servo(servo), _mpu(mpu) {}

void BalancePID::loadGains(Preferences& prefs) {
    prefs.begin(NVS_NS, true);
    kp   = prefs.getFloat("pid_kp", 1.8f);
    ki   = prefs.getFloat("pid_ki", 22.0f);
    kd   = prefs.getFloat("pid_kd", 0.09f);
    trim = prefs.getFloat("pid_tr", 0.0f);
    prefs.end();
}

void BalancePID::saveGains(Preferences& prefs) {
    prefs.begin(NVS_NS, false);
    prefs.putFloat("pid_kp", kp);
    prefs.putFloat("pid_ki", ki);
    prefs.putFloat("pid_kd", kd);
    prefs.putFloat("pid_tr", trim);
    prefs.end();
}

void BalancePID::start() {
    _outputSum    = 0.0f;
    _prevOutput   = 0.0f;
    setpointAccum = 0.0f;
    _lastAngle    = _mpu.angle;
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

    // Drift setpoint toward centre to allow gimbal re-centering (XRobots V2)
    setpointAccum += _prevOutput / SETPOINT_ACCUM_DIV;
    setpointAccum  = constrain(setpointAccum, -SETPOINT_ACCUM_LIMIT, SETPOINT_ACCUM_LIMIT);

    float setpoint = setpointAccum + trim;
    float error    = setpoint - angle;

    // PID_v1-equivalent math with fixed 10ms sample time (matches V2 SetSampleTime(10))
    const float ki_int = ki * SAMPLE_TIME_S;  // integral gain × dt
    const float kd_int = kd / SAMPLE_TIME_S;  // derivative gain / dt

    _outputSum += ki_int * error;
    _outputSum  = constrain(_outputSum, -OUTPUT_LIMIT, OUTPUT_LIMIT);

    float dInput = angle - _lastAngle;  // derivative on measurement, no filter needed (DMP is clean)
    _lastAngle   = angle;

    output = kp * error + _outputSum - kd_int * dInput;
    output = constrain(output, -OUTPUT_LIMIT, OUTPUT_LIMIT);
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
