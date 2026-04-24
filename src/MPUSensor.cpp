#include "MPUSensor.h"
#include "config.h"
#include <math.h>

MPUSensor::MPUSensor(uint8_t addr, int sda, int scl)
    : _addr(addr), _sda(sda), _scl(scl) {}

void MPUSensor::begin(Preferences& prefs) {
    Wire.begin(_sda, _scl);
    Wire.setClock(400000);
    Wire.beginTransmission(_addr);
    Wire.write(0x6B); Wire.write(0x00);
    ok = (Wire.endTransmission() == 0);
    Serial.printf("MPU6050: %s\n", ok ? "found" : "NOT FOUND");

    prefs.begin(NVS_NS, true);
    _bias.ax   = prefs.getFloat("b_ax",     0);
    _bias.ay   = prefs.getFloat("b_ay",     0);
    _bias.az   = prefs.getFloat("b_az",     0);
    _bias.gx   = prefs.getFloat("b_gx",     0);
    _bias.gy   = prefs.getFloat("b_gy",     0);
    _bias.gz   = prefs.getFloat("b_gz",     0);
    _vibStd[0] = prefs.getFloat("v_ax",     0);
    _vibStd[1] = prefs.getFloat("v_ay",     0);
    _vibStd[2] = prefs.getFloat("v_az",     0);
    _vibStd[3] = prefs.getFloat("v_gx",     0);
    _vibStd[4] = prefs.getFloat("v_gy",     0);
    _vibStd[5] = prefs.getFloat("v_gz",     0);
    rollOffset  = prefs.getFloat("roll_off",  0);
    pitchOffset = prefs.getFloat("pitch_off", 0);
    prefs.end();
}

void MPUSensor::rawRead(int16_t out[7]) {
    Wire.beginTransmission(_addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)_addr, (uint8_t)14, (uint8_t)true);
    for (int i = 0; i < 7; i++)
        out[i] = (int16_t)((Wire.read() << 8) | Wire.read());
}

float MPUSensor::dezone(float v, float std) {
    if (std < 0.0001f) return v;
    float t = std * 2.5f;
    if (fabsf(v) <= t) return 0.0f;
    return v > 0 ? v - t : v + t;
}

void MPUSensor::read() {
    if (!ok || calibrating) return;

    int16_t raw[7];
    rawRead(raw);

    float rax = raw[0] / 16384.0f * 9.81f - _bias.ax;
    float ray = raw[1] / 16384.0f * 9.81f - _bias.ay;
    float raz = raw[2] / 16384.0f * 9.81f - _bias.az;
    float rgx = raw[4] / 131.0f           - _bias.gx;
    float rgy = raw[5] / 131.0f           - _bias.gy;
    float rgz = raw[6] / 131.0f           - _bias.gz;

    if (!_emaInit) { _ema = {rax,ray,raz,rgx,rgy,rgz}; _emaInit = true; }
    _ema.ax = EMA_A*rax + (1-EMA_A)*_ema.ax;
    _ema.ay = EMA_A*ray + (1-EMA_A)*_ema.ay;
    _ema.az = EMA_A*raz + (1-EMA_A)*_ema.az;
    _ema.gx = EMA_A*rgx + (1-EMA_A)*_ema.gx;
    _ema.gy = EMA_A*rgy + (1-EMA_A)*_ema.gy;
    _ema.gz = EMA_A*rgz + (1-EMA_A)*_ema.gz;

    filtered.ax = dezone(_ema.ax, _vibStd[0]);
    filtered.ay = dezone(_ema.ay, _vibStd[1]);
    filtered.az = dezone(_ema.az, _vibStd[2]);
    filtered.gx = dezone(_ema.gx, _vibStd[3]);
    filtered.gy = dezone(_ema.gy, _vibStd[4]);
    filtered.gz = dezone(_ema.gz, _vibStd[5]);

    uint32_t now = micros();
    float dt = (_cfLastUs == 0) ? 0.02f
                                : constrain((now - _cfLastUs) / 1e6f, 0.001f, 0.1f);
    _cfLastUs = now;

    float accelRoll  = atan2f(_ema.ay, _ema.az) * 57.2958f;
    float accelPitch = atan2f(-_ema.ax, sqrtf(_ema.ay*_ema.ay + _ema.az*_ema.az)) * 57.2958f;

    cfRoll  = 0.98f * (cfRoll  + _ema.gx * dt) + 0.02f * accelRoll;
    cfPitch = 0.98f * (cfPitch + _ema.gy * dt) + 0.02f * accelPitch;
}

void MPUSensor::calibrate(Preferences& prefs, int samples) {
    if (!ok) return;
    calibrating = true;
    Serial.println("Calibrating MPU6050 — keep still...");
    double sax=0, say=0, saz=0, sgx=0, sgy=0, sgz=0;
    for (int i = 0; i < samples; i++) {
        int16_t raw[7]; rawRead(raw);
        sax += raw[0] / 16384.0 * 9.81;
        say += raw[1] / 16384.0 * 9.81;
        saz += raw[2] / 16384.0 * 9.81;
        sgx += raw[4] / 131.0;
        sgy += raw[5] / 131.0;
        sgz += raw[6] / 131.0;
        delay(2);
    }
    _bias.ax = sax / samples;
    _bias.ay = say / samples;
    _bias.az = saz / samples - 9.81f;
    _bias.gx = sgx / samples;
    _bias.gy = sgy / samples;
    _bias.gz = sgz / samples;
    Serial.printf("Bias: ax=%.3f ay=%.3f az=%.3f gx=%.3f gy=%.3f gz=%.3f\n",
                  _bias.ax, _bias.ay, _bias.az, _bias.gx, _bias.gy, _bias.gz);
    prefs.begin(NVS_NS, false);
    prefs.putFloat("b_ax", _bias.ax); prefs.putFloat("b_ay", _bias.ay);
    prefs.putFloat("b_az", _bias.az); prefs.putFloat("b_gx", _bias.gx);
    prefs.putFloat("b_gy", _bias.gy); prefs.putFloat("b_gz", _bias.gz);
    prefs.end();
    calibrating = false;
}

void MPUSensor::vibCalibrate(Preferences& prefs, int samples) {
    if (!ok) return;
    calibrating = true;
    Serial.println("Vib-cal — measuring noise floor...");
    double mean[6] = {}, M2[6] = {};
    for (int i = 0; i < samples; i++) {
        int16_t raw[7]; rawRead(raw);
        double v[6] = {
            raw[0]/16384.0*9.81 - _bias.ax, raw[1]/16384.0*9.81 - _bias.ay,
            raw[2]/16384.0*9.81 - _bias.az, raw[4]/131.0 - _bias.gx,
            raw[5]/131.0 - _bias.gy,        raw[6]/131.0 - _bias.gz
        };
        for (int j = 0; j < 6; j++) {
            double delta = v[j] - mean[j];
            mean[j] += delta / (i+1);
            M2[j]   += delta * (v[j] - mean[j]);
        }
        delay(4);
    }
    prefs.begin(NVS_NS, false);
    const char* keys[6] = {"v_ax","v_ay","v_az","v_gx","v_gy","v_gz"};
    for (int i = 0; i < 6; i++) {
        _vibStd[i] = sqrtf(M2[i] / samples);
        prefs.putFloat(keys[i], _vibStd[i]);
    }
    prefs.end();
    _emaInit = false;
    Serial.printf("VibStd: ax=%.4f ay=%.4f az=%.4f gx=%.4f gy=%.4f gz=%.4f\n",
                  _vibStd[0],_vibStd[1],_vibStd[2],_vibStd[3],_vibStd[4],_vibStd[5]);
    calibrating = false;
}

void MPUSensor::saveOffsets(Preferences& prefs) {
    prefs.begin(NVS_NS, false);
    prefs.putFloat("roll_off",  rollOffset);
    prefs.putFloat("pitch_off", pitchOffset);
    prefs.end();
}

void MPUSensor::resetAngles(Preferences& prefs) {
    rollOffset  = -cfRoll;
    pitchOffset = -cfPitch;
    saveOffsets(prefs);
}

void MPUSensor::setAngles(float roll, float pitch, Preferences& prefs) {
    rollOffset  = roll  - cfRoll;
    pitchOffset = pitch - cfPitch;
    saveOffsets(prefs);
}
