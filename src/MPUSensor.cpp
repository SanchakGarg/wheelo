#include "MPUSensor.h"
#include "config.h"
#include <math.h>

MPUSensor::MPUSensor(uint8_t addr, int sda, int scl)
    : _addr(addr), _sda(sda), _scl(scl) {}

void MPUSensor::rawRead(int16_t out[7]) {
    Wire.beginTransmission(_addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(_addr, (uint8_t)14, (uint8_t)true);
    for (int i = 0; i < 7; i++)
        out[i] = (int16_t)((Wire.read() << 8) | Wire.read());
}

void MPUSensor::begin(Preferences& prefs) {
    Wire.begin(_sda, _scl);
    Wire.setClock(100000);
    delay(250);

    Serial.printf("I2C SDA=GPIO%d SCL=GPIO%d\n", _sda, _scl);
    int found = 0;
    for (uint8_t a = 1; a < 127; a++) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0) { Serial.printf("  device @ 0x%02X\n", a); found++; }
    }
    if (!found) { Serial.println("  no I2C devices"); ok = false; return; }

    Wire.beginTransmission(_addr);
    if (Wire.endTransmission() != 0) { Serial.println("MPU6050: no ACK"); ok = false; return; }

    Wire.beginTransmission(_addr); Wire.write(0x75); Wire.endTransmission(false);
    Wire.requestFrom(_addr, (uint8_t)1);
    Serial.printf("MPU6050 WHO_AM_I=0x%02X\n", Wire.read());

    Wire.beginTransmission(_addr); Wire.write(0x6B); Wire.write(0x01); Wire.endTransmission();
    delay(10);
    Wire.beginTransmission(_addr); Wire.write(0x1C); Wire.write(0x00); Wire.endTransmission();
    Wire.beginTransmission(_addr); Wire.write(0x1B); Wire.write(0x00); Wire.endTransmission();
    ok = true;

    prefs.begin(NVS_NS, true);
    _biasAx     = prefs.getFloat("b_ax",    0);
    _biasAy     = prefs.getFloat("b_ay",    0);
    _biasAz     = prefs.getFloat("b_az",    0);
    _biasGx     = prefs.getFloat("b_gx",    0);
    _biasGy     = prefs.getFloat("b_gy",    0);
    _biasGz     = prefs.getFloat("b_gz",    0);
    angleOffset = prefs.getFloat("ang_off", 0);
    bool hasBias = prefs.getBool("bias_ok", false);
    prefs.end();

    if (!hasBias) {
        Serial.println("No bias — auto-cal (keep still ~2 s)…");
        calibrate(prefs, 200);
    } else {
        Serial.println("MPU6050 ready");
    }
}

// insertion sort on a small copy — fast enough for MEDIAN_N <= 11
static float medianOf(float* buf, int n) {
    float s[11];
    for (int i = 0; i < n; i++) s[i] = buf[i];
    for (int i = 1; i < n; i++) {
        float key = s[i]; int j = i - 1;
        while (j >= 0 && s[j] > key) { s[j+1] = s[j]; j--; }
        s[j+1] = key;
    }
    return s[n / 2];
}

void MPUSensor::read() {
    if (!ok || calibrating) return;

    int16_t raw[7];
    rawRead(raw);

    float rax = raw[0] / 16384.0f - _biasAx;
    float ray = raw[1] / 16384.0f - _biasAy;
    float raz = raw[2] / 16384.0f - _biasAz;
    float rgx = raw[4] / 131.0f   - _biasGx;

    // Stage 1: EMA on raw sensor readings — reduces vibration amplitude
    if (!_emaInited) {
        _eAx=rax; _eAy=ray; _eAz=raz; _eGx=rgx;
        _emaInited = true;
    } else {
        _eAx = RAW_ALPHA*rax + (1-RAW_ALPHA)*_eAx;
        _eAy = RAW_ALPHA*ray + (1-RAW_ALPHA)*_eAy;
        _eAz = RAW_ALPHA*raz + (1-RAW_ALPHA)*_eAz;
        _eGx = RAW_ALPHA*rgx + (1-RAW_ALPHA)*_eGx;
    }

    uint32_t now = micros();
    float dt = _cfInited ? constrain((now-_lastUs)/1e6f, 0.001f, 0.1f) : 0.01f;
    _lastUs   = now;
    _cfInited = true;

    // Complementary filter
    float accelAngle = atan2f(_eAy, _eAz) * 57.2958f;
    _cfAngle = 0.98f * (_cfAngle + _eGx * dt) + 0.02f * accelAngle;

    // Stage 2: moving median — statistically rejects vibration spikes
    _medBuf[_medHead] = _cfAngle;
    _medHead = (_medHead + 1) % MEDIAN_N;
    if (_medCount < MEDIAN_N) _medCount++;

    angle = medianOf(_medBuf, _medCount) + angleOffset;
}

void MPUSensor::calibrate(Preferences& prefs, int samples) {
    if (!ok) return;
    calibrating = true;
    Serial.printf("Bias cal (%d samples)…\n", samples);
    double ax=0,ay=0,az=0,gx=0,gy=0,gz=0;
    for (int i = 0; i < samples; i++) {
        int16_t raw[7]; rawRead(raw);
        ax+=raw[0]/16384.0; ay+=raw[1]/16384.0; az+=raw[2]/16384.0;
        gx+=raw[4]/131.0;   gy+=raw[5]/131.0;   gz+=raw[6]/131.0;
        delay(2);
    }
    _biasAx=ax/samples;   _biasAy=ay/samples;
    _biasAz=az/samples-1.0f;
    _biasGx=gx/samples;   _biasGy=gy/samples;   _biasGz=gz/samples;
    _cfAngle=0; _cfInited=false; _emaInited=false;
    _medHead=0; _medCount=0;
    Serial.printf("Bias: ax=%.3f ay=%.3f az=%.3f gx=%.3f\n",
                  _biasAx, _biasAy, _biasAz, _biasGx);
    saveBias(prefs);
    calibrating = false;
}

void MPUSensor::resetAngle(Preferences& prefs) {
    angleOffset = -_cfAngle;
    saveOffset(prefs);
}

void MPUSensor::setAngle(float target, Preferences& prefs) {
    angleOffset = target - _cfAngle;
    saveOffset(prefs);
}

void MPUSensor::saveBias(Preferences& prefs) {
    prefs.begin(NVS_NS, false);
    prefs.putFloat("b_ax",_biasAx); prefs.putFloat("b_ay",_biasAy);
    prefs.putFloat("b_az",_biasAz); prefs.putFloat("b_gx",_biasGx);
    prefs.putFloat("b_gy",_biasGy); prefs.putFloat("b_gz",_biasGz);
    prefs.putBool("bias_ok", true);
    prefs.end();
}

void MPUSensor::saveOffset(Preferences& prefs) {
    prefs.begin(NVS_NS, false);
    prefs.putFloat("ang_off", angleOffset);
    prefs.end();
}
