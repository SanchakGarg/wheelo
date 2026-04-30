#include "MPUSensor.h"
#include "config.h"
#include <math.h>

MPUSensor::MPUSensor(uint8_t addr, int sda, int scl)
    : _addr(addr), _sda(sda), _scl(scl) {}

static float lowPass(float prev, float x, float cutoffHz, float dt) {
    const float rc = 1.0f / (2.0f * 3.1415926535f * cutoffHz);
    const float alpha = constrain(dt / (rc + dt), 0.0f, 1.0f);
    return prev + alpha * (x - prev);
}

// 5-element sorting network → median. Branch-only, ~9 compares, no allocs.
// Median rejects the impulse spikes BLDC commutation injects into the gyro
// without introducing the phase lag of an equivalent low-pass filter.
static inline void cas(float &a, float &b) {
    if (a > b) { float t = a; a = b; b = t; }
}
static float median5(float v0, float v1, float v2, float v3, float v4) {
    float a = v0, b = v1, c = v2, d = v3, e = v4;
    cas(a, b); cas(d, e);
    cas(c, e); cas(c, d);
    cas(a, d); cas(a, c);
    cas(b, e); cas(b, d);
    cas(b, c);
    return c;
}

bool MPUSensor::writeReg(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

bool MPUSensor::readReg(uint8_t reg, uint8_t &value) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(_addr, (uint8_t)1, (uint8_t)true) != 1) return false;
    int v = Wire.read();
    if (v < 0) return false;
    value = (uint8_t)v;
    return true;
}

bool MPUSensor::rawRead(int16_t out[7]) {
    Wire.beginTransmission(_addr);
    Wire.write(REG_ACCEL_XOUT);
    if (Wire.endTransmission(false) != 0) return false;

    uint8_t n = Wire.requestFrom(_addr, (uint8_t)14, (uint8_t)true);
    if (n != 14) return false;

    for (int i = 0; i < 7; i++) {
        int hi = Wire.read();
        int lo = Wire.read();
        if (hi < 0 || lo < 0) return false;
        out[i] = (int16_t)((hi << 8) | lo);
    }

    if (MPU_UPSIDE_DOWN) {
        // Chip flipped 180° about its X axis. Accel Y/Z and gyro Y/Z
        // invert; accel X (out[0]), temp (out[3]), and gyro X (out[4])
        // stay the same because the rotation axis IS X.
        out[1] = (int16_t)-out[1];
        out[2] = (int16_t)-out[2];
        out[5] = (int16_t)-out[5];
        out[6] = (int16_t)-out[6];
    }
    return true;
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

    uint8_t who = 0;
    if (readReg(REG_WHO_AM_I, who)) {
        Serial.printf("MPU6050 WHO_AM_I=0x%02X\n", who);
    } else {
        Serial.println("MPU6050 WHO_AM_I read failed");
    }

    if (!writeReg(REG_PWR_MGMT_1, 0x01)) { ok = false; return; }
    delay(10);
    if (!writeReg(REG_CONFIG, MPU_DLPF_CFG)) { ok = false; return; }
    if (!writeReg(REG_SMPLRT_DIV, MPU_SMPLRT_DIV)) { ok = false; return; }
    if (!writeReg(REG_ACCEL_CFG, 0x00)) { ok = false; return; } // +/-2g
    if (!writeReg(REG_GYRO_CFG, 0x00)) { ok = false; return; }  // +/-250 dps
    ok = true;

    prefs.begin(NVS_NS, true);
    _biasAx        = prefs.getFloat("b_ax",      0);
    _biasAy        = prefs.getFloat("b_ay",      0);
    _biasAz        = prefs.getFloat("b_az",      0);
    _biasGx        = prefs.getFloat("b_gx",      0);
    _biasGy        = prefs.getFloat("b_gy",      0);
    _biasGz        = prefs.getFloat("b_gz",      0);
    angleOffset    = prefs.getFloat("ang_off",   0);
    gyroNoiseFloor    = prefs.getFloat("gyro_noise",     1.0f);
    accelInnovDeadband = prefs.getFloat("accel_innov_db", 1.5f);
    bool hasBias   = prefs.getBool("bias_ok", false);
    prefs.end();

    if (!hasBias) {
        Serial.println("No bias — auto-cal (keep still ~2 s)…");
        calibrate(prefs, 200);
    } else {
        Serial.println("MPU6050 ready");
    }
}

void MPUSensor::resetFilters() {
    _eAx = _eAy = _eAz = _eGx = 0.0f;
    _cfAngle = 0.0f;
    _mahonyInt = 0.0f;
    _lastUs = 0;
    _cfInited = false;
    _medIdx = 0;
    _medCount = 0;
    for (size_t i = 0; i < MEDIAN_N; i++) {
        _gxMed[i] = _axMed[i] = _ayMed[i] = _azMed[i] = 0.0f;
    }
    _accelNormFilt = 1.0f;
    rateDps = 0.0f;
    accelAngle = 0.0f;
    accelNormG = 1.0f;
}

bool MPUSensor::read() {
    if (!ok || calibrating) return false;

    int16_t raw[7];
    if (!rawRead(raw)) {
        droppedReads++;
        return false;
    }

    float rax = raw[0] / 16384.0f - _biasAx;
    float ray = raw[1] / 16384.0f - _biasAy;
    float raz = raw[2] / 16384.0f - _biasAz;
    float rgx = raw[4] / 131.0f   - _biasGx;

    uint32_t now = micros();
    float dt = _cfInited ? constrain((now - _lastUs) / 1e6f, 0.005f, 0.025f) : 0.01f;
    _lastUs = now;

    // Median pre-filter: rejects EMF impulse spikes the hardware DLPF lets
    // through. Now applied to ALL three accel axes plus gyro X, so the
    // accel norm computed below is consistent across components.
    _gxMed[_medIdx] = rgx;
    _axMed[_medIdx] = rax;
    _ayMed[_medIdx] = ray;
    _azMed[_medIdx] = raz;
    _medIdx = (_medIdx + 1) % MEDIAN_N;
    if (_medCount < MEDIAN_N) _medCount++;
    if (_medCount == MEDIAN_N) {
        rgx = median5(_gxMed[0], _gxMed[1], _gxMed[2], _gxMed[3], _gxMed[4]);
        rax = median5(_axMed[0], _axMed[1], _axMed[2], _axMed[3], _axMed[4]);
        ray = median5(_ayMed[0], _ayMed[1], _ayMed[2], _ayMed[3], _ayMed[4]);
        raz = median5(_azMed[0], _azMed[1], _azMed[2], _azMed[3], _azMed[4]);
    }

    // Instantaneous norm — exposed for telemetry, but NOT used for
    // divergence detection. Vibration makes this swing 0.7-1.3 g per
    // sample even when the chassis is genuinely still.
    accelNormG = sqrtf(rax * rax + ray * ray + raz * raz);

    // Filtered norm — averages out BLDC vibration so divergence checks
    // and bias tracking only fire when the chassis is REALLY accelerating
    // (impact, fall, manual handling), not just shaking in place.
    _accelNormFilt = lowPass(_accelNormFilt, accelNormG, 1.5f, dt);

    if (!_cfInited) {
        _eAx = rax;
        _eAy = ray;
        _eAz = raz;
        _eGx = rgx;
        accelAngle = atan2f(_eAy, _eAz) * 57.2957795f;
        _cfAngle = accelAngle;
        _cfInited = true;
    } else {
        _eAx = lowPass(_eAx, rax, ACCEL_LP_HZ, dt);
        _eAy = lowPass(_eAy, ray, ACCEL_LP_HZ, dt);
        _eAz = lowPass(_eAz, raz, ACCEL_LP_HZ, dt);
        _eGx = lowPass(_eGx, rgx, GYRO_LP_HZ,  dt);
        accelAngle = atan2f(_eAy, _eAz) * 57.2957795f;
    }

    // rateDps: deadbanded for PID derivative and dashboard display only.
    // Mahony integration below uses _eGx directly (no deadband).
    rateDps = (fabsf(_eGx) < gyroNoiseFloor) ? 0.0f : _eGx;

    // ── Mahony complementary filter ────────────────────────────────────────
    // Replaces the hard innovation deadband + slow bias tracker.
    // Simulation (sim_filter.py) shows 1.4x RMSE improvement over the
    // deadband CF at the same thermal drift rate (0.12 dps/s ramp).
    //
    // Kp: proportional pull toward accel — rejects drift that's already
    //     accumulated in the angle (steady-state error = drift_dps/Kp).
    // Ki: integral correction — builds a running bias estimate so that
    //     the gyro integration itself converges (steady-state error from
    //     ramp drift = drift_rate/Ki = 0.12/0.30 ≈ 0.4 deg).
    // No hard deadband: BLDC vibration is rejected by the 4 Hz accel LP
    // and the 0.32 Hz Mahony crossover, not by gating.

    // Scale trust when accel norm deviates (robot is really accelerating).
    float kp = MAHONY_KP;
    float ki = MAHONY_KI;
    const float norm3 = sqrtf(_eAx * _eAx + _eAy * _eAy + _eAz * _eAz);
    if (norm3 < 0.3f) {
        // Wildly invalid accel — pure gyro integration this sample.
        kp = 0.0f; ki = 0.0f;
    } else if (fabsf(_accelNormFilt - 1.0f) > ACCEL_DIVERGE_G) {
        // Real acceleration event (impact/fall) — reduce but don't freeze.
        kp *= 0.15f; ki *= 0.15f;
    }

    if (norm3 > 0.1f) {
        // Normalize LP-filtered accel and compute cross-product innovation.
        // e_deg ≈ accel_angle - _cfAngle for small angles; exact for large.
        const float ay_n  = _eAy / norm3;
        const float az_n  = _eAz / norm3;
        const float th    = _cfAngle * 0.017453292f;   // deg to rad
        const float e_deg = (ay_n * cosf(th) - az_n * sinf(th)) * 57.2957795f;

        // Integral: online bias estimate in dps (converges to −thermal_drift).
        _mahonyInt += ki * e_deg * dt;
        _mahonyInt  = constrain(_mahonyInt, -MAX_MAHONY_INT, MAX_MAHONY_INT);

        // Integrate: LP-filtered gyro + proportional correction + bias estimate.
        _cfAngle += (_eGx + kp * e_deg + _mahonyInt) * dt;
    } else {
        _cfAngle += _eGx * dt;   // no valid accel — pure gyro
    }

    angle = _cfAngle + angleOffset;
    return true;
}

void MPUSensor::calibrate(Preferences& prefs, int samples) {
    if (!ok) return;
    calibrating = true;
    Serial.printf("Bias cal (%d samples)…\n", samples);
    double ax=0,ay=0,az=0,gx=0,gy=0,gz=0;
    int collected = 0;
    int attempts = 0;
    const int maxAttempts = samples * 4;
    while (collected < samples && attempts < maxAttempts) {
        attempts++;
        int16_t raw[7];
        if (!rawRead(raw)) {
            droppedReads++;
            delay(2);
            continue;
        }
        ax+=raw[0]/16384.0; ay+=raw[1]/16384.0; az+=raw[2]/16384.0;
        gx+=raw[4]/131.0;   gy+=raw[5]/131.0;   gz+=raw[6]/131.0;
        collected++;
        delay(2);
    }

    if (collected == 0) {
        Serial.println("Bias cal failed: no valid MPU samples");
        calibrating = false;
        return;
    }
    if (collected < samples) {
        Serial.printf("Bias cal used %d/%d valid samples\n", collected, samples);
    }

    _biasAx=ax/collected;   _biasAy=ay/collected;
    _biasAz=az/collected-1.0f;
    _biasGx=gx/collected;   _biasGy=gy/collected;   _biasGz=gz/collected;
    resetFilters();
    Serial.printf("Bias: ax=%.3f ay=%.3f az=%.3f gx=%.3f\n",
                  _biasAx, _biasAy, _biasAz, _biasGx);
    saveBias(prefs);
    calibrating = false;
}

bool MPUSensor::characterizeNoise(Preferences& prefs, int samples) {
    if (!ok || calibrating) return false;
    calibrating = true;
    Serial.printf("Noise cal (%d samples, ~%d s)…\n", samples, samples / 100);

    // Separate median buffers for gyro and both accel axes, mirroring read().
    float medGx[MEDIAN_N]={}, medAy[MEDIAN_N]={}, medAz[MEDIAN_N]={};
    uint8_t medIdx = 0, medCount = 0;
    float filtGx = 0.0f, filtAy = 0.0f, filtAz = 1.0f;
    bool inited = false;

    // Warm up: prime the median ring and LP filters before collecting stats.
    for (int i = 0; i < (int)(MEDIAN_N + 15); i++) {
        int16_t raw[7];
        if (!rawRead(raw)) { delay(10); continue; }
        float rgx = raw[4] / 131.0f - _biasGx;
        float ray = raw[1] / 16384.0f - _biasAy;
        float raz = raw[2] / 16384.0f - _biasAz;
        medGx[medIdx] = rgx; medAy[medIdx] = ray; medAz[medIdx] = raz;
        medIdx = (medIdx + 1) % MEDIAN_N;
        if (medCount < MEDIAN_N) medCount++;
        if (!inited) { filtGx = rgx; filtAy = ray; filtAz = raz; inited = true; }
        delay(10);
    }

    // Collect gyro rate AND accel angle through the identical pipeline as read().
    double sumGx=0, sumSqGx=0, sumAng=0, sumSqAng=0;
    int collected = 0;
    uint32_t lastUs = micros();

    for (int attempts = 0; collected < samples && attempts < samples * 4; attempts++) {
        int16_t raw[7];
        if (!rawRead(raw)) { droppedReads++; delay(10); continue; }

        uint32_t now = micros();
        float dt = constrain((now - lastUs) / 1e6f, 0.005f, 0.025f);
        lastUs = now;

        float rgx = raw[4] / 131.0f - _biasGx;
        float ray = raw[1] / 16384.0f - _biasAy;
        float raz = raw[2] / 16384.0f - _biasAz;

        medGx[medIdx] = rgx; medAy[medIdx] = ray; medAz[medIdx] = raz;
        medIdx = (medIdx + 1) % MEDIAN_N;
        if (medCount < MEDIAN_N) medCount++;
        if (medCount == MEDIAN_N) {
            rgx = median5(medGx[0], medGx[1], medGx[2], medGx[3], medGx[4]);
            ray = median5(medAy[0], medAy[1], medAy[2], medAy[3], medAy[4]);
            raz = median5(medAz[0], medAz[1], medAz[2], medAz[3], medAz[4]);
        }

        const float rcG  = 1.0f / (2.0f * 3.1415926535f * GYRO_LP_HZ);
        filtGx += constrain(dt / (rcG + dt), 0.0f, 1.0f) * (rgx - filtGx);

        const float rcA  = 1.0f / (2.0f * 3.1415926535f * ACCEL_LP_HZ);
        const float alpA = constrain(dt / (rcA + dt), 0.0f, 1.0f);
        filtAy += alpA * (ray - filtAy);
        filtAz += alpA * (raz - filtAz);
        float ang = atan2f(filtAy, filtAz) * 57.2957795f;

        sumGx   += filtGx;      sumSqGx  += (double)filtGx * filtGx;
        sumAng  += ang;         sumSqAng += (double)ang * ang;
        collected++;
        delay(10);
    }

    calibrating = false;
    if (collected < 10) { Serial.println("Noise cal failed: too few samples"); return false; }

    // Gyro: 3σ of filtered rate (residual mean ≈ 0 after bias cal, but include it).
    double meanGx = sumGx / collected;
    double stdGx  = sqrt(fmax(0.0, sumSqGx / collected - meanGx * meanGx));
    gyroNoiseFloor = (float)fmax(0.15, fabs(meanGx) + 3.0 * stdGx);

    // Accel angle: we only need the spread (σ), NOT the mean — the mean is the
    // actual tilt angle which is not noise. 3σ covers the BLDC vibration envelope.
    double meanAng = sumAng / collected;
    double stdAng  = sqrt(fmax(0.0, sumSqAng / collected - meanAng * meanAng));
    accelInnovDeadband = (float)fmax(0.5, 3.0 * stdAng);

    Serial.printf("Gyro:  mean=%.4f stddev=%.4f → floor=%.3f °/s\n",
                  (float)meanGx, (float)stdGx, gyroNoiseFloor);
    Serial.printf("Accel: stddev=%.4f → innov_db=%.3f °\n",
                  (float)stdAng, accelInnovDeadband);

    prefs.begin(NVS_NS, false);
    prefs.putFloat("gyro_noise",    gyroNoiseFloor);
    prefs.putFloat("accel_innov_db", accelInnovDeadband);
    prefs.end();
    return true;
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
