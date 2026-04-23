// XIAO ESP32-C3  –  BLDC + ST3215 servo + MPU6050
//
// Half-duplex servo wiring (UART1):
//   D7 / GPIO20  TX  ──[1kΩ]──┬── ST3215 DATA
//   D6 / GPIO21  RX  ──────────┘
//
// I2C MPU6050:
//   D4 / GPIO6   SDA
//   D5 / GPIO7   SCL
//
// ESC: D10 / GPIO10   signal wire
//
// Servo position range:  0 – 4095  (0.088 °/step)
// BLDC throttle cap:     0 – 30 %  (1000 – 1300 µs)
// Servo swing from default: ± 350 counts

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <Preferences.h>
#include <Wire.h>

// ── WiFi ─────────────────────────────────────────────────────────────────────
static const char* WIFI_SSID = "Makerspace";
static const char* WIFI_PASS = "testbricks";

// ── ESC ──────────────────────────────────────────────────────────────────────
static const int ESC_PIN     = D10;
static const int ESC_MIN_US  = 1000;
static const int ESC_MAX_US  = 2000;
static const int ESC_MAX_PCT = 30;
Servo esc;
int   throttlePct = 0;

// ── ST3215 ───────────────────────────────────────────────────────────────────
#define SERVO_SERIAL  Serial1
#define SERVO_TX_PIN  20      // D7
#define SERVO_RX_PIN  21      // D6
#define SERVO_BAUD    1000000UL
#define SERVO_ID      1
#define REG_GOAL_POS  0x2A
static const int SERVO_SWING   = 350;
static const int SERVO_DEFAULT = 1100;   // fallback if NVS empty

int servoDefaultPos = SERVO_DEFAULT;
int servoTargetPos  = SERVO_DEFAULT;

// ── MPU6050 ──────────────────────────────────────────────────────────────────
#define MPU_ADDR  0x68
#define MPU_SDA   6     // D4
#define MPU_SCL   7     // D5
struct MpuVals  { float ax, ay, az, gx, gy, gz; };
MpuVals mpu     = {};
MpuVals bias    = {};      // subtracted from every raw reading
float   vibStd[6] = {};   // per-axis noise std dev from vib-cal; used for dead-zone
bool    mpuOk          = false;
bool    mpuCalibrating = false;
// Complementary filter angles (degrees). Persist when still; accel corrects drift.
float   cfRoll  = 0.0f;
float   cfPitch = 0.0f;
static uint32_t cfLastUs = 0;
// Display offsets — shift the zero reference without touching the CF.
// displayRoll = cfRoll + rollOffset. Saved to NVS.
float rollOffset  = 0.0f;
float pitchOffset = 0.0f;

// ── Auto-balance PID ──────────────────────────────────────────────────────────
bool  autoBalance      = false;
// Gains stored as normalised [0-1]; multiplied by KX_MAX inside runPid().
float pidKp            = 0.20f;  // × 50  → actual 10
float pidKi            = 0.00f;  // × 10
float pidKd            = 0.04f;  // × 50  → actual 2
float pidSetpoint      = 0.0f;
float pidIntegral      = 0.0f;
float pidPrevMeas      = 0.0f;
float pidOutput        = 0.0f;
uint32_t pidLastUs     = 0;
// Gimbal position accumulator (degrees offset from default).
// PID output feeds this each loop; it self-centres so the gimbal never saturates.
float gimbalAccum      = 0.0f;
static const float GIMBAL_ACCUM_RATE   = 800.0f;  // tune: lower = faster gimbal swing
static const float KP_MAX              = 50.0f;
static const float KI_MAX              = 10.0f;
static const float KD_MAX              = 50.0f;
static const float SERVO_DEG_PER_COUNT = 0.088f;
static const float PID_OUTPUT_MAX_DEG  = SERVO_SWING * SERVO_DEG_PER_COUNT; // ≈ 30.8°
static const float PID_ILIMIT          = 500.0f;  // integral clamp (degree·s)
static const float PID_SAFETY_DEG      = 60.0f;   // auto-disable if |roll| exceeds this

// ── NVS / web server ─────────────────────────────────────────────────────────
Preferences prefs;
WebServer   server(80);

// ═══════════════════════════════════════════════════════════════════════════
//  SCServo write  (write-only, no read)
// ═══════════════════════════════════════════════════════════════════════════

static void scWrite(uint8_t id, uint8_t reg, const uint8_t* data, uint8_t dlen) {
  uint8_t len = dlen + 3;   // instr + reg + data + checksum
  uint8_t sum = id + len + 0x03 + reg;
  for (uint8_t i = 0; i < dlen; i++) sum += data[i];
  uint8_t pkt[16], pi = 0;
  pkt[pi++] = 0xFF; pkt[pi++] = 0xFF;
  pkt[pi++] = id;   pkt[pi++] = len;
  pkt[pi++] = 0x03; pkt[pi++] = reg;
  for (uint8_t i = 0; i < dlen; i++) pkt[pi++] = data[i];
  pkt[pi++] = ~sum;
  SERVO_SERIAL.write(pkt, pi);
  SERVO_SERIAL.flush();
}

// ═══════════════════════════════════════════════════════════════════════════
//  BLDC
// ═══════════════════════════════════════════════════════════════════════════

static void applyThrottle(int pct) {
  pct = constrain(pct, 0, ESC_MAX_PCT);
  throttlePct = pct;
  esc.writeMicroseconds(map(pct, 0, 100, ESC_MIN_US, ESC_MAX_US));
}

// ═══════════════════════════════════════════════════════════════════════════
//  Servo
// ═══════════════════════════════════════════════════════════════════════════

static int servoLo() { return max(0,    servoDefaultPos - SERVO_SWING); }
static int servoHi() { return min(4095, servoDefaultPos + SERVO_SWING); }

static void moveServoTo(int pos) {
  pos = constrain(pos, servoLo(), servoHi());
  servoTargetPos = pos;
  uint8_t data[2] = { (uint8_t)(pos & 0xFF), (uint8_t)(pos >> 8) };
  scWrite(SERVO_ID, REG_GOAL_POS, data, 2);
}

// ═══════════════════════════════════════════════════════════════════════════
//  MPU6050
// ═══════════════════════════════════════════════════════════════════════════

static void mpuInit() {
  Wire.begin(MPU_SDA, MPU_SCL);
  Wire.setClock(400000);   // Fast-Mode I2C — needed for ≥500 Hz reads
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0x00);   // clear sleep bit
  mpuOk = (Wire.endTransmission() == 0);
  Serial.printf("MPU6050: %s\n", mpuOk ? "found" : "NOT FOUND");

  // Load saved bias + vibration thresholds from NVS
  prefs.begin("wheelo", true);
  bias.ax   = prefs.getFloat("b_ax", 0);
  bias.ay   = prefs.getFloat("b_ay", 0);
  bias.az   = prefs.getFloat("b_az", 0);
  bias.gx   = prefs.getFloat("b_gx", 0);
  bias.gy   = prefs.getFloat("b_gy", 0);
  bias.gz   = prefs.getFloat("b_gz", 0);
  vibStd[0] = prefs.getFloat("v_ax", 0);
  vibStd[1] = prefs.getFloat("v_ay", 0);
  vibStd[2] = prefs.getFloat("v_az", 0);
  vibStd[3] = prefs.getFloat("v_gx", 0);
  vibStd[4] = prefs.getFloat("v_gy", 0);
  vibStd[5]   = prefs.getFloat("v_gz",     0);
  rollOffset  = prefs.getFloat("roll_off",  0);
  pitchOffset = prefs.getFloat("pitch_off", 0);
  prefs.end();
  Serial.printf("Bias: ax=%.3f ay=%.3f az=%.3f gx=%.3f gy=%.3f gz=%.3f\n",
                bias.ax, bias.ay, bias.az, bias.gx, bias.gy, bias.gz);
  Serial.printf("VibStd: ax=%.4f ay=%.4f az=%.4f gx=%.4f gy=%.4f gz=%.4f\n",
                vibStd[0],vibStd[1],vibStd[2],vibStd[3],vibStd[4],vibStd[5]);
}

// EMA alpha: 0.0 = frozen, 1.0 = no filter. 0.12 cuts BLDC vibration well.
static const float EMA_A = 0.12f;
static MpuVals ema = {};
static bool    emaInit = false;

static void mpuRead() {
  if (!mpuOk || mpuCalibrating) return;
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (uint8_t)true);
  int16_t raw[7];
  for (int i = 0; i < 7; i++)
    raw[i] = (int16_t)((Wire.read() << 8) | Wire.read());
  // raw[3] is temperature — skip
  float rax = raw[0] / 16384.0f * 9.81f - bias.ax;
  float ray = raw[1] / 16384.0f * 9.81f - bias.ay;
  float raz = raw[2] / 16384.0f * 9.81f - bias.az;
  float rgx = raw[4] / 131.0f           - bias.gx;
  float rgy = raw[5] / 131.0f           - bias.gy;
  float rgz = raw[6] / 131.0f           - bias.gz;

  if (!emaInit) { ema = {rax,ray,raz,rgx,rgy,rgz}; emaInit = true; }
  ema.ax = EMA_A*rax + (1.0f-EMA_A)*ema.ax;
  ema.ay = EMA_A*ray + (1.0f-EMA_A)*ema.ay;
  ema.az = EMA_A*raz + (1.0f-EMA_A)*ema.az;
  ema.gx = EMA_A*rgx + (1.0f-EMA_A)*ema.gx;
  ema.gy = EMA_A*rgy + (1.0f-EMA_A)*ema.gy;
  ema.gz = EMA_A*rgz + (1.0f-EMA_A)*ema.gz;

  // Soft dead-zone: attenuate readings that fall inside the measured noise floor.
  // Threshold = 2.5 sigma. Readings above it have the threshold subtracted so
  // the output transitions smoothly instead of snapping.
  auto dezone = [](float v, float std) -> float {
    if (std < 0.0001f) return v;
    float t = std * 2.5f;
    if (fabsf(v) <= t) return 0.0f;
    return v > 0.0f ? v - t : v + t;
  };
  mpu.ax = dezone(ema.ax, vibStd[0]);
  mpu.ay = dezone(ema.ay, vibStd[1]);
  mpu.az = dezone(ema.az, vibStd[2]);
  mpu.gx = dezone(ema.gx, vibStd[3]);
  mpu.gy = dezone(ema.gy, vibStd[4]);
  mpu.gz = dezone(ema.gz, vibStd[5]);

  // Complementary filter — integrate gyro, correct with accel long-term.
  // Uses ema values (pre-dead-zone) for accel angle to avoid atan2 artifacts.
  uint32_t now = micros();
  float dt = (cfLastUs == 0) ? 0.02f : constrain((now - cfLastUs) / 1e6f, 0.001f, 0.1f);
  cfLastUs = now;
  float accelRoll  =  atan2f(ema.ay, ema.az) * 57.2958f;
  float accelPitch = atan2f(-ema.ax, sqrtf(ema.ay*ema.ay + ema.az*ema.az)) * 57.2958f;
  // Use ema.gx/gy (pre-dead-zone) so slow rotations aren't clipped by the
  // vibration threshold before being integrated into the angle.
  cfRoll  = 0.98f * (cfRoll  + ema.gx * dt) + 0.02f * accelRoll;
  cfPitch = 0.98f * (cfPitch + ema.gy * dt) + 0.02f * accelPitch;
}

// Blocks ~2 s. Sensor must be flat and still.
static void calibrateMpu(int samples = 1000) {
  if (!mpuOk) return;
  mpuCalibrating = true;
  Serial.println("Calibrating MPU6050 — keep still...");
  double sax=0,say=0,saz=0,sgx=0,sgy=0,sgz=0;
  for (int i = 0; i < samples; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (uint8_t)true);
    int16_t raw[7];
    for (int j = 0; j < 7; j++)
      raw[j] = (int16_t)((Wire.read() << 8) | Wire.read());
    sax += raw[0] / 16384.0 * 9.81;
    say += raw[1] / 16384.0 * 9.81;
    saz += raw[2] / 16384.0 * 9.81;
    sgx += raw[4] / 131.0;
    sgy += raw[5] / 131.0;
    sgz += raw[6] / 131.0;
    delay(2);
  }
  bias.ax = sax / samples;
  bias.ay = say / samples;
  bias.az = saz / samples - 9.81f;   // Z reads gravity at rest, remove it
  bias.gx = sgx / samples;
  bias.gy = sgy / samples;
  bias.gz = sgz / samples;
  Serial.printf("Done. Bias: ax=%.3f ay=%.3f az=%.3f gx=%.3f gy=%.3f gz=%.3f\n",
                bias.ax, bias.ay, bias.az, bias.gx, bias.gy, bias.gz);
  prefs.begin("wheelo", false);
  prefs.putFloat("b_ax", bias.ax); prefs.putFloat("b_ay", bias.ay);
  prefs.putFloat("b_az", bias.az); prefs.putFloat("b_gx", bias.gx);
  prefs.putFloat("b_gy", bias.gy); prefs.putFloat("b_gz", bias.gz);
  prefs.end();
  mpuCalibrating = false;
}

// Single-pass Welford variance — blocks ~2 s. Sensor flat, motor state doesn't matter.
static void vibCalibrate(int samples = 500) {
  if (!mpuOk) return;
  mpuCalibrating = true;
  Serial.println("Vib-cal — measuring noise floor, keep flat...");
  double mean[6] = {}, M2[6] = {};
  for (int i = 0; i < samples; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (uint8_t)true);
    int16_t raw[7];
    for (int j = 0; j < 7; j++) raw[j] = (int16_t)((Wire.read()<<8)|Wire.read());
    double v[6] = {
      raw[0]/16384.0*9.81 - bias.ax, raw[1]/16384.0*9.81 - bias.ay,
      raw[2]/16384.0*9.81 - bias.az, raw[4]/131.0 - bias.gx,
      raw[5]/131.0 - bias.gy,        raw[6]/131.0 - bias.gz
    };
    for (int j = 0; j < 6; j++) {
      double delta = v[j] - mean[j];
      mean[j] += delta / (i + 1);
      M2[j]   += delta * (v[j] - mean[j]);  // Welford update
    }
    delay(4);
  }
  prefs.begin("wheelo", false);
  const char* keys[6] = {"v_ax","v_ay","v_az","v_gx","v_gy","v_gz"};
  for (int i = 0; i < 6; i++) {
    vibStd[i] = sqrtf(M2[i] / samples);
    prefs.putFloat(keys[i], vibStd[i]);
  }
  prefs.end();
  emaInit = false;   // reset EMA so stale values don't bleed into fresh readings
  Serial.printf("Done. std: ax=%.4f ay=%.4f az=%.4f gx=%.4f gy=%.4f gz=%.4f\n",
                vibStd[0],vibStd[1],vibStd[2],vibStd[3],vibStd[4],vibStd[5]);
  mpuCalibrating = false;
}

// ═══════════════════════════════════════════════════════════════════════════
//  PID controller — runs at 500 Hz when autoBalance is true
//
//  Gimbal velocity mode (SetpointAccum):
//    pidOutput feeds into gimbalAccum each loop instead of directly
//    commanding a position. gimbalAccum self-centres so the gimbal keeps
//    cycling and generating continuous torque rather than parking at an
//    offset and going silent.
// ═══════════════════════════════════════════════════════════════════════════

static void runPid() {
  float dispRoll = cfRoll + rollOffset;

  // Safety: if the robot has fallen over, cut the controller.
  if (fabsf(dispRoll) > PID_SAFETY_DEG) {
    autoBalance = false;
    moveServoTo(servoDefaultPos);
    pidIntegral = 0.0f;
    gimbalAccum = 0.0f;
    Serial.printf("[PID] Safety cut at %.1f deg\n", dispRoll);
    return;
  }

  uint32_t now = micros();
  float dt = (pidLastUs == 0) ? 0.002f : constrain((now - pidLastUs) / 1e6f, 0.0001f, 0.1f);
  pidLastUs = now;

  float error   = pidSetpoint - dispRoll;

  // Scale normalised [0-1] gains to physical units.
  float kp = pidKp * KP_MAX;
  float ki = pidKi * KI_MAX;
  float kd = pidKd * KD_MAX;

  // Integral with anti-windup clamp.
  pidIntegral  += error * dt;
  pidIntegral   = constrain(pidIntegral, -PID_ILIMIT, PID_ILIMIT);

  // Derivative on measurement (not error) to avoid kick on setpoint change.
  float dMeas   = (dispRoll - pidPrevMeas) / dt;
  pidPrevMeas   = dispRoll;

  pidOutput = kp * error + ki * pidIntegral - kd * dMeas;
  pidOutput = constrain(pidOutput, -PID_OUTPUT_MAX_DEG, PID_OUTPUT_MAX_DEG);

  // Velocity mode: accumulate output into gimbal target (degrees from default).
  // Dividing by GIMBAL_ACCUM_RATE converts output magnitude to a slow ramp so
  // the gimbal doesn't slam to an extreme. The clamp keeps it within swing limits.
  gimbalAccum += pidOutput / GIMBAL_ACCUM_RATE;
  gimbalAccum  = constrain(gimbalAccum, -PID_OUTPUT_MAX_DEG, PID_OUTPUT_MAX_DEG);

  moveServoTo(servoDefaultPos - (int)(gimbalAccum / SERVO_DEG_PER_COUNT));
}

// ═══════════════════════════════════════════════════════════════════════════
//  HTML
// ═══════════════════════════════════════════════════════════════════════════

static const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no"/>
<title>Wheelo Control</title>
<style>
:root{color-scheme:dark;
  --bg:#0a0d12;--surface:#111620;--border:#1e2733;
  --blue:#4d9eff;--red:#e05252;--green:#52c87a;--muted:#5a6a80;
  --text:#d0dae8;--text2:#8a9ab0;}
*{box-sizing:border-box;margin:0;padding:0;}
html,body{min-height:100%;background:var(--bg);color:var(--text);
  font-family:'Courier New',Courier,monospace;touch-action:none;}
body{display:flex;flex-direction:column;align-items:center;
  padding:24px 14px 56px;gap:20px;}
.hdr{display:flex;align-items:center;gap:10px;width:min(98vw,880px);}
.hdr-dot{width:8px;height:8px;border-radius:50%;background:var(--green);}
h1{font-size:13px;letter-spacing:.18em;text-transform:uppercase;
  color:var(--muted);font-weight:400;}
.cols{display:flex;gap:16px;width:min(98vw,880px);align-items:flex-start;}
.cols>.panel{flex:1;min-width:0;}
.full{width:min(98vw,880px);}
.panel{background:var(--surface);border:1px solid var(--border);
  border-radius:10px;padding:18px 16px;
  display:flex;flex-direction:column;gap:14px;}
.panel-title{font-size:10px;letter-spacing:.2em;text-transform:uppercase;color:var(--muted);}
.dial-wrap{display:flex;flex-direction:column;align-items:center;gap:8px;}
.dial-svg{width:min(38vw,180px);aspect-ratio:1/1;cursor:pointer;}
.dial-val{font-size:44px;font-weight:700;letter-spacing:-.03em;
  font-variant-numeric:tabular-nums;line-height:1;}
.dial-sub{font-size:11px;color:var(--text2);letter-spacing:.06em;}
.row{display:flex;gap:8px;}
.row>*{flex:1;}
input[type=number]{background:var(--bg);color:var(--text);
  border:1px solid var(--border);border-radius:6px;
  padding:8px 10px;font-size:14px;font-family:inherit;
  font-variant-numeric:tabular-nums;width:100%;}
input:focus{outline:none;border-color:var(--blue);}
button{font-family:inherit;font-size:11px;letter-spacing:.1em;
  text-transform:uppercase;padding:9px 12px;border-radius:6px;
  border:1px solid var(--border);background:#1a2030;color:var(--text);
  cursor:pointer;transition:background .15s;white-space:nowrap;}
button:hover{background:#222d40;}
button.prim{background:#1a3a6e;border-color:var(--blue);color:var(--blue);}
button.prim:hover{background:#204488;}
button.danger{background:#3a1818;border-color:var(--red);color:var(--red);}
button.safe{background:#183328;border-color:var(--green);color:var(--green);}
.vdiv{width:1px;background:var(--border);align-self:stretch;flex-shrink:0;}
.limits{font-size:11px;color:var(--muted);letter-spacing:.04em;}
.limits span{color:var(--text2);}
/* MPU grid */
.mpu-grid{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;}
.mpu-cell{background:var(--bg);border:1px solid var(--border);border-radius:6px;
  padding:8px 10px;display:flex;flex-direction:column;gap:2px;}
.mpu-lbl{font-size:9px;letter-spacing:.14em;text-transform:uppercase;color:var(--muted);}
.mpu-num{font-size:18px;font-variant-numeric:tabular-nums;font-weight:700;}
.mpu-unit{font-size:9px;color:var(--muted);}
/* graph */
.graph-wrap{display:flex;flex-direction:column;gap:6px;}
.graph-title{font-size:9px;letter-spacing:.14em;text-transform:uppercase;color:var(--muted);}
.graph-legend{display:flex;gap:12px;flex-wrap:wrap;}
.legend-item{display:flex;align-items:center;gap:5px;font-size:10px;color:var(--text2);}
.legend-dot{width:10px;height:3px;border-radius:2px;}
canvas{width:100%;height:90px;border-radius:6px;background:var(--bg);
  border:1px solid var(--border);display:block;}
hr{border:none;border-top:1px solid var(--border);width:100%;}
#toast{position:fixed;bottom:22px;left:50%;transform:translateX(-50%);
  background:#1e2d20;border:1px solid var(--green);border-radius:8px;
  padding:9px 18px;font-size:12px;color:var(--green);letter-spacing:.08em;
  opacity:0;transition:opacity .3s;pointer-events:none;white-space:nowrap;}
#toast.show{opacity:1;}
@media(max-width:540px){
  .cols{flex-direction:column;}.vdiv{display:none;}
  .dial-svg{width:min(56vw,220px);}
  .mpu-grid{grid-template-columns:repeat(2,1fr);}
}
</style>
</head>
<body>

<div class="hdr"><div class="hdr-dot"></div><h1>Wheelo &middot; Control Panel</h1></div>

<div class="cols">

  <!-- BLDC -->
  <div class="panel">
    <div class="panel-title">&#9654; BLDC Throttle</div>
    <div class="dial-wrap">
      <svg id="bldc-dial" class="dial-svg" viewBox="-100 -100 200 200">
        <circle cx="0" cy="0" r="82" fill="none" stroke="#1e2733" stroke-width="13"/>
        <path id="bldc-arc" fill="none" stroke="#4d9eff" stroke-width="13" stroke-linecap="round"/>
        <circle id="bldc-knob" r="11" fill="#4d9eff" stroke="#0a0d12" stroke-width="3"/>
      </svg>
      <div id="bldc-val" class="dial-val">0<span style="font-size:22px;color:var(--text2)">%</span></div>
      <div id="bldc-us" class="dial-sub">1000 µs</div>
    </div>
    <div class="limits">Range: <span>0 – 30 %</span> &nbsp;|&nbsp; <span>1000 – 1300 µs</span></div>
    <div class="row">
      <input id="bldc-num" type="number" min="0" max="30" step="1" value="0" inputmode="numeric"/>
      <button class="prim" id="bldc-apply">Apply</button>
    </div>
    <button class="danger" id="bldc-stop">STOP (0%)</button>
  </div>

  <div class="vdiv"></div>

  <!-- Servo -->
  <div class="panel">
    <div class="panel-title">&#9711; ST3215 Position</div>
    <div class="dial-wrap">
      <svg id="srv-dial" class="dial-svg" viewBox="-100 -100 200 200">
        <circle cx="0" cy="0" r="82" fill="none" stroke="#1e2733" stroke-width="13"/>
        <path id="srv-arc" fill="none" stroke="#52c87a" stroke-width="13" stroke-linecap="round"/>
        <circle id="srv-knob" r="11" fill="#52c87a" stroke="#0a0d12" stroke-width="3"/>
      </svg>
      <div id="srv-val" class="dial-val">1100</div>
      <div id="srv-limits" class="dial-sub limits">750 – 1450</div>
    </div>
    <div class="row">
      <input id="srv-num" type="number" min="0" max="4095" step="1" value="1100" inputmode="numeric"/>
      <button class="prim" id="srv-apply">Move</button>
    </div>
    <hr/>
    <div class="limits">Default pos: <span id="srv-defshow">1100</span></div>
    <div class="row">
      <input id="srv-def-num" type="number" min="0" max="4095" step="1"
             placeholder="New default…" inputmode="numeric"/>
      <button class="safe" id="srv-savedef">Save default</button>
    </div>
  </div>

</div>

<!-- Auto-Balance PID -->
<div class="panel full" id="balance-panel">
  <div style="display:flex;align-items:center;justify-content:space-between;flex-wrap:wrap;gap:8px;">
    <div class="panel-title">&#9878; Auto-Balance &mdash; Roll PID</div>
    <button id="bal-toggle" class="safe">START</button>
  </div>
  <div id="bal-status" style="font-size:11px;color:var(--muted);">Stopped &mdash; servo under manual control</div>

  <div class="mpu-grid">
    <div class="mpu-cell">
      <div class="mpu-lbl">Error</div>
      <div class="mpu-num" id="bal-err" style="color:#ff9900;">—</div>
      <div class="mpu-unit">deg</div></div>
    <div class="mpu-cell">
      <div class="mpu-lbl">PID Output</div>
      <div class="mpu-num" id="bal-out">—</div>
      <div class="mpu-unit">deg</div></div>
    <div class="mpu-cell">
      <div class="mpu-lbl">Gimbal Accum</div>
      <div class="mpu-num" id="bal-accum" style="color:#8855cc;">—</div>
      <div class="mpu-unit">deg</div></div>
    <div class="mpu-cell">
      <div class="mpu-lbl">Servo Pos</div>
      <div class="mpu-num" id="bal-srv">—</div>
      <div class="mpu-unit">/ 4095</div></div>
  </div>

  <hr/>
  <div class="panel-title">PID Gains &nbsp;<span style="font-size:9px;color:var(--muted);text-transform:none;letter-spacing:.02em;">[0&ndash;1] &nbsp;&#9660;&#9650; = &plusmn;0.05</span></div>
  <div class="row">
    <div style="display:flex;flex-direction:column;gap:4px;">
      <div style="font-size:9px;color:var(--muted);">Kp &mdash; proportional</div>
      <div style="display:flex;gap:3px;">
        <button class="pid-step" data-id="pid-kp" data-step="-0.05" style="padding:7px 9px;">&#9660;</button>
        <input id="pid-kp" type="number" step="any" min="0" max="1" value="0.20" inputmode="decimal" style="text-align:center;"/>
        <button class="pid-step" data-id="pid-kp" data-step="0.05" style="padding:7px 9px;">&#9650;</button>
      </div>
    </div>
    <div style="display:flex;flex-direction:column;gap:4px;">
      <div style="font-size:9px;color:var(--muted);">Ki &mdash; integral</div>
      <div style="display:flex;gap:3px;">
        <button class="pid-step" data-id="pid-ki" data-step="-0.05" style="padding:7px 9px;">&#9660;</button>
        <input id="pid-ki" type="number" step="any" min="0" max="1" value="0.00" inputmode="decimal" style="text-align:center;"/>
        <button class="pid-step" data-id="pid-ki" data-step="0.05" style="padding:7px 9px;">&#9650;</button>
      </div>
    </div>
    <div style="display:flex;flex-direction:column;gap:4px;">
      <div style="font-size:9px;color:var(--muted);">Kd &mdash; derivative</div>
      <div style="display:flex;gap:3px;">
        <button class="pid-step" data-id="pid-kd" data-step="-0.05" style="padding:7px 9px;">&#9660;</button>
        <input id="pid-kd" type="number" step="any" min="0" max="1" value="0.04" inputmode="decimal" style="text-align:center;"/>
        <button class="pid-step" data-id="pid-kd" data-step="0.05" style="padding:7px 9px;">&#9650;</button>
      </div>
    </div>
  </div>
  <div class="row">
    <div style="display:flex;flex-direction:column;gap:4px;">
      <div style="font-size:9px;color:var(--muted);">Setpoint (°)</div>
      <div style="display:flex;gap:3px;">
        <button class="pid-step" data-id="pid-sp" data-step="-0.5" style="padding:7px 9px;">&#9660;</button>
        <input id="pid-sp" type="number" step="0.5" min="-30" max="30" value="0.0" inputmode="decimal" style="text-align:center;"/>
        <button class="pid-step" data-id="pid-sp" data-step="0.5" style="padding:7px 9px;">&#9650;</button>
      </div>
    </div>
    <div style="display:flex;align-items:flex-end;">
      <button class="prim" id="pid-apply">Apply</button>
    </div>
    <div></div>
  </div>

  <div style="font-size:10px;color:var(--muted);line-height:1.85;border-top:1px solid var(--border);padding-top:10px;">
    <b style="color:#ff9900;">&#9650; Before starting:</b> hold the robot in its balance pose &rarr; press <b style="color:var(--text2);">Reset Angles</b> below so Roll reads 0&deg; &rarr; then hit START.<br/><br/>
    <b style="color:var(--text2);">How PID works:</b><br/>
    &bull; <b style="color:var(--text2);">P</b> (Kp): robot tilts &rarr; servo pushed proportionally. Higher = stronger reaction. Too high = shaking.<br/>
    &bull; <b style="color:var(--text2);">D</b> (Kd): brakes the correction as the robot swings back. Stops overshoot. Add it once P is oscillating.<br/>
    &bull; <b style="color:var(--text2);">I</b> (Ki): slowly cancels a permanent lean that P alone cannot fix. Keep tiny — too much causes slow drift.<br/><br/>
    <b style="color:var(--text2);">Tuning recipe:</b> Kp=1 Ki=0 Kd=0 &rarr; raise Kp until shaking &rarr; raise Kd to damp &rarr; tiny Ki to fix lean.<br/>
    Wrong direction? Flip Kp sign (e.g. &minus;1.0). Safety: auto-stops if |Roll| &gt; 60&deg;.
  </div>
</div>

<!-- MPU6050 -->
<div class="panel full">
  <div style="display:flex;align-items:center;justify-content:space-between;flex-wrap:wrap;gap:8px;">
    <div class="panel-title">&#11835; MPU6050 &nbsp;<span id="mpu-status" style="color:var(--muted)">connecting…</span></div>
    <div style="display:flex;gap:8px;">
      <button id="mpu-cal-btn" class="prim" style="font-size:10px;padding:7px 12px;">Bias Cal</button>
      <button id="mpu-vib-btn" style="font-size:10px;padding:7px 12px;background:#2a1f3a;border-color:#8855cc;color:#aa77ee;">Vib Cal</button>
    </div>
  </div>
  <div id="mpu-cal-msg" style="font-size:11px;color:var(--muted);display:none;"></div>

  <div class="mpu-grid">
    <div class="mpu-cell"><div class="mpu-lbl">Gyro X</div>
      <div class="mpu-num" id="m-gx">—</div><div class="mpu-unit">°/s</div></div>
    <div class="mpu-cell"><div class="mpu-lbl">Gyro Y</div>
      <div class="mpu-num" id="m-gy">—</div><div class="mpu-unit">°/s</div></div>
    <div class="mpu-cell"><div class="mpu-lbl">Gyro Z</div>
      <div class="mpu-num" id="m-gz">—</div><div class="mpu-unit">°/s</div></div>
    <!-- CF angles -->
    <div class="mpu-cell" style="border-color:#ff9900aa;">
      <div class="mpu-lbl" style="color:#ff9900;">Roll</div>
      <div class="mpu-num" id="m-roll" style="color:#ff9900;">—</div>
      <div class="mpu-unit">deg</div></div>
    <div class="mpu-cell" style="border-color:#00ccccaa;">
      <div class="mpu-lbl" style="color:#00cccc;">Pitch</div>
      <div class="mpu-num" id="m-pitch" style="color:#00cccc;">—</div>
      <div class="mpu-unit">deg</div></div>
    <div class="mpu-cell" style="padding:6px;">
      <button id="reset-angles-btn" style="width:100%;height:100%;font-size:9px;
        letter-spacing:.08em;background:#1a2030;border-color:#3a4a5a;color:#5a7a9a;
        border-radius:5px;cursor:pointer;padding:4px;">Reset<br/>Angles</button>
    </div>
  </div>

  <!-- Set initial angle -->
  <div style="display:flex;gap:8px;align-items:center;flex-wrap:wrap;">
    <span style="font-size:10px;letter-spacing:.1em;text-transform:uppercase;color:var(--muted);white-space:nowrap;">Set angles:</span>
    <input id="init-roll"  type="number" step="0.1" min="-360" max="360" placeholder="Roll °"
      inputmode="decimal"
      style="flex:1;min-width:70px;background:var(--bg);color:#ff9900;border:1px solid #ff990066;
             border-radius:6px;padding:7px 8px;font-size:13px;font-family:inherit;"/>
    <input id="init-pitch" type="number" step="0.1" min="-360" max="360" placeholder="Pitch °"
      inputmode="decimal"
      style="flex:1;min-width:70px;background:var(--bg);color:#00cccc;border:1px solid #00cccc66;
             border-radius:6px;padding:7px 8px;font-size:13px;font-family:inherit;"/>
    <button id="set-angles-btn" class="prim" style="font-size:10px;padding:7px 14px;white-space:nowrap;">Set</button>
  </div>

  <!-- Vertical indicator -->
  <div id="vertical-banner" style="display:none;background:#3a1010;border:1px solid #e05252;
    border-radius:8px;padding:10px 18px;text-align:center;font-size:13px;
    letter-spacing:.15em;color:#e05252;text-transform:uppercase;">
    &#9650; Robot is Vertical &#9650;
  </div>

  <div class="graph-wrap">
    <div class="graph-title">Orientation — Complementary Filter &nbsp;<span style="color:var(--muted)">(auto-scale °)</span></div>
    <div class="graph-legend">
      <div class="legend-item"><div class="legend-dot" style="background:#ff9900"></div>Roll</div>
      <div class="legend-item"><div class="legend-dot" style="background:#00cccc"></div>Pitch</div>
    </div>
    <canvas id="gyro-graph"></canvas>
  </div>
</div>

<div id="toast"></div>

<script>
(() => {
'use strict';

// ── Dial factory ─────────────────────────────────────────────────────────────
function makeDial(svgId, arcId, knobId, valId, initMin, initMax, onchange) {
  const svg  = document.getElementById(svgId);
  const arc  = document.getElementById(arcId);
  const knob = document.getElementById(knobId);
  const lbl  = document.getElementById(valId);
  const START = 135, SWEEP = 270, R = 82;
  let min = initMin, max = initMax, val = initMin, dragging = false;

  const polar = d => { const r = d*Math.PI/180; return [Math.cos(r)*R, Math.sin(r)*R]; };

  const render = () => {
    const pct = (max === min) ? 0 : (val - min) / (max - min);
    const end = START + SWEEP * pct;
    const [sx,sy] = polar(START), [ex,ey] = polar(end);
    arc.setAttribute('d', `M ${sx} ${sy} A ${R} ${R} 0 ${end-START>180?1:0} 1 ${ex} ${ey}`);
    knob.setAttribute('cx', ex); knob.setAttribute('cy', ey);
    lbl.textContent = val;
  };

  const setVal = (v, push) => {
    v = Math.max(min, Math.min(max, Math.round(v)));
    if (v === val && !push) { render(); return; }
    val = v; render();
    if (push) onchange(v);
  };

  const setRange = (lo, hi) => {
    min = lo; max = hi;
    val = Math.max(min, Math.min(max, val));
    render();
  };

  const fromEvent = e => {
    const rect = svg.getBoundingClientRect();
    const t = e.touches ? e.touches[0] : e;
    const cx = rect.left + rect.width/2, cy = rect.top + rect.height/2;
    let a = Math.atan2(t.clientY-cy, t.clientX-cx) * 180/Math.PI;
    let d = a - START; if (d < 0) d += 360;
    if (d > SWEEP) d = (d-SWEEP < 360-d) ? SWEEP : 0;
    setVal(min + (d/SWEEP)*(max-min), true);
  };

  svg.addEventListener('mousedown',  e => { dragging=true; fromEvent(e); e.preventDefault(); });
  svg.addEventListener('touchstart', e => { dragging=true; fromEvent(e); e.preventDefault(); },{passive:false});
  window.addEventListener('mousemove', e => { if(dragging){fromEvent(e);e.preventDefault();} });
  window.addEventListener('touchmove', e => { if(dragging){fromEvent(e);e.preventDefault();} },{passive:false});
  window.addEventListener('mouseup',  () => dragging=false);
  window.addEventListener('touchend', () => dragging=false);

  render();
  return { setVal, getVal: ()=>val, setRange, render };
}

// ── Throttled fetch ───────────────────────────────────────────────────────────
function throttledFetch(baseUrl) {
  let busy=false, next=null;
  return async v => {
    if (busy) { next=v; return; }
    busy=true;
    try { await fetch(baseUrl+v); } catch(e){}
    busy=false;
    if (next!==null) { const n=next; next=null; setTimeout(()=>throttledFetch(baseUrl)(n),0); }
  };
}
const sendThrottle = throttledFetch('/set?v=');
const sendServo    = throttledFetch('/servo/set?p=');

// ── Toast ─────────────────────────────────────────────────────────────────────
function toast(msg) {
  const el = document.getElementById('toast');
  el.textContent = msg; el.classList.add('show');
  setTimeout(() => el.classList.remove('show'), 2000);
}

// ── BLDC ─────────────────────────────────────────────────────────────────────
const bldcUSEl = document.getElementById('bldc-us');
const bldcNumEl = document.getElementById('bldc-num');
const bldcValEl = document.getElementById('bldc-val');

const bldcDial = makeDial('bldc-dial','bldc-arc','bldc-knob','bldc-val', 0, 30, v => {
  bldcValEl.innerHTML = v+'<span style="font-size:22px;color:var(--text2)">%</span>';
  bldcUSEl.textContent = (1000 + v*10) + ' µs';
  if (document.activeElement !== bldcNumEl) bldcNumEl.value = v;
  sendThrottle(v);
});

document.getElementById('bldc-apply').addEventListener('click', () => {
  const v = parseInt(bldcNumEl.value, 10);
  if (isNaN(v)) return;
  bldcDial.setVal(v, true);
});
bldcNumEl.addEventListener('keydown', e => { if(e.key==='Enter') document.getElementById('bldc-apply').click(); });
document.getElementById('bldc-stop').addEventListener('click', () => bldcDial.setVal(0, true));

// ── Servo ─────────────────────────────────────────────────────────────────────
const SWING = 350;
let srvDefault = 1100;
const srvNumEl    = document.getElementById('srv-num');
const srvDefNumEl = document.getElementById('srv-def-num');
const srvDefShow  = document.getElementById('srv-defshow');
const srvLimitsEl = document.getElementById('srv-limits');

const srvDial = makeDial('srv-dial','srv-arc','srv-knob','srv-val',
  Math.max(0, srvDefault-SWING), Math.min(4095, srvDefault+SWING), v => {
  if (document.activeElement !== srvNumEl) srvNumEl.value = v;
  sendServo(v);
});

function updateSrvLimits(def) {
  const lo = Math.max(0, def-SWING), hi = Math.min(4095, def+SWING);
  srvDial.setRange(lo, hi);
  srvLimitsEl.textContent = lo + ' – ' + hi;
  srvNumEl.min = lo; srvNumEl.max = hi;
}

document.getElementById('srv-apply').addEventListener('click', () => {
  const v = parseInt(srvNumEl.value, 10);
  if (isNaN(v)) return;
  srvDial.setVal(v, true);
});
srvNumEl.addEventListener('keydown', e => { if(e.key==='Enter') document.getElementById('srv-apply').click(); });

document.getElementById('srv-savedef').addEventListener('click', async () => {
  const v = parseInt(srvDefNumEl.value, 10);
  if (isNaN(v) || v < 0 || v > 4095) { alert('Enter 0 – 4095'); return; }
  try {
    await fetch('/servo/setdefault?p='+v);
    srvDefault = v;
    srvDefShow.textContent = v;
    updateSrvLimits(v);
    toast('Default saved');
  } catch(e) { alert('Save failed'); }
});

// ── Rolling graph ─────────────────────────────────────────────────────────────
const N = 300;   // samples in buffer

function makeGraph(canvasId, channels, minRange) {
  const canvas = document.getElementById(canvasId);
  const ctx    = canvas.getContext('2d');
  const bufs   = channels.map(() => new Float32Array(N).fill(0));
  let   head   = 0, W = 0, H = 0;
  // Peak-hold per channel: decays slowly so the scale zooms in smoothly.
  const peaks  = channels.map(() => minRange * 0.5);

  function resize() {
    W = canvas.width  = canvas.offsetWidth;
    H = canvas.height = 100;
  }
  resize();
  window.addEventListener('resize', resize);

  function push(values) {
    for (let i = 0; i < channels.length; i++) {
      bufs[i][head] = values[i];
      peaks[i] = Math.max(peaks[i] * 0.997, Math.abs(values[i]));
    }
    head = (head + 1) % N;
  }

  function draw() {
    if (!W || !H) return;
    ctx.clearRect(0, 0, W, H);

    // Auto-scale: use the max peak across all channels, with a floor.
    const dispMax = Math.max(...peaks, minRange);
    const yMin = -dispMax, yMax = dispMax, range = dispMax * 2;

    // Grid lines
    const mid = H / 2;
    ctx.strokeStyle = '#1e2733'; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(0,mid); ctx.lineTo(W,mid); ctx.stroke();
    ctx.setLineDash([3,4]); ctx.strokeStyle = '#161e2a';
    ctx.beginPath(); ctx.moveTo(0,H*0.25); ctx.lineTo(W,H*0.25);
                     ctx.moveTo(0,H*0.75); ctx.lineTo(W,H*0.75); ctx.stroke();
    ctx.setLineDash([]);

    // Scale label top-right
    ctx.fillStyle = '#3a4a5a'; ctx.font = '9px monospace'; ctx.textAlign = 'right';
    ctx.fillText('±' + dispMax.toFixed(2), W - 4, 11);
    ctx.textAlign = 'left';

    for (let c = 0; c < channels.length; c++) {
      const buf = bufs[c];
      ctx.strokeStyle = channels[c].color;
      ctx.lineWidth   = 1.5;
      ctx.beginPath();
      for (let x = 0; x < W; x++) {
        const idx = (head + Math.floor(x * N / W)) % N;
        const y   = H - ((buf[idx] - yMin) / range) * H;
        if (x === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
      }
      ctx.stroke();
    }
  }

  return { push, draw };
}

const gyroGraph = makeGraph('gyro-graph', [
  { color:'#ff9900' }, { color:'#00cccc' },
], 5);     // roll and pitch in degrees, minimum ±5°

// ── MPU poll ──────────────────────────────────────────────────────────────────
const mpuStatus      = document.getElementById('mpu-status');
const mpuCalMsg      = document.getElementById('mpu-cal-msg');
const mpuCalBtn      = document.getElementById('mpu-cal-btn');
const vertBanner     = document.getElementById('vertical-banner');
const mpuIds         = ['m-gx','m-gy','m-gz'];

// Data fetch loop — recursive setTimeout so requests never pile up.
async function fetchMpu() {
  try {
    const r = await fetch('/mpu');
    const j = await r.json();
    if (!j.ok) { mpuStatus.textContent = 'not found'; mpuStatus.style.color='var(--red)'; }
    else if (j.cal) { mpuStatus.textContent = 'calibrating…'; mpuStatus.style.color='var(--blue)'; }
    else {
      mpuStatus.textContent = 'live';
      mpuStatus.style.color = 'var(--green)';
      const vals = [j.gx, j.gy, j.gz];
      vals.forEach((v,i) => document.getElementById(mpuIds[i]).textContent = v.toFixed(2));
      document.getElementById('m-roll').textContent  = j.roll.toFixed(1);
      document.getElementById('m-pitch').textContent = j.pitch.toFixed(1);
      vertBanner.style.display = j.vertical ? 'block' : 'none';
      gyroGraph.push([j.roll, j.pitch]);
    }
  } catch(e) {}
  setTimeout(fetchMpu, 80);   // next fetch after this one finishes — no pileup
}
fetchMpu();

// Render loop — runs at display refresh rate, independent of network.
function renderLoop() {
  gyroGraph.draw();
  requestAnimationFrame(renderLoop);
}
requestAnimationFrame(renderLoop);

// ── Calibrate buttons ─────────────────────────────────────────────────────────
const mpuVibBtn = document.getElementById('mpu-vib-btn');

async function runCal(url, msg, successMsg) {
  mpuCalBtn.disabled = true; mpuVibBtn.disabled = true;
  mpuCalMsg.textContent = msg; mpuCalMsg.style.display = 'block';
  mpuStatus.textContent = 'calibrating…'; mpuStatus.style.color = 'var(--blue)';
  try {
    const r = await fetch(url);
    const j = await r.json();
    if (j.done) { toast(successMsg); }
  } catch(e) { toast('Failed — check connection'); }
  mpuCalMsg.style.display = 'none';
  mpuStatus.textContent = 'live'; mpuStatus.style.color = 'var(--green)';
  mpuCalBtn.disabled = false; mpuVibBtn.disabled = false;
}

mpuCalBtn.addEventListener('click', () =>
  runCal('/mpu/calibrate', 'Bias cal — keep flat & still (~2 s)…', 'Bias calibrated ✓ saved'));

mpuVibBtn.addEventListener('click', () =>
  runCal('/mpu/vibcal', 'Vib cal — measuring noise floor (~2 s)…', 'Vib noise floor saved ✓'));

document.getElementById('reset-angles-btn').addEventListener('click', async () => {
  try { await fetch('/mpu/resetangles'); toast('Angles reset to 0°'); }
  catch(e) {}
});

document.getElementById('set-angles-btn').addEventListener('click', async () => {
  const roll  = parseFloat(document.getElementById('init-roll').value);
  const pitch = parseFloat(document.getElementById('init-pitch').value);
  if (isNaN(roll) || isNaN(pitch)) { toast('Enter both Roll and Pitch'); return; }
  try {
    await fetch('/mpu/setangles?roll=' + roll + '&pitch=' + pitch);
    toast('Angles set: Roll ' + roll + '°  Pitch ' + pitch + '°');
  } catch(e) { toast('Failed'); }
});

// ── Auto-Balance PID ──────────────────────────────────────────────────────────
const balToggle  = document.getElementById('bal-toggle');
const balStatus  = document.getElementById('bal-status');
const balErrEl   = document.getElementById('bal-err');
const balOutEl   = document.getElementById('bal-out');
const balAccumEl = document.getElementById('bal-accum');
const balSrvEl   = document.getElementById('bal-srv');
const pidKpEl    = document.getElementById('pid-kp');
const pidKiEl    = document.getElementById('pid-ki');
const pidKdEl    = document.getElementById('pid-kd');
const pidSpEl    = document.getElementById('pid-sp');
let   balRunning     = false;
let   balGainsLoaded = false;  // gains only synced from server once at startup

function updateBalUI(j) {
  balRunning = j.running;
  if (j.error      !== undefined) balErrEl.textContent   = j.error.toFixed(2);
  if (j.output     !== undefined) balOutEl.textContent   = j.output.toFixed(1);
  if (j.gimbalAccum!== undefined) balAccumEl.textContent = j.gimbalAccum.toFixed(2);
  if (j.servoPos   !== undefined) balSrvEl.textContent   = j.servoPos;
  // Only overwrite the gain inputs once — after that, the user owns those fields.
  if (!balGainsLoaded && j.kp !== undefined) {
    pidKpEl.value = j.kp.toFixed(4);
    pidKiEl.value = j.ki.toFixed(4);
    pidKdEl.value = j.kd.toFixed(4);
    pidSpEl.value = j.setpoint.toFixed(1);
    balGainsLoaded = true;
  }
  if (j.running) {
    balToggle.textContent='STOP'; balToggle.className='danger';
    balStatus.textContent='Running — PID active'; balStatus.style.color='var(--green)';
  } else {
    balToggle.textContent='START'; balToggle.className='safe';
    balStatus.textContent='Stopped — servo under manual control'; balStatus.style.color='var(--muted)';
  }
}

async function fetchBalance() {
  try { const r=await fetch('/balance/state'); updateBalUI(await r.json()); } catch(e) {}
  setTimeout(fetchBalance, 200);
}
fetchBalance();

balToggle.addEventListener('click', async () => {
  const url = balRunning ? '/balance/stop' : '/balance/start';
  try {
    await fetch(url);
    toast(balRunning ? 'Balance stopped' : 'Balance started');
  } catch(e) { toast('Error'); }
});

document.getElementById('pid-apply').addEventListener('click', async () => {
  const kp=parseFloat(pidKpEl.value), ki=parseFloat(pidKiEl.value);
  const kd=parseFloat(pidKdEl.value), sp=parseFloat(pidSpEl.value);
  if ([kp,ki,kd,sp].some(isNaN)) { toast('Enter all values'); return; }
  try {
    const r = await fetch('/balance/pid?kp='+kp+'&ki='+ki+'&kd='+kd+'&sp='+sp);
    if (!r.ok) { toast('Server error ' + r.status); return; }
    const j = await r.json();
    // Confirm back with what the server actually stored
    pidKpEl.value = j.kp.toFixed(4);
    pidKiEl.value = j.ki.toFixed(4);
    pidKdEl.value = j.kd.toFixed(4);
    pidSpEl.value = j.sp.toFixed(1);
    toast('Saved: Kp=' + j.kp.toFixed(4) + ' Ki=' + j.ki.toFixed(4) + ' Kd=' + j.kd.toFixed(4));
  } catch(e) { toast('Failed — is ESP32 connected?'); }
});

// ▼▲ step buttons — each button carries its target input id and step size
document.querySelectorAll('.pid-step').forEach(btn => {
  btn.addEventListener('click', () => {
    const inp  = document.getElementById(btn.dataset.id);
    const step = parseFloat(btn.dataset.step);
    const cur  = parseFloat(inp.value) || 0;
    // Ki needs 3 decimal places, others 2
    const dp = 4;
    inp.value = (cur + step).toFixed(dp);
  });
});

// ── Boot state ────────────────────────────────────────────────────────────────
fetch('/state').then(r=>r.json()).then(j=>{
  if (j.pct !== undefined) {
    bldcDial.setVal(j.pct, false);
    bldcValEl.innerHTML = j.pct+'<span style="font-size:22px;color:var(--text2)">%</span>';
    bldcUSEl.textContent = (1000+j.pct*10)+' µs';
    bldcNumEl.value = j.pct;
  }
  if (j.servoDefault !== undefined) {
    srvDefault = j.servoDefault;
    srvDefShow.textContent = j.servoDefault;
    srvDefNumEl.value = j.servoDefault;
    updateSrvLimits(j.servoDefault);
  }
  if (j.servoPos !== undefined) {
    srvDial.setVal(j.servoPos, false);
    srvNumEl.value = j.servoPos;
  }
}).catch(()=>{});

window.addEventListener('resize', () => gyroGraph.draw());

})();
</script>
</body>
</html>
)HTML";

// ═══════════════════════════════════════════════════════════════════════════
//  HTTP handlers
// ═══════════════════════════════════════════════════════════════════════════

static void handleRoot() { server.send_P(200, "text/html", INDEX_HTML); }

static void handleSet() {
  if (!server.hasArg("v")) { server.send(400,"text/plain","missing v"); return; }
  applyThrottle(server.arg("v").toInt());
  server.send(200, "text/plain", String(throttlePct));
}

static void handleState() {
  String j = "{\"pct\":"          + String(throttlePct)
           + ",\"servoPos\":"     + String(servoTargetPos)
           + ",\"servoDefault\":" + String(servoDefaultPos)
           + ",\"servoLo\":"      + String(servoLo())
           + ",\"servoHi\":"      + String(servoHi())
           + "}";
  server.send(200, "application/json", j);
}

static void handleServoSet() {
  if (!server.hasArg("p")) { server.send(400,"text/plain","missing p"); return; }
  moveServoTo(server.arg("p").toInt());
  server.send(200, "text/plain", String(servoTargetPos));
}

static void handleServoSetDefault() {
  if (!server.hasArg("p")) { server.send(400,"text/plain","missing p"); return; }
  int pos = constrain(server.arg("p").toInt(), 0, 4095);
  servoDefaultPos = pos;
  prefs.begin("wheelo", false);
  prefs.putInt("servo_default", pos);
  prefs.end();
  // Clamp current target to new limits
  if (servoTargetPos < servoLo() || servoTargetPos > servoHi())
    moveServoTo(servoDefaultPos);
  server.send(200, "text/plain", String(pos));
}

static void handleMpu() {
  float dispRoll  = cfRoll  + rollOffset;
  float dispPitch = cfPitch + pitchOffset;
  bool  vert = fabsf(dispRoll) > 70.0f || fabsf(dispPitch) > 70.0f;
  char buf[220];
  snprintf(buf, sizeof(buf),
    "{\"ok\":%s,\"cal\":%s"
    ",\"gx\":%.3f,\"gy\":%.3f,\"gz\":%.3f"
    ",\"roll\":%.2f,\"pitch\":%.2f,\"vertical\":%s}",
    mpuOk ? "true" : "false",
    mpuCalibrating ? "true" : "false",
    mpu.gx, mpu.gy, mpu.gz,
    dispRoll, dispPitch, vert ? "true" : "false");
  server.send(200, "application/json", buf);
}

static void saveAngleOffsets() {
  prefs.begin("wheelo", false);
  prefs.putFloat("roll_off",  rollOffset);
  prefs.putFloat("pitch_off", pitchOffset);
  prefs.end();
}

static void handleMpuResetAngles() {
  // Offset the display so current physical position reads 0°.
  rollOffset  = -cfRoll;
  pitchOffset = -cfPitch;
  saveAngleOffsets();
  server.send(200, "application/json", "{\"done\":true}");
}

static void handleMpuSetAngles() {
  if (!server.hasArg("roll") || !server.hasArg("pitch")) {
    server.send(400, "text/plain", "missing roll or pitch"); return;
  }
  float desiredRoll  = server.arg("roll").toFloat();
  float desiredPitch = server.arg("pitch").toFloat();
  // Shift offset so display = desired without touching the CF.
  rollOffset  = desiredRoll  - cfRoll;
  pitchOffset = desiredPitch - cfPitch;
  saveAngleOffsets();
  float dispRoll  = cfRoll  + rollOffset;
  float dispPitch = cfPitch + pitchOffset;
  char buf[70];
  snprintf(buf, sizeof(buf), "{\"done\":true,\"roll\":%.2f,\"pitch\":%.2f}", dispRoll, dispPitch);
  server.send(200, "application/json", buf);
}

static void handleMpuCalibrate() {
  if (!mpuOk) { server.send(503, "text/plain", "MPU not found"); return; }
  if (mpuCalibrating) { server.send(409, "text/plain", "already calibrating"); return; }
  calibrateMpu(1000);
  char buf[160];
  snprintf(buf, sizeof(buf),
    "{\"done\":true,\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f"
    ",\"gx\":%.3f,\"gy\":%.3f,\"gz\":%.3f}",
    bias.ax, bias.ay, bias.az, bias.gx, bias.gy, bias.gz);
  server.send(200, "application/json", buf);
}

static void handleMpuVibCal() {
  if (!mpuOk) { server.send(503, "text/plain", "MPU not found"); return; }
  if (mpuCalibrating) { server.send(409, "text/plain", "busy"); return; }
  vibCalibrate(500);
  char buf[200];
  snprintf(buf, sizeof(buf),
    "{\"done\":true,\"std\":[%.4f,%.4f,%.4f,%.4f,%.4f,%.4f]}",
    vibStd[0],vibStd[1],vibStd[2],vibStd[3],vibStd[4],vibStd[5]);
  server.send(200, "application/json", buf);
}

static void handleBalanceStart() {
  if (!mpuOk) { server.send(503, "text/plain", "MPU not found"); return; }
  autoBalance = true;
  pidIntegral = 0.0f;
  gimbalAccum = 0.0f;
  pidPrevMeas = cfRoll + rollOffset;
  pidLastUs   = 0;
  server.send(200, "application/json", "{\"running\":true}");
}

static void handleBalanceStop() {
  autoBalance = false;
  pidIntegral = 0.0f;
  gimbalAccum = 0.0f;
  moveServoTo(servoDefaultPos);
  server.send(200, "application/json", "{\"running\":false}");
}

static void handleBalancePid() {
  if (server.hasArg("kp")) pidKp = server.arg("kp").toFloat();
  if (server.hasArg("ki")) pidKi = server.arg("ki").toFloat();
  if (server.hasArg("kd")) pidKd = server.arg("kd").toFloat();
  if (server.hasArg("sp")) pidSetpoint = server.arg("sp").toFloat();
  pidIntegral = 0.0f;   // reset windup when gains change
  prefs.begin("wheelo", false);
  prefs.putFloat("pid_kp", pidKp);
  prefs.putFloat("pid_ki", pidKi);
  prefs.putFloat("pid_kd", pidKd);
  prefs.end();
  char buf[100];
  snprintf(buf, sizeof(buf),
    "{\"done\":true,\"kp\":%.4f,\"ki\":%.4f,\"kd\":%.4f,\"sp\":%.4f}",
    pidKp, pidKi, pidKd, pidSetpoint);
  server.send(200, "application/json", buf);
}

static void handleBalanceState() {
  float dispRoll = cfRoll + rollOffset;
  char buf[220];
  snprintf(buf, sizeof(buf),
    "{\"running\":%s,\"error\":%.2f,\"output\":%.1f,\"gimbalAccum\":%.2f,\"servoPos\":%d"
    ",\"kp\":%.4f,\"ki\":%.4f,\"kd\":%.4f,\"setpoint\":%.4f}",
    autoBalance ? "true" : "false",
    pidSetpoint - dispRoll, pidOutput, gimbalAccum, servoTargetPos,
    pidKp, pidKi, pidKd, pidSetpoint);
  server.send(200, "application/json", buf);
}

// ═══════════════════════════════════════════════════════════════════════════
//  setup / loop
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  delay(200);

  // NVS
  prefs.begin("wheelo", true);
  servoDefaultPos = prefs.getInt("servo_default", SERVO_DEFAULT);
  pidKp           = prefs.getFloat("pid_kp", 0.20f);
  pidKi           = prefs.getFloat("pid_ki", 0.00f);
  pidKd           = prefs.getFloat("pid_kd", 0.04f);
  prefs.end();
  Serial.printf("Servo default from NVS: %d\n", servoDefaultPos);

  // ESC
  ESP32PWM::allocateTimer(0);
  esc.setPeriodHertz(50);
  esc.attach(ESC_PIN, ESC_MIN_US, ESC_MAX_US);
  applyThrottle(0);
  Serial.println("ESC armed at 0%");

  // Servo UART
  SERVO_SERIAL.begin(SERVO_BAUD, SERIAL_8N1, SERVO_RX_PIN, SERVO_TX_PIN);
  delay(50);
  Serial.printf("Moving servo to default %d ...\n", servoDefaultPos);
  servoTargetPos = servoDefaultPos;
  moveServoTo(servoDefaultPos);

  // MPU6050
  mpuInit();

  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("WiFi");
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis()-t0 < 30000) {
    delay(300); Serial.print('.');
    applyThrottle(0);
  }
  if (WiFi.status() == WL_CONNECTED)
    Serial.printf("\nIP: %s\n", WiFi.localIP().toString().c_str());
  else
    Serial.println("\nWiFi failed.");

  server.on("/",                 handleRoot);
  server.on("/set",              handleSet);
  server.on("/state",            handleState);
  server.on("/servo/set",        handleServoSet);
  server.on("/servo/setdefault", handleServoSetDefault);
  server.on("/mpu",              handleMpu);
  server.on("/mpu/calibrate",   handleMpuCalibrate);
  server.on("/mpu/vibcal",      handleMpuVibCal);
  server.on("/mpu/resetangles",  handleMpuResetAngles);
  server.on("/mpu/setangles",    handleMpuSetAngles);
  server.on("/balance/start",    handleBalanceStart);
  server.on("/balance/stop",     handleBalanceStop);
  server.on("/balance/pid",      handleBalancePid);
  server.on("/balance/state",    handleBalanceState);
  server.begin();
  Serial.println("HTTP server started.");
}

void loop() {
  server.handleClient();

  // Drain any servo echo bytes from RX buffer
  while (SERVO_SERIAL.available()) SERVO_SERIAL.read();

  // Read MPU at 500 Hz; run PID immediately after if enabled
  static uint32_t lastMpu = 0;
  if (millis() - lastMpu >= 2) {
    mpuRead();
    if (autoBalance) runPid();
    lastMpu = millis();
  }
}
