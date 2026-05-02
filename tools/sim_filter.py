#!/usr/bin/env python3
"""
Scissored CMG gyro filter simulation.

Compares:
  A) Current complementary filter (CF) with hard innovation deadband + bias tracker
  B) Mahony complementary filter (continuous PI correction, no deadband)

Scenario: robot sits at 0 deg pitch with flywheel spinning.
  - Gyro: perfect calibration at t=0, thermal bias drifts at DRIFT_RATE dps/s
  - Accel: 0 deg true pitch + BLDC vibration + mechanical resonance

Expected result:
  - CF drifts past the deadband then corrects noisily (the observed 8 deg drift)
  - Mahony holds near 0 deg throughout, limited noise from accel vibration
"""

import math
import numpy as np

# ── Simulation parameters ───────────────────────────────────────────────────
DT   = 0.01   # 100 Hz sample rate
T    = 120.0  # total sim time (seconds)
N    = int(T / DT)
SEED = 42

# ── Filter constants (match C++) ─────────────────────────────────────────────
GYRO_LP_HZ   = 8.0
ACCEL_LP_HZ  = 4.0
MEDIAN_N     = 5

# Current CF
CF_ACCEL_TRUST_STILL  = 0.35   # Hz
CF_ACCEL_TRUST_MOVING = 0.02   # Hz
CF_INNOV_DB           = 1.5    # deg — default before noise cal
CF_BIAS_ALPHA         = 0.0008
CF_STILL_GNORM        = 0.04   # g tolerance for bias tracking
CF_DIVERGE_G          = 0.12

# Mahony replacement
MAHONY_KP     = 2.0    # deg/s per deg error — 0.32 Hz crossover
# Ki > drift_rate / acceptable_error: 0.12 dps/s / 0.5 deg = 0.24 minimum.
MAHONY_KI     = 0.30   # integral gain (steady-state error = 0.12/0.30 = 0.4 deg)
MAX_MAHONY_INT= 20.0   # dps — clamp on integral

# ── Noise / drift model ──────────────────────────────────────────────────────
GYRO_WHITE_STD   = 0.25   # dps RMS (after hardware DLPF + median — residual)
DRIFT_RATE       = 0.12   # dps/s thermal drift (≈8 deg in ~70 s)

# Accel noise model — values are BEFORE software LP.
# After the 4 Hz LP these attenuate significantly.
BLDC_FREQ        = 60.0   # Hz commutation
BLDC_AMP_AY      = 0.28   # g peak on ay
BLDC_AMP_AZ      = 0.14   # g peak on az
RESONANCE_FREQ   = 2.0    # Hz chassis resonance
RESONANCE_AMP    = 0.06   # g peak (low-freq, harder to reject)
ACCEL_WHITE_STD  = 0.008  # g RMS per axis

# ── Generate raw sensor streams ──────────────────────────────────────────────
rng = np.random.default_rng(SEED)
t   = np.arange(N) * DT

# Gyro: thermal bias grows from 0 (perfect calibration at t=0)
gyro_bias = np.cumsum(np.full(N, DRIFT_RATE * DT))  # ramp
gyro_raw  = gyro_bias + rng.normal(0, GYRO_WHITE_STD, N)

# Accel: true pitch 0 deg → ay_true=0, az_true=1
# BLDC vibration is periodic EMF coupling; resonance is mechanical
ay_vib = (BLDC_AMP_AY * np.sin(2*np.pi*BLDC_FREQ * t)
          + RESONANCE_AMP * np.sin(2*np.pi*RESONANCE_FREQ * t))
az_vib = BLDC_AMP_AZ * np.sin(2*np.pi*BLDC_FREQ * t + 1.1)
ax_raw  = rng.normal(0, ACCEL_WHITE_STD, N)
ay_raw  = ay_vib + rng.normal(0, ACCEL_WHITE_STD, N)
az_raw  = 1.0 + az_vib + rng.normal(0, ACCEL_WHITE_STD, N)

# ── LP helper ────────────────────────────────────────────────────────────────
def lp(prev, x, hz):
    rc    = 1.0 / (2 * math.pi * hz)
    alpha = DT / (rc + DT)
    return prev + alpha * (x - prev)

def median5(buf):
    return float(np.median(buf))

# ════════════════════════════════════════════════════════════════════════════
# Filter A: Current CF (innovation deadband + bias tracker)
# ════════════════════════════════════════════════════════════════════════════
def run_current_cf():
    eGx = 0.0; eAx = 0.0; eAy = 0.0; eAz = 1.0
    cfAngle = None
    biasGx  = 0.0
    aNFilt  = 1.0
    mg = [0.0]*MEDIAN_N; ma = [0.0]*MEDIAN_N
    my = [0.0]*MEDIAN_N; mz = [0.0]*MEDIAN_N
    mi = 0; mc = 0
    angles = np.zeros(N)

    for i in range(N):
        rgx = gyro_raw[i] - biasGx
        rax = ax_raw[i]; ray = ay_raw[i]; raz = az_raw[i]

        mg[mi]=rgx; ma[mi]=rax; my[mi]=ray; mz[mi]=raz
        mi = (mi+1) % MEDIAN_N
        if mc < MEDIAN_N: mc += 1
        if mc == MEDIAN_N:
            rgx = median5(mg); rax = median5(ma)
            ray = median5(my); raz = median5(mz)

        aNorm  = math.sqrt(rax**2 + ray**2 + raz**2)
        aNFilt = lp(aNFilt, aNorm, 1.5)

        if cfAngle is None:
            eAy = ray; eAz = raz; eGx = rgx
            cfAngle = math.atan2(eAy, eAz) * 57.2957795
        else:
            eAx = lp(eAx, rax, ACCEL_LP_HZ)
            eAy = lp(eAy, ray, ACCEL_LP_HZ)
            eAz = lp(eAz, raz, ACCEL_LP_HZ)
            eGx = lp(eGx, rgx, GYRO_LP_HZ)

        accelAngle = math.atan2(eAy, eAz) * 57.2957795

        # Bias tracking: gated on accel norm only
        if abs(aNFilt - 1.0) < CF_STILL_GNORM:
            biasGx += CF_BIAS_ALPHA * eGx

        # Accel trust (complementary filter alpha)
        if abs(aNFilt - 1.0) > CF_DIVERGE_G:
            hz = CF_ACCEL_TRUST_MOVING
        else:
            hz = CF_ACCEL_TRUST_STILL
        trust = DT / (1.0/(2*math.pi*hz) + DT)

        predicted  = cfAngle + eGx * DT
        innovation = accelAngle - predicted

        if abs(innovation) < CF_INNOV_DB:
            cfAngle = predicted
        else:
            cfAngle = predicted + trust * innovation

        angles[i] = cfAngle

    return angles

# ════════════════════════════════════════════════════════════════════════════
# Filter B: Mahony complementary filter (no hard deadband)
# ════════════════════════════════════════════════════════════════════════════
def run_mahony():
    eGx = 0.0; eAx = 0.0; eAy = 0.0; eAz = 1.0
    mahonyAngle = None
    mahonyInt   = 0.0
    aNFilt      = 1.0
    mg = [0.0]*MEDIAN_N; ma = [0.0]*MEDIAN_N
    my = [0.0]*MEDIAN_N; mz = [0.0]*MEDIAN_N
    mi = 0; mc = 0
    angles = np.zeros(N)

    for i in range(N):
        rgx = gyro_raw[i]   # no pre-subtracted bias — Mahony corrects via integral
        rax = ax_raw[i]; ray = ay_raw[i]; raz = az_raw[i]

        mg[mi]=rgx; ma[mi]=rax; my[mi]=ray; mz[mi]=raz
        mi = (mi+1) % MEDIAN_N
        if mc < MEDIAN_N: mc += 1
        if mc == MEDIAN_N:
            rgx = median5(mg); rax = median5(ma)
            ray = median5(my); raz = median5(mz)

        aNorm  = math.sqrt(rax**2 + ray**2 + raz**2)
        aNFilt = lp(aNFilt, aNorm, 1.5)

        eAx = lp(eAx, rax, ACCEL_LP_HZ)
        eAy = lp(eAy, ray, ACCEL_LP_HZ)
        eAz = lp(eAz, raz, ACCEL_LP_HZ)
        eGx = lp(eGx, rgx, GYRO_LP_HZ)

        if mahonyAngle is None:
            mahonyAngle = math.atan2(eAy, eAz) * 57.2957795
            angles[i] = mahonyAngle
            continue

        # Accel trust scaling
        kp = MAHONY_KP
        ki = MAHONY_KI
        norm3 = math.sqrt(eAx**2 + eAy**2 + eAz**2)
        if norm3 < 0.3:
            kp = 0.0; ki = 0.0   # wildly invalid accel
        elif abs(aNFilt - 1.0) > CF_DIVERGE_G:
            kp *= 0.15; ki *= 0.15  # actual acceleration event

        # Normalize LP-filtered accel
        if norm3 > 0.1:
            ay_n = eAy / norm3
            az_n = eAz / norm3
        else:
            ay_n = 0.0; az_n = 1.0

        # Cross-product innovation (pitch only): e ≈ accel_angle - estimated_angle
        th    = mahonyAngle * 0.017453292  # deg → rad
        e_deg = (ay_n * math.cos(th) - az_n * math.sin(th)) * 57.2957795

        # Mahony integral (online gyro bias estimate in dps)
        mahonyInt += ki * e_deg * DT
        mahonyInt  = max(-MAX_MAHONY_INT, min(MAX_MAHONY_INT, mahonyInt))

        # Integrate: raw gyro + proportional correction + integral bias correction
        omega       = eGx + kp * e_deg + mahonyInt
        mahonyAngle += omega * DT

        angles[i] = mahonyAngle

    return angles

# ── Run ──────────────────────────────────────────────────────────────────────
print("=" * 64)
print("  Scissored CMG gyro filter simulation")
print("=" * 64)
print(f"Duration : {T:.0f} s  |  Sample rate: {1/DT:.0f} Hz  |  N={N}")
print(f"Drift    : {DRIFT_RATE} dps/s -> final bias {DRIFT_RATE*T:.1f} dps after {T:.0f} s")
print(f"BLDC vib : ±{BLDC_AMP_AY:.2f}g @{BLDC_FREQ:.0f} Hz, ±{RESONANCE_AMP:.2f}g @{RESONANCE_FREQ:.0f} Hz")
print(f"Gyro noise: {GYRO_WHITE_STD} dps RMS (post-DLPF residual)")
print()

cf_angles  = run_current_cf()
mah_angles = run_mahony()

SS = int(5.0 / DT)   # skip first 5 s warmup

def report(label, arr):
    ss   = arr[SS:]
    rmse = math.sqrt(float(np.mean(ss**2)))
    print(f"  {label}")
    print(f"    Mean offset : {ss.mean():+.3f} deg  (bias from true 0 deg)")
    print(f"    Std dev     : {ss.std():.3f} deg  (noise/jitter)")
    print(f"    RMSE        : {rmse:.3f} deg")
    print(f"    3-sigma band: ±{3*ss.std():.3f} deg")
    print(f"    Max/min     : {ss.max():+.2f} / {ss.min():+.2f} deg")

print("=== Steady-state performance (after 5 s warmup) ===")
report("Current CF (deadband=1.5 deg, slow bias track)", cf_angles)
print()
report(f"Mahony (Kp={MAHONY_KP}, Ki={MAHONY_KI})", mah_angles)

print()
print("=== Drift over time (true angle = 0 deg throughout) ===")
print(f"  {'t':>4}   {'gyro_bias':>10}   {'CF angle':>10}   {'Mahony angle':>12}")
for ts in [5, 10, 20, 30, 60, 90, 120]:
    idx  = min(int(ts/DT)-1, N-1)
    bias = gyro_bias[idx]
    print(f"  {ts:4d}s   {bias:10.2f}   {cf_angles[idx]:10.2f}   {mah_angles[idx]:12.2f}")

print()
print("=== Mahony integral (bias estimate) over time ===")
print("  (Should converge toward –gyro_bias to cancel drift)")

# Re-run Mahony and capture integral history
def run_mahony_with_int():
    eGx = 0.0; eAx = 0.0; eAy = 0.0; eAz = 1.0
    mahonyAngle = None; mahonyInt = 0.0; aNFilt = 1.0
    mg = [0.0]*MEDIAN_N; ma = [0.0]*MEDIAN_N
    my = [0.0]*MEDIAN_N; mz = [0.0]*MEDIAN_N
    mi = 0; mc = 0
    ints = np.zeros(N)
    for i in range(N):
        rgx = gyro_raw[i]
        rax = ax_raw[i]; ray = ay_raw[i]; raz = az_raw[i]
        mg[mi]=rgx; ma[mi]=rax; my[mi]=ray; mz[mi]=raz
        mi = (mi+1) % MEDIAN_N
        if mc < MEDIAN_N: mc += 1
        if mc == MEDIAN_N:
            rgx = median5(mg); rax = median5(ma)
            ray = median5(my); raz = median5(mz)
        aNorm  = math.sqrt(rax**2 + ray**2 + raz**2)
        aNFilt = lp(aNFilt, aNorm, 1.5)
        eAx = lp(eAx, rax, ACCEL_LP_HZ); eAy = lp(eAy, ray, ACCEL_LP_HZ)
        eAz = lp(eAz, raz, ACCEL_LP_HZ); eGx = lp(eGx, rgx, GYRO_LP_HZ)
        if mahonyAngle is None:
            mahonyAngle = math.atan2(eAy, eAz) * 57.2957795
            ints[i] = 0.0; continue
        kp = MAHONY_KP; ki = MAHONY_KI
        norm3 = math.sqrt(eAx**2 + eAy**2 + eAz**2)
        if norm3 < 0.3: kp=0.0; ki=0.0
        elif abs(aNFilt - 1.0) > CF_DIVERGE_G: kp*=0.15; ki*=0.15
        if norm3 > 0.1: ay_n=eAy/norm3; az_n=eAz/norm3
        else: ay_n=0.0; az_n=1.0
        th = mahonyAngle * 0.017453292
        e_deg = (ay_n * math.cos(th) - az_n * math.sin(th)) * 57.2957795
        mahonyInt += ki * e_deg * DT
        mahonyInt = max(-MAX_MAHONY_INT, min(MAX_MAHONY_INT, mahonyInt))
        mahonyAngle += (eGx + kp * e_deg + mahonyInt) * DT
        ints[i] = mahonyInt
    return ints

mah_ints = run_mahony_with_int()
print(f"  {'t':>4}   {'true_bias':>10}   {'mahony_int':>12}   {'tracking %':>10}")
for ts in [10, 30, 60, 90, 120]:
    idx  = min(int(ts/DT)-1, N-1)
    bias = gyro_bias[idx]
    mi_  = mah_ints[idx]
    pct  = -mi_ / bias * 100 if abs(bias) > 0.01 else 0.0
    print(f"  {ts:4d}s   {bias:10.2f}   {mi_:12.4f}   {pct:9.1f}%")

print()
print("=== Summary ===")
cf_ss  = cf_angles[SS:]
mah_ss = mah_angles[SS:]
print(f"  Current CF:  RMSE={math.sqrt(float(np.mean(cf_ss**2))):.2f} deg,  "
      f"max drift={abs(cf_ss).max():.2f} deg")
print(f"  Mahony:      RMSE={math.sqrt(float(np.mean(mah_ss**2))):.2f} deg,  "
      f"max drift={abs(mah_ss).max():.2f} deg")
improvement = math.sqrt(float(np.mean(cf_ss**2))) / max(math.sqrt(float(np.mean(mah_ss**2))), 0.001)
print(f"  Improvement: {improvement:.1f}x RMSE reduction")
print()

try:
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    t_arr = np.arange(N) * DT

    axes[0].plot(t_arr, cf_angles,  label='Current CF', alpha=0.8)
    axes[0].plot(t_arr, mah_angles, label='Mahony',     alpha=0.8)
    axes[0].axhline(0, color='k', linestyle='--', linewidth=0.8, label='True (0°)')
    axes[0].set_ylabel('Estimated angle (deg)')
    axes[0].set_title('Filter comparison — scissored CMG gyro simulation')
    axes[0].legend(); axes[0].grid(True)

    axes[1].plot(t_arr, gyro_bias, label='True thermal bias (dps)', color='red')
    axes[1].plot(t_arr, -mah_ints, label='–Mahony integral (bias estimate)', color='green', alpha=0.8)
    axes[1].set_ylabel('Gyro bias (dps)')
    axes[1].legend(); axes[1].grid(True)

    axes[2].plot(t_arr, np.degrees(np.arctan2(ay_raw, az_raw)), label='Raw accel angle', alpha=0.4)
    axes[2].plot(t_arr, cf_angles - mah_angles, label='CF – Mahony (tracking error)', alpha=0.8)
    axes[2].set_ylabel('Degrees')
    axes[2].set_xlabel('Time (s)')
    axes[2].legend(); axes[2].grid(True)

    plt.tight_layout()
    plt.savefig('sim_filter_results.png', dpi=120)
    print("Plot saved: sim_filter_results.png")
    plt.show()
except ImportError:
    print("(matplotlib not available — install it to see the plot)")
