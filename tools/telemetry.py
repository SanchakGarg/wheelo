#!/usr/bin/env python3
"""Stream Wheelo balance telemetry to CSV over WiFi.

Polls the /balance/state HTTP endpoint on the robot and logs each sample with a
wall-clock timestamp. Useful for PID tuning and post-mortem analysis of
oscillation patterns.

Setup (one-time):
    pip install requests

Examples:
    python telemetry.py
    python telemetry.py --host 192.168.1.42       # use IP if wheelo.local fails
    python telemetry.py --rate 50 --duration 20   # 50 Hz for 20 seconds
    python telemetry.py --out kp35_kd010.csv      # custom filename

Notes:
- Default polling rate is 50 Hz. The PID inside the robot runs at 100 Hz, so
  we sample at roughly half-rate. Higher rates work but the ESP32 web server
  starts dropping requests above ~80 Hz.
- mDNS (wheelo.local) needs Bonjour on Windows. If it fails, use --host with
  the actual IP printed on the serial monitor at boot.
- Press Ctrl-C to stop and flush the CSV cleanly.
"""

import argparse
import csv
import signal
import sys
import time
from datetime import datetime

try:
    import requests
except ImportError:
    print("ERROR: requests not installed. Run: pip install requests", file=sys.stderr)
    sys.exit(1)


COLUMNS = [
    "t_ms",         # ms since script start
    "wall_time",    # ISO timestamp (local clock)
    "running",      # PID active?
    "roll_deg",     # derived: setpoint - error when angle is not reported
    "angle_deg",    # filtered complementary angle from MPU path
    "rate_dps",     # filtered gyro rate used by PID derivative
    "accel_angle_deg",
    "accel_norm_g",
    "dropped_reads",
    "error_deg",    # PID error term
    "output_deg",   # PID output (servo deflection in degrees)
    "sp_accum",     # SetpointAccum drift
    "servo_pos",    # raw servo position (steps)
    "kp", "ki", "kd",
    "setpoint_deg",
]


def main():
    ap = argparse.ArgumentParser(description="Wheelo balance telemetry logger")
    ap.add_argument("--host", default="wheelo.local",
                    help="Robot hostname or IP (default: wheelo.local)")
    ap.add_argument("--rate", type=float, default=50.0,
                    help="Target sample rate in Hz (default: 50)")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="Stop after N seconds (0 = run until Ctrl-C)")
    ap.add_argument("--out", default=None,
                    help="CSV output file (default: wheelo_YYYYMMDD_HHMMSS.csv)")
    args = ap.parse_args()

    if args.out is None:
        args.out = f"wheelo_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

    url = f"http://{args.host}/balance/state"
    interval = 1.0 / args.rate

    sess = requests.Session()
    try:
        r = sess.get(url, timeout=2.0)
        r.raise_for_status()
        r.json()
    except Exception as e:
        print(f"ERROR: cannot reach {url}: {e}", file=sys.stderr)
        print("If wheelo.local fails, pass --host <ip> with the IP from the serial monitor.",
              file=sys.stderr)
        sys.exit(1)

    print(f"Connected to {url}")
    print(f"Logging at {args.rate:g} Hz -> {args.out}")
    print("Press Ctrl-C to stop.\n")

    stop = {"flag": False}
    def handler(*_): stop["flag"] = True
    signal.signal(signal.SIGINT, handler)

    f = open(args.out, "w", newline="")
    writer = csv.DictWriter(f, fieldnames=COLUMNS)
    writer.writeheader()

    t0 = time.perf_counter()
    samples = 0
    errors = 0
    last_status = t0
    next_t = t0

    try:
        while not stop["flag"]:
            now = time.perf_counter()
            if now < next_t:
                time.sleep(max(0.0, next_t - now))
            next_t += interval

            t_req = time.perf_counter()
            try:
                resp = sess.get(url, timeout=0.5)
                if resp.status_code != 200:
                    errors += 1
                    continue
                j = resp.json()
            except Exception:
                errors += 1
                continue

            elapsed_ms = (t_req - t0) * 1000.0
            sp = float(j.get("setpoint", 0.0))
            err = float(j.get("error", 0.0))
            roll = sp - err
            angle = float(j.get("angle", roll))

            writer.writerow({
                "t_ms":        f"{elapsed_ms:.2f}",
                "wall_time":   datetime.now().isoformat(timespec="milliseconds"),
                "running":     j.get("running"),
                "roll_deg":    f"{roll:.4f}",
                "angle_deg":   f"{angle:.4f}",
                "rate_dps":    f"{float(j.get('rate', 0.0)):.4f}",
                "accel_angle_deg": f"{float(j.get('accelAngle', 0.0)):.4f}",
                "accel_norm_g": f"{float(j.get('accelNorm', 0.0)):.4f}",
                "dropped_reads": j.get("dropped", 0),
                "error_deg":   f"{err:.4f}",
                "output_deg":  f"{float(j.get('output', 0.0)):.4f}",
                "sp_accum":    f"{float(j.get('spAccum', 0.0)):.4f}",
                "servo_pos":   j.get("servoPos"),
                "kp":          j.get("kp"),
                "ki":          j.get("ki"),
                "kd":          j.get("kd"),
                "setpoint_deg": sp,
            })
            samples += 1

            if now - last_status >= 1.0:
                eff = samples / (now - t0)
                sys.stdout.write(
                    f"\r  {samples:6d} samples | {eff:6.2f} Hz effective | "
                    f"errors={errors:4d} | t={now-t0:6.1f}s"
                )
                sys.stdout.flush()
                last_status = now

            if args.duration > 0 and (now - t0) >= args.duration:
                break
    finally:
        f.close()
        total = time.perf_counter() - t0
        eff = samples / total if total > 0 else 0.0
        print(f"\n\nDone. {samples} samples over {total:.1f}s "
              f"(effective {eff:.1f} Hz, {errors} request errors).")
        print(f"Saved: {args.out}")


if __name__ == "__main__":
    main()
