"""
Differential-drive odometry from left/right wheel encoder deltas.
No IMU required. Tune TRACK_WIDTH_M, wheel diameter, and ticks/rev in config.
"""

import math

import config


def meters_per_tick():
    """Linear distance one wheel travels per encoder tick (both wheels assumed equal)."""
    circ = math.pi * config.WHEEL_DIAMETER_M
    return circ / max(config.ENCODER_TICKS_PER_REV, 1)


def integrate_differential(d_left_m, d_right_m, x, y, heading_deg):
    """
    Apply left/right arc lengths (meters, forward positive per wheel) over one interval.
    heading_deg: 0=E, 90=N, 180=W, 270=S (same convention as rest of stack).
    """
    L = max(config.TRACK_WIDTH_M, 1e-6)
    th = math.radians(heading_deg)
    d_center = 0.5 * (d_left_m + d_right_m)
    d_theta = (d_right_m - d_left_m) / L
    th_mid = th + 0.5 * d_theta
    x += d_center * math.cos(th_mid)
    y += d_center * math.sin(th_mid)
    heading_deg = (heading_deg + math.degrees(d_theta)) % 360.0
    return x, y, heading_deg


def ticks_delta_to_wheel_meters(dl_ticks, dr_ticks):
    mpt = meters_per_tick()
    d_left_m = dl_ticks * mpt * config.ENCODER_LEFT_SIGN
    d_right_m = dr_ticks * mpt * config.ENCODER_RIGHT_SIGN
    return d_left_m, d_right_m


if __name__ == "__main__":
    print("Odometry calibration (from config.py):")
    print(f"  TRACK_WIDTH_M        = {config.TRACK_WIDTH_M}")
    print(f"  WHEEL_DIAMETER_M     = {config.WHEEL_DIAMETER_M}")
    print(f"  ENCODER_TICKS_PER_REV= {config.ENCODER_TICKS_PER_REV}")
    print(f"  meters_per_tick()    = {meters_per_tick():.6f} m/tick")
    x, y, h = 0.0, 0.0, 0.0
    x, y, h = integrate_differential(0.1, 0.1, x, y, h)
    print(f"  sample straight 0.1 m each wheel: pos=({x:.4f}, {y:.4f}) heading={h:.2f}°")
    x, y, h = integrate_differential(-0.05, 0.05, x, y, h)
    print(f"  sample pivot-ish: pos=({x:.4f}, {y:.4f}) heading={h:.2f}°")
