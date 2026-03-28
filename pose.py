"""
Pose: field coordinates (m) + heading (deg). Updates via encoders when ODOMETRY_MODE=='encoder',
else time * METERS_PER_SECOND / DEGREES_PER_SECOND.
"""

import math
import time

import config
import encoder_state
import odometry

robot_x, robot_y = FIELD_START = config.FIELD["start"]
robot_heading = 0.0


def reset_pose(x=None, y=None, heading=None):
    global robot_x, robot_y, robot_heading
    if x is not None:
        robot_x = x
    if y is not None:
        robot_y = y
    if heading is not None:
        robot_heading = heading
    normalize_heading()
    encoder_state.reset_baseline()


def normalize_heading():
    global robot_heading
    robot_heading = robot_heading % 360.0


def print_pose():
    print(f"POSE -> x={robot_x:.2f}, y={robot_y:.2f}, heading={robot_heading:.1f}")


def _read_ticks():
    import drive_control

    return drive_control.motor.read_wheel_ticks()


def update_pose_forward(seconds):
    global robot_x, robot_y
    dist = config.METERS_PER_SECOND * seconds
    angle = math.radians(robot_heading)
    robot_x += dist * math.cos(angle)
    robot_y += dist * math.sin(angle)


def update_pose_backward(seconds):
    global robot_x, robot_y
    dist = config.METERS_PER_SECOND * seconds
    angle = math.radians(robot_heading)
    robot_x -= dist * math.cos(angle)
    robot_y -= dist * math.sin(angle)


def update_pose_left(seconds):
    global robot_heading
    robot_heading += config.DEGREES_PER_SECOND * seconds
    normalize_heading()


def update_pose_right(seconds):
    global robot_heading
    robot_heading -= config.DEGREES_PER_SECOND * seconds
    normalize_heading()


def drive_distance_signed(signed_meters, speed=None):
    """
    Move along current heading: positive = forward, negative = backward.
    Encoder mode: integrates ticks until traveled distance reaches target.
    """
    global robot_x, robot_y, robot_heading

    if abs(signed_meters) < 1e-6:
        return

    import drive_control

    if speed is None:
        speed = config.FORWARD_SPEED if signed_meters > 0 else config.BACKUP_SPEED

    if config.ODOMETRY_MODE != "encoder":
        dist = abs(signed_meters)
        t = dist / max(config.METERS_PER_SECOND, 0.05)
        if signed_meters > 0:
            drive_control.forward(speed)
        else:
            drive_control.backward(speed)
        time.sleep(t)
        drive_control.stop()
        if signed_meters > 0:
            update_pose_forward(t)
        else:
            update_pose_backward(t)
        return

    target = abs(signed_meters)
    traveled = 0.0
    stall_t0 = None

    if signed_meters > 0:
        drive_control.forward(speed)
    else:
        drive_control.backward(speed)

    t0 = time.time()
    max_time = target / max(config.METERS_PER_SECOND, 0.05) * 2.5 + 2.0

    while traveled < target - config.DISTANCE_TOLERANCE_M:
        if time.time() - t0 > max_time:
            rem = max(0.0, target - traveled)
            print("[pose] drive_distance timeout - time fallback for remainder")
            if rem > 0:
                fake_t = rem / max(config.METERS_PER_SECOND, 0.05)
                if signed_meters > 0:
                    update_pose_forward(fake_t)
                else:
                    update_pose_backward(fake_t)
            break

        time.sleep(config.ODOMETRY_POLL_INTERVAL_S)

        dl, dr = encoder_state.consume_tick_delta(_read_ticks)
        if dl == 0 and dr == 0:
            if stall_t0 is None:
                stall_t0 = time.time()
            elif time.time() - stall_t0 > config.ENCODER_STALL_TIME_S:
                dt = config.ODOMETRY_POLL_INTERVAL_S
                traveled += config.METERS_PER_SECOND * dt
                if signed_meters > 0:
                    update_pose_forward(dt)
                else:
                    update_pose_backward(dt)
            continue

        stall_t0 = None
        dlm, drm = odometry.ticks_delta_to_wheel_meters(dl, dr)
        robot_x, robot_y, robot_heading = odometry.integrate_differential(
            dlm, drm, robot_x, robot_y, robot_heading
        )
        step = 0.5 * (dlm + drm)
        if signed_meters > 0 and step > 0:
            traveled += step
        elif signed_meters < 0 and step < 0:
            traveled += -step

    drive_control.stop()
    encoder_state.consume_tick_delta(_read_ticks)


def drive_for(seconds, speed=config.FORWARD_SPEED):
    if seconds <= 0:
        return
    dist = config.METERS_PER_SECOND * seconds
    drive_distance_signed(dist, speed=speed)


def back_for(seconds, speed=config.BACKUP_SPEED):
    if seconds <= 0:
        return
    dist = config.METERS_PER_SECOND * seconds
    drive_distance_signed(-dist, speed=speed)


def left_for(seconds, speed=config.TURN_SPEED):
    if seconds <= 0:
        return
    delta = config.DEGREES_PER_SECOND * seconds
    rotate_degrees_left(delta, speed)


def right_for(seconds, speed=config.TURN_SPEED):
    if seconds <= 0:
        return
    delta = config.DEGREES_PER_SECOND * seconds
    rotate_degrees_right(delta, speed)


def rotate_degrees_left(deg, speed=config.TURN_SPEED):
    """Increase heading (CCW in standard math with 0=E, 90=N)."""
    if deg <= 0:
        return
    target = (robot_heading + deg) % 360.0
    rotate_to_heading(target, speed)


def rotate_degrees_right(deg, speed=config.TURN_SPEED):
    if deg <= 0:
        return
    target = (robot_heading - deg) % 360.0
    rotate_to_heading(target, speed)


def angle_to_target(target_x, target_y):
    dx = target_x - robot_x
    dy = target_y - robot_y
    return math.degrees(math.atan2(dy, dx))


def distance_to_target(target_x, target_y):
    dx = target_x - robot_x
    dy = target_y - robot_y
    return math.sqrt(dx * dx + dy * dy)


def shortest_turn_angle(current_deg, target_deg):
    return (target_deg - current_deg + 180) % 360 - 180


def rotate_to_heading(target_heading, speed=config.TURN_SPEED):
    global robot_x, robot_y, robot_heading

    diff = shortest_turn_angle(robot_heading, target_heading)
    if abs(diff) < config.HEADING_TOLERANCE_DEG:
        return

    print(f"Rotating from {robot_heading:.1f} toward {target_heading:.1f} (d={diff:.1f})")

    import drive_control

    if config.ODOMETRY_MODE != "encoder":
        turn_seconds = abs(diff) / max(config.DEGREES_PER_SECOND, 1e-6)
        if diff > 0:
            drive_control.left(speed)
        else:
            drive_control.right(speed)
        time.sleep(turn_seconds)
        drive_control.stop()
        if diff > 0:
            update_pose_left(turn_seconds)
        else:
            update_pose_right(turn_seconds)
        print_pose()
        return

    stall_t0 = None
    t0 = time.time()
    max_time = abs(diff) / max(config.DEGREES_PER_SECOND, 1e-6) * 2.5 + 3.0

    while abs(shortest_turn_angle(robot_heading, target_heading)) > config.HEADING_TOLERANCE_DEG:
        if time.time() - t0 > max_time:
            print("[pose] rotate timeout - snapping heading (check encoders / signs)")
            robot_heading = target_heading % 360.0
            break

        if shortest_turn_angle(robot_heading, target_heading) > 0:
            drive_control.left(speed)
        else:
            drive_control.right(speed)

        time.sleep(config.ODOMETRY_POLL_INTERVAL_S)

        dl, dr = encoder_state.consume_tick_delta(_read_ticks)
        if dl == 0 and dr == 0:
            if stall_t0 is None:
                stall_t0 = time.time()
            elif time.time() - stall_t0 > config.ENCODER_STALL_TIME_S:
                dt = config.ODOMETRY_POLL_INTERVAL_S
                w = config.DEGREES_PER_SECOND * dt
                if shortest_turn_angle(robot_heading, target_heading) > 0:
                    robot_heading = (robot_heading + w) % 360.0
                else:
                    robot_heading = (robot_heading - w) % 360.0
            continue

        stall_t0 = None
        dlm, drm = odometry.ticks_delta_to_wheel_meters(dl, dr)
        robot_x, robot_y, robot_heading = odometry.integrate_differential(
            dlm, drm, robot_x, robot_y, robot_heading
        )

    drive_control.stop()
    encoder_state.consume_tick_delta(_read_ticks)
    print_pose()


if __name__ == "__main__":
    import argparse

    p = argparse.ArgumentParser(description="Pose / encoder odometry smoke test.")
    p.add_argument(
        "command",
        choices=["status", "drive"],
        default="status",
        nargs="?",
    )
    p.add_argument(
        "--meters",
        type=float,
        default=0.1,
        help="For drive: signed distance (+ = forward).",
    )
    p.add_argument(
        "--yes",
        action="store_true",
        help="Required for drive (robot must be clear to move).",
    )
    args = p.parse_args()

    if args.command == "status":
        print(f"ODOMETRY_MODE = {config.ODOMETRY_MODE!r}")
        print(f"start pose FIELD['start'] = {config.FIELD['start']}")
        print_pose()
        raise SystemExit(0)

    if not args.yes:
        print("Refusing to drive without --yes")
        raise SystemExit(1)
    drive_distance_signed(args.meters)
    print_pose()
