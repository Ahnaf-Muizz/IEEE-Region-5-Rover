"""
High-level drive commands. Maps semantic forward/back to your motor library naming.
"""

import time

import motor_control

import config

motor = motor_control.MotorController()


def safe_motor_call(label, fn, *args, **kwargs):
    try:
        return fn(*args, **kwargs)
    except OSError as e:
        print(f"[MOTOR ERROR] {label}: {e}")
        return False
    except Exception as e:
        print(f"[MOTOR ERROR] {label}: {e}")
        return False


def forward(speed=config.FORWARD_SPEED):
    safe_motor_call("forward", motor.forward, speed)


def backward(speed=config.BACKUP_SPEED):
    safe_motor_call("reverse", motor.reverse, speed)


def left(speed=config.TURN_SPEED):
    safe_motor_call("turn_left", motor.turn_left, speed)


def right(speed=config.TURN_SPEED):
    safe_motor_call("turn_right", motor.turn_right, speed)


def stop():
    safe_motor_call("stop", motor.stop)


def drive_for(seconds, speed=config.FORWARD_SPEED):
    if seconds <= 0:
        return
    forward(speed)
    time.sleep(seconds)
    stop()


def back_for(seconds, speed=config.BACKUP_SPEED):
    if seconds <= 0:
        return
    backward(speed)
    time.sleep(seconds)
    stop()


def left_for(seconds, speed=config.TURN_SPEED):
    if seconds <= 0:
        return
    left(speed)
    time.sleep(seconds)
    stop()


def right_for(seconds, speed=config.TURN_SPEED):
    if seconds <= 0:
        return
    right(speed)
    time.sleep(seconds)
    stop()


if __name__ == "__main__":
    import argparse

    p = argparse.ArgumentParser(description="Motor smoke test (time-based, no pose update).")
    p.add_argument(
        "action",
        choices=["info", "forward", "backward", "left", "right", "stop"],
        nargs="?",
        default="info",
        help="info = print only; others move the robot briefly.",
    )
    p.add_argument(
        "--seconds",
        type=float,
        default=0.25,
        help="Duration for motion actions (default 0.25).",
    )
    p.add_argument(
        "--yes",
        action="store_true",
        help="Required for any motion (safety).",
    )
    p.add_argument("--speed", type=int, default=None, help="PWM/speed argument to motor driver.")
    args = p.parse_args()

    if args.action == "info":
        print("MotorController methods: forward, reverse, turn_left, turn_right, stop, read_wheel_ticks")
        print("Use: python drive_control.py forward --yes")
        raise SystemExit(0)

    if not args.yes:
        print("Refusing to move without --yes (robot must be safe to move).")
        raise SystemExit(1)

    spd = args.speed if args.speed is not None else config.FORWARD_SPEED
    t = max(0.01, args.seconds)
    if args.action == "stop":
        stop()
    elif args.action == "forward":
        drive_for(t, spd)
    elif args.action == "backward":
        back_for(t, args.speed if args.speed is not None else config.BACKUP_SPEED)
    elif args.action == "left":
        left_for(t, args.speed if args.speed is not None else config.TURN_SPEED)
    elif args.action == "right":
        right_for(t, args.speed if args.speed is not None else config.TURN_SPEED)
    print("done.")
