#!/usr/bin/env python3
"""
Keyboard control for rover drivetrain using L298N backend.

Controls map:
  1 = Forward
  2 = Backward
  3 = Turn Right
  4 = Turn Left
  5 = Stop
  q = Quit
"""

import sys
import termios
import tty

import config
import drive_control


def get_keypress():
    """Read one key without pressing Enter."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def print_controls():
    print("\n" + "=" * 40)
    print("   L298N KEYBOARD MOTOR TEST")
    print("=" * 40)
    print("  1  ->  Forward")
    print("  2  ->  Backward")
    print("  3  ->  Turn Right")
    print("  4  ->  Turn Left")
    print("  5  ->  Stop")
    print("  q  ->  Quit")
    print("=" * 40)
    print("Waiting for input...\n")


def main():
    print_controls()
    try:
        while True:
            key = get_keypress()

            if key == "1":
                drive_control.forward(config.FORWARD_SPEED)
                print(f"FORWARD  speed={config.FORWARD_SPEED}")
            elif key == "2":
                drive_control.backward(config.BACKUP_SPEED)
                print(f"BACKWARD speed={config.BACKUP_SPEED}")
            elif key == "3":
                drive_control.right(config.TURN_SPEED)
                print(f"TURN RIGHT speed={config.TURN_SPEED}")
            elif key == "4":
                drive_control.left(config.TURN_SPEED)
                print(f"TURN LEFT  speed={config.TURN_SPEED}")
            elif key == "5":
                drive_control.stop()
                print("STOP")
            elif key == "q":
                print("\nQuitting...")
                break
            else:
                print(f"Unknown key '{key}' - use 1/2/3/4/5 or q")
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        drive_control.stop()
        print("Motors stopped. Goodbye!")


if __name__ == "__main__":
    main()
