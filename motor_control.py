#!/usr/bin/env python3
"""
Motor control for a tank chassis over I2C.

This module is compatible with the rover stack's expected interface:
forward, reverse, turn_left, turn_right, stop, read_wheel_ticks.
"""

import atexit

try:
    from smbus2 import SMBus

    I2C_AVAILABLE = True
except ImportError:
    print("Warning: smbus2 not installed!")
    print("Install with: pip3 install smbus2 --break-system-packages")
    I2C_AVAILABLE = False
    SMBus = None


class MotorController:
    """Controls tracked chassis with I2C motor driver."""

    STATE_IDLE = "idle"
    STATE_FORWARD = "forward"
    STATE_REVERSE = "reverse"
    STATE_TURN_RIGHT = "turn_right"
    STATE_TURN_LEFT = "turn_left"
    STATE_STOP = "stop"

    SPEED_FULL = 100
    SPEED_HIGH = 75
    SPEED_MEDIUM = 50
    SPEED_LOW = 25

    # Motor registers (hardware-specific)
    LEFT_MOTOR = 51
    RIGHT_MOTOR = 52

    def __init__(self, i2c_address=0x34, i2c_bus=1):
        self.i2c_address = i2c_address
        self.i2c_bus_number = i2c_bus
        self.current_state = self.STATE_STOP
        self.current_speed = self.SPEED_FULL
        self.bus = None
        self._warned_no_encoder = False

        if I2C_AVAILABLE:
            try:
                self.bus = SMBus(i2c_bus)
                if self._test_connection():
                    print(f"Motor driver found at address 0x{i2c_address:02X}")
                else:
                    print(f"Device at 0x{i2c_address:02X} not responding")
                    print("Check wiring, power supply, and I2C address.")
            except Exception as e:
                print(f"Failed to initialize I2C: {e}")
                self.bus = None
        else:
            print("smbus2 not available - simulation mode")

        atexit.register(self.cleanup)

    def _test_connection(self):
        if self.bus is None:
            return False
        try:
            self.bus.read_byte(self.i2c_address)
            return True
        except Exception:
            return False

    def _convert_speed(self, speed_percent, forward=True):
        """
        Convert speed percentage to motor driver byte.

        Protocol:
        - 0 = stop
        - 1-127 = backward (1 fastest, 127 slowest)
        - 128-255 = forward (128 slowest, 255 fastest)
        """
        speed_percent = max(0, min(100, int(speed_percent)))
        if speed_percent == 0:
            return 0

        if forward:
            value = 128 + int((speed_percent / 100.0) * 127)
            return min(255, value)

        value = 127 - int((speed_percent / 100.0) * 126)
        return max(1, value)

    def _write_motor(self, motor_register, speed_percent, forward=True):
        motor_value = self._convert_speed(speed_percent, forward)

        if self.bus is None:
            direction = "FORWARD" if forward else "REVERSE"
            motor_name = "LEFT" if motor_register == self.LEFT_MOTOR else "RIGHT"
            print(
                f"[SIM] {motor_name} motor: {direction} at {speed_percent}% "
                f"(value={motor_value})"
            )
            return

        try:
            self.bus.write_byte_data(self.i2c_address, motor_register, motor_value)
        except Exception as e:
            print(f"Error writing to motor: {e}")

    def stop(self):
        if self.bus is not None:
            try:
                self.bus.write_byte_data(self.i2c_address, self.LEFT_MOTOR, 0)
                self.bus.write_byte_data(self.i2c_address, self.RIGHT_MOTOR, 0)
            except Exception as e:
                print(f"Error stopping motors: {e}")
        else:
            print("[SIM] all motors stopped")
        self.current_state = self.STATE_STOP

    def idle(self):
        self.stop()
        self.current_state = self.STATE_IDLE

    def forward(self, speed_percent=None):
        speed = speed_percent if speed_percent is not None else self.current_speed
        speed = max(0, min(100, int(speed)))
        self._write_motor(self.LEFT_MOTOR, speed, forward=True)
        self._write_motor(self.RIGHT_MOTOR, speed, forward=True)
        self.current_state = self.STATE_FORWARD

    def reverse(self, speed_percent=None):
        speed = speed_percent if speed_percent is not None else self.current_speed
        speed = max(0, min(100, int(speed)))
        self._write_motor(self.LEFT_MOTOR, speed, forward=False)
        self._write_motor(self.RIGHT_MOTOR, speed, forward=False)
        self.current_state = self.STATE_REVERSE

    def turn_right(self, speed_percent=None):
        speed = speed_percent if speed_percent is not None else self.current_speed
        speed = max(0, min(100, int(speed)))
        self._write_motor(self.LEFT_MOTOR, speed, forward=True)
        self._write_motor(self.RIGHT_MOTOR, speed, forward=False)
        self.current_state = self.STATE_TURN_RIGHT

    def turn_left(self, speed_percent=None):
        speed = speed_percent if speed_percent is not None else self.current_speed
        speed = max(0, min(100, int(speed)))
        self._write_motor(self.LEFT_MOTOR, speed, forward=False)
        self._write_motor(self.RIGHT_MOTOR, speed, forward=True)
        self.current_state = self.STATE_TURN_LEFT

    def slow_down(self, target_speed_percent):
        target_speed_percent = max(0, min(100, int(target_speed_percent)))
        self.set_speed(target_speed_percent)

        if self.current_state == self.STATE_FORWARD:
            self.forward()
        elif self.current_state == self.STATE_REVERSE:
            self.reverse()
        elif self.current_state == self.STATE_TURN_RIGHT:
            self.turn_right()
        elif self.current_state == self.STATE_TURN_LEFT:
            self.turn_left()

    def set_speed(self, speed_percent):
        self.current_speed = max(0, min(100, int(speed_percent)))

    def get_state(self):
        return self.current_state

    def get_speed(self):
        return self.current_speed

    def read_wheel_ticks(self):
        """
        Return cumulative (left_ticks, right_ticks).

        This specific I2C interface does not expose encoder counters in the
        provided protocol, so we return zero deltas. The rover stack handles
        this by falling back to time-based odometry when ticks stall.
        """
        if not self._warned_no_encoder:
            print(
                "read_wheel_ticks(): encoder registers not implemented for this driver; "
                "returning (0, 0)."
            )
            self._warned_no_encoder = True
        return (0, 0)

    def cleanup(self):
        try:
            self.stop()
        except Exception:
            pass
        if self.bus is not None:
            try:
                self.bus.close()
            except Exception:
                pass
            self.bus = None


if __name__ == "__main__":
    m = MotorController()
    print("read_wheel_ticks() =", m.read_wheel_ticks())
    print("Motor controller ready.")
