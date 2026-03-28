#!/usr/bin/env python3
"""
Motor control for a tank chassis over I2C.

Integrated from the working keyboard control behavior:
- left motor register: 51
- right motor register: 52
- stop raw value: 0
- forward raw value: 50
- reverse raw value: 200

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
    RAW_STOP = 0
    RAW_FORWARD = 50
    RAW_REVERSE = 200

    def __init__(
        self,
        i2c_address=0x34,
        i2c_bus=1,
        raw_forward=RAW_FORWARD,
        raw_reverse=RAW_REVERSE,
        raw_stop=RAW_STOP,
        fixed_raw_speed=True,
    ):
        self.i2c_address = i2c_address
        self.i2c_bus_number = i2c_bus
        self.current_state = self.STATE_STOP
        self.current_speed = self.SPEED_FULL
        self.bus = None
        self._warned_no_encoder = False
        self.raw_forward = max(0, min(255, int(raw_forward)))
        self.raw_reverse = max(0, min(255, int(raw_reverse)))
        self.raw_stop = max(0, min(255, int(raw_stop)))
        # Default True to match the known-good keyboard control exactly.
        self.fixed_raw_speed = bool(fixed_raw_speed)

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

    def _speed_to_raw(self, target_raw, speed_percent):
        """
        Translate rover speed percent to a raw byte.

        fixed_raw_speed=True preserves keyboard_control behavior exactly:
        any non-zero speed uses the known-good raw direction byte.
        """
        speed_percent = max(0, min(100, int(speed_percent)))
        if speed_percent == 0:
            return self.raw_stop
        if self.fixed_raw_speed:
            return target_raw
        return int(round(self.raw_stop + (target_raw - self.raw_stop) * (speed_percent / 100.0)))

    def _write_motor_raw(self, motor_register, raw_value):
        motor_value = max(0, min(255, int(raw_value)))

        if self.bus is None:
            motor_name = "LEFT" if motor_register == self.LEFT_MOTOR else "RIGHT"
            print(f"[SIM] {motor_name} motor raw={motor_value}")
            return

        try:
            self.bus.write_byte_data(self.i2c_address, motor_register, motor_value)
        except Exception as e:
            print(f"Error writing to motor: {e}")

    def _write_pair(self, left_raw, right_raw):
        self._write_motor_raw(self.LEFT_MOTOR, left_raw)
        self._write_motor_raw(self.RIGHT_MOTOR, right_raw)

    def stop(self):
        self._write_pair(self.raw_stop, self.raw_stop)
        self.current_state = self.STATE_STOP

    def idle(self):
        self.stop()
        self.current_state = self.STATE_IDLE

    def forward(self, speed_percent=None):
        speed = speed_percent if speed_percent is not None else self.current_speed
        speed = max(0, min(100, int(speed)))
        val = self._speed_to_raw(self.raw_forward, speed)
        self._write_pair(val, val)
        self.current_state = self.STATE_FORWARD

    def reverse(self, speed_percent=None):
        speed = speed_percent if speed_percent is not None else self.current_speed
        speed = max(0, min(100, int(speed)))
        val = self._speed_to_raw(self.raw_reverse, speed)
        self._write_pair(val, val)
        self.current_state = self.STATE_REVERSE

    def turn_right(self, speed_percent=None):
        speed = speed_percent if speed_percent is not None else self.current_speed
        speed = max(0, min(100, int(speed)))
        # Match working keyboard_control.py:
        # right turn = left reverse, right forward.
        left_val = self._speed_to_raw(self.raw_reverse, speed)
        right_val = self._speed_to_raw(self.raw_forward, speed)
        self._write_pair(left_val, right_val)
        self.current_state = self.STATE_TURN_RIGHT

    def turn_left(self, speed_percent=None):
        speed = speed_percent if speed_percent is not None else self.current_speed
        speed = max(0, min(100, int(speed)))
        # Match working keyboard_control.py:
        # left turn = left forward, right reverse.
        left_val = self._speed_to_raw(self.raw_forward, speed)
        right_val = self._speed_to_raw(self.raw_reverse, speed)
        self._write_pair(left_val, right_val)
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
