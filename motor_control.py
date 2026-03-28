#!/usr/bin/env python3
"""
Motor control for L298N dual H-bridge using Raspberry Pi GPIO (BCM pins).

Compatible interface for rover stack:
forward, reverse, turn_left, turn_right, stop, read_wheel_ticks.
"""

import atexit

import config

try:
    import RPi.GPIO as GPIO

    GPIO_AVAILABLE = True
except Exception:
    GPIO_AVAILABLE = False
    GPIO = None
    print("Warning: RPi.GPIO not available. Motor control running in simulation mode.")


class MotorController:
    """Controls left/right motor channels through an L298N."""

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

    def __init__(
        self,
        left_in1=config.L298N_LEFT_IN1,
        left_in2=config.L298N_LEFT_IN2,
        left_en=config.L298N_LEFT_EN,
        right_in1=config.L298N_RIGHT_IN1,
        right_in2=config.L298N_RIGHT_IN2,
        right_en=config.L298N_RIGHT_EN,
        pwm_hz=config.L298N_PWM_FREQUENCY_HZ,
        use_pwm=config.L298N_USE_PWM,
        left_invert=config.L298N_LEFT_INVERT,
        right_invert=config.L298N_RIGHT_INVERT,
    ):
        self.left_in1 = int(left_in1)
        self.left_in2 = int(left_in2)
        self.left_en = int(left_en)
        self.right_in1 = int(right_in1)
        self.right_in2 = int(right_in2)
        self.right_en = int(right_en)
        self.pwm_hz = max(50, int(pwm_hz))
        self.use_pwm = bool(use_pwm)
        self.left_invert = bool(left_invert)
        self.right_invert = bool(right_invert)

        self.current_state = self.STATE_STOP
        self.current_speed = self.SPEED_FULL
        self._warned_no_encoder = False
        self._gpio_ready = False
        self._pwm_left = None
        self._pwm_right = None

        if GPIO_AVAILABLE:
            try:
                GPIO.setwarnings(False)
                GPIO.setmode(GPIO.BCM)

                for pin in (
                    self.left_in1,
                    self.left_in2,
                    self.left_en,
                    self.right_in1,
                    self.right_in2,
                    self.right_en,
                ):
                    GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)

                if self.use_pwm:
                    self._pwm_left = GPIO.PWM(self.left_en, self.pwm_hz)
                    self._pwm_right = GPIO.PWM(self.right_en, self.pwm_hz)
                    self._pwm_left.start(0)
                    self._pwm_right.start(0)
                else:
                    GPIO.output(self.left_en, GPIO.HIGH)
                    GPIO.output(self.right_en, GPIO.HIGH)

                self._gpio_ready = True
                print("L298N GPIO initialized (BCM mode).")
            except Exception as e:
                print(f"Failed to initialize GPIO for L298N: {e}")
                self._gpio_ready = False
        else:
            print("RPi.GPIO unavailable - simulation mode")

        self.stop()
        atexit.register(self.cleanup)

    @staticmethod
    def _clamp_speed(speed_percent):
        return max(0, min(100, int(speed_percent)))

    def _set_enable_duty(self, left_duty, right_duty):
        if not self._gpio_ready:
            return
        if self.use_pwm:
            self._pwm_left.ChangeDutyCycle(self._clamp_speed(left_duty))
            self._pwm_right.ChangeDutyCycle(self._clamp_speed(right_duty))
        else:
            GPIO.output(self.left_en, GPIO.HIGH if left_duty > 0 else GPIO.LOW)
            GPIO.output(self.right_en, GPIO.HIGH if right_duty > 0 else GPIO.LOW)

    def _apply_side(self, is_left, forward, duty):
        if not self._gpio_ready:
            side = "LEFT" if is_left else "RIGHT"
            direction = "FWD" if forward else "REV"
            print(f"[SIM] {side} {direction} duty={self._clamp_speed(duty)}")
            return

        if is_left:
            in1, in2 = self.left_in1, self.left_in2
            invert = self.left_invert
        else:
            in1, in2 = self.right_in1, self.right_in2
            invert = self.right_invert

        duty = self._clamp_speed(duty)
        if duty <= 0:
            GPIO.output(in1, GPIO.LOW)
            GPIO.output(in2, GPIO.LOW)
            return

        logical_forward = not forward if invert else forward
        if logical_forward:
            GPIO.output(in1, GPIO.HIGH)
            GPIO.output(in2, GPIO.LOW)
        else:
            GPIO.output(in1, GPIO.LOW)
            GPIO.output(in2, GPIO.HIGH)

    def _drive(self, left_forward, right_forward, speed_percent, state_name, sim_label):
        speed = self._clamp_speed(speed_percent if speed_percent is not None else self.current_speed)
        if speed <= 0:
            self.stop()
            return

        if not self._gpio_ready:
            print(f"[SIM] {sim_label} speed={speed}%")

        self._apply_side(True, left_forward, speed)
        self._apply_side(False, right_forward, speed)
        self._set_enable_duty(speed, speed)
        self.current_state = state_name

    def stop(self):
        if self._gpio_ready:
            GPIO.output(self.left_in1, GPIO.LOW)
            GPIO.output(self.left_in2, GPIO.LOW)
            GPIO.output(self.right_in1, GPIO.LOW)
            GPIO.output(self.right_in2, GPIO.LOW)
            self._set_enable_duty(0, 0)
        else:
            print("[SIM] stop")
        self.current_state = self.STATE_STOP

    def idle(self):
        self.stop()
        self.current_state = self.STATE_IDLE

    def forward(self, speed_percent=None):
        self._drive(True, True, speed_percent, self.STATE_FORWARD, "forward")

    def reverse(self, speed_percent=None):
        self._drive(False, False, speed_percent, self.STATE_REVERSE, "reverse")

    def turn_right(self, speed_percent=None):
        self._drive(True, False, speed_percent, self.STATE_TURN_RIGHT, "turn_right")

    def turn_left(self, speed_percent=None):
        self._drive(False, True, speed_percent, self.STATE_TURN_LEFT, "turn_left")

    def slow_down(self, target_speed_percent):
        target_speed_percent = self._clamp_speed(target_speed_percent)
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
        self.current_speed = self._clamp_speed(speed_percent)

    def get_state(self):
        return self.current_state

    def get_speed(self):
        return self.current_speed

    def read_wheel_ticks(self):
        """
        L298N does not include encoder feedback.
        Return (0, 0) so pose code can use time-mode fallback.
        """
        if not self._warned_no_encoder:
            print("read_wheel_ticks(): no encoder interface on L298N backend; returning (0, 0).")
            self._warned_no_encoder = True
        return (0, 0)

    def cleanup(self):
        try:
            self.stop()
        except Exception:
            pass

        if self._gpio_ready:
            try:
                if self._pwm_left is not None:
                    self._pwm_left.stop()
                if self._pwm_right is not None:
                    self._pwm_right.stop()
                GPIO.cleanup(
                    [
                        self.left_in1,
                        self.left_in2,
                        self.left_en,
                        self.right_in1,
                        self.right_in2,
                        self.right_en,
                    ]
                )
            except Exception:
                pass

        self._gpio_ready = False


if __name__ == "__main__":
    m = MotorController()
    print("read_wheel_ticks() =", m.read_wheel_ticks())
    print("L298N motor controller ready.")
