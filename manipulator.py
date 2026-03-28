"""
Servo manipulator control for rock pickup and container dropoff.

Public API used by mission code:
- initialize()
- pickup_rock()
- drop_rock_in_container()
- shutdown()
"""

import time

import config

try:
    from gpiozero import AngularServo
    from gpiozero.pins.pigpio import PiGPIOFactory

    SERVO_AVAILABLE = True
except Exception:
    SERVO_AVAILABLE = False
    AngularServo = None
    PiGPIOFactory = None
    print("[manipulator] gpiozero/pigpio unavailable; running in simulation mode.")

_manipulator = None


def _cfg(name, default=None, aliases=()):
    if hasattr(config, name):
        return getattr(config, name)
    for alias in aliases:
        if hasattr(config, alias):
            return getattr(config, alias)
    return default


class Manipulator:
    def __init__(self):
        self.enabled = bool(_cfg("MANIPULATOR_ENABLED", True))
        self.simulate = bool(_cfg("MANIPULATOR_SIMULATE", False)) or not SERVO_AVAILABLE
        self._factory = None
        self._arm = None
        self._wrist = None
        self._claw = None
        self._ready = False

        self.pin_arm = int(_cfg("SERVO_ARM_PIN", 5, aliases=("SERVO_BASE_PIN",)))
        self.pin_wrist = int(_cfg("SERVO_WRIST_PIN", 6))
        self.pin_claw = int(_cfg("SERVO_CLAW_PIN", 19))
        self.min_pulse = float(_cfg("SERVO_MIN_PULSE_S", 0.0005))
        self.max_pulse = float(_cfg("SERVO_MAX_PULSE_S", 0.0025))
        self.move_delay = float(_cfg("SERVO_SETTLE_S", 0.35, aliases=("SERVO_MOVE_DELAY_S",)))
        self.pick_settle = float(_cfg("SERVO_PICKUP_SETTLE_S", 0.45))
        self.grab_delay = float(_cfg("SERVO_GRAB_DELAY_S", 0.35))
        self.drop_settle = float(_cfg("SERVO_DROP_SETTLE_S", 0.4))
        self.release_delay = float(_cfg("SERVO_RELEASE_DELAY_S", 0.35))

        self.arm_stow = float(_cfg("ARM_STOW_ANGLE", 95.0, aliases=("SERVO_BASE_HOME_DEG",)))
        self.arm_pick = float(_cfg("ARM_PICK_ANGLE", 150.0, aliases=("SERVO_BASE_PICKUP_DEG",)))
        self.arm_lift = float(_cfg("ARM_LIFT_ANGLE", 80.0, aliases=("SERVO_BASE_LIFT_DEG",)))
        self.arm_drop = float(_cfg("ARM_DROP_ANGLE", 120.0, aliases=("SERVO_BASE_DROPOFF_DEG",)))
        self.wrist_stow = float(_cfg("WRIST_STOW_ANGLE", 95.0, aliases=("SERVO_WRIST_HOME_DEG",)))
        self.wrist_pick = float(_cfg("WRIST_PICK_ANGLE", 140.0, aliases=("SERVO_WRIST_DOWN_DEG",)))
        self.wrist_drop = float(_cfg("WRIST_DROP_ANGLE", 70.0, aliases=("SERVO_WRIST_DROPOFF_DEG",)))
        self.claw_open = float(_cfg("CLAW_OPEN_ANGLE", 100.0, aliases=("SERVO_CLAW_OPEN_DEG",)))
        self.claw_closed = float(_cfg("CLAW_CLOSED_ANGLE", 50.0, aliases=("SERVO_CLAW_CLOSED_DEG",)))

        if not self.enabled:
            print("[manipulator] disabled in config.")
            return

        if self.simulate:
            print("[manipulator] simulation mode active.")
            return

        try:
            # pigpio gives stable pulses for servos on Raspberry Pi.
            self._factory = PiGPIOFactory()
            self._arm = AngularServo(
                self.pin_arm,
                min_angle=0,
                max_angle=180,
                min_pulse_width=self.min_pulse,
                max_pulse_width=self.max_pulse,
                pin_factory=self._factory,
            )
            self._wrist = AngularServo(
                self.pin_wrist,
                min_angle=0,
                max_angle=180,
                min_pulse_width=self.min_pulse,
                max_pulse_width=self.max_pulse,
                pin_factory=self._factory,
            )
            self._claw = AngularServo(
                self.pin_claw,
                min_angle=0,
                max_angle=180,
                min_pulse_width=self.min_pulse,
                max_pulse_width=self.max_pulse,
                pin_factory=self._factory,
            )
            self._ready = True
            print("[manipulator] servos initialized.")
        except Exception as e:
            print(f"[manipulator] init failed ({e}); simulation mode active.")
            self.simulate = True
            self._ready = False

    @staticmethod
    def _clamp_angle(angle_deg):
        return max(0.0, min(180.0, float(angle_deg)))

    @staticmethod
    def _sleep(seconds):
        time.sleep(max(0.0, float(seconds)))

    def _set_servo(self, name, angle_deg):
        angle_deg = self._clamp_angle(angle_deg)
        if self.simulate or not self._ready:
            print(f"[manipulator] {name} -> {angle_deg:.1f} deg")
            return

        servo = {"arm": self._arm, "wrist": self._wrist, "claw": self._claw}[name]
        servo.angle = angle_deg

    def home(self):
        self._set_servo("arm", self.arm_stow)
        self._set_servo("wrist", self.wrist_stow)
        self._set_servo("claw", self.claw_open)
        self._sleep(self.move_delay)

    def pickup_rock(self):
        if not self.enabled:
            print("[manipulator] pickup skipped (disabled).")
            return True
        print("[manipulator] pickup sequence start")
        self._set_servo("arm", self.arm_pick)
        self._sleep(self.move_delay)
        self._set_servo("claw", self.claw_open)
        self._sleep(self.move_delay)
        self._set_servo("wrist", self.wrist_pick)
        self._sleep(self.pick_settle)
        self._set_servo("claw", self.claw_closed)
        self._sleep(self.grab_delay)
        self._set_servo("arm", self.arm_lift)
        self._sleep(self.move_delay)
        print("[manipulator] pickup sequence complete")
        return True

    def drop_in_container(self):
        if not self.enabled:
            print("[manipulator] dropoff skipped (disabled).")
            return True
        print("[manipulator] dropoff sequence start")
        self._set_servo("arm", self.arm_drop)
        self._sleep(self.move_delay)
        self._set_servo("wrist", self.wrist_drop)
        self._sleep(self.drop_settle)
        self._set_servo("claw", self.claw_open)
        self._sleep(self.release_delay)
        self._set_servo("arm", self.arm_stow)
        self._set_servo("wrist", self.wrist_stow)
        self._sleep(self.move_delay)
        print("[manipulator] dropoff sequence complete")
        return True

    def cleanup(self):
        try:
            if self.enabled:
                self.home()
        except Exception:
            pass

        for servo in (self._arm, self._wrist, self._claw):
            try:
                if servo is not None:
                    servo.detach()
            except Exception:
                pass
        self._arm = None
        self._wrist = None
        self._claw = None
        self._ready = False


def initialize():
    global _manipulator
    if _manipulator is None:
        _manipulator = Manipulator()
        _manipulator.home()
    return True


def pickup_rock():
    if _manipulator is None:
        initialize()
    return _manipulator.pickup_rock()


def drop_rock_in_container():
    if _manipulator is None:
        initialize()
    return _manipulator.drop_in_container()


def shutdown():
    global _manipulator
    if _manipulator is not None:
        _manipulator.cleanup()
        _manipulator = None

