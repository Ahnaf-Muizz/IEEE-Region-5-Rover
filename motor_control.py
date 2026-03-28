"""
Replace this stub with your real 4-channel encoder driver on the Pi.

Required: forward, reverse, turn_left, turn_right, stop.
For coordinate / encoder navigation: implement read_wheel_ticks().

If you have four motors, return the sum (or average × 2) of left-side ticks as left,
and same for right, so straight-line motion increases both counts forward.
"""


class MotorController:
    def forward(self, speed):
        print(f"[stub] forward({speed})")

    def reverse(self, speed):
        print(f"[stub] reverse({speed})")

    def turn_left(self, speed):
        print(f"[stub] turn_left({speed})")

    def turn_right(self, speed):
        print(f"[stub] turn_right({speed})")

    def stop(self):
        print("[stub] stop()")

    def read_wheel_ticks(self):
        """
        Cumulative quadrature counts: forward motion should increase both.
        Return (left_ticks, right_ticks). Stub never moves counts — pose will
        time-fallback if no motion is seen (see config.ENCODER_STALL_TIME_S).
        """
        return (0, 0)


if __name__ == "__main__":
    m = MotorController()
    print("read_wheel_ticks() =", m.read_wheel_ticks())
    print("Replace this module with your driver; keep the same public methods.")
