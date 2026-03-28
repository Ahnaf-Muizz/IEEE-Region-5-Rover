"""
Last encoder sample for delta computation (avoids circular imports with pose).
"""

_last_l = None
_last_r = None


def reset_baseline():
    global _last_l, _last_r
    _last_l = None
    _last_r = None


def consume_tick_delta(read_fn):
    """
    read_fn() -> (left_ticks, right_ticks) integers (cumulative from driver).
    Returns (dl, dr) tick deltas since last call. First call returns (0,0) after seeding.
    """
    global _last_l, _last_r
    l, r = read_fn()
    if _last_l is None:
        _last_l, _last_r = l, r
        return 0, 0
    dl = l - _last_l
    dr = r - _last_r
    # crude overflow guard (32-bit-ish jump)
    if abs(dl) > 100000 or abs(dr) > 100000:
        _last_l, _last_r = l, r
        return 0, 0
    _last_l, _last_r = l, r
    return dl, dr


if __name__ == "__main__":
    reset_baseline()
    _demo_n = 0

    def fake_read():
        global _demo_n
        _demo_n += 1
        return (100 + _demo_n * 10, 100 + _demo_n * 10)

    print("encoder_state demo (fake increasing ticks):")
    for i in range(4):
        dl, dr = consume_tick_delta(fake_read)
        print(f"  call {i}: delta = ({dl}, {dr})")
