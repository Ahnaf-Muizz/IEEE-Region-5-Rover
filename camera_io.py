"""
Camera capture: USB webcam (Arducam) via OpenCV, or optional Pi Camera Module 3 via Picamera2.
All frames are returned as BGR uint8 for OpenCV.
"""

import platform
import time

import cv2

import config

_cap = None
_picam2 = None


def open_camera():
    global _cap, _picam2

    if config.CAMERA_BACKEND == "picamera2":
        from picamera2 import Picamera2

        _picam2 = Picamera2()
        cam_cfg = _picam2.create_preview_configuration(
            main={"size": (config.FRAME_WIDTH, config.FRAME_HEIGHT)}
        )
        _picam2.configure(cam_cfg)
        _picam2.start()
        time.sleep(1.0)
        return

    backend = cv2.CAP_ANY
    if platform.system() == "Linux":
        backend = cv2.CAP_V4L2

    _cap = cv2.VideoCapture(config.CAMERA_INDEX, backend)
    if not _cap.isOpened():
        _cap.open(config.CAMERA_INDEX)

    _cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.FRAME_WIDTH)
    _cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.FRAME_HEIGHT)
    # USB cameras often ignore exact size; still try.

    for _ in range(10):
        ok, frame = _cap.read()
        if ok and frame is not None:
            break
        time.sleep(0.05)

    if not _cap.isOpened():
        raise RuntimeError(
            f"Could not open camera index {config.CAMERA_INDEX}. "
            "Try another CAMERA_INDEX in config.py or check permissions (video group on Linux)."
        )


def read_bgr():
    """Return BGR frame or None if capture failed."""
    global _cap, _picam2

    if config.CAMERA_BACKEND == "picamera2":
        if _picam2 is None:
            return None
        rgb = _picam2.capture_array()
        if rgb is None:
            return None
        if rgb.ndim == 2:
            return cv2.cvtColor(rgb, cv2.COLOR_GRAY2BGR)
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

    if _cap is None:
        return None
    ok, frame = _cap.read()
    if not ok or frame is None:
        return None
    return frame


def close_camera():
    global _cap, _picam2

    if _cap is not None:
        _cap.release()
        _cap = None

    if _picam2 is not None:
        try:
            _picam2.stop()
        except Exception:
            pass
        _picam2 = None


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Test camera capture.")
    parser.add_argument("--backend", default=config.CAMERA_BACKEND, choices=["usb", "picamera2"])
    args = parser.parse_args()
    config.CAMERA_BACKEND = "picamera2" if args.backend == "picamera2" else "usb"
    config.SHOW_CAMERA = True

    open_camera()
    print("Press q to quit.")
    try:
        while True:
            frame = read_bgr()
            if frame is None:
                print("No frame")
                break
            cv2.imshow("camera_io test", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        close_camera()
        cv2.destroyAllWindows()
