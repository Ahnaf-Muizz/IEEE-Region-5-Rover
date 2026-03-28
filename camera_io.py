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
_warned_picam_gray = False


def open_camera():
    global _cap, _picam2

    if config.CAMERA_BACKEND == "picamera2":
        from picamera2 import Picamera2

        _picam2 = Picamera2()
        cam_cfg = _picam2.create_preview_configuration(
            main={
                "size": (config.FRAME_WIDTH, config.FRAME_HEIGHT),
                "format": config.PICAMERA2_MAIN_FORMAT,
            }
        )
        _picam2.configure(cam_cfg)
        try:
            _picam2.set_controls(
                {
                    "AeEnable": bool(config.PICAMERA2_AE_ENABLE),
                    "AwbEnable": bool(config.PICAMERA2_AWB_ENABLE),
                    "Saturation": float(config.PICAMERA2_SATURATION),
                    "Contrast": float(config.PICAMERA2_CONTRAST),
                }
            )
        except Exception:
            # Some camera drivers/versions may not expose all controls.
            pass
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
    global _cap, _picam2, _warned_picam_gray

    if config.CAMERA_BACKEND == "picamera2":
        if _picam2 is None:
            return None
        frame = _picam2.capture_array()
        if frame is None:
            return None
        if frame.ndim == 3 and frame.shape[2] == 3:
            # Keep color in normal lighting. Only collapse to grayscale in extreme darkness.
            try:
                if bool(getattr(config, "PICAMERA2_FORCE_GRAY_DARK_ONLY", True)):
                    gray_probe = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
                    dark_thr = float(getattr(config, "GRAYSCALE_DARK_THRESHOLD", 3.0))
                    dark_thr = max(0.0, min(255.0, dark_thr))
                    if float(gray_probe.mean()) <= dark_thr:
                        return cv2.cvtColor(gray_probe, cv2.COLOR_GRAY2BGR)
            except Exception:
                pass
        if frame.ndim == 2:
            # If we ever get luma-only output, this indicates camera format/control mismatch.
            # Try YUV420 decode first, else keep compatibility with grayscale->BGR.
            if frame.shape[0] == (config.FRAME_HEIGHT * 3) // 2:
                return cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_I420)
            if not _warned_picam_gray:
                print(
                    "[camera_io] warning: Picamera frame is grayscale; "
                    "check PICAMERA2_MAIN_FORMAT and camera controls in config.py"
                )
                _warned_picam_gray = True
            return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        if frame.ndim == 3 and frame.shape[2] == 4:
            return cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
        if frame.ndim == 3 and frame.shape[2] == 3:
            return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        return None

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
