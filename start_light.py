"""
Start LED detection: brightness step in center ROI (Game Manual — south wall LED by Landing Site).
"""

import time

import cv2
import numpy as np

import camera_io
import config


def wait_for_start_light(baseline_frames=30, change_threshold=25, required_frames=5):
    bright_threshold = 200
    min_bright_pixels = 1200

    print("Calibrating baseline brightness...")
    values = []

    for _ in range(baseline_frames):
        frame = camera_io.read_bgr()
        if frame is None:
            time.sleep(0.05)
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        h, w = gray.shape
        region = gray[h // 3 : 2 * h // 3, w // 3 : 2 * w // 3]

        brightness = float(np.mean(region))
        values.append(brightness)
        time.sleep(0.05)

    if not values:
        print("No frames for baseline; skipping start light wait.")
        return True

    baseline = sum(values) / len(values)
    print(f"Baseline brightness: {baseline:.1f}")
    print("Waiting for brightness change...")
    wait_deadline = time.time() + float(getattr(config, "START_LIGHT_TIMEOUT_S", 10.0))
    fallback_after_timeout = bool(getattr(config, "START_LIGHT_FALLBACK_AFTER_TIMEOUT", True))

    detections = 0

    while True:
        if fallback_after_timeout and time.time() >= wait_deadline:
            print("START LIGHT timeout reached; proceeding with autonomous sequence.")
            return True

        frame = camera_io.read_bgr()
        if frame is None:
            time.sleep(0.05)
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        h, w = gray.shape
        region = gray[h // 3 : 2 * h // 3, w // 3 : 2 * w // 3]

        brightness = float(np.mean(region))
        change = brightness - baseline
        bright_pixels = int(np.sum(region >= bright_threshold))

        print(f"Brightness: {brightness:.1f} Change: {change:.1f} BrightPixels: {bright_pixels}")

        if change > change_threshold and bright_pixels > min_bright_pixels:
            detections += 1
        else:
            detections = 0

        if detections >= required_frames:
            print("START LIGHT DETECTED")
            return True

        time.sleep(0.05)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Test Start LED detection (blocks until trigger).")
    parser.add_argument("--backend", default=config.CAMERA_BACKEND)
    args = parser.parse_args()
    if args.backend in ("usb", "picamera2"):
        config.CAMERA_BACKEND = args.backend

    camera_io.open_camera()
    try:
        wait_for_start_light()
    finally:
        camera_io.close_camera()
