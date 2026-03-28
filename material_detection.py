"""
Purple Astral Material (Nebulite / Geodinium) segmentation in BGR frames.
"""

import math

import cv2
import numpy as np

import config


def _mask_purple(hsv_frame):
    mask = cv2.inRange(hsv_frame, config.PURPLE_LOWER, config.PURPLE_UPPER)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.GaussianBlur(mask, (5, 5), 0)
    return mask


def get_best_purple_material(frame_bgr):
    # Force RGB->HSV path to match known-good field behavior.
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_RGB2HSV)
    mask = _mask_purple(hsv)

    if config.SHOW_DEBUG_MASK:
        cv2.imshow("Purple Mask", mask)
        cv2.waitKey(1)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best = None
    best_score = -1

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < config.MATERIAL_MIN_AREA:
            continue

        perimeter = cv2.arcLength(cnt, True)
        if perimeter <= 0:
            continue

        circularity = 4 * math.pi * area / (perimeter * perimeter)

        hull = cv2.convexHull(cnt)
        hull_area = cv2.contourArea(hull)
        if hull_area <= 0:
            continue

        solidity = area / hull_area
        x, y, w, h = cv2.boundingRect(cnt)

        if y + h < config.FRAME_HEIGHT // 3:
            continue

        if h <= 0:
            continue

        aspect = w / float(h)
        cx = x + w // 2
        cy = y + h // 2

        if circularity < config.MATERIAL_MIN_CIRCULARITY:
            continue
        if solidity < config.MATERIAL_MIN_SOLIDITY:
            continue
        if aspect < 0.5 or aspect > 1.8:
            continue

        score = area + 300 * solidity + 200 * circularity
        if score > best_score:
            best_score = score
            best = {
                "center": (cx, cy),
                "area": area,
                "bbox": (x, y, w, h),
                "circularity": circularity,
                "solidity": solidity,
            }

    return best


def material_steering_error_px(material):
    """Positive if blob is right of drivetrain steering line (camera offset applied)."""
    if material is None:
        return None
    cx, _ = material["center"]
    return cx - config.steering_center_x()


if __name__ == "__main__":
    import argparse
    import camera_io

    parser = argparse.ArgumentParser(description="Purple material detection test.")
    parser.add_argument("--backend", default=config.CAMERA_BACKEND)
    args = parser.parse_args()
    if args.backend in ("usb", "picamera2"):
        config.CAMERA_BACKEND = args.backend
    config.SHOW_CAMERA = True
    config.SHOW_DEBUG_MASK = True

    camera_io.open_camera()
    print("Press q to quit. Tune PURPLE_* in config.py if needed.")
    try:
        while True:
            frame = camera_io.read_bgr()
            if frame is None:
                continue
            m = get_best_purple_material(frame)
            disp = frame.copy()
            scx = config.steering_center_x()
            cv2.line(disp, (scx, 0), (scx, config.FRAME_HEIGHT), (255, 0, 0), 1)
            if m:
                x, y, w, h = m["bbox"]
                cv2.rectangle(disp, (x, y), (x + w, y + h), (255, 0, 255), 2)
                err = material_steering_error_px(m)
                cv2.putText(
                    disp,
                    f"err={err}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 255, 0),
                    2,
                )
            cv2.imshow("material test", disp)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        camera_io.close_camera()
        cv2.destroyAllWindows()
