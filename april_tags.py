"""
AprilTag detection (tag36h11). Competition: wall IDs 5,6,7; mast telemetry 0–4.
"""

import math

import cv2

import config

APRILTAG_BACKEND = None
DetectorClass = None
_detector = None

try:
    from pupil_apriltags import Detector as PupilDetector

    APRILTAG_BACKEND = "pupil"
    DetectorClass = PupilDetector
except Exception:
    try:
        import apriltag

        APRILTAG_BACKEND = "apriltag"
        DetectorClass = apriltag
    except Exception:
        APRILTAG_BACKEND = None
        DetectorClass = None


def get_detector():
    global _detector
    if _detector is not None:
        return _detector

    if APRILTAG_BACKEND == "pupil":
        _detector = DetectorClass(families="tag36h11")
    elif APRILTAG_BACKEND == "apriltag":
        options = DetectorClass.DetectorOptions(families="tag36h11")
        _detector = DetectorClass.Detector(options)
    else:
        raise RuntimeError(
            "No AprilTag library found. On Pi: pip install pupil-apriltags  (or apriltag)"
        )
    return _detector


def detect_tags(frame_bgr):
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    return get_detector().detect(gray)


def get_tag_id(tag):
    if hasattr(tag, "tag_id"):
        return int(tag.tag_id)
    if hasattr(tag, "tagID"):
        return int(tag.tagID)
    return None


def get_tag_center(tag):
    c = tag.center
    return int(c[0]), int(c[1])


def get_tag_area(tag):
    pts = [(float(p[0]), float(p[1])) for p in tag.corners]
    area = 0.0
    for i in range(len(pts)):
        x1, y1 = pts[i]
        x2, y2 = pts[(i + 1) % len(pts)]
        area += x1 * y2 - x2 * y1
    return abs(area) * 0.5


def get_best_tag_by_ids(frame_bgr, valid_ids):
    tags = detect_tags(frame_bgr)
    good = []
    for tag in tags:
        tid = get_tag_id(tag)
        if tid in valid_ids:
            good.append(tag)
    if not good:
        return None
    good.sort(key=lambda t: get_tag_area(t), reverse=True)
    return good[0]


def get_best_telemetry_tag(frame_bgr):
    return get_best_tag_by_ids(frame_bgr, [0, 1, 2, 3, 4])


def get_best_wall_tag(frame_bgr):
    return get_best_tag_by_ids(frame_bgr, [5, 6, 7])


def tag_heading_error_px(tag):
    """Pixel error vs. image center (not drivetrain center). Use for wall localization."""
    cx, _ = get_tag_center(tag)
    return cx - (config.FRAME_WIDTH // 2)


if __name__ == "__main__":
    import argparse
    import camera_io

    parser = argparse.ArgumentParser(description="Live AprilTag test.")
    parser.add_argument("--backend", default=config.CAMERA_BACKEND)
    args = parser.parse_args()
    if args.backend:
        config.CAMERA_BACKEND = args.backend if args.backend in ("usb", "picamera2") else "usb"
    config.SHOW_CAMERA = True

    camera_io.open_camera()
    print("Press q to quit.")
    try:
        while True:
            frame = camera_io.read_bgr()
            if frame is None:
                continue
            for tag in detect_tags(frame):
                tid = get_tag_id(tag)
                cx, cy = get_tag_center(tag)
                cv2.circle(frame, (cx, cy), 8, (0, 255, 0), -1)
                cv2.putText(
                    frame,
                    f"id={tid}",
                    (cx + 10, cy),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                )
            cv2.imshow("apriltag test", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        camera_io.close_camera()
        cv2.destroyAllWindows()
