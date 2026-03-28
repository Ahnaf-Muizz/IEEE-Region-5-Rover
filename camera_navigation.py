"""
Vision-based navigation: wall-tag localization, telemetry search, purple material approach.
Uses config.steering_center_x() for camera lateral offset vs. drivetrain.
"""

import time

import cv2

import april_tags
import config
import material_detection
import pose
from drive_control import stop


def match_time_expired(start_time):
    return (time.time() - start_time) >= config.MATCH_DURATION


def get_distance_cm():
    """No ultrasonic — always clear."""
    return 999.0


def show_frame(frame_bgr, tag=None, material=None):
    if not config.SHOW_CAMERA:
        return

    display = frame_bgr.copy()
    scx = config.steering_center_x()
    cv2.line(display, (scx, 0), (scx, config.FRAME_HEIGHT), (255, 0, 0), 1)
    fc = config.FRAME_WIDTH // 2
    cv2.line(display, (fc, 0), (fc, config.FRAME_HEIGHT), (128, 128, 255), 1)

    if tag is not None:
        cx, cy = april_tags.get_tag_center(tag)
        area = int(april_tags.get_tag_area(tag))
        tid = april_tags.get_tag_id(tag)
        cv2.circle(display, (cx, cy), 8, (0, 255, 0), -1)
        cv2.putText(
            display,
            f"ID={tid} area={area}",
            (max(10, cx - 60), max(20, cy - 20)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )

    if material is not None:
        x, y, w, h = material["bbox"]
        cx, cy = material["center"]
        area = int(material["area"])
        circ = material["circularity"]
        sol = material["solidity"]
        cv2.rectangle(display, (x, y), (x + w, y + h), (255, 0, 255), 2)
        cv2.circle(display, (cx, cy), 8, (255, 0, 255), -1)
        cv2.putText(
            display,
            f"PURPLE a={area} c={circ:.2f} s={sol:.2f}",
            (max(10, x), max(20, y - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 0, 255),
            2,
        )

    cv2.imshow("Robot Camera", display)
    cv2.waitKey(1)


def correct_pose_from_wall_tag(tag):
    import encoder_state

    tid = april_tags.get_tag_id(tag)
    cx, _ = april_tags.get_tag_center(tag)
    ref_x = config.steering_center_x()
    error = cx - ref_x

    if tid == 5:
        pose.robot_x, pose.robot_y = config.FIELD["wall_tag_5"]
        pose.robot_heading = 90.0
    elif tid == 6:
        pose.robot_x, pose.robot_y = config.FIELD["wall_tag_6"]
        pose.robot_heading = 270.0
    elif tid == 7:
        pose.robot_x, pose.robot_y = config.FIELD["wall_tag_7"]
        pose.robot_heading = 0.0

    half_w = config.FRAME_WIDTH / 2.0
    pose.robot_heading += (error / half_w) * 12.0
    pose.normalize_heading()

    encoder_state.reset_baseline()
    print(f"Localized from wall tag {tid}")
    pose.print_pose()


def try_localize_from_wall_tags(frame_bgr):
    if not config.USE_WALL_TAG_POSE_CORRECTION:
        return False

    tag = april_tags.get_best_wall_tag(frame_bgr)
    show_frame(frame_bgr, tag=tag)

    if tag is not None:
        correct_pose_from_wall_tag(tag)
        return True
    return False


def avoid_obstacle(last_turn_right):
    print("Obstacle branch (no ranging sensor) — skip.")
    return last_turn_right


def drive_toward_coordinate(target_x, target_y, match_start_time):
    last_turn_right = False

    while True:
        if match_time_expired(match_start_time):
            print("Match time expired during navigation.")
            stop()
            return "match_over"

        dist = pose.distance_to_target(target_x, target_y)
        if dist <= 0.18:
            stop()
            print("Arrived near target.")
            pose.print_pose()
            return "reached"

        target_heading = pose.angle_to_target(target_x, target_y)
        pose.rotate_to_heading(target_heading)

        frame = camera_io_read()
        if frame is not None:
            try_localize_from_wall_tags(frame)

        distance_cm = get_distance_cm()
        if distance_cm < config.OBSTACLE_DISTANCE_CM:
            last_turn_right = avoid_obstacle(last_turn_right)
            frame2 = camera_io_read()
            if frame2 is not None:
                try_localize_from_wall_tags(frame2)
            continue

        # Coordinate-first leg: drive a bounded chord toward the goal (encoder distance when enabled).
        step_m = min(0.55, max(0.08, dist - 0.15))
        print(f"Driving step {step_m:.2f} m toward x={target_x:.2f}, y={target_y:.2f}")
        pose.drive_distance_signed(step_m)
        pose.print_pose()

        frame3 = camera_io_read()
        if frame3 is not None:
            try_localize_from_wall_tags(frame3)


def camera_io_read():
    import camera_io

    return camera_io.read_bgr()


def search_for_telemetry(match_start_time):
    print("Searching for telemetry tag...")
    start = time.time()

    while time.time() - start < config.SEARCH_TIMEOUT:
        if match_time_expired(match_start_time):
            stop()
            return None, "match_over"

        if get_distance_cm() < config.CRITICAL_DISTANCE_CM:
            stop()
            return None, "obstacle"

        frame = camera_io_read()
        if frame is None:
            time.sleep(0.05)
            continue

        telemetry_tag = april_tags.get_best_telemetry_tag(frame)
        wall_tag = april_tags.get_best_wall_tag(frame)

        if telemetry_tag is not None:
            show_frame(frame, tag=telemetry_tag)
            tid = april_tags.get_tag_id(telemetry_tag)
            print(f"Found telemetry tag: {tid}")
            stop()
            return tid, "found"

        if wall_tag is not None and config.USE_WALL_TAG_POSE_CORRECTION:
            show_frame(frame, tag=wall_tag)
            correct_pose_from_wall_tag(wall_tag)
        else:
            show_frame(frame, tag=wall_tag if wall_tag is not None else None)

        pose.left_for(0.10, config.TURN_SPEED)

    stop()
    return None, "timeout"


def go_to_material_search_zone(match_start_time):
    print("Going to material search zone")
    tx, ty = config.FIELD["material_search_zone"]
    return drive_toward_coordinate(tx, ty, match_start_time)


def search_for_astral_material(match_start_time):
    print("Searching for Astral Material...")
    start = time.time()
    sweep_left = True

    while time.time() - start < config.MATERIAL_SEARCH_TIMEOUT:
        if match_time_expired(match_start_time):
            stop()
            return "match_over", None

        if get_distance_cm() < config.CRITICAL_DISTANCE_CM:
            stop()
            return "obstacle", None

        frame = camera_io_read()
        if frame is None:
            time.sleep(0.05)
            continue

        material = material_detection.get_best_purple_material(frame)
        wall_tag = april_tags.get_best_wall_tag(frame)

        if material is not None:
            show_frame(frame, material=material)
            print("Astral Material found")
            stop()
            return "found", material

        if wall_tag is not None and config.USE_WALL_TAG_POSE_CORRECTION:
            correct_pose_from_wall_tag(wall_tag)

        show_frame(frame, material=None)

        if sweep_left:
            pose.left_for(0.8, config.TURN_SPEED)
        else:
            pose.right_for(0.8, config.TURN_SPEED)

        sweep_left = not sweep_left

        if get_distance_cm() > config.OBSTACLE_DISTANCE_CM:
            pose.drive_for(0.8, config.APPROACH_SPEED)
        else:
            stop()
            return "obstacle", None

    stop()
    return "timeout", None


def approach_astral_material(match_start_time):
    print("Approaching Astral Material...")
    start = time.time()

    while time.time() - start < config.MATERIAL_APPROACH_TIMEOUT:
        if match_time_expired(match_start_time):
            stop()
            return "match_over"

        if get_distance_cm() < config.CRITICAL_DISTANCE_CM:
            stop()
            return "obstacle"

        frame = camera_io_read()
        if frame is None:
            time.sleep(0.05)
            continue

        material = material_detection.get_best_purple_material(frame)
        show_frame(frame, material=material)

        if material is None:
            stop()
            return "lost"

        err = material_detection.material_steering_error_px(material)
        area = material["area"]

        print(f"material error={err} area={int(area)} dist={get_distance_cm():.1f}")

        if get_distance_cm() <= config.OBSTACLE_DISTANCE_CM or area >= config.MATERIAL_CLOSE_AREA:
            stop()
            print("Reached Astral Material pickup position.")
            return "reached"

        if abs(err) > config.MATERIAL_CENTER_TOLERANCE:
            if err < 0:
                pose.left_for(0.8, config.TURN_SPEED)
            else:
                pose.right_for(0.8, config.TURN_SPEED)
        else:
            pose.drive_for(0.8, config.APPROACH_SPEED)

    stop()
    return "timeout"


def go_to_pad(pad_id, match_start_time):
    print("Going to pad", pad_id)

    if pad_id == 0:
        tx, ty = config.FIELD["pad_0"]
    elif pad_id == 1:
        tx, ty = config.FIELD["pad_1"]
    elif pad_id == 2:
        tx, ty = config.FIELD["pad_2"]
    elif pad_id == 3:
        tx, ty = config.FIELD["pad_3"]
    else:
        tx, ty = config.FIELD["pad_4"]

    return drive_toward_coordinate(tx, ty, match_start_time)


if __name__ == "__main__":
    import argparse

    import camera_io

    p = argparse.ArgumentParser(
        description="Vision navigation smoke test (camera only, no wheel motion by default)."
    )
    p.add_argument(
        "command",
        choices=["preview"],
        default="preview",
        nargs="?",
    )
    p.add_argument("--backend", default=None, help="usb or picamera2 (overrides config).")
    p.add_argument(
        "--show-tags",
        action="store_true",
        help="Draw best wall tag; apply USE_WALL_TAG_POSE_CORRECTION if True in config.",
    )
    args = p.parse_args()

    if args.backend in ("usb", "picamera2"):
        config.CAMERA_BACKEND = "picamera2" if args.backend == "picamera2" else "usb"
    config.SHOW_CAMERA = True

    camera_io.open_camera()
    print("preview: q to quit. Wall-tag pose updates only if USE_WALL_TAG_POSE_CORRECTION.")
    try:
        while True:
            frame = camera_io.read_bgr()
            if frame is None:
                time.sleep(0.05)
                continue
            tag = None
            material = None
            if args.show_tags:
                tag = april_tags.get_best_wall_tag(frame)
                if tag is not None and config.USE_WALL_TAG_POSE_CORRECTION:
                    correct_pose_from_wall_tag(tag)
            else:
                material = material_detection.get_best_purple_material(frame)
            show_frame(frame, tag=tag, material=material)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        camera_io.close_camera()
        cv2.destroyAllWindows()
