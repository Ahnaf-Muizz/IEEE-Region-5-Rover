#!/usr/bin/env python3
"""
Mining Mayhem autonomous navigation orchestration.
Run subsystems alone for bench tests, or full mission on the Pi.

Examples:
  python main.py --mode camera
  python main.py --mode tags
  python main.py --mode material
  python main.py --mode start_light
  python main.py --mode full
  python main.py --mode odometry

Run modules directly (from this folder):
  python camera_io.py
  python april_tags.py
  python material_detection.py
  python start_light.py
  python odometry.py
  python encoder_state.py
  python motor_control.py
  python drive_control.py info
  python drive_control.py forward --yes
  python pose.py status
  python pose.py drive --yes --meters 0.1
  python camera_navigation.py preview
  python camera_navigation.py preview --show-tags
"""

import argparse
import time

import camera_io
import config
import drive_control
import encoder_state
import manipulator
import pose
import start_light
from camera_geometry import camera_world_to_rover_center
from camera_navigation import (
    approach_astral_material,
    execute_tag_behavior,
    go_to_material_search_zone,
    go_to_dropoff_tag6,
    go_to_random_beacon_from_0_to_4,
    match_time_expired,
    search_for_astral_material,
    search_for_telemetry,
    try_localize_from_wall_tags,
)


def run_full_mission():
    beacon_id = None
    last_turn_right = False

    start_light.wait_for_start_light()

    try:
        manipulator.initialize()
        match_start_time = time.time()
        encoder_state.reset_baseline()

        print("Initial pose:")
        pose.print_pose()

        print("Leaving start area...")
        if config.ODOMETRY_MODE == "encoder":
            pose.drive_distance_signed(config.LEAVE_START_DISTANCE_M)
        else:
            pose.drive_for(config.LEAVE_START_TIME)
        pose.print_pose()

        frame = camera_io.read_bgr()
        if frame is not None:
            try_localize_from_wall_tags(frame)

        while beacon_id is None:
            if match_time_expired(match_start_time):
                print("Match time expired during telemetry search.")
                drive_control.stop()
                return

            tag_id, result = search_for_telemetry(match_start_time)

            if result == "found":
                if tag_id in (0, 1, 2, 3, 4):
                    beacon_id = tag_id
                    break
                execute_tag_behavior(tag_id)

            elif result == "obstacle":
                last_turn_right = not last_turn_right

            elif result == "timeout":
                print("Telemetry search timed out, repositioning.")
                pose.drive_for(0.5)
                f2 = camera_io.read_bgr()
                if f2 is not None:
                    try_localize_from_wall_tags(f2)

            elif result == "match_over":
                print("Match time expired.")
                drive_control.stop()
                return

        print("Beacon boundary tag selected =", beacon_id)
        tx, ty = go_to_random_beacon_from_0_to_4(beacon_id)
        pose.robot_x, pose.robot_y = tx, ty
        pose.normalize_heading()
        encoder_state.reset_baseline()
        print("Beacon confirmed. Pose snapped to selected boundary tag.")
        pose.print_pose()

        nav_result = go_to_material_search_zone(match_start_time)
        if nav_result != "reached":
            print("Could not reach material search zone.")
            drive_control.stop()
            return

        material_found = False
        while not material_found:
            if match_time_expired(match_start_time):
                print("Match time expired during material search.")
                drive_control.stop()
                return

            result, material = search_for_astral_material(match_start_time)

            if result == "found":
                approach_result = approach_astral_material(match_start_time)

                if approach_result == "reached":
                    material_found = True
                    print("Astral Material objective reached. Running pickup sequence...")
                    if not manipulator.pickup_rock():
                        print("Pickup sequence failed or disabled.")

                elif approach_result == "obstacle":
                    last_turn_right = not last_turn_right

                elif approach_result in ["lost", "timeout"]:
                    print("Lost Astral Material, searching again.")

                elif approach_result == "match_over":
                    print("Match time expired.")
                    drive_control.stop()
                    return

            elif result == "obstacle":
                last_turn_right = not last_turn_right

            elif result == "timeout":
                print("Material search timed out, relocalizing and retrying.")
                f3 = camera_io.read_bgr()
                if f3 is not None:
                    try_localize_from_wall_tags(f3)
                pose.drive_for(0.4)

            elif result == "match_over":
                print("Match time expired.")
                drive_control.stop()
                return

        drop_result = go_to_dropoff_tag6(match_start_time)
        if drop_result == "reached":
            print("Arrived at dropoff point (Tag 6). Running dropoff sequence...")
            if not manipulator.drop_into_container():
                print("Dropoff sequence failed or disabled.")
        else:
            print("Failed to reach dropoff point (Tag 6).")

    except KeyboardInterrupt:
        print("Stopped by user.")
    finally:
        drive_control.stop()
        manipulator.shutdown()


def main():
    parser = argparse.ArgumentParser(description="Mining Mayhem rover navigation")
    parser.add_argument(
        "--mode",
        choices=["full", "camera", "tags", "material", "start_light", "odometry"],
        default="full",
        help="Subsystem to run (default: full mission). odometry = short encoder distance test.",
    )
    parser.add_argument(
        "--backend",
        default=None,
        help='Camera backend override: "usb" or "picamera2".',
    )
    parser.add_argument("--show", action="store_true", help="Enable OpenCV debug windows.")
    parser.add_argument(
        "--no-servo",
        action="store_true",
        help="Disable servo manipulator actions (pickup/dropoff) for dry runs.",
    )
    args = parser.parse_args()

    if args.backend in ("usb", "picamera2"):
        config.CAMERA_BACKEND = "picamera2" if args.backend == "picamera2" else "usb"
    if args.show:
        config.SHOW_CAMERA = True
    if args.no_servo:
        config.MANIPULATOR_ENABLED = False

    needs_camera = args.mode != "odometry"
    if needs_camera:
        camera_io.open_camera()

    try:
        if args.mode == "full":
            run_full_mission()
        elif args.mode == "camera":
            import cv2

            print("Camera test: use --show for live window (q to quit). Without --show, grabs 20 frames.")

            n = 0
            while True:
                f = camera_io.read_bgr()
                if f is None:
                    continue
                if config.SHOW_CAMERA:
                    cv2.imshow("main camera test", f)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
                else:
                    n += 1
                    if n <= 20:
                        print(f"frame {n} shape={getattr(f, 'shape', None)}")
                    if n >= 20:
                        break
                    time.sleep(0.05)
        elif args.mode == "tags":
            import april_tags
            import cv2

            print("AprilTag test: --show for live window (q to quit); else 30-frame sample.")
            n = 0
            while True:
                f = camera_io.read_bgr()
                if f is None:
                    continue
                tags = april_tags.detect_tags(f)
                for tag in tags:
                    tid = april_tags.get_tag_id(tag)
                    cx, cy = april_tags.get_tag_center(tag)
                    cv2.circle(f, (cx, cy), 8, (0, 255, 0), -1)
                    cv2.putText(
                        f,
                        f"id={tid}",
                        (cx + 10, cy),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2,
                    )
                if not config.SHOW_CAMERA:
                    n += 1
                    print(f"frame {n}: {len(tags)} tag(s)")
                    if n >= 30:
                        break
                    time.sleep(0.05)
                    continue
                cv2.imshow("tags", f)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
        elif args.mode == "material":
            import cv2
            import material_detection

            print("Material test: --show for live window (q to quit); else 30-frame sample.")
            n = 0
            while True:
                f = camera_io.read_bgr()
                if f is None:
                    continue
                m = material_detection.get_best_purple_material(f)
                if not config.SHOW_CAMERA:
                    n += 1
                    print(f"frame {n}: material={'yes' if m else 'no'}")
                    if n >= 30:
                        break
                    time.sleep(0.05)
                    continue
                disp = f.copy()
                scx = config.steering_center_x()
                cv2.line(disp, (scx, 0), (scx, config.FRAME_HEIGHT), (255, 0, 0), 2)
                if m:
                    x, y, w, h = m["bbox"]
                    cv2.rectangle(disp, (x, y), (x + w, y + h), (255, 0, 255), 2)
                cv2.imshow("material", disp)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
        elif args.mode == "start_light":
            start_light.wait_for_start_light()
        elif args.mode == "odometry":
            print("Encoder odometry bench test (~0.15 m forward, then pose).")
            print(f"ODOMETRY_MODE={config.ODOMETRY_MODE!r}")
            encoder_state.reset_baseline()
            pose.print_pose()
            pose.drive_distance_signed(0.15)
            pose.print_pose()
    finally:
        if needs_camera:
            camera_io.close_camera()
        try:
            import cv2

            cv2.destroyAllWindows()
        except Exception:
            pass


if __name__ == "__main__":
    main()
