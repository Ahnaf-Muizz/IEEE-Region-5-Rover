"""
Tunable constants for Mining Mayhem (IEEE Region 5) navigation.
Calibrate FIELD coordinates and dead-reckoning on your practice field.
"""

import numpy as np

# --- Camera ---
# 0 = first USB camera. On Pi, try 0; if the Arducam is second device, use 1.
CAMERA_INDEX = 0
# "usb" = OpenCV VideoCapture (Arducam USB). "picamera2" = Pi Camera Module only.
CAMERA_BACKEND = "usb"
# Prefer explicit color format for Picamera2 to avoid luma-only/grayscale frames.
PICAMERA2_MAIN_FORMAT = "RGB888"
# Keep auto exposure / white-balance on unless you intentionally tune manual controls.
PICAMERA2_AE_ENABLE = True
PICAMERA2_AWB_ENABLE = True
PICAMERA2_SATURATION = 1.1
PICAMERA2_CONTRAST = 1.0
# Only allow grayscale/low-light fallback when the scene is almost pitch dark.
# OpenCV grayscale brightness scale is 0..255.
PICAMERA2_FORCE_GRAY_DARK_ONLY = True
PICAMERA2_GRAY_DARKNESS_THRESHOLD = 3.0

# If the lens is to the RIGHT of the robot centerline (forward view), rocks on the
# centerline appear LEFT of the image center. Use a NEGATIVE offset (pixels) to move
# the steering target left, or POSITIVE if the opposite. Tune while driving at pickup range.
# effective_steering_x = (FRAME_WIDTH // 2) + CAMERA_STEERING_OFFSET_PX
CAMERA_STEERING_OFFSET_PX = -35

FRAME_WIDTH = 640
FRAME_HEIGHT = 480
SHOW_CAMERA = False
SHOW_DEBUG_MASK = False
# Match proven pipeline: interpret frames as RGB for purple HSV conversion.
MATERIAL_COLOR_ORDER = "rgb"  # options: "rgb", "bgr"
# Camera mounting offset from rover center (meters).
# Coordinate convention:
#   +x = forward, +y = left
# User measurement:
#   x-axis offset = 71.31 mm  -> forward
#   y-axis offset = 50.19 mm  -> right (so left is negative)
CAMERA_OFFSET_X_M = 0.07131
CAMERA_OFFSET_Y_M = -0.05019
USE_CAMERA_OFFSET_COMPENSATION = True

# --- Match ---
MATCH_DURATION = 180.0  # Game Manual: 3 minutes

# --- Odometry: encoder differential drive (no IMU) ---
# "encoder" = distance/heading moves use wheel ticks; "time" = legacy seconds × rates.
# L298N has no encoder interface by itself, so default to time mode.
ODOMETRY_MODE = "time"

# Distance between left and right wheel contact patches (meters). Measure carefully.
TRACK_WIDTH_M = 0.16
WHEEL_DIAMETER_M = 0.065
# Full quadrature counts per one wheel revolution (datasheet × 4 if using 4x decoding).
ENCODER_TICKS_PER_REV = 980
# Flip sign if forward drive makes counts decrease.
ENCODER_LEFT_SIGN = 1
ENCODER_RIGHT_SIGN = 1

ODOMETRY_POLL_INTERVAL_S = 0.02
# Stop turning / distance when within these tolerances
HEADING_TOLERANCE_DEG = 4.0
DISTANCE_TOLERANCE_M = 0.04
# If ticks never change (stub driver / wiring), fall back to time-based pose after this many seconds of motion
ENCODER_STALL_TIME_S = 0.35

# Field is ~4' × 8' plywood; inner usable area is smaller — your FIELD dict should match tape + layout.
FIELD_INNER_WIDTH_M = 1.22
FIELD_INNER_LENGTH_M = 2.44

# Vision snaps pose to wall tags — off = pure coordinate + odometry only.
USE_WALL_TAG_POSE_CORRECTION = False

# --- Navigation timing / fallback when ODOMETRY_MODE == "time" or encoder stall fallback ---
METERS_PER_SECOND = 0.35
DEGREES_PER_SECOND = 110.0

# Leave Landing Site: prefer encoder distance when ODOMETRY_MODE == "encoder"
LEAVE_START_DISTANCE_M = 0.52
LEAVE_START_TIME = 1.5
BACKUP_TIME = 0.7
TURN_AVOID_TIME = 0.55

SEARCH_TIMEOUT = 8.0
MATERIAL_SEARCH_TIMEOUT = 20.0
MATERIAL_APPROACH_TIMEOUT = 12.0

CENTER_TOLERANCE = 50
TAG_CLOSE_AREA = 3000

# Ultrasonic removed — keep large thresholds so logic never triggers obstacle branch
OBSTACLE_DISTANCE_CM = 999
CRITICAL_DISTANCE_CM = 999

# --- Purple Astral Material (HSV). OpenCV H: 0–179. Tune under your lighting. ---
PURPLE_LOWER = np.array([120, 60, 40], dtype=np.uint8)
PURPLE_UPPER = np.array([165, 255, 255], dtype=np.uint8)

MATERIAL_MIN_AREA = 900
MATERIAL_MIN_CIRCULARITY = 0.45
MATERIAL_MIN_SOLIDITY = 0.70
MATERIAL_CLOSE_AREA = 12000
MATERIAL_CENTER_TOLERANCE = 60
MATERIAL_FORWARD_STEP = 0.35

# --- Field frame (meters). Heading: 0=E, 90=N, 180=W, 270=S
# VERIFY against Game Manual 2 figures and your taped field; ±1 in per rules.
FIELD = {
    "start": (0.20, 0.20),
    "mast": (0.20, 1.20),  # fallback/default beacon mast reference
    "pad_0": (0.35, 0.45),
    "pad_1": (0.35, 0.75),
    "pad_2": (0.35, 1.05),
    "pad_3": (0.35, 1.35),
    "pad_4": (0.35, 1.65),
    "wall_tag_5": (2.20, 1.90),
    "wall_tag_6": (2.20, 0.10),
    "wall_tag_7": (2.30, 1.00),
    "material_search_zone": (1.20, 1.00),
    # Boundary tag references (IDs 0-4). Update with measured field coordinates.
    "tag_0": (0.35, 0.45),
    "tag_1": (0.35, 0.75),
    "tag_2": (0.35, 1.05),
    "tag_3": (0.35, 1.35),
    "tag_4": (0.35, 1.65),
    # Functional objectives per user mapping.
    "dropoff": (0.35, 1.05),      # Tag 6 region
    "cave_entrance": (2.00, 1.00),  # Tag 7 region
}

# --- AprilTag role mapping per current game plan ---
# IDs 0-4: field boundary / random beacon mast selection
TAG_BEACON_IDS = [0, 1, 2, 3, 4]
# ID 5: "go forwards"
TAG_FORWARD_ID = 5
# ID 6: dropoff point
TAG_DROPOFF_ID = 6
# ID 7: cave entrance + continue straight
TAG_CAVE_ENTRY_ID = 7

# Role-action tuning
TAG5_FORWARD_SECONDS = 0.8
TAG7_STRAIGHT_SECONDS = 0.9

# --- Motor PWM tuning (passed to motor_control) ---
FORWARD_SPEED = 100
TURN_SPEED = 90
APPROACH_SPEED = 90
BACKUP_SPEED = 90

# --- L298N GPIO backend (BCM numbering) ---
# Adjust pinout to your wiring on Raspberry Pi.
L298N_LEFT_IN1 = 17
L298N_LEFT_IN2 = 27
L298N_LEFT_EN = 18
L298N_RIGHT_IN1 = 23
L298N_RIGHT_IN2 = 24
L298N_RIGHT_EN = 13
L298N_PWM_FREQUENCY_HZ = 1000
L298N_USE_PWM = True
# Set True if either motor side spins opposite your expected direction.
L298N_LEFT_INVERT = False
L298N_RIGHT_INVERT = False


def steering_center_x():
    return (FRAME_WIDTH // 2) + CAMERA_STEERING_OFFSET_PX
