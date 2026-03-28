"""
Camera-to-rover geometry helpers.

All offsets are in meters, expressed in rover body frame:
- +forward axis: toward rover front
- +right axis: toward rover right side
"""

import math

import config


def camera_to_rover_center(x_cam, y_cam, heading_deg):
    """
    Convert a camera reference point in world frame to rover center in world frame.

    heading convention matches the rest of the stack:
    0=E, 90=N, 180=W, 270=S.
    """
    if not getattr(config, "USE_CAMERA_OFFSET_COMPENSATION", True):
        return x_cam, y_cam

    th = math.radians(heading_deg)
    forward = float(config.CAMERA_OFFSET_FORWARD_M)
    right = float(config.CAMERA_OFFSET_RIGHT_M)

    # Camera position = rover center + forward*F + right*R.
    # So rover center = camera position - that rotated offset.
    dx_world = (forward * math.cos(th)) + (right * math.sin(th))
    dy_world = (forward * math.sin(th)) - (right * math.cos(th))
    return x_cam - dx_world, y_cam - dy_world


def camera_to_robot_center(x_cam, y_cam, heading_deg):
    """Alias for compatibility with existing imports."""
    return camera_to_rover_center(x_cam, y_cam, heading_deg)


def camera_world_to_rover_center(x_cam, y_cam, heading_deg):
    """Backward-compatible alias."""
    return camera_to_rover_center(x_cam, y_cam, heading_deg)
