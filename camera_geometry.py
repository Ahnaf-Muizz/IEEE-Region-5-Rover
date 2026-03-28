"""
Camera-to-rover geometry helpers.

All offsets are in meters, expressed in rover body frame:
- +x: forward
- +y: left
"""

import math

import config


def camera_world_to_rover_center(x_cam, y_cam, heading_deg):
    """
    Convert a camera reference point in world frame to rover center in world frame.

    heading convention matches the rest of the stack:
    0=E, 90=N, 180=W, 270=S.
    """
    th = math.radians(heading_deg)
    dx_body = config.CAMERA_OFFSET_X_M
    dy_body = config.CAMERA_OFFSET_Y_M

    # Body-frame offset rotated into world frame.
    dx_world = dx_body * math.cos(th) - dy_body * math.sin(th)
    dy_world = dx_body * math.sin(th) + dy_body * math.cos(th)
    return x_cam - dx_world, y_cam - dy_world
