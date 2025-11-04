import cv2
import numpy as np
import pyrealsense2 as rs
import time

def get_axis_position(tag_pos, tag_rvec, offset_vec):
    """
    tag_pos: 3D position of tag center (from tvec)
    tag_rvec: rotation vector of tag (from rvec)
    offset_vec: vector from tag center to axis in tag's local frame (meters)
    Returns: 3D position of axis in camera/world frame
    """
    R, _ = cv2.Rodrigues(tag_rvec)
    axis_pos = tag_pos + R @ offset_vec
    return axis_pos

def get_z_axis(rvec):
    R, _ = cv2.Rodrigues(rvec)
    z_axis = R @ np.array([0, 0, 1])  # Tag's local z-axis in camera/world frame
    return z_axis / np.linalg.norm(z_axis)


def rotate_base_z_local(R_base, degrees, about='y'):
    """
    Rotate the base tag's local z-axis by `degrees` about one of the base's local axes.
    about: 'x' | 'y' | 'z' (local axes of the base tag)
    Returns the rotated z-axis direction in the camera/world frame (unit vector).
    """
    theta = np.deg2rad(degrees)
    axes = {
        'x': np.array([1.0, 0.0, 0.0], dtype=float),
        'y': np.array([0.0, 1.0, 0.0], dtype=float),
        'z': np.array([0.0, 0.0, 1.0], dtype=float),
    }
    if about not in axes:
        raise ValueError("about must be one of 'x', 'y', 'z'")
    # Small rotation defined in the base's local frame
    R_delta_local = cv2.Rodrigues(axes[about] * theta)[0]
    z_local = np.array([0.0, 0.0, 1.0], dtype=float)
    # Apply local rotation, then map to world
    splay_z = R_base @ (R_delta_local @ z_local)
    return splay_z / np.linalg.norm(splay_z)


def calculate_angles(ids, corners, K, dist):
    if ids is None or len(ids) == 0:
        # No markers detected, return default values
        return None, None, None, None, None, None

    dip_id = 0
    pip_id = 1
    mcp_id = 2
    base_id = 3

    rvecs, tvecs, _obj = cv2.aruco.estimatePoseSingleMarkers(corners, 0.014, K, dist)

    angle_DP = None
    angle_PM = None
    angle_MS = None
    angle_SP = None
    splay_pos = None
    splay_z = None

    dip_z = None
    pip_z = None
    mcp_z = None

    # Process BASE independently
    try:
        rvecs_base, tvecs_base, _obj_base = cv2.aruco.estimatePoseSingleMarkers(corners, 0.03, K, dist)
        base_idx = list(ids.flatten()).index(base_id)
        base_rvec = rvecs_base[base_idx][0]
        # Rotate base's local z-axis by -10 degrees about its local +Y ("backward" pitch)
        R_base, _ = cv2.Rodrigues(base_rvec)
        splay_z = rotate_base_z_local(R_base, 10.0, about='x')
        # Translate splay_z by x=-0.029, y=0.027,z=-0.015
        splay_offset = np.array([-0.029, 0.030, -0.015], dtype=float)
        base_pos = tvecs_base[base_idx][0]
        splay_pos = get_axis_position(base_pos, base_rvec, splay_offset)
    except (ValueError, IndexError):
        pass

    # Process DIP independently
    try:
        dip_idx = list(ids.flatten()).index(dip_id)
        dip_rvec = rvecs[dip_idx][0]
        dip_z = get_z_axis(dip_rvec)
    except (ValueError, IndexError):
        pass

    # Process PIP independently
    try:
        pip_idx = list(ids.flatten()).index(pip_id)
        pip_rvec = rvecs[pip_idx][0]
        pip_z = get_z_axis(pip_rvec)
    except (ValueError, IndexError):
        pass

    # Process MCP independently
    try:
        mcp_idx = list(ids.flatten()).index(mcp_id)
        mcp_rvec = rvecs[mcp_idx][0]
        mcp_z = get_z_axis(mcp_rvec)
    except (ValueError, IndexError):
        pass

    # Calculate angles only if required vectors are available
    try:
        if dip_z is not None and pip_z is not None:
            angle_DP_rad = np.arccos(np.clip(np.dot(dip_z, pip_z), -1.0, 1.0))
            angle_DP = np.degrees(angle_DP_rad)

        if pip_z is not None and mcp_z is not None:
            angle_PM_rad = np.arccos(np.clip(np.dot(pip_z, mcp_z), -1.0, 1.0))
            angle_PM = np.degrees(angle_PM_rad)

        if mcp_z is not None and splay_z is not None:
            angle_MS_rad = np.arccos(np.clip(np.dot(mcp_z, splay_z), -1.0, 1.0))
            angle_MS = np.degrees(angle_MS_rad)
    except Exception:
        pass

    return angle_DP, angle_PM, angle_MS, angle_SP, splay_pos, splay_z


