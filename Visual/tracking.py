# import cv2
# import numpy as np
# import pyrealsense2 as rs
# import time
# from aruco_detect import *


# COLOR_W, COLOR_H, COLOR_FPS = 640, 480, 30
# ARUCO_DICT = cv2.aruco.DICT_4X4_50
# DRAW_AXIS = True
# marker_size_m = 0.020  # 20mm
# tip_id = 0
# mid_id = 1
# low_id = 2

# motor_id = 20
# device = '/dev/ttyUSB0'
# baud = 2000000
# start_pos = 1200
# end_pos = 1800
# steps = 100
# DRY_RUN = True


# def activate_camera():
#     pipe = rs.pipeline()
#     cfg = rs.config()
#     cfg.enable_stream(rs.stream.color, COLOR_W, COLOR_H, rs.format.bgr8, COLOR_FPS)
#     profile = pipe.start(cfg)
#     print(f"Streaming color {COLOR_W}x{COLOR_H}@{COLOR_FPS}…  (q=quit, s=snapshot)")

#     K, dist = get_color_intrinsics(profile)
    
#     detector = make_aruco_detector(ARUCO_DICT)
    
#     snap_id = 0
#     fps_smooth = None
#     t_prev = time.time()
    
#     return pipe, detector, K, dist, snap_id, fps_smooth, t_prev

# def motor_setup():
#     client = DynamixelClient([motor_id], port=device)
#     client.connect()
#     print("Enabling torque...")
#     client.set_torque_enabled(True)
#     time.sleep(0.1)
#     return client

# def get_x_axis(rvec):
#     R, _ = cv2.Rodrigues(rvec)
#     x_axis = R @ np.array([1, 0, 0])  # Tag's local x-axis in camera/world frame
#     return x_axis / np.linalg.norm(x_axis)


# def main():
#     pipe, detector, K, dist, snap_id, fps_smooth, t_prev = activate_camera()
#     client = motor_setup()
#     position = []
#     dip = []
#     pip = []
#     move_position = start_pos

#     for i in range(50):
#         if move_position >= start_pos:
#             move_position -= (end_pos - start_pos) / steps
#         else:
#             move_position += (end_pos - start_pos) / steps
        
#         position.append(move_position)
#         print(f"Moving to position {move_position}")
#         if DRY_RUN:
#             print(f"DRY RUN: would set motor {motor_id} -> {move_position}")
#         else:
#             client.set_pos_indv(motor_id, move_position)
#             time.sleep(0.1)
#         angle = []
#         for j in range(10):
#             frames = pipe.wait_for_frames()
#             c = frames.get_color_frame()
#             if not c:
#                 continue

#             frame = np.asanyarray(c.get_data())
#             gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

#             corners, ids, rejected = detect_markers(detector, gray)

#             if ids is not None and len(ids) > 0:
#                 rvecs, tvecs = estimate_marker_pose(corners, ids, K, dist, marker_size_m)
#                 # finger tip, subtract x axis
#                 try:
#                     tip_idx = list(ids.flatten()).index(tip_id)
#                     tip_rvec = rvecs[tip_idx][0]
#                     tip_x_axis = get_x_axis(tip_rvec)
                    
#                 # finger mid
#                 try:
#                     mid_idx = list(ids.flatten()).index(mid_id)
#                     mid_rvec = rvecs[mid_idx][0]
#                     mid_x_axis = get_x_axis(mid_rvec)
                    
#                 # finger low
#                 try:
#                     low_idx = list(ids.flatten()).index(low_id)
#                     low_rvec = rvecs[low_idx][0]
#                     low_x_axis = get_x_axis(low_rvec)
                    
#                 # calculate angles
#                 try:
#                     dip_angle = np.arccos(np.clip(np.dot(tip_x_axis, mid_x_axis), -1.0, 1.0)) * (180.0 / np.pi)
#                     pip_angle = np.arccos(np.clip(np.dot(mid_x_axis, low_x_axis), -1.0, 1.0)) * (180.0 / np.pi)
#                     angle.append((dip_angle, pip_angle))
#                 except ValueError:
#                     pass
                
#             time.sleep(0.1)
        
#         # Filter outliers and compute average angles
#         if angle:
#             angles_array = np.array(angle)  # shape: (N, 2) where N <= 10
#             dip_angles = angles_array[:, 0]
#             pip_angles = angles_array[:, 1]
            
#             # Remove outliers using IQR method (Interquartile Range)
#             def filter_outliers(data):
#                 if len(data) < 3:
#                     return data  # not enough data to filter
#                 q1 = np.percentile(data, 25)
#                 q3 = np.percentile(data, 75)
#                 iqr = q3 - q1
#                 lower = q1 - 1.5 * iqr
#                 upper = q3 + 1.5 * iqr
#                 mask = (data >= lower) & (data <= upper)
#                 return data[mask]
            
#             dip_filtered = filter_outliers(dip_angles)
#             pip_filtered = filter_outliers(pip_angles)
            
#             if len(dip_filtered) > 0 and len(pip_filtered) > 0:
#                 dip_avg = np.mean(dip_filtered)
#                 pip_avg = np.mean(pip_filtered)
#                 dip.append(dip_avg)
#                 pip.append(pip_avg)
#                 print(f"Position {i}: DIP={dip_avg:.2f}° (n={len(dip_filtered)}), PIP={pip_avg:.2f}° (n={len(pip_filtered)})")
#             else:
#                 print(f"Position {i}: No valid angles after filtering")
#         else:
#             print(f"Position {i}: No angles detected")
            
#     position = np.array(position)
#     dip = np.array(dip)
#     pip = np.array(pip)

#     pipe.stop()
#     cv2.destroyAllWindows()
    
#     # save results
#     np.savez_compressed("tracking_results.npz", position=position, dip=dip, pip=pip)
#     print("Results saved to tracking_results.npz", pip=pip)
    
# if __name__ == "__main__":
#     main()

#!/usr/bin/env python3
import time
import numpy as np
import cv2
import pyrealsense2 as rs

# These helpers are assumed to exist in your environment:
# - get_color_intrinsics(profile) -> (K, dist)
# - make_aruco_detector(ARUCO_DICT)
# - detect_markers(detector, gray) -> (corners, ids, rejected)
# - estimate_marker_pose(corners, ids, K, dist, marker_size_m) -> (rvecs, tvecs)
from aruco_detect import *

# --------------------------
# Configuration
# --------------------------
# Camera
COLOR_W, COLOR_H, COLOR_FPS = 640, 480, 30
ARUCO_DICT = cv2.aruco.DICT_4X4_50
MARKER_SIZE_M = 0.020  # 20 mm tags

# Marker IDs for finger segments
TIP_ID = 0
MID_ID = 1
LOW_ID = 2

# Motor / motion
MOTOR_ID = 20
PORT = "/dev/ttyUSB0"
BAUD = 2_000_000
START_POS = 2735
END_POS = 1333
NUM_POSITIONS = 10          # discrete points along the stroke
CYCLES = 500                  # forward+reverse pairs (total passes = 2*CYCLES)
SAMPLES_PER_POS = 10        # frames to sample at each stop
SETTLE_PER_POS = 0.5       # s to wait after move (when not DRY)
DRY_RUN = False              # if True, do not open or move the motor

SAVE_PATH = "tracking_results_up_spring.npz"

# --------------------------
# Optional Dynamixel client import
# --------------------------
DxlClient = None
if not DRY_RUN:
    try:
        # Replace with your actual client module/class
        from ruka_hand.utils.dynamixel_util import DynamixelClient as DxlClient
    except Exception as e:
        raise RuntimeError(
            f"DRY_RUN is False but Dynamixel client couldn't be imported: {e}"
        )

# --------------------------
# Utilities
# --------------------------
def get_x_axis(rvec):
    """Return unit x-axis of the tag (local frame) expressed in camera frame."""
    R, _ = cv2.Rodrigues(rvec)
    x = R @ np.array([1.0, 0.0, 0.0])
    n = np.linalg.norm(x)
    if n == 0:
        return None
    return x / n

def iqr_filter(data):
    """Return values within 1.5*IQR of Q1..Q3; pass-through if <3 samples."""
    data = np.asarray(data)
    if data.size < 3:
        return data
    q1 = np.percentile(data, 25)
    q3 = np.percentile(data, 75)
    iqr = q3 - q1
    lower = q1 - 1.5 * iqr
    upper = q3 + 1.5 * iqr
    return data[(data >= lower) & (data <= upper)]

def pingpong_indices(n: int, cycles: int):
    """
    Yield (idx, dir_flag) where dir_flag=0 for forward, 1 for reverse.
    Sequence: 0..N-1, N-2..1, 0..N-1, ...
    """
    if n < 2:
        raise ValueError("num_positions must be >= 2.")
    fwd = list(range(0, n))          # include endpoints
    rev = list(range(n - 2, 0, -1))  # exclude endpoints
    for _ in range(cycles):
        for i in fwd:
            yield i, 0
        for i in rev:
            yield i, 1

def finite_stats(arr):
    a = np.asarray(arr)
    a = a[np.isfinite(a)]
    if a.size == 0:
        return np.nan, np.nan
    return float(np.nanmean(np.abs(a))), float(np.nanmax(np.abs(a)))

# --------------------------
# Setup helpers
# --------------------------
def activate_camera():
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, COLOR_W, COLOR_H, rs.format.bgr8, COLOR_FPS)
    profile = pipe.start(cfg)
    print(f"Streaming color {COLOR_W}x{COLOR_H}@{COLOR_FPS}…")
    K, dist = get_color_intrinsics(profile)
    detector = make_aruco_detector(ARUCO_DICT)
    # let auto-exposure stabilize a moment
    time.sleep(0.5)
    return pipe, detector, K, dist

def motor_setup():
    if DRY_RUN:
        return None
    client = DxlClient([MOTOR_ID], port=PORT)
    client.connect()
    print("Enabling torque...")
    client.set_torque_enabled(True)
    time.sleep(0.5)
    return client

# --------------------------
# Main
# --------------------------
def main():
    start_time = time.strftime("%Y-%m-%d %H:%M:%S")
    print(f"\n===== Experiment started at {start_time} =====\n")
    pipe, detector, K, dist = activate_camera()
    client = motor_setup()

    positions_cmd = np.linspace(START_POS, END_POS, NUM_POSITIONS)

    # Logs
    logged_idx = []
    logged_dir = []   # 0=fwd, 1=rev
    logged_cmd = []
    dip_series = []
    pip_series = []

    try:
        step_counter = 0
        for idx, dflag in pingpong_indices(NUM_POSITIONS, CYCLES):
            step_counter += 1
            target = float(positions_cmd[idx])
            direction = "FWD" if dflag == 0 else "REV"
            print(f"[{step_counter}] {direction} -> idx {idx}  target {target:.1f}")

            if DRY_RUN:
                print(f"DRY RUN: would set motor {MOTOR_ID} -> {target:.1f}")
            else:
                client.set_pos_indv(MOTOR_ID, int(round(target)))
                time.sleep(SETTLE_PER_POS)

            # Collect frames/angles at this stop
            pairs = []
            for _ in range(SAMPLES_PER_POS):
                frames = pipe.wait_for_frames()
                c = frames.get_color_frame()
                if not c:
                    continue

                frame = np.asanyarray(c.get_data())
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                corners, ids, _rej = detect_markers(detector, gray)
                if ids is None or len(ids) == 0:
                    time.sleep(0.02)
                    continue

                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, MARKER_SIZE_M, K, dist)
                
                # Build id -> rvec
                id_list = ids.flatten().tolist()
                id_to_rvec = {
                    m_id: (rvecs[k][0] if rvecs[k].ndim == 2 else rvecs[k])
                    for k, m_id in enumerate(id_list)
                }

                # Require all three tags
                required = (TIP_ID, MID_ID, LOW_ID)
                if not all(rid in id_to_rvec for rid in required):
                    time.sleep(0.02)
                    continue

                tip_x = get_x_axis(id_to_rvec[TIP_ID])
                mid_x = get_x_axis(id_to_rvec[MID_ID])
                low_x = get_x_axis(id_to_rvec[LOW_ID])
                if tip_x is None or mid_x is None or low_x is None:
                    time.sleep(0.02)
                    continue

                # Optional: enforce consistent axis sign to avoid 180° flips
                # Align mid_x to previous mid_x if you cache it; omitted for brevity.

                dip_angle = np.degrees(np.arccos(np.clip(np.dot(tip_x, mid_x), -1.0, 1.0)))
                pip_angle = np.degrees(np.arccos(np.clip(np.dot(mid_x, low_x), -1.0, 1.0)))
                pairs.append((dip_angle, pip_angle))

                time.sleep(0.02)  # vary frames slightly

            # Aggregate with outlier rejection
            if len(pairs) == 0:
                print(f"idx {idx} ({direction}): No angles detected")
                continue

            arr = np.asarray(pairs)
            dip_f = iqr_filter(arr[:, 0])
            pip_f = iqr_filter(arr[:, 1])
            if dip_f.size == 0 or pip_f.size == 0:
                print(f"idx {idx} ({direction}): No valid angles after filtering")
                continue

            dip_avg = float(np.mean(dip_f))
            pip_avg = float(np.mean(pip_f))

            logged_idx.append(idx)
            logged_dir.append(dflag)
            logged_cmd.append(target)
            dip_series.append(dip_avg)
            pip_series.append(pip_avg)

            print(f"idx {idx} ({direction}): DIP={dip_avg:.2f}° (n={dip_f.size}), "
                  f"PIP={pip_avg:.2f}° (n={pip_f.size})")

            time.sleep(0.02)

    finally:
        # Cleanup
        try:
            pipe.stop()
        except Exception:
            pass
        cv2.destroyAllWindows()
        if client is not None:
            try:
                client.set_torque_enabled(False)
            except Exception:
                pass

    # Convert logs to arrays
    logged_idx = np.asarray(logged_idx, dtype=int)
    logged_dir = np.asarray(logged_dir, dtype=int)
    logged_cmd = np.asarray(logged_cmd, dtype=float)
    dip_series = np.asarray(dip_series, dtype=float)
    pip_series = np.asarray(pip_series, dtype=float)

    # Hysteresis stats per discrete index
    N = NUM_POSITIONS
    dip_fwd = np.full(N, np.nan)
    dip_rev = np.full(N, np.nan)
    pip_fwd = np.full(N, np.nan)
    pip_rev = np.full(N, np.nan)

    for i in range(N):
        fmask = (logged_idx == i) & (logged_dir == 0)
        rmask = (logged_idx == i) & (logged_dir == 1)
        if np.any(fmask):
            dip_fwd[i] = np.mean(dip_series[fmask])
            pip_fwd[i] = np.mean(pip_series[fmask])
        if np.any(rmask):
            dip_rev[i] = np.mean(dip_series[rmask])
            pip_rev[i] = np.mean(pip_series[rmask])

    dip_hyst = dip_fwd - dip_rev
    pip_hyst = pip_fwd - pip_rev

    dip_mean_abs, dip_max_abs = finite_stats(dip_hyst)
    pip_mean_abs, pip_max_abs = finite_stats(pip_hyst)

    print(f"Hysteresis summary (|FWD-REV|): "
          f"DIP mean={dip_mean_abs:.2f}°, max={dip_max_abs:.2f}°; "
          f"PIP mean={pip_mean_abs:.2f}°, max={pip_max_abs:.2f}°")

    # Save all artifacts
    np.savez_compressed(
        SAVE_PATH,
        positions_cmd=np.asarray(positions_cmd, dtype=float),
        logged_idx=logged_idx,
        logged_dir=logged_dir,
        logged_cmd=logged_cmd,
        dip=dip_series,
        pip=pip_series,
        dip_fwd=dip_fwd,
        dip_rev=dip_rev,
        pip_fwd=pip_fwd,
        pip_rev=pip_rev,
        dip_hyst=dip_hyst,
        pip_hyst=pip_hyst,
        config=np.array([COLOR_W, COLOR_H, COLOR_FPS, START_POS, END_POS,
                         NUM_POSITIONS, CYCLES, SAMPLES_PER_POS, SETTLE_PER_POS, MARKER_SIZE_M], dtype=float)
    )
    print(f"Saved {SAVE_PATH}")
    end_time = time.strftime("%Y-%m-%d %H:%M:%S")
    print(f"\n===== Experiment finished at {end_time} =====\n")

if __name__ == "__main__":
    main()