import cv2
import numpy as np
import pyrealsense2 as rs
import time

def make_aruco_detector(dict_id):
    """
    Returns (kind, dict_or_detector, params_or_None)
    kind = 'legacy' uses detectMarkers(dict, params)
    kind = 'new'    uses ArucoDetector(dict, params).detectMarkers()
    """
    aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
    if hasattr(cv2.aruco, "DetectorParameters_create"):
        params = cv2.aruco.DetectorParameters_create()
        return ("legacy", aruco_dict, params)
    else:
        params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, params)
        return ("new", detector, None)
    
    
def detect_markers(detector, gray):
    kind, obj, params = detector
    if kind == "legacy":
        return cv2.aruco.detectMarkers(gray, obj, parameters=params)
    else:
        return obj.detectMarkers(gray)
    
    
def get_color_intrinsics(profile):
    """Extract camera matrix K and distortion coeffs from RealSense color stream."""
    color_stream = profile.get_stream(rs.stream.color)  # rs.video_stream_profile
    intr = color_stream.as_video_stream_profile().get_intrinsics()
    K = np.array([[intr.fx, 0, intr.ppx],
                  [0, intr.fy, intr.ppy],
                  [0,      0,       1]], dtype=np.float32)
    dist = np.array(intr.coeffs, dtype=np.float32)  # [k1,k2,p1,p2,k3] (+k4..)
    return K, dist


def drawDetectedMarkers(frame, corners, ids, K, dist, base_id, base_size_m, other_size_m):
    for i, mid in enumerate(ids.flatten()):
        marker_length = base_size_m if mid == base_id else other_size_m
        rvecs, tvecs, _obj = cv2.aruco.estimatePoseSingleMarkers(
            [corners[i]], marker_length, K, dist
        )
        rvec, tvec = rvecs[0], tvecs[0]
        cv2.drawFrameAxes(frame, K, dist, rvec, tvec, marker_length)
        tx, ty, tz = tvec.flatten()
        # cv2.putText(frame,
        #             f"ID {int(mid)}  x:{tx:.3f} y:{ty:.3f} z:{tz:.3f} m",
        #             (10, 30 + 24*int(mid % 10)),
        #             cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2, cv2.LINE_AA)
        
        if mid == base_id:
            # --- Draw a custom 3D line in marker's frame (e.g., from center to +X axis) ---
            # Define 3D points in marker coordinate system (origin and 3cm along X)
            line_3d = np.float32([[0,0,0], [-0.029,0.030,-0.015]])
            # Project to 2D image points
            line_2d, _ = cv2.projectPoints(line_3d, rvec, tvec, K, dist)
            p1 = tuple(line_2d[0].ravel().astype(int))
            p2 = tuple(line_2d[1].ravel().astype(int))
            cv2.line(frame, p1, p2, (0,0,255), 2)  # Red line