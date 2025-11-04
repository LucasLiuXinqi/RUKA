import cv2
import numpy as np
import pyrealsense2 as rs
import time
from aruco_detect import*
from calculate_angle import*

# ====== Config (tweak these if you need) ======================================
COLOR_W, COLOR_H, COLOR_FPS = 640, 480, 30
ARUCO_DICT = cv2.aruco.DICT_4X4_1000   # DICT_4X4_50, DICT_5X5_100, etc. 
DRAW_AXIS = True                      # set False to skip drawing axes
base_id = 3
dip_id = 0
pip_id = 1
mcp_id = 2
# ==============================================================================


def main():
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, COLOR_W, COLOR_H, rs.format.bgr8, COLOR_FPS)
    profile = pipe.start(cfg)
    print(f"Streaming color {COLOR_W}x{COLOR_H}@{COLOR_FPS}…  (q=quit, s=snapshot)")

    K, dist = get_color_intrinsics(profile)
    
    detector = make_aruco_detector(ARUCO_DICT)
    
    snap_id = 0
    fps_smooth = None
    t_prev = time.time()


    while True:
        frames = pipe.wait_for_frames()
        c = frames.get_color_frame()
        if not c:
            continue

        frame = np.asanyarray(c.get_data())
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = detect_markers(detector, gray)

        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            if DRAW_AXIS:
                drawDetectedMarkers(frame, corners, ids, K, dist, base_id, base_size_m=0.03, other_size_m=0.014)

        angle_DP, angle_PM, angle_MS, angle_SP, splay_pos, splay_z = calculate_angles(ids, corners, K, dist)

        # Plot the splayed markers only if valid values are returned
        if splay_pos is not None and splay_z is not None:   
            # Project 3D points to 2D image plane
            p_start_2d, _ = cv2.projectPoints(splay_pos, np.zeros(3), np.zeros(3), K, dist)
            p_end_2d, _ = cv2.projectPoints(splay_pos + splay_z * 0.03, np.zeros(3), np.zeros(3), K, dist)

            # Convert to tuple for cv2.line
            p_start = tuple(p_start_2d.ravel().astype(int))
            p_end = tuple(p_end_2d.ravel().astype(int))

            # Draw the line
            cv2.line(frame, p_start, p_end, (0,255,255), 2, cv2.LINE_AA)
            
        # Display angles only if they are valid
        if angle_DP is not None:
            cv2.putText(frame, f"Angle DP: {angle_DP:.1f} deg", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3, cv2.LINE_AA)
        if angle_PM is not None:
            cv2.putText(frame, f"Angle PM: {angle_PM:.1f} deg", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3, cv2.LINE_AA)
        if angle_MS is not None:
            cv2.putText(frame, f"Angle MS: {angle_MS:.1f} deg", (10, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3, cv2.LINE_AA)
        if angle_SP is not None:
            cv2.putText(frame, f"Angle SP: {angle_SP:.1f} deg", (10, 120),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3, cv2.LINE_AA)
            
            

        # FPS
        now = time.time()
        fps = 1.0 / max(1e-6, (now - t_prev))
        t_prev = now
        fps_smooth = fps if fps_smooth is None else (0.9*fps_smooth + 0.1*fps)
        # cv2.putText(frame, f"FPS: {fps_smooth:.1f}", (10, frame.shape[0]-10),
        #             cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2, cv2.LINE_AA)

        cv2.imshow("D405 Color + ArUco", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            name = f"aruco_snap_{snap_id:03d}.png"
            cv2.imwrite(name, frame)
            print(f"Saved {name}")
            snap_id += 1
        
if __name__ == "__main__":
    main()

