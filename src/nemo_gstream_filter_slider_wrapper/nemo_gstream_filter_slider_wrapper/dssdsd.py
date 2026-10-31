#!/usr/bin/env python3
"""
nemo_perception.py — Ultra-Optimized Pi 4 Vision Node
Replaces: receiver.py, receiver1.py, Tac2026.py, transmitters, and underwater_enhance.py
"""

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import threading
import time

# ─────────────────────────────────────────────
#  CONFIG & CONSTANTS
# ─────────────────────────────────────────────
CAM0_DEV = "/dev/video4"  # Forward Camera
CAM1_DEV = "/dev/video2"  # Downward Camera

# Hard-limit resolution to reduce CPU load by 75% compared to 640x480
PROCESS_W, PROCESS_H = 320, 240
FPS_LIMIT = 10

# ArUco Configuration
ARUCO_DICTS = [cv2.aruco.DICT_4X4_100, cv2.aruco.DICT_5X5_100, cv2.aruco.DICT_6X6_100, cv2.aruco.DICT_7X7_100]
PIPE_HSV_RANGES = [
    (np.array([18, 90, 70], dtype=np.uint8), np.array([40, 255, 255], dtype=np.uint8)),
    (np.array([ 5, 100, 60], dtype=np.uint8), np.array([18, 255, 255], dtype=np.uint8)),
]
PIPE_MIN_AREA = 200  # Scaled down for 320x240

# ─────────────────────────────────────────────
#  PRE-INITIALIZE DETECTORS (OpenCV < 4.7 & 4.7+ Compatible)
# ─────────────────────────────────────────────
aruco_dicts = [cv2.aruco.getPredefinedDictionary(d) for d in ARUCO_DICTS]

# Handle both OpenCV versions cleanly
if hasattr(cv2.aruco, "DetectorParameters_create"):
    aruco_params = cv2.aruco.DetectorParameters_create()
else:
    aruco_params = cv2.aruco.DetectorParameters()

aruco_params.minMarkerPerimeterRate = 0.05
aruco_params.maxMarkerPerimeterRate = 4.0

if hasattr(cv2.aruco, "CORNER_REFINE_NONE"):
    aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
    
# ─────────────────────────────────────────────
#  PIPELINE DETECTION ALGORITHM
# ─────────────────────────────────────────────
def detect_pipeline(hsv_frame):
    mask = np.zeros(hsv_frame.shape[:2], dtype=np.uint8)
    for lo, hi in PIPE_HSV_RANGES:
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv_frame, lo, hi))

    # Reduced morphological iterations to save CPU
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best_cnt, best_rect, best_score = None, None, -1
    for c in cnts:
        area = cv2.contourArea(c)
        if area < PIPE_MIN_AREA: continue
        if area > best_score:
            best_score = area
            best_cnt = c
            best_rect = cv2.minAreaRect(c)

    if best_cnt is not None:
        rw, rh = best_rect[1]
        angle = best_rect[2] + (90 if rw > rh else 0)
        M = cv2.moments(best_cnt)
        if M["m00"] > 0:
            return True, angle, int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
    return False, 0.0, 0, 0

# ─────────────────────────────────────────────
#  CAMERA THREAD
# ─────────────────────────────────────────────
class CameraWorker(threading.Thread):
    def __init__(self, cam_id, device_path, node):
        super().__init__(daemon=True)
        self.cam_id = cam_id
        self.node = node
        # Read directly via V4L2, bypassing GStreamer completely
        self.cap = cv2.VideoCapture(device_path, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, FPS_LIMIT)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        def run(self):
            fps_counter = 0
            fps_timer = time.time()
            measured_fps = 0.0

            while rclpy.ok():
                loop_start = time.time()
                ret, frame = self.cap.read()
                if not ret:
                    time.sleep(0.01)
                    continue
            
                # Immediately downscale to process resolution
                small_frame = cv2.resize(frame, (PROCESS_W, PROCESS_H))
            
                # 1. Pipeline Detection
                hsv = cv2.cvtColor(small_frame, cv2.COLOR_BGR2HSV)
                pipe_found, angle, cx, cy = detect_pipeline(hsv)

                # 2. ArUco Detection
                gray = cv2.cvtColor(small_frame, cv2.COLOR_BGR2GRAY)
                for detector in detectors:
                    corners, ids, _ = detector.detectMarkers(gray)

                # FPS and Latency Calculation
                fps_counter += 1
                loop_latency_ms = (time.time() - loop_start) * 1000.0

                if (time.time() - fps_timer) >= 1.0:
                    measured_fps = fps_counter / (time.time() - fps_timer)
                    fps_counter = 0
                    fps_timer = time.time()
                    self.node.get_logger().info(
                        f"[Cam{self.cam_id} Performance] FPS: {measured_fps:.1f} | Loop Latency: {loop_latency_ms:.1f} ms"
                    )
    def __init__(self):
        super().__init__('nemo_perception')
        self.get_logger().info("Starting optimized dual-camera vision stack...")

        self.cam0_thread = CameraWorker(0, CAM0_DEV, self)
        self.cam1_thread = CameraWorker(1, CAM1_DEV, self)

        self.cam0_thread.start()
        #self.cam1_thread.start()

def main(args=None):
    rclpy.init(args=args)
    node = NemoPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()