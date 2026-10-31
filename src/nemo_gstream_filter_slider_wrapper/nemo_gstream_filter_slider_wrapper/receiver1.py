#!/usr/bin/env python3
"""
receiver.py  —  TAC 2026  |  Camera 1  Logitech Downward  (port 5001)
=============================================================
Re-written with DearPyGui and Real-Time Dynamic Auto-Tuning.
Tailored for 2D USB Webcams.
"""

import threading
import queue
import time
import numpy as np
import cv2
import cv2.aruco as aruco
import logging
from datetime import datetime

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

import dearpygui.dearpygui as dpg

# ── shared enhancement module
from .underwater_enhance import enhance, enhance_for_aruco, set_runtime_param

# Changed to 1 to bypass RealSense-specific distortion matrices in the enhance module
CAM_ID = 1
VIDEO_W, VIDEO_H = 640, 480

# ─────────────────────────────────────────────
#  PALETTE & TAG DEFINITIONS
# ─────────────────────────────────────────────
PIPELINE_TAGS = {
    56: {"order": 1, "name": "PIPELINE ENTRY", "msg": "Pipeline detected — move to next section"},
    5:  {"order": 2, "name": "SECTION 2",      "msg": "Section 2 reached — proceed forward"},
    20: {"order": 3, "name": "SECTION 3",      "msg": "Section 3 reached — scan surroundings"},
    32: {"order": 4, "name": "PIPELINE EXIT",  "msg": "Pipeline exit — inspection complete"},
}

def get_tag_role(aruco_id):
    if aruco_id in PIPELINE_TAGS:
        return "PIPELINE", PIPELINE_TAGS[aruco_id]
    return "DOCKING", None

# ─────────────────────────────────────────────
#  ARUCO SETUP
# ─────────────────────────────────────────────
ARUCO_DICTS         = [aruco.DICT_4X4_1000, aruco.DICT_5X5_1000,
                       aruco.DICT_6X6_1000, aruco.DICT_7X7_1000]
ARUCO_DICT_NAMES    = ["4x4", "5x5", "6x6", "7x7"]
ARUCO_DICT_PRIORITY = [1, 2, 3, 4]

aruco_params = aruco.DetectorParameters_create()
aruco_params.minMarkerPerimeterRate       = 0.03
aruco_params.maxMarkerPerimeterRate       = 10.0
aruco_params.adaptiveThreshWinSizeMin     = 3
aruco_params.adaptiveThreshWinSizeMax     = 53
aruco_params.adaptiveThreshWinSizeStep    = 4
aruco_params.adaptiveThreshConstant       = 7
aruco_params.minCornerDistanceRate        = 0.05
aruco_params.polygonalApproxAccuracyRate  = 0.03
aruco_params.errorCorrectionRate          = 0.6
aruco_params.minMarkerDistanceRate        = 0.05
aruco_params.cornerRefinementMethod       = aruco.CORNER_REFINE_SUBPIX

aruco_dicts = [aruco.getPredefinedDictionary(d) for d in ARUCO_DICTS]

# ─────────────────────────────────────────────
#  CONSTANTS
# ─────────────────────────────────────────────
CENTER_TOLERANCE      = 60
MAX_LOST              = 8
NIGHT_THRESHOLD       = 80
MIN_MARKER_AREA       = 500
MARKER_RESULT_TTL     = 0.10
SPATIAL_MERGE_DIST    = 80
DETECT_SCALE          = 0.5
PIPELINE_MSG_COOLDOWN = 3.0

PIPE_HSV_RANGES = [
    (np.array([18, 90, 70],  dtype=np.uint8), np.array([40, 255, 255], dtype=np.uint8)),
    (np.array([ 5, 100, 60], dtype=np.uint8), np.array([18, 255, 255], dtype=np.uint8)),
]
PIPE_MIN_AREA       = 800
PIPE_MIN_ASPECT     = 2.2
PIPE_MIN_FILL_RATIO = 0.45
PIPE_MIN_SOLIDITY   = 0.65
PIPE_MIN_LENGTH     = 80

# ─────────────────────────────────────────────
#  STATE & GLOBALS
# ─────────────────────────────────────────────
_master_filter = [False]
_auto_tune     = [False]
_tac_flag      = [False]

_slider_gamma      = [150]
_slider_red_gain   = [140]
_slider_clahe      = [30]
_slider_brightness = [100]
_slider_deblur     = [55]

class MissionState:
    def __init__(self):
        self.pipeline_found, self.pipeline_angle = False, 0.0
        self.marker_sequence, self.marker_count = [], 0
        self.pinger_side, self._seen_ids_set = "unknown", set()
    def register_tag(self, aruco_id, role):
        if aruco_id not in self._seen_ids_set:
            self._seen_ids_set.add(aruco_id)
            self.marker_sequence.append(aruco_id)
            self.marker_count = len(self.marker_sequence)
            if role == "PIPELINE":
                self.pipeline_found = True
    def update_pipeline_angle(self, angle_deg): self.pipeline_angle = angle_deg
    def update_pinger_side(self, side): self.pinger_side = side

mission = MissionState()
pipeline_last_msg_ts = {}

# ─────────────────────────────────────────────
#  HELPERS & LOGIC
# ─────────────────────────────────────────────
def force_put(q, item):
    while True:
        try: q.get_nowait()
        except queue.Empty: break
    try: q.put_nowait(item)
    except queue.Full: pass

def is_valid_marker(pts, scale=1.0):
    area = cv2.contourArea(pts.astype(np.float32))
    if area < MIN_MARKER_AREA * (scale ** 2): return False
    sides = [np.linalg.norm(pts[(i + 1) % 4] - pts[i]) for i in range(4)]
    mean = np.mean(sides)
    if mean == 0 or np.any(np.abs(sides - mean) / mean > 0.40): return False
    rect = cv2.minAreaRect(pts.astype(np.float32))
    rw, rh = rect[1]
    if min(rw, rh) == 0 or max(rw, rh) / min(rw, rh) > 1.6: return False
    return True

def contour_score_for_pipe(c):
    area = cv2.contourArea(c)
    if area < PIPE_MIN_AREA: return -1, None
    rect = cv2.minAreaRect(c)
    rw, rh = rect[1]
    if rw <= 1 or rh <= 1: return -1, None
    long_side, short_side = max(rw, rh), min(rw, rh)
    if short_side <= 1 or (long_side / short_side) < PIPE_MIN_ASPECT or long_side < PIPE_MIN_LENGTH:
        return -1, None
    if (area / (rw * rh)) < PIPE_MIN_FILL_RATIO: return -1, None
    hull_area = cv2.contourArea(cv2.convexHull(c))
    if hull_area <= 1 or (area / hull_area) < PIPE_MIN_SOLIDITY: return -1, None
    return (area * 0.002 + (long_side / short_side) * 120 +
            (area / (rw * rh)) * 200 + (area / hull_area) * 150 + long_side * 0.8), rect

def compute_pipeline_mask(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lo, hi in PIPE_HSV_RANGES:
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lo, hi))
    mask = cv2.GaussianBlur(mask, (5, 5), 0)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT,    (11, 11)))
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best_cnt, best_rect, best_score = None, None, -1
    for c in cnts:
        score, rect = contour_score_for_pipe(c)
        if score > best_score:
            best_score, best_cnt, best_rect = score, c, rect

    pipe_found, angle, cx_pipe, cy_pipe = False, 0.0, 0, 0
    if best_cnt is not None:
        pipe_found = True
        rw, rh = best_rect[1]
        angle = best_rect[2] + (90 if rw > rh else 0)
        M = cv2.moments(best_cnt)
        if M["m00"] > 0:
            cx_pipe, cy_pipe = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
    return pipe_found, angle, cx_pipe, cy_pipe

# ─────────────────────────────────────────────
#  AUTO-TUNE ALGORITHM
# ─────────────────────────────────────────────
def auto_tune_filters(raw_frame):
    small = cv2.resize(raw_frame, (160, 120))
    b, g, r = cv2.split(small)
    gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)

    mean_y = np.mean(gray)
    std_y = np.std(gray)
    mean_r = np.mean(r)
    mean_bg = (np.mean(b) + np.mean(g)) / 2.0 + 1e-5

    t_gamma = 150
    t_bright = 100
    if mean_y < 90:
        t_gamma = 180 + int(90 - mean_y)
        t_bright = 100 + int((90 - mean_y) / 2)
    elif mean_y > 150:
        t_gamma = 100
        t_bright = 100 - int((mean_y - 150) / 2)

    ratio = mean_r / mean_bg
    t_red = 140
    if ratio < 0.75:
        t_red = 140 + int((0.75 - ratio) * 250)

    t_clahe = 30
    if std_y < 45:
        t_clahe = 30 + int((45 - std_y) * 1.5)

    lap_var = cv2.Laplacian(gray, cv2.CV_64F).var()
    t_deblur = 55
    if lap_var < 150:
        t_deblur = 55 + int((150 - lap_var) / 3)

    t_gamma = max(50, min(300, t_gamma))
    t_bright = max(0, min(200, t_bright))
    t_red = max(50, min(300, t_red))
    t_clahe = max(0, min(100, t_clahe))
    t_deblur = max(0, min(200, t_deblur))

    alpha = 0.05
    _slider_gamma[0]      = int(alpha * t_gamma + (1 - alpha) * _slider_gamma[0])
    _slider_brightness[0] = int(alpha * t_bright + (1 - alpha) * _slider_brightness[0])
    _slider_red_gain[0]   = int(alpha * t_red + (1 - alpha) * _slider_red_gain[0])
    _slider_clahe[0]      = int(alpha * t_clahe + (1 - alpha) * _slider_clahe[0])
    _slider_deblur[0]     = int(alpha * t_deblur + (1 - alpha) * _slider_deblur[0])

    dpg.set_value("s_gamma", _slider_gamma[0])
    dpg.set_value("s_red", _slider_red_gain[0])
    dpg.set_value("s_clahe", _slider_clahe[0])
    dpg.set_value("s_bright", _slider_brightness[0])
    dpg.set_value("s_deblur", _slider_deblur[0])

    set_runtime_param(CAM_ID, "gamma", _slider_gamma[0] / 100.0)
    set_runtime_param(CAM_ID, "red_gain", _slider_red_gain[0] / 100.0)
    set_runtime_param(CAM_ID, "clahe_clip", _slider_clahe[0] / 10.0)
    set_runtime_param(CAM_ID, "brightness", _slider_brightness[0] - 100)
    set_runtime_param(CAM_ID, "deblur_s", _slider_deblur[0] / 100.0)

# ─────────────────────────────────────────────
#  ARUCO THREAD WORKERS
# ─────────────────────────────────────────────
aruco_input_queues = [queue.Queue(maxsize=2) for _ in ARUCO_DICTS]
aruco_result_queue = queue.Queue(maxsize=8)

def make_aruco_worker(det_idx):
    aruco_dict  = aruco_dicts[det_idx]
    priority    = ARUCO_DICT_PRIORITY[det_idx]
    dict_name   = ARUCO_DICT_NAMES[det_idx]
    in_q        = aruco_input_queues[det_idx]

    def worker():
        while True:
            try:
                enh_gray_s, scale = in_q.get(timeout=1.0)
            except queue.Empty:
                continue

            corners, ids, _ = aruco.detectMarkers(enh_gray_s, aruco_dict, parameters=aruco_params)
            if ids is None or len(ids) == 0:
                continue

            ts = time.monotonic()
            for i in range(len(ids)):
                c = corners[i][0]
                if not is_valid_marker(c, scale): continue
                tx = int(c[:, 0].mean() / scale)
                ty = int(c[:, 1].mean() / scale)
                aruco_id = int(ids[i][0])
                role, _ = get_tag_role(aruco_id)
                try:
                    aruco_result_queue.put_nowait(({
                        "priority": priority,
                        "area":     cv2.contourArea(c),
                        "tx": tx, "ty": ty,
                        "label":    f"ArUco {dict_name}:{aruco_id}",
                        "aruco_id": aruco_id,
                        "role":     role,
                        "corner":   (corners[i] / scale).astype(np.float32),
                    }, ts))
                except queue.Full:
                    pass
    return worker

# ─────────────────────────────────────────────
#  ROS 2 NODE
# ─────────────────────────────────────────────
class GstRxNode(Node):
    def __init__(self):
        super().__init__("gst_rx_node_cam1")
        self.declare_parameter("port",       5001)
        self.declare_parameter("latency_ms", 10)
        self.declare_parameter("sync",       False)
        self.bridge = CvBridge()

    def get_pipeline_string(self) -> str:
        port      = int(self.get_parameter("port").value)
        latency   = int(self.get_parameter("latency_ms").value)
        sync_str  = "true" if bool(self.get_parameter("sync").value) else "false"
        return (f'udpsrc port={port} '
                f'caps="application/x-rtp, media=video, encoding-name=H264, payload=96" ! '
                f'rtpjitterbuffer latency={latency} ! '
                f'rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! '
                f'video/x-raw, format=BGR ! appsink drop=True max-buffers=1 sync={sync_str}')

# ─────────────────────────────────────────────
#  DPG CALLBACKS
# ─────────────────────────────────────────────
def cb_master(s, a):
    _master_filter[0] = a
    if not a:
        _auto_tune[0] = False
        dpg.set_value("cb_auto", False)
        toggle_sliders(True)

def cb_auto(s, a):
    if not _master_filter[0]:
        dpg.set_value(s, False)
        return
    _auto_tune[0] = a
    toggle_sliders(not a)

def toggle_sliders(state):
    dpg.configure_item("s_gamma", enabled=state)
    dpg.configure_item("s_red", enabled=state)
    dpg.configure_item("s_clahe", enabled=state)
    dpg.configure_item("s_bright", enabled=state)
    dpg.configure_item("s_deblur", enabled=state)

def cb_tac(s, a): _tac_flag[0] = a

def cb_gamma(s, a): 
    _slider_gamma[0] = a
    set_runtime_param(CAM_ID, "gamma", a / 100.0)
def cb_red(s, a): 
    _slider_red_gain[0] = a
    set_runtime_param(CAM_ID, "red_gain", a / 100.0)
def cb_clahe(s, a): 
    _slider_clahe[0] = a
    set_runtime_param(CAM_ID, "clahe_clip", a / 10.0)
def cb_bright(s, a): 
    _slider_brightness[0] = a
    set_runtime_param(CAM_ID, "brightness", a - 100)
def cb_deblur(s, a): 
    _slider_deblur[0] = a
    set_runtime_param(CAM_ID, "deblur_s", a / 100.0)

# ─────────────────────────────────────────────
#  MAIN LOOP
# ─────────────────────────────────────────────
def main():
    # --- Setup Logging to Shared File ---
    shared_log_file = "aruco_detections.txt"
    logging.basicConfig(
        filename=shared_log_file,
        level=logging.INFO,
        format="%(asctime)s.%(msecs)03d | %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S"
    )
    logging.info("--- Cam 1 (Logitech) Node Started ---")

    rclpy.init()
    node = GstRxNode()
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    pipeline_str = node.get_pipeline_string()
    cap = cv2.VideoCapture(pipeline_str, cv2.CAP_GSTREAMER)

    for i in range(len(ARUCO_DICTS)):
        threading.Thread(target=make_aruco_worker(i), daemon=True).start()

    # --- Setup DearPyGui ---
    dpg.create_context()
    
    texture_data = np.zeros(VIDEO_W * VIDEO_H * 4, dtype=np.float32)
    with dpg.texture_registry(show=False):
        dpg.add_dynamic_texture(width=VIDEO_W, height=VIDEO_H, default_value=texture_data, tag="video_texture")

    with dpg.window(tag="Primary Window"):
        with dpg.group(horizontal=True):
            with dpg.child_window(width=VIDEO_W + 20, height=VIDEO_H + 20):
                dpg.add_image("video_texture")

            with dpg.child_window(width=-1):
                dpg.add_text("SYSTEM OVERRIDES", color=(180, 160, 100))
                dpg.add_checkbox(label="Master Filter Enabled", default_value=False, callback=cb_master)
                dpg.add_checkbox(label="Auto-Tune Dynamic Filters", tag="cb_auto", default_value=False, callback=cb_auto)
                dpg.add_checkbox(label="TAC Mode (Turbid Harbor Override)", default_value=False, callback=cb_tac)
                dpg.add_separator()
                dpg.add_spacer(height=5)
                dpg.add_text("MANUAL ADJUSTMENTS", color=(100, 160, 180))
                dpg.add_slider_int(tag="s_gamma", label="Gamma", default_value=150, max_value=300, callback=cb_gamma)
                dpg.add_slider_int(tag="s_red", label="Red Gain", default_value=140, max_value=300, callback=cb_red)
                dpg.add_slider_int(tag="s_clahe", label="CLAHE", default_value=30, max_value=100, callback=cb_clahe)
                dpg.add_slider_int(tag="s_bright", label="Brightness", default_value=100, max_value=200, callback=cb_bright)
                dpg.add_slider_int(tag="s_deblur", label="Debblur", default_value=55, max_value=200, callback=cb_deblur)
                dpg.add_separator()
                dpg.add_spacer(height=5)
                dpg.add_text("TELEMETRY", color=(80, 180, 80))
                dpg.add_text("FPS: --", tag="t_fps")
                dpg.add_text("Pipeline: SEARCHING", tag="t_pipe")
                dpg.add_text("Target ID: NONE", tag="t_id")
                dpg.add_text("Target Role: --", tag="t_role")

    dpg.create_viewport(title='TAC 2026 Ground Station - Cam1 Logitech Downward', width=1100, height=540)
    dpg.setup_dearpygui()
    dpg.show_viewport()
    dpg.set_primary_window("Primary Window", True)

    last_marker_result, last_marker_ts = None, 0.0
    prev_time = time.time()
    lost_frames = 0
    prev_detected_id = None

    while dpg.is_dearpygui_running():
        ret, raw_frame = cap.read()
        if not ret:
            dpg.render_dearpygui_frame()
            time.sleep(0.01)
            continue

        now = time.monotonic()
        ct = time.time()
        fps = 1.0 / max(ct - prev_time, 1e-6)
        prev_time = ct

        dpg.set_value("t_fps", f"FPS: {int(fps)}")

        if _master_filter[0] and _auto_tune[0]:
            auto_tune_filters(raw_frame)

        if _master_filter[0]:
            frame = enhance(raw_frame, CAM_ID, tac_mode=_tac_flag[0])
            enh_gray = enhance_for_aruco(raw_frame, CAM_ID, tac_mode=_tac_flag[0])
        else:
            frame = raw_frame.copy()
            enh_gray = cv2.cvtColor(raw_frame, cv2.COLOR_BGR2GRAY)

        pipe_found, pipe_angle, pipe_cx, pipe_cy = compute_pipeline_mask(frame)
        if pipe_found:
            mission.update_pipeline_angle(pipe_angle)
            mission.pipeline_found = True
            cv2.circle(frame, (pipe_cx, pipe_cy), 5, (200, 200, 160), -1)
            dpg.set_value("t_pipe", f"Pipeline: DETECTED ({pipe_angle:.1f}°)")
        else:
            dpg.set_value("t_pipe", "Pipeline: SEARCHING")

        enh_gray_s = cv2.resize(enh_gray, (0, 0), fx=DETECT_SCALE, fy=DETECT_SCALE)
        for q in aruco_input_queues:
            force_put(q, (enh_gray_s, DETECT_SCALE))

        fresh_aruco = []
        while True:
            try:
                r, ts = aruco_result_queue.get_nowait()
                if (now - ts) < MARKER_RESULT_TTL:
                    fresh_aruco.append((r, ts))
            except queue.Empty:
                break

        if fresh_aruco:
            best_r, best_score = None, (-1, -1)
            for r, _ in fresh_aruco:
                if (r["priority"], r["area"]) > best_score:
                    best_score = (r["priority"], r["area"])
                    best_r = r

            if best_r:
                last_marker_result, last_marker_ts = best_r, now
                aid, role = best_r["aruco_id"], best_r["role"]
                
                # Simplified log format (Only logs the ID, no "Acquired" wording)
                if aid != prev_detected_id:
                    logging.info(f"Cam 1 - ID: {aid}")

                mission.register_tag(aid, role)
                prev_detected_id = aid

        if last_marker_result is not None and (now - last_marker_ts) < MARKER_RESULT_TTL:
            r = last_marker_result
            aruco.drawDetectedMarkers(frame, [r["corner"]], np.array([[r["aruco_id"]]]))
            dpg.set_value("t_id", f"Target ID: {r['aruco_id']}")
            dpg.set_value("t_role", f"Target Role: {r['role']}")
            lost_frames = 0
        else:
            lost_frames += 1
            if lost_frames >= MAX_LOST:
                # The "Lost" logging has been completely removed here.
                last_marker_result = None
                prev_detected_id = None
                dpg.set_value("t_id", "Target ID: NONE")
                dpg.set_value("t_role", "Target Role: --")

        frame_rgba = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
        texture_data = np.ascontiguousarray(frame_rgba, dtype=np.float32) / 255.0
        dpg.set_value("video_texture", texture_data.flatten())

        dpg.render_dearpygui_frame()

    cap.release()
    dpg.destroy_context()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()