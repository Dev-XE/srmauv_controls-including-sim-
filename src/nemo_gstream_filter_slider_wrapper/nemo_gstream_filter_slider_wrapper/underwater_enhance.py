"""
underwater_enhance.py  — TAC 2026  shared enhancement module
=============================================================
Imported by both receiver.py (Groov-e cam, port 5000)
         and receiver-1.py (Logitech C270, port 5001).

CAMERA PROBLEMS FIXED:
  1. Curved acrylic blur        -> deblur kernel + unsharp mask
  2. Blue/green color cast      -> gray-world white balance + red boost
  3. Low contrast / haze        -> CLAHE on LAB luminance channel
  4. Backscatter particles      -> median filter
  5. Low brightness underwater  -> gamma correction (LUT, pre-built)
  6. C270 barrel distortion     -> undistort remap (pre-computed)
  7. Motion blur (ROV moving)   -> adaptive sharpening per sharpness score
  8. Turbid harbour water (TAC) -> aggressive CLAHE + dark-channel dehaze

Both cameras share the same algorithmic pipeline but carry
SEPARATE calibration constants so each lens is corrected
independently.  The only toggle at runtime is `tac_mode`
(press 'T' in either receiver window).
"""

import cv2
import numpy as np


# ═══════════════════════════════════════════════════════════════════
#  PER-CAMERA CALIBRATION
#  Replace with values from cv2.calibrateCamera() on a checkerboard.
# ═══════════════════════════════════════════════════════════════════

# ── Camera 0  Groov-e  (receiver.py  port 5000)
CAM0_MATRIX = np.array([
    [600.0,   0.0, 320.0],
    [  0.0, 600.0, 240.0],
    [  0.0,   0.0,   1.0],
], dtype=np.float32)
# k1, k2, p1, p2, k3  — measured for Groov-e lens
CAM0_DIST = np.array([-0.30, 0.10, 0.0, 0.0, 0.0], dtype=np.float32)

# ── Camera 1  Logitech C270  (receiver-1.py  port 5001)
CAM1_MATRIX = np.array([
    [554.256,   0.0,   320.0],
    [  0.0,   554.256, 240.0],
    [  0.0,     0.0,     1.0],
], dtype=np.float32)
# C270 has notable barrel distortion — tighten k1 after real calibration
CAM1_DIST = np.array([-0.38, 0.15, 0.0, 0.0, -0.035], dtype=np.float32)

FRAME_SIZE = (640, 480)   # must match v4l2src width/height in transmitters


# ═══════════════════════════════════════════════════════════════════
#  PRE-COMPUTED LOOKUP TABLES  (built once at import, zero per-frame cost)
# ═══════════════════════════════════════════════════════════════════

def _build_undistort(cam_matrix, dist, size):
    new_mat, _ = cv2.getOptimalNewCameraMatrix(cam_matrix, dist, size, 0)
    m1, m2 = cv2.initUndistortRectifyMap(cam_matrix, dist, None, new_mat, size, cv2.CV_16SC2)
    return m1, m2

def _build_gamma_lut(gamma: float) -> np.ndarray:
    return np.array(
        [(i / 255.0) ** (1.0 / gamma) * 255 for i in range(256)], dtype=np.uint8
    )

# Undistort maps
_MAPS = {
    0: _build_undistort(CAM0_MATRIX, CAM0_DIST, FRAME_SIZE),
    1: _build_undistort(CAM1_MATRIX, CAM1_DIST, FRAME_SIZE),
}

# ── RUNTIME-ADJUSTABLE PARAMETERS (modified via trackbars in receiver)
# Default values; actual values set by slider callbacks
_runtime_params = {
    0: {"gamma": 1.5, "red_gain": 1.4, "clahe_clip": 3.0, "deblur_s": 0.55, "brightness": 0},
    1: {"gamma": 1.5, "red_gain": 1.6, "clahe_clip": 3.0, "deblur_s": 0.75, "brightness": 0},
}

def get_runtime_params(cam_id: int):
    """Return current runtime-adjustable parameters for a camera."""
    return _runtime_params.get(cam_id, _runtime_params[0])

def set_runtime_param(cam_id: int, key: str, value: float):
    """Set a runtime parameter for a camera."""
    if cam_id in _runtime_params and key in _runtime_params[cam_id]:
        _runtime_params[cam_id][key] = value

# CLAHE objects — normal and TAC-aggressive
_CLAHE_NORM = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
_CLAHE_TAC  = cv2.createCLAHE(clipLimit=5.0, tileGridSize=(4, 4))

# Camera matrices exposed for pose estimation in receivers
CAMERA_MATRICES = {0: CAM0_MATRIX, 1: CAM1_MATRIX}
DIST_COEFFS     = {0: CAM0_DIST,   1: CAM1_DIST}


# ═══════════════════════════════════════════════════════════════════
#  INDIVIDUAL ENHANCEMENT STEPS
# ═══════════════════════════════════════════════════════════════════

def _undistort(frame: np.ndarray, cam_id: int) -> np.ndarray:
    """Fix #6 — barrel distortion using pre-computed remap."""
    m1, m2 = _MAPS[cam_id]
    return cv2.remap(frame, m1, m2, cv2.INTER_LINEAR)


def _remove_backscatter(frame: np.ndarray) -> np.ndarray:
    """Fix #4 — median filter removes bright particle blobs."""
    return cv2.medianBlur(frame, 3)


def _white_balance_red_boost(frame: np.ndarray, red_gain: float) -> np.ndarray:
    """Fix #2 — gray-world white balance then boost red channel."""
    f = frame.astype(np.float32)
    b_m, g_m, r_m = np.mean(f[:,:,0]), np.mean(f[:,:,1]), np.mean(f[:,:,2])
    gray = (b_m + g_m + r_m) / 3.0
    f[:,:,0] = np.clip(f[:,:,0] * (gray / (b_m + 1e-6)), 0, 255)
    f[:,:,1] = np.clip(f[:,:,1] * (gray / (g_m + 1e-6)), 0, 255)
    f[:,:,2] = np.clip(f[:,:,2] * (gray / (r_m + 1e-6)) * red_gain, 0, 255)
    return f.astype(np.uint8)


def _gamma(frame: np.ndarray, gamma: float) -> np.ndarray:
    """Fix #5 — brightness lift via gamma LUT. gamma > 1 brightens, < 1 darkens."""
    lut = np.array([(i / 255.0) ** (1.0 / gamma) * 255 for i in range(256)], dtype=np.uint8)
    return cv2.LUT(frame, lut)


def _clahe_lab(frame: np.ndarray, tac: bool) -> np.ndarray:
    """Fix #3 & #8 — CLAHE on L channel only (preserves hue)."""
    clahe = _CLAHE_TAC if tac else _CLAHE_NORM
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    return cv2.cvtColor(cv2.merge([clahe.apply(l), a, b]), cv2.COLOR_LAB2BGR)


def _deblur_acrylic(frame: np.ndarray, strength: float) -> np.ndarray:
    """Fix #1 — Laplacian-based deblur kernel for curved acrylic housing."""
    s = strength
    k = np.array([[0, -s, 0], [-s, 1 + 4*s, -s], [0, -s, 0]], dtype=np.float32)
    return np.clip(cv2.filter2D(frame, -1, k), 0, 255).astype(np.uint8)


def _adaptive_sharpen(frame: np.ndarray) -> np.ndarray:
    """Fix #7 — sharpening strength scaled to measured frame sharpness."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    score = cv2.Laplacian(gray, cv2.CV_64F).var()
    if score > 350:          # already sharp — leave alone
        return frame
    elif score > 100:
        strength = 1.3       # mild motion blur
    else:
        strength = 2.0       # heavy blur / ROV moving fast

    blurred = cv2.GaussianBlur(frame, (3, 3), 0)
    return np.clip(
        cv2.addWeighted(frame, 1.0 + strength, blurred, -strength, 0), 0, 255
    ).astype(np.uint8)


def _adjust_brightness(frame: np.ndarray, delta: int) -> np.ndarray:
    """Adjust brightness by a delta value (-255 to +255)."""
    if delta == 0:
        return frame
    return np.clip(frame.astype(np.int16) + delta, 0, 255).astype(np.uint8)


def _dehaze(frame: np.ndarray, omega: float = 0.80) -> np.ndarray:
    """Fix #8 TAC extra — dark-channel prior dehaze for turbid harbour water."""
    img = frame.astype(np.float32) / 255.0
    patch = 7
    dark = cv2.erode(np.min(img, axis=2), np.ones((patch, patch), np.uint8))
    flat = dark.flatten()
    n = max(1, int(flat.size * 0.001))
    idx = np.argpartition(flat, -n)[-n:]
    A = np.clip(np.max(img.reshape(-1, 3)[idx], axis=0), 0.5, 1.0)
    norm_min = cv2.erode(
        np.min(img / A[np.newaxis, np.newaxis, :], axis=2),
        np.ones((patch, patch), np.uint8)
    )
    t = np.clip(1.0 - omega * norm_min, 0.15, 1.0)
    t3 = np.stack([t]*3, axis=2)
    J = np.clip((img - A) / t3 + A, 0, 1)
    return (J * 255).astype(np.uint8)


# ═══════════════════════════════════════════════════════════════════
#  MASTER PIPELINE  — call this once per frame in each receiver
# ═══════════════════════════════════════════════════════════════════

def enhance(frame: np.ndarray, cam_id: int, tac_mode: bool = False) -> np.ndarray:
    """
    Full underwater enhancement pipeline for one camera.
    Uses runtime-adjustable parameters from trackbars.

    Parameters
    ----------
    frame    : raw BGR frame straight from GStreamer appsink
    cam_id   : 0 = Groov-e (port 5000),  1 = C270 (port 5001)
    tac_mode : True = turbid harbour mode (aggressive CLAHE + dehaze)

    Returns
    -------
    Enhanced BGR frame, same shape as input.

    Pipeline order (matters):
      undistort → backscatter → brightness → white-balance+red → gamma →
      CLAHE → [dehaze] → deblur-acrylic → adaptive-sharpen
    """
    p = get_runtime_params(cam_id)

    out = _undistort(frame, cam_id)
    out = _remove_backscatter(out)
    out = _adjust_brightness(out, int(p["brightness"]))
    out = _white_balance_red_boost(out, p["red_gain"])
    out = _gamma(out, p["gamma"])

    # CLAHE with runtime-adjustable clip limit
    clahe = cv2.createCLAHE(clipLimit=p["clahe_clip"], tileGridSize=(8, 8))
    lab = cv2.cvtColor(out, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    out = cv2.cvtColor(cv2.merge([clahe.apply(l), a, b]), cv2.COLOR_LAB2BGR)

    if tac_mode:
        out = _dehaze(out)
        # Second CLAHE pass in TAC mode
        clahe_tac = cv2.createCLAHE(clipLimit=p["clahe_clip"] + 2.0, tileGridSize=(4, 4))
        lab = cv2.cvtColor(out, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        out = cv2.cvtColor(cv2.merge([clahe_tac.apply(l), a, b]), cv2.COLOR_LAB2BGR)

    out = _deblur_acrylic(out, p["deblur_s"])
    out = _adaptive_sharpen(out)
    return out


def enhance_for_aruco(frame: np.ndarray, cam_id: int, tac_mode: bool = False) -> np.ndarray:
    """
    Same pipeline with stronger CLAHE + deblur for ArUco detection branch.
    Uses runtime-adjustable parameters. Returns grayscale ready for detectMarkers().
    """
    p = get_runtime_params(cam_id)

    out = _undistort(frame, cam_id)
    out = _remove_backscatter(out)
    out = _adjust_brightness(out, int(p["brightness"]))
    out = _white_balance_red_boost(out, p["red_gain"])
    out = _gamma(out, p["gamma"])

    # Stronger CLAHE for marker detection
    clip_limit = (p["clahe_clip"] + 2.0) if tac_mode else (p["clahe_clip"] + 1.0)
    clahe_a = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=(4, 4))
    lab = cv2.cvtColor(out, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    out = cv2.cvtColor(cv2.merge([clahe_a.apply(l), a, b]), cv2.COLOR_LAB2BGR)

    if tac_mode:
        out = _dehaze(out)

    # Stronger deblur for marker edges
    s = p["deblur_s"] + 0.15
    k = np.array([[0, -s, 0], [-s, 1+4*s, -s], [0, -s, 0]], dtype=np.float32)
    out = np.clip(cv2.filter2D(out, -1, k), 0, 255).astype(np.uint8)

    gray = cv2.cvtColor(out, cv2.COLOR_BGR2GRAY)
    # Adaptive sharpen on gray
    score = cv2.Laplacian(gray, cv2.CV_64F).var()
    strength = 2.5 if score < 80 else (1.5 if score < 300 else 0.8)
    blurred = cv2.GaussianBlur(gray, (3, 3), 0)
    gray = np.clip(
        cv2.addWeighted(gray, 1.0 + strength, blurred, -strength, 0), 0, 255
    ).astype(np.uint8)
    return gray