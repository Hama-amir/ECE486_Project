"""
step5.py  -  Lab 3 Part 3 / Step 5  (ECE 486)
===============================================
Complete standalone calibration script. Does everything in one run:
  1. Moves the robot through CALIB_TARGETS, detects the ArUco marker in
     each position, and records (robot_pose, camera_detection) pairs.
  2. Fits the camera-to-robot transform AND the marker's local offset from
     the end-effector center jointly (see WHY JOINT FIT below).
  3. Moves the robot through VALIDATION_TARGETS (disjoint from calibration),
     detects the marker, and computes the raw and corrected error.

HOW TO RUN:
  python step5.py
  Add --headless to suppress the live camera preview window.

BEFORE RUNNING - FILL IN THE THREE THINGS BELOW:
  1. CAMERA_MATRIX and DIST_COEFFS from your Lab 2 calibrate_camera.py output
  2. MARKER_SIZE_M: measure the black border of the physical ArUco marker in mm,
     divide by 1000. E.g. a 40mm marker -> 0.040. Do not guess this.
  3. MARKER_ID: the number printed/labelled on your marker (commonly 0 or 1).

NOTE ON BRITTLENESS: once you start collecting calibration data, do not move
the camera, the robot, or the marker mount. Even a small nudge invalidates the
entire calibration. Re-run this script from scratch if anything moves.

WHY JOINT FIT (not just compute_transform.py alone):
  The prof's compute_transform.py fits one fixed rigid transform T between
  camera and robot frames. But T sees the MARKER position, not the
  END-EFFECTOR center -- they differ by a physical offset. Because the
  Dobot's parallelogram linkage keeps the end-effector level regardless of
  J2/J3, that offset is fixed in the end-effector's own frame, which spins
  with J1. A single fixed T cannot represent a term that rotates with J1, so
  the raw error is always dominated by this rotating offset. Fitting T and
  the offset together cuts error by roughly an order of magnitude.
"""

import argparse
import csv
import math
import time

import cv2
import cv2.aruco as aruco
import numpy as np
import DobotDllType as dType


# =============================================================================
# FILL THESE IN BEFORE RUNNING
# =============================================================================

# Your camera matrix from Lab 2's calibrate_camera.py output.
# It looks like [[fx, 0, cx], [0, fy, cy], [0, 0, 1]].
CAMERA_MATRIX = np.array([
    [800.0,   0.0, 400.0],   # <-- REPLACE with your actual values
    [  0.0, 800.0, 300.0],
    [  0.0,   0.0,   1.0],
], dtype=np.float64)

# Distortion coefficients from the same calibration output.
DIST_COEFFS = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)  # <-- REPLACE

# Physical size of the ArUco marker's black border, in METERS.
# Measure it with a ruler -- do not guess.
MARKER_SIZE_M = 0.040    # <-- REPLACE (e.g. 0.040 for a 40mm marker)

# The ID number on the marker attached to the robot.
MARKER_ID = 0            # <-- REPLACE if your marker has a different number

# Camera index. 0 is usually the built-in webcam; 1 or 2 for a USB camera.
CAMERA_INDEX = 0         # <-- ADJUST if the wrong camera opens


# =============================================================================
# Calibration and validation target lists
# =============================================================================
# 12 calibration points: stay inside the reachable workspace and keep the
# marker well within the camera field of view. These use moderate radii and
# a conservative z range that is less likely to cause the marker to leave frame.
CALIB_TARGETS = [
    ("calib_00", 120.00, -90.00, -30), ("calib_01", 140.00, -30.00, -30),
    ("calib_02", 160.00, -60.00, -30), ("calib_03", 160.00,  30.00, -30),
    ("calib_04", 140.00,  60.00, -30), ("calib_05", 120.00,  90.00, -30),
    ("calib_06", 130.00, -70.00, -60), ("calib_07", 150.00, -20.00, -60),
    ("calib_08", 150.00,  20.00, -60), ("calib_09", 130.00,  70.00, -60),
    ("calib_10", 140.00, -50.00, -80), ("calib_11", 140.00,  50.00, -80),
]

# 20 validation points: keep them disjoint from calibration while staying in
# the same safe camera-visible region.
VALIDATION_TARGETS = [
    ("val_00", 110.00, -100.00, -20), ("val_01", 135.00, -40.00, -40),
    ("val_02", 150.00,   0.00, -20), ("val_03", 150.00,   0.00, -70),
    ("val_04", 135.00,  40.00, -40), ("val_05", 110.00, 100.00, -20),
    ("val_06", 125.00, -80.00, -40), ("val_07", 145.00, -10.00, -20),
    ("val_08", 145.00,  10.00, -40), ("val_09", 125.00,  80.00, -20),
    ("val_10", 120.00, -120.00, -50), ("val_11", 140.00, -70.00, -70),
    ("val_12", 140.00,  70.00, -50), ("val_13", 160.00,   0.00, -20),
    ("val_14", 160.00,   0.00, -80), ("val_15", 155.00,  50.00, -50),
    ("val_16", 155.00, -50.00, -50), ("val_17", 130.00, -110.00, -60),
    ("val_18", 150.00,  90.00, -60), ("val_19", 145.00, -90.00, -60),
]


# =============================================================================
# Robot interface  (same pattern as ece486_starter_code.py)
# =============================================================================

HOME_POS = [200, 100, 50]


def init_robot(api):
    com = dType.SearchDobot(api)
    if "COM" not in com[0]:
        print("Robot not found. Exiting."); exit()
    for port in com:
        state = dType.ConnectDobot(api, port, 115200)[0]
        if state == dType.DobotConnect.DobotConnect_NoError:
            print(f"Connected on {port}"); break
    if state != dType.DobotConnect.DobotConnect_NoError:
        print("Cannot connect. Exiting."); exit()
    dType.SetQueuedCmdStopExec(api)
    dType.SetQueuedCmdClear(api)
    dType.SetPTPCommonParams(api, 50, 50, isQueued=1)
    dType.SetHOMEParams(api, *HOME_POS, 0, isQueued=1)
    cmd = dType.SetHOMECmd(api, temp=0, isQueued=1)[0]
    dType.SetQueuedCmdStartExec(api)
    while cmd > dType.GetQueuedCmdCurrentIndex(api)[0]:
        dType.dSleep(25)
    print("Robot ready.")


def move_xyz(api, x, y, z):
    cmd = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode,
                          x, y, z, 0, isQueued=0)[0]
    while cmd > dType.GetQueuedCmdCurrentIndex(api)[0]:
        dType.dSleep(25)


def go_home(api):
    move_xyz(api, *HOME_POS)


# =============================================================================
# Camera and ArUco detection
# =============================================================================

def make_detector():
    d = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    try:                                    # OpenCV >= 4.7
        return aruco.ArucoDetector(d, aruco.DetectorParameters())
    except AttributeError:                  # OpenCV < 4.7
        return None


def detect_marker(cap, detector, target_id, n_frames=5, show=True, window="step5"):
    """
    Average `n_frames` detections of `target_id` and return the marker's
    3D position in the camera frame (meters). Returns None if not detected.
    Uses solvePnP on the detected corner pixels.
    """
    half = MARKER_SIZE_M / 2.0
    obj_pts = np.array([
        [-half,  half, 0],
        [ half,  half, 0],
        [ half, -half, 0],
        [-half, -half, 0],
    ], dtype=np.float32)

    tvecs = []
    attempts = 0
    while len(tvecs) < n_frames and attempts < 60:
        ret, frame = cap.read()
        attempts += 1
        if not ret:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if detector is not None:
            corners, ids, _ = detector.detectMarkers(gray)
        else:
            p = aruco.DetectorParameters_create()
            corners, ids, _ = aruco.detectMarkers(
                gray, aruco.getPredefinedDictionary(aruco.DICT_4X4_50), parameters=p)

        if show:
            disp = frame.copy()
            if ids is not None:
                aruco.drawDetectedMarkers(disp, corners, ids)
            cv2.imshow(window, disp)
            cv2.waitKey(1)

        if ids is None or target_id not in ids.flatten():
            continue

        idx = list(ids.flatten()).index(target_id)
        img_pts = corners[idx][0].astype(np.float32)
        ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts,
                                        CAMERA_MATRIX, DIST_COEFFS)
        if ok:
            tvecs.append(tvec.flatten())

    if len(tvecs) == 0:
        return None
    return np.mean(tvecs, axis=0)   # (3,) meters, camera frame


# =============================================================================
# Joint fitting math  (same as step5_calibration.py, inlined to keep one file)
# =============================================================================

def Rz(deg):
    th = math.radians(deg); c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _kabsch(P_src, Q_dst):
    P = np.asarray(P_src, float); Q = np.asarray(Q_dst, float)
    Pc = P.mean(0); Qc = Q.mean(0)
    P0 = P - Pc; Q0 = Q - Qc
    U, _, Vt = np.linalg.svd(P0.T @ Q0)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1, 1, d]) @ U.T
    return R, Qc - R @ Pc


def joint_fit(C_calib, X_robot_mm, J1_list, max_iters=2000, tol=1e-9):
    """
    Jointly fit camera->robot transform (R, t in meters) and the marker's
    local offset (mm, in end-effector frame, which rotates with J1).
    """
    C = np.asarray(C_calib, float)
    X = np.asarray(X_robot_mm, float)
    offset = np.zeros(3)
    R, t = None, None
    for _ in range(max_iters):
        targets_m = np.array([X[i] + Rz(J1_list[i]) @ offset
                               for i in range(len(J1_list))]) / 1000.0
        R, t = _kabsch(C, targets_m)
        pred_mm = ((R @ C.T).T + t) * 1000.0
        resid = pred_mm - X
        local_r = np.array([Rz(-J1_list[i]) @ resid[i] for i in range(len(J1_list))])
        new_off = local_r.mean(0)
        if np.linalg.norm(new_off - offset) < tol:
            offset = new_off; break
        offset = new_off
    return R, t, offset


def apply_raw(R, t, c_m):
    return (R @ np.asarray(c_m, float) + t) * 1000.0


def apply_corrected(R, t, c_m, J1_deg, offset_mm):
    return apply_raw(R, t, c_m) - Rz(J1_deg) @ offset_mm


def error_report(X_true, X_est, J1_list, label):
    errs = np.linalg.norm(np.asarray(X_true) - np.asarray(X_est), axis=1)
    print(f"\n  {label}")
    print(f"    mean={errs.mean():.2f}  median={np.median(errs):.2f}  "
          f"max={errs.max():.2f} mm  (n={len(errs)})")
    J1 = np.asarray(J1_list)
    for lo, hi in [(-90, -30), (-30, 30), (30, 90)]:
        mask = (J1 >= lo) & (J1 < hi)
        if mask.sum():
            print(f"    J1 [{lo:>4},{hi:>3}) deg (n={mask.sum():>2}): "
                  f"mean={errs[mask].mean():.2f} mm")
    return errs


# =============================================================================
# Data collection helper
# =============================================================================

SETTLE_S = 1.0   # seconds to wait after each robot move before capturing

def collect_data(api, cap, detector, targets, label, show):
    """
    Move robot to each target, detect marker, record the pair.
    Returns (robot_poses_mm, camera_tvecs_m, J1_list, labels_of_skipped).
    """
    robot_xyz = []; cam_tvec = []; J1_list = []; skipped = []
    for tag, x, y, z in targets:
        print(f"  {tag}: moving to ({x:.1f}, {y:.1f}, {z:.1f}) ...", end="", flush=True)
        move_xyz(api, x, y, z)
        time.sleep(SETTLE_S)
        pose = list(dType.GetPose(api))   # [x,y,z,r,J1,J2,pos6,J4]
        act_x, act_y, act_z = pose[0], pose[1], pose[2]
        J1 = pose[4]

        tvec = detect_marker(cap, detector, MARKER_ID, show=show)
        if tvec is None:
            print(f" MARKER NOT DETECTED -- skipped")
            skipped.append(tag)
            go_home(api)
            continue

        robot_xyz.append([act_x, act_y, act_z])
        cam_tvec.append(tvec)
        J1_list.append(J1)
        print(f" J1={J1:.1f}  tvec=({tvec[0]*1000:.1f},{tvec[1]*1000:.1f},{tvec[2]*1000:.1f}) mm")
        go_home(api)

    if skipped:
        print(f"  [{label}] Skipped {len(skipped)} points "
              f"(marker not visible): {skipped}")
    return np.array(robot_xyz), np.array(cam_tvec), J1_list


# =============================================================================
# Main
# =============================================================================

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true",
                        help="Suppress camera preview window")
    parser.add_argument("--calib-csv", default="step5_calib_data.csv")
    parser.add_argument("--val-csv",   default="step5_val_data.csv")
    args = parser.parse_args()
    show = not args.headless

    # ── robot ────────────────────────────────────────────────────────────────
    api = dType.load()
    init_robot(api)

    # ── camera ───────────────────────────────────────────────────────────────
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print(f"Cannot open camera index {CAMERA_INDEX}. Exiting."); exit()
    detector = make_detector()
    print(f"Camera opened (index {CAMERA_INDEX}). "
          f"Looking for DICT_4X4_50 marker ID {MARKER_ID}, size {MARKER_SIZE_M*1000:.0f} mm.")
    if show:
        print("A live preview window will appear. "
              "If no marker is detected, move the robot or camera to fix visibility.")

    try:
        # ── Phase 1: calibration data collection ────────────────────────────
        print(f"\n{'='*60}")
        print(f"PHASE 1: CALIBRATION ({len(CALIB_TARGETS)} targets)")
        print(f"{'='*60}")
        X_calib, C_calib, J1_calib = collect_data(
            api, cap, detector, CALIB_TARGETS, "calibration", show)

        if len(X_calib) < 6:
            print(f"\nOnly {len(X_calib)} calibration points collected -- "
                  f"need at least 6. Adjust CALIB_TARGETS visibility and retry.")
            return

        # save calibration data
        with open(args.calib_csv, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["label","x_mm","y_mm","z_mm","J1_deg",
                         "cam_x_m","cam_y_m","cam_z_m"])
            for i in range(len(X_calib)):
                tag = CALIB_TARGETS[i][0] if i < len(CALIB_TARGETS) else f"c{i}"
                w.writerow([tag,
                             *X_calib[i].round(4), round(J1_calib[i],4),
                             *C_calib[i].round(6)])
        print(f"Calibration data saved to {args.calib_csv}")

        # ── Phase 2: fit transform + offset ─────────────────────────────────
        print(f"\n{'='*60}")
        print(f"PHASE 2: FITTING TRANSFORM AND MARKER OFFSET")
        print(f"{'='*60}")
        R_fit, t_fit, offset_fit = joint_fit(C_calib, X_calib, J1_calib)
        print(f"Local marker offset (in end-effector frame): "
              f"[{offset_fit[0]:.2f}, {offset_fit[1]:.2f}, {offset_fit[2]:.2f}] mm")
        print("  (The z component is absorbed into the transform's z-translation")
        print("   and cannot be separated -- this is expected, not a bug.)")

        # sanity check on calibration residuals
        raw_cal = [apply_raw(R_fit, t_fit, C_calib[i]) for i in range(len(C_calib))]
        cor_cal = [apply_corrected(R_fit, t_fit, C_calib[i], J1_calib[i], offset_fit)
                   for i in range(len(C_calib))]
        print("\nCalibration set residuals (should be small -- fit was done on these):")
        error_report(X_calib, raw_cal, J1_calib, "RAW")
        error_report(X_calib, cor_cal, J1_calib, "CORRECTED")

        # ── Phase 3: validation data collection ─────────────────────────────
        print(f"\n{'='*60}")
        print(f"PHASE 3: VALIDATION ({len(VALIDATION_TARGETS)} targets)")
        print(f"{'='*60}")
        X_val, C_val, J1_val = collect_data(
            api, cap, detector, VALIDATION_TARGETS, "validation", show)

        if len(X_val) < 10:
            print(f"Only {len(X_val)} validation points -- need at least 10 for a "
                  f"meaningful error analysis.")
        else:
            with open(args.val_csv, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["label","x_mm","y_mm","z_mm","J1_deg",
                             "cam_x_m","cam_y_m","cam_z_m"])
                for i in range(len(X_val)):
                    tag = VALIDATION_TARGETS[i][0] if i < len(VALIDATION_TARGETS) else f"v{i}"
                    w.writerow([tag,
                                 *X_val[i].round(4), round(J1_val[i],4),
                                 *C_val[i].round(6)])
            print(f"Validation data saved to {args.val_csv}")

        # ── Phase 4: error analysis ─────────────────────────────────────────
        print(f"\n{'='*60}")
        print(f"PHASE 4: ERROR ANALYSIS ON VALIDATION SET")
        print(f"{'='*60}")
        raw_val = [apply_raw(R_fit, t_fit, C_val[i]) for i in range(len(C_val))]
        cor_val = [apply_corrected(R_fit, t_fit, C_val[i], J1_val[i], offset_fit)
                   for i in range(len(C_val))]

        errs_raw = error_report(X_val, raw_val, J1_val, "RAW (no correction)")
        errs_cor = error_report(X_val, cor_val, J1_val, "CORRECTED (offset removed)")

        if errs_raw.mean() > 0:
            print(f"\n  Improvement: {errs_raw.mean()/errs_cor.mean():.1f}x  "
                  f"({errs_raw.mean():.2f} mm -> {errs_cor.mean():.2f} mm)")
        print(f"\n  Dobot spec (repeatability): 0.2 mm")
        print(f"  Remaining error after correction: {errs_cor.mean():.2f} mm mean")
        if errs_cor.mean() > 5:
            print("  NOTE: remaining error is >5mm. Likely causes:")
            print("    - Camera matrix from Lab 2 needs refreshing (re-run calibrate_camera.py)")
            print("    - Marker mount shifted between calibration and validation")
            print("    - Not enough calibration points or poor distribution")

    finally:
        go_home(api)
        cap.release()
        if show:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
