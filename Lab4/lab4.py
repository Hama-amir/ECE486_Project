"""
lab4.py  -  ECE 486 Lab 4: Vision-Based Movement
==================================================================================
Everything for both parts of Lab 4 in one file:
  - Extended restricted-workspace check (marker bounding box INTERSECTED with the
    Lab 1 original workspace, per the lab's "Understanding the Problem" section)
  - Part 1: single-marker dot-and-measure workflow, with an iteration log for
    the manual ruler-measurement correction loop
  - Part 2: arbitrary-length marker-ID sequence runner

This version's detection code is aligned to the prof's actual find_aruco.py
(uploaded and checked against, not guessed): same DICT_4X4_50 dictionary, same
cv2.aruco.ArucoDetector + estimatePoseSingleMarkers call, same R.npy/T.npy
loading pattern, same transform_camera_to_world() math (R @ X_c + T, no hidden
scale factor). Differences from a live preview script: this batches multiple
frames per marker and averages, since Part 1 is precision-sensitive in a way a
live debug overlay doesn't need to be.

=====================================================================================
READ THIS BEFORE RUNNING. EVERY VALUE BELOW MARKED "PLACEHOLDER" IS NOT A REAL
NUMBER. Nothing in this file was invented to "look plausible" -- values that are
station-specific, pen-specific, or must be physically measured are left as
placeholders with an explicit note on how the lab document (or the confirmed
find_aruco.py) says to obtain them. Using this file before filling them in
correctly WILL move the robot to wrong positions or into the table.
=====================================================================================

PLACEHOLDERS YOU MUST FILL IN:

  1. CAMERA_MATRIX, DIST_COEFFS
     Source: your calibrate_camera.py output (Lab 2). Station-specific.
     find_aruco.py itself hardcodes camera_matrix=[[1418,0,354],[0,790,184],[0,0,1]]
     with the comment "assuming some default values, you should calibrate your
     camera" -- confirming directly, in the prof's own script, that this is a
     dummy placeholder, not a real value to reuse. Do not copy those numbers.

  2. R.npy, T.npy  (files, not variables)
     find_aruco.py loads these with np.load("R.npy") / np.load("T.npy") -- this
     file does the same. They must exist in the working directory and must come
     from compute_transform.py run FRESH this session ("Remember to begin by
     re-calibrating the robot to the camera frame using the scripts from last
     lab" -- Lab 4 PDF). A stale R.npy/T.npy from a previous session is invalid.

  3. WORLD_POS_SCALE_TO_MM  -- a genuine ambiguity, not resolved by find_aruco.py
     find_aruco.py computes X_w = R @ X_c + T and only ever prints it as debug
     text -- it never sends it to the robot, so it never needed to know whether
     that result is in meters or millimeters. This script DOES need to command
     the robot (which takes millimeters, per GetPose/move_xyz throughout this
     entire course), so this matters and I cannot determine it without seeing
     compute_transform.py, which I do not have. Verify it yourself in under a
     minute: run with --debug-print-only, look at the printed X_w magnitude for
     a marker, and compare it to the known Lab 1 workspace scale (r between 140
     and 260mm). If the printed numbers are ~0.1-0.3, R/T were fit in meters and
     WORLD_POS_SCALE_TO_MM should be 1000.0. If they already look like ~150-260,
     no scaling is needed and it should be 1.0. Left as None on purpose so the
     script refuses to move until you've checked this.

  4. MARKER_SIZE_M
     find_aruco.py hardcodes 0.05 (5cm) with NO comment flagging it as a
     placeholder (unlike camera_matrix, which is explicitly flagged). This might
     mean 5cm is the real, correct marker size for this course's printouts, or
     it might just be another convenient default left in starter code -- the
     file alone doesn't say which. Measure your actual printed marker's black
     border with a ruler to confirm. If it's close to 50mm, that's a good sign
     0.05 was real; if not, trust your ruler over the starter script.

  5. CAMERA_INDEX
     find_aruco.py uses cv2.VideoCapture(0) -- defaulted to 0 here to match.
     Still station/OS-dependent; adjust if the wrong camera opens.

  6. Z_SAFE_TRAVEL, Z_DOT_CONTACT
     THE LAB EXPLICITLY FORBIDS FINDING THESE WITH YOUR OWN PYTHON CODE:
     "You should use Dobot Link to move the robot manually and record its
     positions for this part. Do not use python code you wrote yourself! That
     code is what you are restricting with this new workspace, so you can't
     pre-program a motion if you don't know what those restrictions are."
     You must jog the robot by hand in DobotLink and read GetPose to find:
       Z_SAFE_TRAVEL : a height where the pen can move freely in xy without
                        touching the table. The lab notes this may need to be
                        ABOVE z=0 depending on your pen mount -- that's fine.
       Z_DOT_CONTACT : the height where the pen tip just touches the paper.
                        The pen holder has ~1cm of spring travel once contact
                        is made; the lab caps SAFE usage at 0.5cm (5mm) of that
                        travel. So the true lower limit used by the workspace
                        check should be no more than 5mm past first contact --
                        do not command the pen further than that.
     These are left as None below and the script will refuse to run motion
     commands until you fill them in, on purpose.

  7. OUT_OF_VIEW_POS
     Defaults to [200, 100, 50] -- this is NOT invented. It is the exact
     home_pos from ece486_starter_code.py (Lab 1), and Lab 1's own PDF states
     this position was chosen specifically because "later labs in this course
     will use vision with cameras mounted to the desks, this position moves
     the robot out of the field of view of the camera as much as possible."
     That said: this was designed WITHOUT a pen attached. The xy component
     should still clear the camera, but verify this default actually keeps
     the robot (and the pen) out of your station's specific camera frame --
     do not assume it without checking.

  8. PEN_OFFSET_CORRECTION
     Starts at (0,0,0) mm. This is what Part 1 empirically determines through
     the ruler-measurement iteration loop described in the lab -- there is no
     way to know this in advance, it is the literal experimental result of
     Part 1. See run_part1_iteration() below for how it gets updated between
     runs.

CONFIRMED, NOT PLACEHOLDERS:
  - Original Lab 1 workspace bounds: -120 <= z <= 0, 140 <= r <= 260, x >= 0
    (Lab 1 PDF, verified numerically with real robot data earlier in this course)
  - home_pos = [200, 100, 50] (Lab 1 PDF / ece486_starter_code.py)
  - ArUco dictionary = DICT_4X4_50 (confirmed directly in find_aruco.py)
  - Detection method = cv2.aruco.ArucoDetector + estimatePoseSingleMarkers
    (confirmed directly in find_aruco.py, not solvePnP -- earlier version of
    this file used solvePnP manually; that was mathematically workable but not
    verified against the real starter script, so it has been changed to match)
  - R/T loading = np.load("R.npy"), np.load("T.npy") (confirmed in find_aruco.py)
  - transform_camera_to_world(X_c, R, T) = R @ X_c + T, no hidden scale factor
    inside R or T themselves (confirmed in find_aruco.py's own function)

HOW TO RUN:
    python lab4.py --debug-print-only          # check WORLD_POS_SCALE_TO_MM first
    python lab4.py --part 1 --marker-id 0
    python lab4.py --part 2
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
# PLACEHOLDERS -- fill these in before running. See docstring above for source
# of each one. Do not guess values here; wrong numbers move the robot wrong.
# =============================================================================

CAMERA_MATRIX = None   # <-- REPLACE. e.g. np.array([[fx,0,cx],[0,fy,cy],[0,0,1]])
DIST_COEFFS   = None   # <-- REPLACE. e.g. np.array([k1,k2,p1,p2,k3])

# R.npy / T.npy are loaded from disk in load_cam2robot_transform() below, exactly
# matching find_aruco.py's np.load("R.npy") / np.load("T.npy"). Run
# compute_transform.py fresh this session so these files exist and are current.
R_T_DIRECTORY = "."      # <-- ADJUST if R.npy/T.npy are saved somewhere else

# Genuine unit ambiguity -- see docstring point 3. Must be verified empirically
# with --debug-print-only before this script is trusted to move the robot.
WORLD_POS_SCALE_TO_MM = None   # <-- REPLACE: either 1000.0 or 1.0, see docstring.

MARKER_SIZE_M = None    # <-- REPLACE. Measure your printed marker's black border, mm/1000.
                          #     find_aruco.py hardcodes 0.05 with no placeholder comment --
                          #     see docstring point 4 for why that isn't fully trustworthy.

ARUCO_DICT_ID = aruco.DICT_4X4_50   # Confirmed directly in find_aruco.py.

CAMERA_INDEX = 0         # Matches find_aruco.py's cv2.VideoCapture(0). Adjust if needed.

# Pen z-limits -- MUST be found by manually jogging with DobotLink. See docstring.
# Left as None on purpose so the script refuses to move until you fill these in.
Z_SAFE_TRAVEL = None     # <-- REPLACE (mm). May be > 0 depending on pen mount.
Z_DOT_CONTACT = None     # <-- REPLACE (mm). First-contact height. Do not press
                          #     more than 5mm further into the spring than this.

# Sourced from Lab 1 PDF / ece486_starter_code.py -- see docstring note on why
# this default may still need station-specific verification with a pen attached.
OUT_OF_VIEW_POS = [200.0, 100.0, 50.0]

# Empirical Part 1 output. Starts at zero. Updated between iterations based on
# ruler measurements -- see run_part1_iteration().
PEN_OFFSET_CORRECTION = np.array([0.0, 0.0, 0.0])


# =============================================================================
# CONFIRMED VALUES -- sourced from the Lab 1 PDF, not placeholders.
# =============================================================================

Z_MIN, Z_MAX = -120.0, 0.0     # Lab 1 original workspace z bounds
R_MIN, R_MAX = 140.0, 260.0    # Lab 1 original workspace radius bounds
X_MIN = 0.0                    # Lab 1 original workspace x bound


# =============================================================================
# Restricted workspace logic
#
# Lab 4 restriction 1 (xy): "the intersection of a box in the xy plane and the
# robot's original workspace projected onto the xy plane." The box is the
# min/max x,y of the 3 markers. This is the intersection of a rectangle (convex)
# with the original annular workspace's outer disk (convex), the x>=0 half-plane
# (convex), and the inner-radius hole (NOT convex). Endpoint checks alone are
# only sufficient for the convex parts; the inner hole still needs the exact
# closest-point-on-segment check, same as Lab 1's workspace_function.py.
#
# Lab 4 restriction 2 (z): the pen's allowed travel range, found by jogging
# (Z_SAFE_TRAVEL and Z_DOT_CONTACT, both placeholders above). This is a plain
# interval, which is convex, so endpoint checking is always sufficient for z.
# =============================================================================

def in_original_xy(x, y):
    """Lab 1 original workspace, projected onto the xy plane (ignores z)."""
    r = math.hypot(x, y)
    return (R_MIN <= r <= R_MAX) and (x >= X_MIN)


def compute_marker_box(marker_positions_robot_frame):
    """
    marker_positions_robot_frame: list of (x,y,z) tuples for all 3 markers,
    already transformed into the robot base frame. Returns (xmin,xmax,ymin,ymax).
    Requires all 3 markers to have been located -- if any were undetected at
    calibration time, the box cannot be safely computed and this will raise.
    """
    if len(marker_positions_robot_frame) < 3:
        raise ValueError(
            f"Only {len(marker_positions_robot_frame)} marker(s) located; need all "
            f"3 to compute the restricted-workspace box. Re-check camera visibility."
        )
    xs = [p[0] for p in marker_positions_robot_frame]
    ys = [p[1] for p in marker_positions_robot_frame]
    return (min(xs), max(xs), min(ys), max(ys))


def in_box(x, y, box):
    xmin, xmax, ymin, ymax = box
    return xmin <= x <= xmax and ymin <= y <= ymax


def in_restricted_xy(x, y, box):
    """Lab 4 restriction 1: box AND original-workspace-xy, intersected."""
    return in_box(x, y, box) and in_original_xy(x, y)


def in_restricted_z(z):
    """Lab 4 restriction 2: the pen's measured travel range."""
    if Z_SAFE_TRAVEL is None or Z_DOT_CONTACT is None:
        raise RuntimeError(
            "Z_SAFE_TRAVEL / Z_DOT_CONTACT are not set. These must be found by "
            "jogging the robot manually with DobotLink -- see the file docstring. "
            "Refusing to evaluate a z-restriction with unknown limits."
        )
    z_lo = min(Z_SAFE_TRAVEL, Z_DOT_CONTACT)
    z_hi = max(Z_SAFE_TRAVEL, Z_DOT_CONTACT)
    return z_lo <= z <= z_hi


def in_restricted_workspace(x, y, z, box):
    return in_restricted_xy(x, y, box) and in_restricted_z(z)


def closest_radius_on_segment(p0_xy, p1_xy):
    """Exact minimum (x,y) radius anywhere on the straight segment p0->p1."""
    x0, y0 = p0_xy
    dx = p1_xy[0] - x0
    dy = p1_xy[1] - y0
    denom = dx * dx + dy * dy
    if denom == 0.0:
        return math.hypot(x0, y0)
    t = -(x0 * dx + y0 * dy) / denom
    t = max(0.0, min(1.0, t))
    return math.hypot(x0 + t * dx, y0 + t * dy)


def segment_in_restricted_xy(p0, p1, box):
    """
    Whole-segment safety check. See the module comment above the workspace
    section for why only the inner-radius hole needs this treatment.
    Returns (ok: bool, reason: str).
    """
    if not in_restricted_xy(*p0, box):
        return False, "start point outside restricted xy region"
    if not in_restricted_xy(*p1, box):
        return False, "end point outside restricted xy region"
    r_min = closest_radius_on_segment(p0, p1)
    if r_min < R_MIN:
        return False, f"path crosses inner hole (closest r={r_min:.1f} < {R_MIN:.0f})"
    return True, "ok"


# =============================================================================
# ArUco detection and camera->robot transform
#
# This section is now aligned to the confirmed, real find_aruco.py: same
# dictionary (DICT_4X4_50), same detector class (ArucoDetector), same pose
# function (estimatePoseSingleMarkers, not solvePnP), same R/T loading
# (np.load("R.npy")/np.load("T.npy")), same transform math (R @ X_c + T).
# The one addition beyond find_aruco.py is averaging over several frames
# before committing to a robot move, since find_aruco.py is a live debug
# preview and doesn't need that; Part 1 here is precision-sensitive.
# =============================================================================

def load_cam2robot_transform():
    """Load R, T exactly as find_aruco.py does: np.load('R.npy'), np.load('T.npy')."""
    import os
    r_path = os.path.join(R_T_DIRECTORY, "R.npy")
    t_path = os.path.join(R_T_DIRECTORY, "T.npy")
    if not os.path.exists(r_path) or not os.path.exists(t_path):
        raise RuntimeError(
            f"R.npy and/or T.npy not found in '{R_T_DIRECTORY}'. Run "
            f"compute_transform.py fresh this session first -- see docstring."
        )
    return np.load(r_path), np.load(t_path)


def make_detector():
    """Exactly matches find_aruco.py: getPredefinedDictionary + ArucoDetector."""
    d = aruco.getPredefinedDictionary(ARUCO_DICT_ID)
    return aruco.ArucoDetector(d, aruco.DetectorParameters())


def detect_markers_once(cap, detector, n_frames=5):
    """
    Capture and average n_frames detections. Returns {marker_id: tvec}
    in whatever units estimatePoseSingleMarkers returns for the given
    MARKER_SIZE_M (meters if MARKER_SIZE_M is in meters, matching
    find_aruco.py's convention).
    """
    if MARKER_SIZE_M is None:
        raise RuntimeError("MARKER_SIZE_M is not set. Measure your printout first.")
    if CAMERA_MATRIX is None or DIST_COEFFS is None:
        raise RuntimeError("CAMERA_MATRIX / DIST_COEFFS not set. Run calibrate_camera.py first.")

    detections = {}   # id -> list of tvecs
    for _ in range(n_frames):
        ret, frame = cap.read()
        if not ret:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)
        if ids is None:
            continue
        # Batched call across all markers in this frame, matching find_aruco.py.
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            corners, MARKER_SIZE_M, CAMERA_MATRIX, DIST_COEFFS)
        for i, mid in enumerate(ids.flatten()):
            detections.setdefault(int(mid), []).append(tvecs[i].flatten())

    return {mid: np.mean(vecs, axis=0) for mid, vecs in detections.items()}


def transform_camera_to_world(X_c, R, T):
    """Identical to find_aruco.py's function of the same name: R @ X_c + T."""
    return R @ X_c + T


def cam_to_robot(tvec, R, T):
    """
    Camera-frame point -> robot-frame position in millimeters.
    Applies find_aruco.py's exact transform_camera_to_world(), then applies
    WORLD_POS_SCALE_TO_MM -- the one addition find_aruco.py doesn't need,
    since it only ever displays this value rather than commanding the robot
    with it. See docstring point 3 for how to determine this scale factor.
    """
    if WORLD_POS_SCALE_TO_MM is None:
        raise RuntimeError(
            "WORLD_POS_SCALE_TO_MM is not set. Run with --debug-print-only "
            "first and compare the printed magnitude to the known r in "
            "[140,260]mm workspace scale -- see docstring point 3."
        )
    world = transform_camera_to_world(np.asarray(tvec, dtype=float), R, T)
    return world * WORLD_POS_SCALE_TO_MM


def locate_all_markers_robot_frame(cap, detector, R, T):
    """Detect all visible markers and return {id: (x,y,z) mm, robot frame}."""
    cam_detections = detect_markers_once(cap, detector)
    return {mid: tuple(cam_to_robot(t, R, T)) for mid, t in cam_detections.items()}


# =============================================================================
# Robot interface (same setup/motion pattern as prior labs' real-robot scripts)
# =============================================================================

def init_robot(api):
    com = dType.SearchDobot(api)
    if "COM" not in com[0]:
        print("Robot not found. Exiting.")
        exit()
    state = dType.DobotConnect.DobotConnect_NoError
    for port in com:
        state = dType.ConnectDobot(api, port, 115200)[0]
        if state == dType.DobotConnect.DobotConnect_NoError:
            print(f"Connected on {port}")
            break
    if state != dType.DobotConnect.DobotConnect_NoError:
        print("Cannot connect. Exiting.")
        exit()
    dType.SetQueuedCmdStopExec(api)
    dType.SetQueuedCmdClear(api)
    dType.SetPTPCommonParams(api, 50, 50, isQueued=1)
    dType.SetHOMEParams(api, *[200, 100, 50], 0, isQueued=1)
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


def get_pose_xyz(api):
    pose = dType.GetPose(api)   # [x,y,z,r,J1,J2,pos6,J4]
    return float(pose[0]), float(pose[1]), float(pose[2])


def move_to_out_of_view(api):
    move_xyz(api, *OUT_OF_VIEW_POS)


# =============================================================================
# Part 1: single marker, dot, ruler-measure-and-iterate
#
# The lab's procedure is explicitly manual and iterative: place a dot, measure
# the offset from the marker center with a ruler, adjust, repeat. This script
# cannot perform the ruler measurement for you -- it logs everything it CAN
# know (the computed target, the correction applied) and leaves the measured
# columns blank for you to fill in by hand after each run, which is exactly
# the data the report's Part 1 error table needs.
# =============================================================================

def check_z_limits_set():
    if Z_SAFE_TRAVEL is None or Z_DOT_CONTACT is None:
        raise RuntimeError(
            "Z_SAFE_TRAVEL / Z_DOT_CONTACT are not set. Jog the robot manually "
            "in DobotLink to find these values first -- see the file docstring. "
            "This script will not move the robot with unknown z-limits."
        )


def run_part1_iteration(api, cap, detector, R, T, marker_id, iteration_num,
                         log_path="lab4_part1_log.csv"):
    """
    One iteration of Part 1: locate the marker, move to it (target + current
    PEN_OFFSET_CORRECTION), dot, retract. Logs the computed target and the
    correction used. YOU must measure the actual dot position with a ruler
    after this runs and record it, then update PEN_OFFSET_CORRECTION above
    before the next call for the correction loop to actually converge.
    """
    check_z_limits_set()

    markers = locate_all_markers_robot_frame(cap, detector, R, T)
    if marker_id not in markers:
        print(f"Marker {marker_id} not detected. Nothing moved.")
        return

    mx, my, mz = markers[marker_id]
    box = compute_marker_box(list(markers.values())) if len(markers) >= 3 else None

    target = np.array([mx, my, mz]) + PEN_OFFSET_CORRECTION
    tx, ty, tz = target

    if box is not None:
        ok, reason = in_restricted_xy(tx, ty, box), None
        if not ok:
            print(f"Target ({tx:.2f},{ty:.2f}) rejected by restricted xy workspace. Not moving.")
            return
    else:
        print("WARNING: fewer than 3 markers visible -- cannot compute the full "
              "restricted box. Proceeding with original-workspace check only.")
        if not in_original_xy(tx, ty):
            print(f"Target ({tx:.2f},{ty:.2f}) rejected by original workspace. Not moving.")
            return

    print(f"Marker {marker_id} at robot-frame ({mx:.2f},{my:.2f},{mz:.2f}) mm")
    print(f"Commanded target (with correction): ({tx:.2f},{ty:.2f},{tz:.2f}) mm")

    move_xyz(api, tx, ty, Z_SAFE_TRAVEL)
    move_xyz(api, tx, ty, Z_DOT_CONTACT)
    time.sleep(0.3)
    move_xyz(api, tx, ty, Z_SAFE_TRAVEL)
    move_to_out_of_view(api)

    write_header = not _file_has_content(log_path)
    with open(log_path, "a", newline="") as f:
        w = csv.writer(f)
        if write_header:
            w.writerow(["iteration", "marker_id", "marker_x_mm", "marker_y_mm", "marker_z_mm",
                        "correction_x_mm", "correction_y_mm", "correction_z_mm",
                        "commanded_x_mm", "commanded_y_mm", "commanded_z_mm",
                        "MEASURED_dot_x_mm_FILL_IN", "MEASURED_dot_y_mm_FILL_IN",
                        "error_mm_FILL_IN_OR_COMPUTE"])
        w.writerow([iteration_num, marker_id, round(mx,3), round(my,3), round(mz,3),
                    *PEN_OFFSET_CORRECTION.round(3).tolist(),
                    round(tx,3), round(ty,3), round(tz,3),
                    "", "", ""])
    print(f"Logged to {log_path}. Measure the real dot position with a ruler, "
          f"fill in the blank columns, then update PEN_OFFSET_CORRECTION above "
          f"before the next iteration.")


def _file_has_content(path):
    try:
        with open(path, "r") as f:
            return len(f.read()) > 0
    except FileNotFoundError:
        return False


# =============================================================================
# Part 2: sequence runner
# =============================================================================

def parse_id_sequence_from_stdin():
    """
    Reads whitespace/newline separated ints from stdin until a negative one is
    entered (per lab: 'When the user enters a negative number, stop taking in
    IDs'). The negative terminator itself is not included in the sequence.
    """
    print("Enter marker IDs one at a time (or space-separated on one line). "
          "Enter a negative number to stop:")
    ids = []
    raw = input("> ").split()
    for tok in raw:
        v = int(tok)
        if v < 0:
            return ids
        ids.append(v)
    # if the line didn't include a negative terminator, keep reading lines
    while True:
        raw = input("> ").split()
        stop = False
        for tok in raw:
            v = int(tok)
            if v < 0:
                stop = True
                break
            ids.append(v)
        if stop:
            break
    return ids


def run_part2_sequence(api, cap, detector, R, T, sequence, log_path="lab4_part2_log.csv"):
    """
    Caches all marker positions ONCE at the start (the lab explicitly advises
    against re-imaging mid-trajectory). Moves through the sequence, skipping
    markers that are undetected or outside the restricted workspace. If every
    marker in the sequence is invalid, does not move at all.
    """
    check_z_limits_set()

    markers = locate_all_markers_robot_frame(cap, detector, R, T)
    if len(markers) < 3:
        print(f"WARNING: only {len(markers)} of 3 markers detected at capture time. "
              f"Restricted-box computation and any marker not in this set will be unavailable.")

    box = compute_marker_box(list(markers.values())) if len(markers) >= 3 else None

    def target_is_valid(x, y, z):
        if box is not None:
            return in_restricted_xy(x, y, box) and in_restricted_z(Z_SAFE_TRAVEL) and in_restricted_z(Z_DOT_CONTACT)
        return in_original_xy(x, y)

    valid_moves = []
    for mid in sequence:
        if mid not in markers:
            print(f"  marker {mid}: not detected -- skipping")
            continue
        mx, my, mz = markers[mid]
        tx, ty, tz = np.array([mx, my, mz]) + PEN_OFFSET_CORRECTION
        if not target_is_valid(tx, ty, tz):
            print(f"  marker {mid}: target ({tx:.1f},{ty:.1f}) outside restricted workspace -- skipping")
            continue
        valid_moves.append((mid, tx, ty))

    if not valid_moves:
        print("No valid markers in the sequence. Robot will not move.")
        return

    rows = []
    for mid, tx, ty in valid_moves:
        print(f"  moving to marker {mid} at ({tx:.2f},{ty:.2f})")
        move_xyz(api, tx, ty, Z_SAFE_TRAVEL)
        move_xyz(api, tx, ty, Z_DOT_CONTACT)
        time.sleep(0.3)
        act_x, act_y, act_z = get_pose_xyz(api)
        move_xyz(api, tx, ty, Z_SAFE_TRAVEL)
        rows.append([mid, tx, ty, Z_DOT_CONTACT, act_x, act_y, act_z])

    move_to_out_of_view(api)

    with open(log_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["marker_id", "target_x_mm", "target_y_mm", "target_z_mm",
                    "reported_pen_x_mm", "reported_pen_y_mm", "reported_pen_z_mm"])
        w.writerows(rows)
    print(f"\n{len(valid_moves)}/{len(sequence)} moves executed. Logged to {log_path}.")


# =============================================================================
# Main
# =============================================================================

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--part", type=int, choices=[1, 2],
                        help="Required unless --debug-print-only is used")
    parser.add_argument("--marker-id", type=int, help="Required for --part 1")
    parser.add_argument("--iteration", type=int, default=1,
                        help="Iteration number for Part 1 logging")
    parser.add_argument("--debug-print-only", action="store_true",
                        help="Detect markers, print raw R@X_c+T, and exit. "
                             "No robot connection, no motion. Use this FIRST "
                             "to determine WORLD_POS_SCALE_TO_MM -- see docstring.")
    args = parser.parse_args()

    if args.debug_print_only:
        R, T = load_cam2robot_transform()
        cap = cv2.VideoCapture(CAMERA_INDEX)
        if not cap.isOpened():
            print(f"Cannot open camera index {CAMERA_INDEX}. Exiting.")
            return
        detector = make_detector()
        if MARKER_SIZE_M is None or CAMERA_MATRIX is None or DIST_COEFFS is None:
            print("Set MARKER_SIZE_M, CAMERA_MATRIX, DIST_COEFFS before running this check.")
            cap.release()
            return
        cam_detections = detect_markers_once(cap, detector)
        cap.release()
        if not cam_detections:
            print("No markers detected. Check camera, lighting, printout placement.")
            return
        print("Raw R @ X_c + T (BEFORE any WORLD_POS_SCALE_TO_MM is applied):")
        for mid, tvec in cam_detections.items():
            world = transform_camera_to_world(tvec, R, T)
            mag = np.linalg.norm(world)
            print(f"  marker {mid}: {world}   magnitude={mag:.4f}")
        print("\nKnown robot workspace scale (Lab 1, confirmed): r between 140 and 260 mm.")
        print("If the magnitudes above are ~0.1-0.3   -> set WORLD_POS_SCALE_TO_MM = 1000.0")
        print("If the magnitudes above are already ~150-260 -> set WORLD_POS_SCALE_TO_MM = 1.0")
        return

    if args.part is None:
        parser.error("--part is required unless using --debug-print-only")
    if args.part == 1 and args.marker_id is None:
        parser.error("--part 1 requires --marker-id")

    R, T = load_cam2robot_transform()

    api = dType.load()
    init_robot(api)

    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print(f"Cannot open camera index {CAMERA_INDEX}. Exiting.")
        return
    detector = make_detector()

    try:
        if args.part == 1:
            run_part1_iteration(api, cap, detector, R, T, args.marker_id, args.iteration)
        else:
            sequence = parse_id_sequence_from_stdin()
            print(f"Sequence: {sequence}")
            run_part2_sequence(api, cap, detector, R, T, sequence)
    finally:
        move_to_out_of_view(api)
        cap.release()


if __name__ == "__main__":
    main()
