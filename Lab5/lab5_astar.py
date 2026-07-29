"""
lab5_astar.py  -  ECE 486 Lab 5, Option 1: A* Path Planning (pen attachment)
=====================================================================================
One file, three modes:

  python lab5_astar.py --self-test
      Runs the synthetic verification suite (A* correctness, no-corner-cutting,
      decimation safety, robot-mode cell classification, refusal behaviors).
      No camera, no robot, no placeholders needed.

  python lab5_astar.py --offline FIELD.png --src 1 --dst 2 [--out annotated.png]
      Runs the ENTIRE detection->obstacles->grid->A*->validation pipeline on one
      of the prof's provided field PNGs, in pixel coordinates, and saves an
      annotated image of the plan. This is the prof-sanctioned at-home test path:
      "if you're just testing using the png files themselves ... all this means
      is that you will have a scale ambiguity ... this doesn't affect the
      underlying logic of your algorithms." No robot, no camera, no R/T needed.

  python lab5_astar.py --debug-print-only
      Same units check as lab4.py: detects the real printed markers through the
      real camera, prints raw R @ X_c + T magnitudes, and tells you how to set
      WORLD_POS_SCALE_TO_MM by comparing against the known 140-260 mm workspace
      scale. Requires camera + intrinsics + fresh R.npy/T.npy. Robot not moved.

  python lab5_astar.py --run --src 1 --dst 2 [--trial N]
      The real thing, at the bench, after ALL placeholders are filled:
      capture -> world model -> restricted workspace -> inflated obstacles ->
      grid -> A* -> validate -> draw with the pen -> log the trial to CSV.

=====================================================================================
PLACEHOLDERS -- the script REFUSES to run robot mode until these are filled.
Nothing here is invented; each value's source is stated.

  1. CAMERA_MATRIX, DIST_COEFFS
     From calibrate_camera.py (Lab 2). Station-specific. find_aruco.py's
     hardcoded matrix is flagged by the prof's own comment as a dummy default --
     never copy it.

  2. R.npy / T.npy  (files in R_T_DIRECTORY)
     From compute_transform.py, run FRESH this session (Lab 5 requirement 1:
     "Calibrate the robot"; Lab 4: "Remember to begin by re-calibrating").
     Loaded exactly as find_aruco.py does: np.load("R.npy"), np.load("T.npy").

  3. WORLD_POS_SCALE_TO_MM
     Genuine unit ambiguity carried over from Lab 4: find_aruco.py only ever
     PRINTS R @ X_c + T, never commands the robot with it, so nothing in the
     provided materials fixes whether that product is meters or millimetres.
     Resolve in ~10 seconds with --debug-print-only (magnitudes ~0.15-0.26 =>
     1000.0; magnitudes already ~150-260 => 1.0).

  4. Z_SAFE_TRAVEL, Z_DOT_CONTACT  (mm)
     The Lab 4 prohibition applies verbatim and this script obeys it: these are
     found ONLY by manually jogging in DobotLink ("Do not use python code you
     wrote yourself!"). Z_DOT_CONTACT must be measured on the TRANSPARENCY
     (Lab 5 Option 1 requires a transparency sheet over the printout), not bare
     paper. Spring travel ~1 cm, lab-capped at 0.5 cm; gentle consistent
     contact (~2-3 mm compression) is the sensible drawing point.

  5. PEN_OFFSET_CORRECTION  (mm, xy)
     Starts (0,0). This is the OUTPUT of the mandatory Lab-5-requirement-2 step
     ("Use your procedure from lab 4 to reduce errors as much as possible"):
     run the Lab 4 Part 1 dot-and-measure loop first, put the converged
     correction here.

  6. WORST_DOT_ERROR_MM, PEN_LINE_HALF_WIDTH_MM
     Bench-measured inflation inputs (see required_clearance_mm below).
     WORST_DOT_ERROR_MM = the worst single post-correction dot error from your
     error-reduction pass THIS session. PEN_LINE_HALF_WIDTH_MM = half the width
     of one drawn test line, measured once with a ruler. Robot mode refuses to
     run while these are None. (For scale only: the Lab 4 session produced
     ~1.5 mm dots and ~2.5-3 mm mean residual error -- but you must use THIS
     session's measurements, not those.)

  7. EXTRA_SAFETY_MM
     An explicit design choice, not a measurement. Default 2.0 mm. It also
     (deliberately, with ~100x margin) absorbs the corner-sampling
     under-approximation documented at cell_is_free().

CONFIRMED, NOT PLACEHOLDERS (source in parentheses):
  - Original workspace: -120<=z<=0, 140<=r<=260, x>=0 (Lab 1 PDF; verified
    against real robot data in Labs 1-3)
  - Restricted workspace = marker bounding box INTERSECT original workspace,
    per Lab 4 PDF; Lab 5 requirement 3 reuses it verbatim
  - Obstacle rule: "the pen tip may not enter any boundary of the ArUco
    marker"; only markers that are neither source nor destination are
    obstacles (Lab 5 PDF, Option 1)
  - Algorithm: "You must use the A* path planning algorithm, which will
    require you to discretize the search space" (Lab 5 PDF)
  - Marker physical size: exactly 5 cm x 5 cm printed (prof's field notes;
    print at 5/6 ~ 83.3%); this also retroactively confirms find_aruco.py's
    0.05
  - ArUco dictionary DICT_4X4_50 (find_aruco.py; independently confirmed by
    running detection on all three provided field PNGs)
  - Detection: cv2.aruco.ArucoDetector + estimatePoseSingleMarkers
    (find_aruco.py); camera->world math R @ X_c + T (find_aruco.py)
  - Out-of-view position [200,100,50] (Lab 1 PDF / ece486_starter_code.py;
    chosen there specifically to clear the camera view -- re-verify with the
    pen mounted)
  - Dobot API call pattern (SearchDobot/ConnectDobot 115200/queued-cmd
    polling/PTPMOVLXYZMode) -- ece486_starter_code.py, used in every prior lab
"""

import argparse
import csv
import heapq
import math
import os
import sys
import time
from pathlib import Path

import numpy as np

LAB1_SOFTWARE_DIR = Path(__file__).resolve().parent.parent / "Lab1" / "ece486_software"
if str(LAB1_SOFTWARE_DIR) not in sys.path:
    sys.path.insert(0, str(LAB1_SOFTWARE_DIR))

try:
    import cv2
    import cv2.aruco as aruco
except ImportError:  # self-test mode can run without OpenCV
    cv2 = None
    aruco = None


# =============================================================================
# PLACEHOLDERS -- fill at the bench. Robot mode refuses to run until they are.
# =============================================================================

CAMERA_MATRIX = np.array([
    [659.15181972, 0, 310.30344121],
    [0, 658.80133178, 222.92663724],
    [0, 0, 1],
], dtype=np.float32)
                          
DIST_COEFFS = np.array([
    [5.01473487e-02, 2.73181783e-01, -1.76984102e-03, -3.81410830e-03, -1.98538389e+00]
], dtype=np.float32)


R_T_DIRECTORY = str(LAB1_SOFTWARE_DIR)  # default: shared Lab 1 calibration folder
WORLD_POS_SCALE_TO_MM = 1000.0   # <-- REPLACE after --debug-print-only: 1000.0 or 1.0

CAMERA_INDEX = 0            # matches find_aruco.py's cv2.VideoCapture(0)

Z_SAFE_TRAVEL = 50        # <-- REPLACE (mm): DobotLink manual jog ONLY
Z_DOT_CONTACT = 5        # <-- REPLACE (mm): DobotLink manual jog ONLY, on transparency

PEN_OFFSET_CORRECTION = np.array([0.0, 0.0])   # <-- UPDATE with converged Lab-4-procedure correction (mm, xy)

WORST_DOT_ERROR_MM     = 40   # <-- REPLACE: worst post-correction dot error, THIS session
PEN_LINE_HALF_WIDTH_MM = 0.75   # <-- REPLACE: half of one measured drawn-line width

EXTRA_SAFETY_MM = 2.0       # explicit design margin (choice, not measurement)

GRID_CELL_MM = 2.0          # discretization pitch. Design choice; see report:
                            # page of ~216x279mm -> ~1.5e4 cells, A* in ms;
                            # one cell diagonal (2.83mm) is charged to the
                            # inflation budget below so discretization can
                            # never eat into the true clearance.


# =============================================================================
# CONFIRMED constants (sources in module docstring)
# =============================================================================

Z_MIN, Z_MAX = -120.0, 0.0
R_MIN, R_MAX = 140.0, 260.0
X_MIN = 0.0

MARKER_SIDE_MM = 50.0                       # exactly 5 cm printed (prof's note)
MARKER_HALF_DIAG_MM = MARKER_SIDE_MM * math.sqrt(2) / 2.0   # 35.355...: radius of the
# circumscribed circle of the 5 cm obstacle square. Blocking this disc covers
# the forbidden square at ANY in-plane rotation without estimating orientation
# (design decision; the over-blocked corner slivers are irrelevant at page scale).

OUT_OF_VIEW_POS = [200.0, 100.0, 50.0]

ARUCO_DICT_ID = 0  # placeholder int; real value set below if cv2 present
if aruco is not None:
    ARUCO_DICT_ID = aruco.DICT_4X4_50


# =============================================================================
# Inflation budget
# =============================================================================

def required_clearance_mm(worst_dot_error_mm, pen_half_width_mm,
                           cell_mm=GRID_CELL_MM, extra_mm=EXTRA_SAFETY_MM):
    """
    Radius of the blocked disc around each obstacle marker's center, in mm:

      marker half-diagonal   35.36  covers the forbidden 5cm square at any rotation
    + worst dot error         (measured THIS session, Lab-4-procedure output)
    + pen line half-width     (measured once)
    + one cell diagonal       (grid discretization slack -- the path is planned
                               through cell corners/centers up to one diagonal
                               away from the continuous ideal)
    + explicit extra margin   (EXTRA_SAFETY_MM design choice; also absorbs the
                               micron-scale corner-sampling bulge, see
                               cell_is_free)

    Every term is either a stated geometric fact, a measurement you own, or an
    explicit choice -- exactly the justification chain the report's Strategy
    section needs.
    """
    return (MARKER_HALF_DIAG_MM
            + worst_dot_error_mm
            + pen_half_width_mm
            + cell_mm * math.sqrt(2)
            + extra_mm)


# =============================================================================
# Restricted-workspace geometry (Lab 4 logic, verified there; reused verbatim)
# =============================================================================

def in_original_xy(x, y):
    r = math.hypot(x, y)
    return (R_MIN <= r <= R_MAX) and (x >= X_MIN)


def compute_marker_box(marker_positions):
    if len(marker_positions) < 3:
        raise ValueError(
            f"Only {len(marker_positions)} marker(s) located; the Lab 4 "
            f"restricted-workspace box needs all 3. Fix visibility first.")
    xs = [p[0] for p in marker_positions]
    ys = [p[1] for p in marker_positions]
    return (min(xs), max(xs), min(ys), max(ys))


def in_box(x, y, box):
    xmin, xmax, ymin, ymax = box
    return xmin <= x <= xmax and ymin <= y <= ymax


def in_restricted_xy(x, y, box):
    return in_box(x, y, box) and in_original_xy(x, y)


# =============================================================================
# Grid construction -- unit-agnostic core shared by robot (mm) and offline (px)
# =============================================================================

def cell_is_free(ix, iy, origin, cell, free_corner_fn, obstacles, clearance):
    """
    A cell is free iff all 4 of its corners (a) satisfy free_corner_fn (the
    workspace test) and (b) lie outside every obstacle's blocked disc.

    HONESTY NOTE on corner sampling vs convex boundaries: testing only corners
    under-approximates a convex forbidden region that bulges between two
    corners. The bulge depth over a chord of length d against a boundary of
    curvature radius Rc is d^2/(8*Rc). With d = one cell diagonal (2.83 mm at
    the 2 mm default): vs the inner-hole boundary (Rc=140 mm) that is
    0.0071 mm; vs an inflated obstacle disc (Rc ~ 40-50 mm) it is ~0.025 mm.
    Both are 2-3 orders of magnitude below EXTRA_SAFETY_MM = 2.0 mm, which is
    charged to the inflation budget precisely so this rounding can never
    matter. Numbers stated so the report can state them too.
    """
    x0 = origin[0] + ix * cell
    y0 = origin[1] + iy * cell
    for cx, cy in ((x0, y0), (x0 + cell, y0), (x0, y0 + cell), (x0 + cell, y0 + cell)):
        if not free_corner_fn(cx, cy):
            return False
        for (ox, oy) in obstacles:
            if math.hypot(cx - ox, cy - oy) < clearance:
                return False
    return True


def build_grid(bounds, cell, free_corner_fn, obstacles, clearance):
    """
    bounds = (xmin, xmax, ymin, ymax) of the region to discretize.
    Returns (occ, origin, nx, ny): occ[iy][ix] True = FREE.
    """
    xmin, xmax, ymin, ymax = bounds
    nx = max(1, int(math.ceil((xmax - xmin) / cell)))
    ny = max(1, int(math.ceil((ymax - ymin) / cell)))
    origin = (xmin, ymin)
    occ = [[cell_is_free(ix, iy, origin, cell, free_corner_fn, obstacles, clearance)
            for ix in range(nx)] for iy in range(ny)]
    return occ, origin, nx, ny


def cell_center(ix, iy, origin, cell):
    return (origin[0] + (ix + 0.5) * cell, origin[1] + (iy + 0.5) * cell)


def nearest_free_cell(occ, nx, ny, seed):
    """Expanding ring search from the seed cell to the closest free cell."""
    sx, sy = seed
    sx = min(max(sx, 0), nx - 1)
    sy = min(max(sy, 0), ny - 1)
    if occ[sy][sx]:
        return (sx, sy)
    for radius in range(1, max(nx, ny)):
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if max(abs(dx), abs(dy)) != radius:
                    continue
                x, y = sx + dx, sy + dy
                if 0 <= x < nx and 0 <= y < ny and occ[y][x]:
                    return (x, y)
    return None


# =============================================================================
# A*  -- 8-connected, octile heuristic, no corner cutting
# =============================================================================

SQRT2 = math.sqrt(2)


def octile(a, b):
    dx = abs(a[0] - b[0]); dy = abs(a[1] - b[1])
    return max(dx, dy) + (SQRT2 - 1.0) * min(dx, dy)


def astar(occ, nx, ny, start, goal):
    """
    Returns (path_cells, stats) or (None, stats).
    - Moves: 4 orthogonal (cost 1) + 4 diagonal (cost sqrt2).
    - No corner cutting: the diagonal (dx,dy) is only allowed when BOTH
      orthogonal neighbours (x+dx,y) and (x,y+dy) are free, so the path can
      never squeeze between two diagonally-touching blocked cells / graze an
      obstacle corner.
    - Heuristic: octile distance -- admissible and consistent for exactly this
      move set, hence A* returns a minimum-cost path and never reopens closed
      nodes (the one-line optimality argument for the report).
    """
    if not occ[start[1]][start[0]] or not occ[goal[1]][goal[0]]:
        return None, {"expanded": 0, "cost": None}
    open_heap = [(octile(start, goal), 0.0, start)]
    g = {start: 0.0}
    parent = {start: None}
    closed = set()
    expanded = 0
    while open_heap:
        f, gc, cur = heapq.heappop(open_heap)
        if cur in closed:
            continue
        closed.add(cur)
        expanded += 1
        if cur == goal:
            path = []
            n = cur
            while n is not None:
                path.append(n)
                n = parent[n]
            path.reverse()
            return path, {"expanded": expanded, "cost": g[cur]}
        cx, cy = cur
        for dx, dy, cost in ((1,0,1),(-1,0,1),(0,1,1),(0,-1,1),
                              (1,1,SQRT2),(1,-1,SQRT2),(-1,1,SQRT2),(-1,-1,SQRT2)):
            x, y = cx + dx, cy + dy
            if not (0 <= x < nx and 0 <= y < ny) or not occ[y][x]:
                continue
            if dx != 0 and dy != 0:      # no corner cutting
                if not occ[cy][cx + dx] or not occ[cy + dy][cx]:
                    continue
            ng = g[cur] + cost
            if (x, y) not in g or ng < g[(x, y)] - 1e-12:
                g[(x, y)] = ng
                parent[(x, y)] = cur
                heapq.heappush(open_heap, (ng + octile((x, y), goal), ng, (x, y)))
    return None, {"expanded": expanded, "cost": None}


def decimate(path_cells):
    """Collapse maximal collinear runs; keeps endpoints and turn points only.
    Pure execution-count optimization: the surviving waypoints trace the
    identical polyline, so clearance is untouched (verified by validate_path
    in the self-tests and again at runtime before any motion)."""
    if len(path_cells) <= 2:
        return list(path_cells)
    out = [path_cells[0]]
    for i in range(1, len(path_cells) - 1):
        ax, ay = path_cells[i][0] - out[-1][0], path_cells[i][1] - out[-1][1]
        bx, by = path_cells[i+1][0] - path_cells[i][0], path_cells[i+1][1] - path_cells[i][1]
        if ax * by - ay * bx != 0 or (ax, ay) == (0, 0) or \
           (ax * bx + ay * by) <= 0 or \
           (abs(ax) > 0 and abs(bx) > 0 and (ax // abs(ax) != bx // abs(bx))) or \
           (abs(ay) > 0 and abs(by) > 0 and (ay // abs(ay) != by // abs(by))):
            out.append(path_cells[i])
    out.append(path_cells[-1])
    return out


def validate_path(points_xy, free_corner_fn, obstacles, clearance, step):
    """
    Defense in depth: independently re-check the FINAL polyline by dense
    sampling (every `step` units along every segment) against the same
    workspace test and obstacle discs the grid was built from. Run in the
    self-tests AND immediately before commanding any motion.
    Returns (ok, min_obstacle_distance).
    """
    min_d = float("inf")
    for i in range(len(points_xy) - 1):
        (x0, y0), (x1, y1) = points_xy[i], points_xy[i + 1]
        seg = math.hypot(x1 - x0, y1 - y0)
        n = max(1, int(math.ceil(seg / step)))
        for k in range(n + 1):
            t = k / n
            x, y = x0 + t * (x1 - x0), y0 + t * (y1 - y0)
            if not free_corner_fn(x, y):
                return False, min_d
            for (ox, oy) in obstacles:
                d = math.hypot(x - ox, y - oy)
                min_d = min(min_d, d)
                if d < clearance:
                    return False, min_d
    return True, min_d


# =============================================================================
# Shared planning driver
# =============================================================================

def plan(markers, src_id, dst_id, bounds, cell, free_corner_fn, clearance):
    """
    markers: {id: (x, y)} in whatever plane units the caller works in.
    Obstacles = every detected marker that is neither source nor destination
    (Lab 5: "All other ArUco markers on the page are obstacles").
    Returns dict with cells, waypoints, obstacles, stats -- or raises with a
    plain-language reason.
    """
    if src_id not in markers:
        raise RuntimeError(f"Source marker {src_id} not detected.")
    if dst_id not in markers:
        raise RuntimeError(f"Destination marker {dst_id} not detected.")
    obstacles = [pos for mid, pos in markers.items() if mid not in (src_id, dst_id)]

    t0 = time.time()
    occ, origin, nx, ny = build_grid(bounds, cell, free_corner_fn, obstacles, clearance)

    def to_cell(p):
        return (int((p[0] - origin[0]) / cell), int((p[1] - origin[1]) / cell))

    start = nearest_free_cell(occ, nx, ny, to_cell(markers[src_id]))
    goal = nearest_free_cell(occ, nx, ny, to_cell(markers[dst_id]))
    if start is None or goal is None:
        raise RuntimeError("No free cell near source and/or destination -- "
                           "obstacle inflation may have sealed an endpoint.")

    cells, stats = astar(occ, nx, ny, start, goal)
    stats["planning_time_s"] = time.time() - t0
    if cells is None:
        raise RuntimeError("A* found no path: free space between source and "
                           "destination is fully separated by obstacles/limits.")

    way_cells = decimate(cells)
    waypoints = [cell_center(ix, iy, origin, cell) for ix, iy in way_cells]

    ok, min_d = validate_path(waypoints, free_corner_fn, obstacles, clearance,
                               step=cell / 4.0)
    if not ok:
        raise RuntimeError("Post-plan validation FAILED -- refusing to draw. "
                           "(This should be impossible; investigate before running.)")

    path_len = sum(math.hypot(waypoints[i+1][0]-waypoints[i][0],
                                waypoints[i+1][1]-waypoints[i][1])
                    for i in range(len(waypoints)-1))
    return {
        "waypoints": waypoints,
        "cells": cells,
        "obstacles": obstacles,
        "stats": stats,
        "path_length": path_len,
        "min_obstacle_dist": min_d,
        "clearance_used": clearance,
        "grid_shape": (nx, ny),
    }


# =============================================================================
# OFFLINE MODE -- pixel-coordinate pipeline on the provided field PNGs
# =============================================================================

# Offline demo inflation inputs. These stand in for the bench-measured values
# ONLY so the pipeline can be exercised end-to-end at home, exactly as the prof
# sanctions ("scale ambiguity ... doesn't affect the underlying logic"). They
# are labelled in the output and are NOT defaults for robot mode, which refuses
# to run until the real measured values are typed in.
OFFLINE_DEMO_DOT_ERROR_MM = 3.0
OFFLINE_DEMO_PEN_HALF_WIDTH_MM = 0.75


def detect_markers_in_png(image_path):
    """Detect markers directly in a field PNG. Returns ({id:(cx,cy)px},
    mean_side_px). Uses corner geometry only -- no pose estimation, hence no
    camera matrix and no physical-scale assumption; px-per-mm comes from the
    known 50 mm printed side vs the detected pixel side."""
    img = cv2.imread(image_path)
    if img is None:
        raise RuntimeError(f"Could not read {image_path}")
    d = aruco.getPredefinedDictionary(ARUCO_DICT_ID)
    det = aruco.ArucoDetector(d, aruco.DetectorParameters())
    corners, ids, _ = det.detectMarkers(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
    if ids is None:
        raise RuntimeError("No markers detected in the image.")
    markers, sides = {}, []
    for c, mid in zip(corners, ids.flatten()):
        pts = c[0]
        markers[int(mid)] = (float(pts[:, 0].mean()), float(pts[:, 1].mean()))
        sides.append(float(np.linalg.norm(pts[1] - pts[0])))
    return markers, float(np.mean(sides)), img


def run_offline(image_path, src_id, dst_id, out_path):
    markers, side_px, img = detect_markers_in_png(image_path)
    print(f"Detected markers (px): { {k: (round(v[0],1), round(v[1],1)) for k,v in markers.items()} }")
    print(f"Mean marker side: {side_px:.1f} px  ->  px_per_mm = {side_px/MARKER_SIDE_MM:.3f} "
          f"(from the known {MARKER_SIDE_MM:.0f} mm printed side)")

    px_per_mm = side_px / MARKER_SIDE_MM
    clearance_mm = required_clearance_mm(OFFLINE_DEMO_DOT_ERROR_MM,
                                          OFFLINE_DEMO_PEN_HALF_WIDTH_MM)
    clearance_px = clearance_mm * px_per_mm
    cell_px = GRID_CELL_MM * px_per_mm
    print(f"[OFFLINE-DEMO inflation] clearance = {clearance_mm:.2f} mm = {clearance_px:.1f} px "
          f"(demo dot-error {OFFLINE_DEMO_DOT_ERROR_MM} mm and pen half-width "
          f"{OFFLINE_DEMO_PEN_HALF_WIDTH_MM} mm are stand-ins; the bench run uses "
          f"YOUR measured values)")

    h, w = img.shape[:2]
    margin = cell_px
    bounds = (margin, w - margin, margin, h - margin)

    def free_corner_fn(x, y):
        return bounds[0] <= x <= bounds[1] and bounds[2] <= y <= bounds[3]

    result = plan(markers, src_id, dst_id, bounds, cell_px, free_corner_fn, clearance_px)

    s = result["stats"]
    print(f"\nA*: expanded {s['expanded']} nodes in {s['planning_time_s']*1000:.0f} ms "
          f"on a {result['grid_shape'][0]}x{result['grid_shape'][1]} grid")
    print(f"Path: {len(result['cells'])} cells -> {len(result['waypoints'])} waypoints "
          f"after decimation; length = {result['path_length']/px_per_mm:.1f} mm-equivalent")
    print(f"Validated min distance to any obstacle center: "
          f"{result['min_obstacle_dist']/px_per_mm:.1f} mm-equivalent "
          f"(required >= {clearance_mm:.1f}); clearance beyond the marker square itself: "
          f"{(result['min_obstacle_dist']/px_per_mm - MARKER_HALF_DIAG_MM):.1f} mm-equivalent")

    # annotate
    for (ox, oy) in result["obstacles"]:
        cv2.circle(img, (int(ox), int(oy)), int(clearance_px), (0, 0, 255), 2)
        cv2.circle(img, (int(ox), int(oy)), int(MARKER_HALF_DIAG_MM * px_per_mm), (0, 0, 160), 1)
    pts = np.array([[int(x), int(y)] for x, y in result["waypoints"]])
    cv2.polylines(img, [pts], False, (0, 180, 0), 3)
    for x, y in result["waypoints"]:
        cv2.circle(img, (int(x), int(y)), 4, (255, 0, 0), -1)
    for mid, (mx, my) in markers.items():
        label = "SRC" if mid == src_id else ("DST" if mid == dst_id else "OBS")
        cv2.putText(img, f"{label} {mid}", (int(mx) - 40, int(my) - int(side_px/2) - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)
    cv2.imwrite(out_path, img)
    print(f"Annotated plan saved to {out_path}")
    return result


# =============================================================================
# ROBOT MODE
# =============================================================================

def load_cam2robot_transform():
    candidates = []
    if R_T_DIRECTORY:
        candidates.append(Path(R_T_DIRECTORY).expanduser())
    candidates.extend([
        Path.cwd(),
        Path(__file__).resolve().parent,
        LAB1_SOFTWARE_DIR,
    ])

    seen = set()
    for base in candidates:
        if not base:
            continue
        base = base.resolve()
        if base in seen:
            continue
        seen.add(base)
        r_path = base / "R.npy"
        t_path = base / "T.npy"
        if r_path.exists() and t_path.exists():
            return np.load(r_path), np.load(t_path)

    raise RuntimeError(
        f"R.npy/T.npy not found. Looked in: {', '.join(str(p) for p in candidates)}. "
        f"Run compute_transform.py FRESH this session first."
    )


def check_robot_placeholders():
    missing = []
    if CAMERA_MATRIX is None or DIST_COEFFS is None:
        missing.append("CAMERA_MATRIX / DIST_COEFFS (from calibrate_camera.py)")
    if WORLD_POS_SCALE_TO_MM is None:
        missing.append("WORLD_POS_SCALE_TO_MM (run --debug-print-only to determine)")
    if Z_SAFE_TRAVEL is None or Z_DOT_CONTACT is None:
        missing.append("Z_SAFE_TRAVEL / Z_DOT_CONTACT (DobotLink manual jog ONLY)")
    if WORST_DOT_ERROR_MM is None or PEN_LINE_HALF_WIDTH_MM is None:
        missing.append("WORST_DOT_ERROR_MM / PEN_LINE_HALF_WIDTH_MM "
                       "(measure during the mandatory Lab-4-procedure error pass)")
    if missing:
        raise RuntimeError("Robot mode refused -- unfilled placeholders:\n  - "
                           + "\n  - ".join(missing))


def transform_camera_to_world(X_c, R, T):
    """Identical to find_aruco.py: R @ X_c + T."""
    return R @ X_c + T


def detect_markers_robot(cap, n_frames=5):
    """estimatePoseSingleMarkers, batched per frame, averaged over n_frames --
    the find_aruco.py method, plus averaging because a one-shot debug preview
    and a precision plan have different needs."""
    d = aruco.getPredefinedDictionary(ARUCO_DICT_ID)
    det = aruco.ArucoDetector(d, aruco.DetectorParameters())
    acc = {}
    for _ in range(n_frames):
        ret, frame = cap.read()
        if not ret:
            continue
        corners, ids, _ = det.detectMarkers(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        if ids is None:
            continue
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            corners, MARKER_SIDE_MM / 1000.0, CAMERA_MATRIX, DIST_COEFFS)
        for i, mid in enumerate(ids.flatten()):
            acc.setdefault(int(mid), []).append(tvecs[i].flatten())
    return {mid: np.mean(v, axis=0) for mid, v in acc.items()}


def robot_connect():
    import DobotDllType as dType
    api = dType.load()
    com = dType.SearchDobot(api)
    if not com or "COM" not in com[0]:
        raise RuntimeError("Robot not found.")
    state = None
    for port in com:
        state = dType.ConnectDobot(api, port, 115200)[0]
        if state == dType.DobotConnect.DobotConnect_NoError:
            print(f"Connected on {port}")
            break
    if state != dType.DobotConnect.DobotConnect_NoError:
        raise RuntimeError("Cannot connect to robot.")
    dType.SetQueuedCmdStopExec(api)
    dType.SetQueuedCmdClear(api)
    dType.SetPTPCommonParams(api, 50, 50, isQueued=1)
    dType.SetHOMEParams(api, *OUT_OF_VIEW_POS, 0, isQueued=1)
    cmd = dType.SetHOMECmd(api, temp=0, isQueued=1)[0]
    dType.SetQueuedCmdStartExec(api)
    while cmd > dType.GetQueuedCmdCurrentIndex(api)[0]:
        dType.dSleep(25)
    return api, dType


def move_xyz(api, dType, x, y, z):
    cmd = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, x, y, z, 0, isQueued=0)[0]
    while cmd > dType.GetQueuedCmdCurrentIndex(api)[0]:
        dType.dSleep(25)


def run_robot(src_id, dst_id, trial_num, log_path="lab5_astar_trials.csv"):
    check_robot_placeholders()
    R, T = load_cam2robot_transform()

    api, dType = robot_connect()
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera index {CAMERA_INDEX}.")

    try:
        # -- world model: single capture, robot out of frame (Lab 4 caching rule)
        move_xyz(api, dType, *OUT_OF_VIEW_POS)
        time.sleep(0.5)
        cam = detect_markers_robot(cap)
        if len(cam) < 3:
            raise RuntimeError(f"Only {len(cam)} markers detected; need all 3 "
                               f"for the restricted-workspace box. Fix visibility.")
        markers3 = {mid: (transform_camera_to_world(t, R, T) * WORLD_POS_SCALE_TO_MM)
                    for mid, t in cam.items()}
        markers = {mid: (float(p[0]), float(p[1])) for mid, p in markers3.items()}
        print(f"Markers (robot frame, mm): "
              f"{ {k: (round(v[0],1), round(v[1],1)) for k,v in markers.items()} }")

        box = compute_marker_box(list(markers3.values()))

        def free_corner_fn(x, y):
            return in_restricted_xy(x, y, box)

        clearance = required_clearance_mm(WORST_DOT_ERROR_MM, PEN_LINE_HALF_WIDTH_MM)
        print(f"Inflated obstacle radius: {clearance:.2f} mm "
              f"(= {MARKER_HALF_DIAG_MM:.2f} half-diag + {WORST_DOT_ERROR_MM} err "
              f"+ {PEN_LINE_HALF_WIDTH_MM} pen + {GRID_CELL_MM*SQRT2:.2f} cell "
              f"+ {EXTRA_SAFETY_MM} extra)")

        result = plan(markers, src_id, dst_id, box, GRID_CELL_MM,
                      free_corner_fn, clearance)
        s = result["stats"]
        print(f"A*: {s['expanded']} nodes, {s['planning_time_s']*1000:.0f} ms; "
              f"{len(result['waypoints'])} waypoints; "
              f"length {result['path_length']:.1f} mm; "
              f"min obstacle-center distance {result['min_obstacle_dist']:.1f} mm")

        # -- execute: corrected commands (Lab 4 semantics: commanded = plan + correction)
        wps = [(x + PEN_OFFSET_CORRECTION[0], y + PEN_OFFSET_CORRECTION[1])
               for (x, y) in result["waypoints"]]
        x0, y0 = wps[0]
        move_xyz(api, dType, x0, y0, Z_SAFE_TRAVEL)
        move_xyz(api, dType, x0, y0, Z_DOT_CONTACT)
        for (x, y) in wps[1:]:
            move_xyz(api, dType, x, y, Z_DOT_CONTACT)
        xe, ye = wps[-1]
        move_xyz(api, dType, xe, ye, Z_SAFE_TRAVEL)
        move_xyz(api, dType, *OUT_OF_VIEW_POS)
        print("Path drawn. Robot moved out of view.")

        # -- trial log. Ruler-measured columns left blank on purpose: physical
        #    ground truth comes from the transparency, not from GetPose, which
        #    echoes commanded angles (Labs 2-4 finding) and proves nothing here.
        header = ["trial", "src", "dst", "planning_time_s", "expanded",
                  "path_length_mm", "n_waypoints",
                  "min_planned_clearance_beyond_square_mm",
                  "MEASURED_pass_fail", "MEASURED_min_clearance_mm",
                  "MEASURED_start_offset_mm", "MEASURED_end_offset_mm"]
        new = not os.path.exists(log_path)
        with open(log_path, "a", newline="") as f:
            w = csv.writer(f)
            if new:
                w.writerow(header)
            w.writerow([trial_num, src_id, dst_id,
                        round(s["planning_time_s"], 3), s["expanded"],
                        round(result["path_length"], 1), len(result["waypoints"]),
                        round(result["min_obstacle_dist"] - MARKER_HALF_DIAG_MM, 2),
                        "", "", "", ""])
        print(f"Trial logged to {log_path}. Measure the transparency with a ruler "
              f"and fill in the MEASURED_ columns.")
    finally:
        try:
            move_xyz(api, dType, *OUT_OF_VIEW_POS)
        except Exception:
            pass
        cap.release()


def run_debug_print_only():
    if CAMERA_MATRIX is None or DIST_COEFFS is None:
        raise RuntimeError("Set CAMERA_MATRIX / DIST_COEFFS first.")
    R, T = load_cam2robot_transform()
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera index {CAMERA_INDEX}.")
    cam = detect_markers_robot(cap)
    cap.release()
    if not cam:
        print("No markers detected."); return
    print("Raw R @ X_c + T (BEFORE any scaling):")
    for mid, t in cam.items():
        w = transform_camera_to_world(t, R, T)
        print(f"  marker {mid}: {np.round(w, 4)}   magnitude={np.linalg.norm(w):.4f}")
    print("\nKnown workspace scale: r in [140, 260] mm.")
    print("Magnitudes ~0.15-0.26  -> WORLD_POS_SCALE_TO_MM = 1000.0")
    print("Magnitudes ~150-260    -> WORLD_POS_SCALE_TO_MM = 1.0")


# =============================================================================
# SELF-TESTS
# =============================================================================

def _self_test():
    print("=" * 70)
    print("SELF-TEST SUITE")
    print("=" * 70)
    failures = 0

    def check(name, cond):
        nonlocal failures
        print(f"  [{'PASS' if cond else 'FAIL'}] {name}")
        if not cond:
            failures += 1

    # --- 1. A* on an open grid: octile-optimal straight/diagonal cost ---
    nx = ny = 20
    occ = [[True]*nx for _ in range(ny)]
    path, st = astar(occ, nx, ny, (2, 2), (12, 2))
    check("open grid horizontal: cost == 10", path is not None and abs(st["cost"] - 10) < 1e-9)
    path, st = astar(occ, nx, ny, (2, 2), (10, 10))
    check("open grid diagonal: cost == 8*sqrt2", path is not None and abs(st["cost"] - 8*SQRT2) < 1e-9)

    # --- 2. wall forces detour; path never enters blocked cells ---
    occ = [[True]*nx for _ in range(ny)]
    for y in range(0, 15):
        occ[y][10] = False           # wall with a gap at the top
    path, st = astar(occ, nx, ny, (2, 2), (18, 2))
    ok = path is not None and all(occ[y][x] for x, y in path)
    check("wall detour found, no blocked cell on path", ok)
    check("detour is longer than straight line", st["cost"] > 16)

    # --- 3. sealed goal -> graceful None ---
    occ2 = [[True]*nx for _ in range(ny)]
    gx, gy = 15, 15
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            if (dx, dy) != (0, 0):
                occ2[gy+dy][gx+dx] = False
    path, st = astar(occ2, nx, ny, (2, 2), (gx, gy))
    check("sealed goal returns None (no crash)", path is None)

    # --- 4. no corner cutting ---
    occ3 = [[True]*5 for _ in range(5)]
    occ3[1][2] = False
    occ3[2][1] = False
    path, st = astar(occ3, 5, 5, (1, 1), (3, 3))
    diag_cut = path is not None and ((1,1) in path and (2,2) in path and
                                      path[path.index((1,1))+1] == (2, 2))
    check("diagonal through touching blocked pair is refused", not diag_cut)

    # --- 5. decimation preserves endpoints and validation still passes ---
    occ = [[True]*nx for _ in range(ny)]
    for y in range(5, 20):
        occ[y][9] = False
    path, st = astar(occ, nx, ny, (2, 10), (17, 10))
    check("decimation test path exists", path is not None)
    if path is not None:
        dec = decimate(path)
        check("decimation keeps endpoints", dec[0] == path[0] and dec[-1] == path[-1])
        origin, cell = (0.0, 0.0), 1.0
        pts_full = [cell_center(x, y, origin, cell) for x, y in path]
        pts_dec = [cell_center(x, y, origin, cell) for x, y in dec]
        def ff(x, y): return 0 <= x <= nx and 0 <= y <= ny and \
            occ[min(int(y), ny-1)][min(int(x), nx-1)]
        okf, _ = validate_path(pts_full, ff, [], 0.0, 0.25)
        okd, _ = validate_path(pts_dec, ff, [], 0.0, 0.25)
        check("dense validation passes on full path", okf)
        check("dense validation passes on decimated path", okd)
        check("decimated is not longer than full",
              sum(math.hypot(pts_dec[i+1][0]-pts_dec[i][0], pts_dec[i+1][1]-pts_dec[i][1])
                  for i in range(len(pts_dec)-1))
              <= sum(math.hypot(pts_full[i+1][0]-pts_full[i][0], pts_full[i+1][1]-pts_full[i][1])
                     for i in range(len(pts_full)-1)) + 1e-9)

    # --- 6. robot-mode cell classification: box INTERSECT annulus w/ hole + obstacle ---
    box = (100.0, 260.0, -120.0, 120.0)
    def free_fn(x, y): return in_restricted_xy(x, y, box)
    obstacles = [(200.0, 0.0)]
    clearance = required_clearance_mm(3.0, 0.75)   # synthetic-but-realistic demo numbers
    bounds = box
    occ, origin, gx_, gy_ = build_grid(bounds, 2.0, free_fn, obstacles, clearance)
    def cell_of(x, y): return (int((x-origin[0])/2.0), int((y-origin[1])/2.0))
    cx, cy = cell_of(120.0, 10.0)        # r=120.4 < 140: inner hole
    check("cell in inner hole is blocked", not occ[cy][cx])
    cx, cy = cell_of(200.0, 0.0)         # obstacle center
    check("cell at obstacle center is blocked", not occ[cy][cx])
    cx, cy = cell_of(200.0, 90.0)        # r=219: inside everything, clear of obstacle
    check("cell in free interior is free", occ[cy][cx])
    cx, cy = cell_of(200.0, clearance - 4.0)   # inside inflated disc
    check("cell just inside inflated radius is blocked", not occ[cy][cx])
    cx, cy = cell_of(200.0, clearance + 6.0)   # outside inflated disc
    check("cell just outside inflated radius is free", occ[cy][cx])

    # --- 7. end-to-end plan on the synthetic robot-mode region ---
    markers = {1: (130.0, -80.0), 2: (240.0, 60.0), 3: (200.0, 0.0)}
    # note: marker 1 at r=152.6, marker 2 at r=247.4 -- both inside the annulus
    try:
        res = plan(markers, 1, 2, box, 2.0, free_fn, clearance)
        beyond = res["min_obstacle_dist"] - MARKER_HALF_DIAG_MM
        check("synthetic plan validates", True)
        check(f"min clearance beyond obstacle square ({beyond:.1f} mm) >= "
              f"error+pen+cell margins ({clearance - MARKER_HALF_DIAG_MM - EXTRA_SAFETY_MM:.1f})",
              res["min_obstacle_dist"] >= clearance - 1e-9)
        check("plan endpoints near markers (<6 mm)",
              math.hypot(res["waypoints"][0][0]-markers[1][0],
                          res["waypoints"][0][1]-markers[1][1]) < 6.0 and
              math.hypot(res["waypoints"][-1][0]-markers[2][0],
                          res["waypoints"][-1][1]-markers[2][1]) < 6.0)
    except RuntimeError as e:
        print(f"  [FAIL] synthetic plan raised: {e}")
        failures += 1

    # --- 8. refusal behaviors ---
    try:
        check_robot_placeholders()
        check("robot placeholders refusal fires", False)
    except RuntimeError:
        check("robot placeholders refusal fires", True)
    try:
        load_cam2robot_transform()
        r_missing = not (os.path.exists(os.path.join(R_T_DIRECTORY, "R.npy")))
        check("R/T load behaves (files present)", not r_missing)
    except RuntimeError:
        check("R/T missing-file refusal fires", True)

    print("-" * 70)
    print(f"{'ALL TESTS PASSED' if failures == 0 else f'{failures} FAILURE(S)'}")
    return failures == 0


# =============================================================================
# CLI
# =============================================================================

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--self-test", action="store_true")
    p.add_argument("--offline", metavar="FIELD_PNG")
    p.add_argument("--debug-print-only", action="store_true")
    p.add_argument("--run", action="store_true")
    p.add_argument("--src", type=int)
    p.add_argument("--dst", type=int)
    p.add_argument("--trial", type=int, default=1)
    p.add_argument("--out", default="lab5_offline_plan.png")
    a = p.parse_args()

    if a.self_test:
        ok = _self_test()
        raise SystemExit(0 if ok else 1)
    if a.offline:
        if a.src is None or a.dst is None:
            p.error("--offline requires --src and --dst")
        run_offline(a.offline, a.src, a.dst, a.out)
        return
    if a.debug_print_only:
        run_debug_print_only()
        return
    if a.run:
        if a.src is None or a.dst is None:
            p.error("--run requires --src and --dst")
        run_robot(a.src, a.dst, a.trial)
        return
    p.print_help()


if __name__ == "__main__":
    main()
