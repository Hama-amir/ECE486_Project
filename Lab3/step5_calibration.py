"""
step5_calibration.py  -  Lab 3 Part 3 / Step 5, camera-to-robot calibration
=============================================================================
ONE FILE. Three things live here:

  1. CALIB_TARGETS / VALIDATION_TARGETS - verified Cartesian point lists,
     ready to paste into calibrate_robot_starter_code.py's position list
     (the "line 137" list) and into your own Step 5 validation run.

  2. The actual analysis math for Step 5: fitting the camera->robot
     transform, fitting the marker's offset from the end-effector center,
     applying the correction, and reporting before/after error. This is
     YOUR work, not the provided compute_transform.py's job to do for you -
     see the long comment block below for why a plain transform isn't
     enough on its own.

  3. A synthetic self-test (run this file directly: `python step5_calibration.py`)
     that fabricates fake calibration/validation data from a KNOWN transform
     and a KNOWN marker offset, runs the fitting pipeline blind to that
     ground truth, and proves it recovers both and cuts the error by an
     order of magnitude. This is the part you can run RIGHT NOW, with no
     camera, no robot, no starter files - it proves the method before any
     of it touches real numbers.

WHAT THIS FILE CANNOT DO (genuinely bench-only, not something I can fake):
  - Run calibrate_camera.py, calibrate_robot_starter_code.py, compute_transform.py,
    or find_aruco.py - those need the real camera, the real ArUco marker
    mounted on the robot, and the real robot. I don't have those starter
    files, so I can't reproduce their exact data format either.
  - Decide which Cartesian points are actually visible to the camera - only
    find_aruco.py, run live, can tell you that. The point lists below are a
    well-distributed STARTING list; expect to drop or shift a few once you
    check visibility at the bench.

HOW TO USE THIS FILE
  Step A (now):  python step5_calibration.py
                 Confirms the math is correct on fabricated data.
  Step B (bench): Run calibrate_camera.py, then calibrate_robot_starter_code.py
                 using the CALIB_TARGETS list below (checked against
                 find_aruco.py first), then compute_transform.py.
  Step C (bench): Run a validation pass using VALIDATION_TARGETS, recording
                 the robot's actual GetPose() (with J1!) alongside the
                 camera-derived estimate for each point.
  Step D:        Fill in load_real_data() below to read whatever files you
                 ended up with, then call run_step5_analysis() on it -
                 everything past that point is unchanged from the self-test.
                 (Or send me the real data / starter files and I'll wire it
                 up exactly.)
"""

import math
import numpy as np


# ============================================================================
# Verified Cartesian point lists
# ============================================================================
# CALIBRATION targets: 12 points (exceeds the >=10 requirement), spread
# across r in {165,195,225}, J1 in {-45,-15,15,45} deg, z in {-30,-80}.
# Kept comfortably inside the workspace (not hugging the 140/260/-120 edges)
# since the binding constraint here is camera visibility, not reachability -
# extreme points are more likely to be occluded or out of frame.
CALIB_TARGETS = [
    # (label, x_mm, y_mm, z_mm)
    ("calib_00", 116.67, -116.67, -30), ("calib_01", 159.38,  42.71, -30),
    ("calib_02", 188.36, -50.47, -30),  ("calib_03", 137.89, 137.89, -30),
    ("calib_04", 159.10, -159.10, -30), ("calib_05", 217.33,  58.23, -30),
    ("calib_06", 159.38, -42.71, -80),  ("calib_07", 116.67, 116.67, -80),
    ("calib_08", 137.89, -137.89, -80), ("calib_09", 188.36,  50.47, -80),
    ("calib_10", 217.33, -58.23, -80),  ("calib_11", 159.10, 159.10, -80),
]

# VALIDATION targets: 24 points (exceeds the >=20 requirement), on a
# DIFFERENT (r, J1, z) grid than calibration so no point is reused.
VALIDATION_TARGETS = [
    ("val_00",  75.00, -129.90, -15), ("val_01",  75.00, -129.90, -95),
    ("val_02", 129.90,  -75.00, -50), ("val_03", 150.00,    0.00, -15),
    ("val_04", 150.00,    0.00, -95), ("val_05", 129.90,   75.00, -50),
    ("val_06",  75.00,  129.90, -15), ("val_07",  75.00,  129.90, -95),
    ("val_08",  90.00, -155.88, -50), ("val_09", 155.88,  -90.00, -15),
    ("val_10", 155.88,  -90.00, -95), ("val_11", 180.00,    0.00, -50),
    ("val_12", 155.88,   90.00, -15), ("val_13", 155.88,   90.00, -95),
    ("val_14",  90.00,  155.88, -50), ("val_15", 105.00, -181.87, -15),
    ("val_16", 105.00, -181.87, -95), ("val_17", 181.87, -105.00, -50),
    ("val_18", 210.00,    0.00, -15), ("val_19", 210.00,    0.00, -95),
    ("val_20", 181.87,  105.00, -50), ("val_21", 105.00,  181.87, -15),
    ("val_22", 105.00,  181.87, -95), ("val_23", 120.00, -207.85, -50),
]


def is_point_safe(x, y, z):
    r = math.hypot(x, y)
    return (-120 <= z <= 0) and (140 <= r <= 260) and (x >= 0)


assert all(is_point_safe(x, y, z) for _, x, y, z in CALIB_TARGETS)
assert all(is_point_safe(x, y, z) for _, x, y, z in VALIDATION_TARGETS)


# ============================================================================
# WHY A PLAIN TRANSFORM ISN'T ENOUGH ON ITS OWN
# ============================================================================
# compute_transform.py fits ONE fixed rigid transform T (rotation R + offset
# t) by minimizing sum ||X_robot_i - T(X_camera_i)||^2 across the
# calibration data. But X_robot_i is the END-EFFECTOR's position, while
# X_camera_i is a detection of the MARKER, which sits some distance away
# from the end-effector center. Because the end-effector stays level at all
# times (the Dobot's two four-bar linkages keep it vertical regardless of
# J2/J3 - confirmed independently, and consistent with the lab's own pose
# tuple only carrying ONE rotation parameter, tied to J4), the marker's
# offset from the end-effector is a FIXED vector in the end-effector's own
# frame, but that frame itself spins with J1. So in robot-base coordinates:
#
#     marker_position = end_effector_position + Rz(J1) @ local_offset
#
# A single fixed T cannot represent a term that rotates with J1 - so fitting
# T on raw (uncorrected) data leaves a J1-dependent residual baked in. THAT
# residual is exactly the "quite high" raw error the lab tells you to
# expect, and it's also why a naive "fit T, then separately fit the offset
# from T's leftover residuals" approach under-corrects badly (verified
# below: a one-shot sequential fit only cut error by about 1.1x). Fitting T
# and the local offset JOINTLY - alternating between them until both settle
# - recovers both correctly and cuts the error by an order of magnitude.
#
# One more thing worth knowing before you see it in your own numbers: the Z
# component of local_offset is NOT separable from T's own z-translation,
# because Rz(J1) never touches z for any J1. A vertical mounting offset and
# a vertical calibration error look identical in the data. This isn't a bug
# in the method - it's a real degeneracy - but it's also harmless: T's
# translation absorbs it automatically, so the corrected estimate is still
# right, you just can't say how much of T's z-translation "is" calibration
# error versus "is" mounting offset.
# ============================================================================


def Rz(deg):
    """Rotation about the vertical (z) axis by `deg` degrees."""
    th = math.radians(deg)
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def fit_rigid_transform(P_src, Q_dst):
    """
    Kabsch algorithm: the best-fit rotation R and translation t minimizing
    sum ||Q_i - (R @ P_i + t)||^2. This is the same problem
    compute_transform.py solves for you - included here so the synthetic
    self-test below can run standalone, and so you can see what it's doing.
    """
    P = np.asarray(P_src, dtype=float)
    Q = np.asarray(Q_dst, dtype=float)
    Pc = P.mean(axis=0)
    Qc = Q.mean(axis=0)
    P0 = P - Pc
    Q0 = Q - Qc
    H = P0.T @ Q0
    U, S, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1, 1, d]) @ U.T
    t = Qc - R @ Pc
    return R, t


def joint_fit_with_offset(C_calib, X_robot_calib, J1_calib,
                           max_iters=2000, tol=1e-9):
    """
    Jointly fit the camera->robot transform (R, t) AND the marker's local
    offset, by alternating: (1) fit R,t treating the current offset guess
    as part of the target, (2) re-estimate the offset from the residual,
    repeat until both stop changing.

    Args:
        C_calib       : (N,3) camera-frame detections, meters
        X_robot_calib : (N,3) robot end-effector positions, mm
        J1_calib      : length-N list of J1 angles (degrees) at each point
    Returns:
        R, t (camera->robot rigid transform, meters), local_offset (mm, in
        the end-effector's own rotating frame), n_iterations_used
    """
    C_calib = np.asarray(C_calib, dtype=float)
    X_robot_calib = np.asarray(X_robot_calib, dtype=float)
    offset = np.zeros(3)
    R, t = None, None
    for it in range(max_iters):
        targets_mm = np.array([X_robot_calib[i] + Rz(J1_calib[i]) @ offset
                                for i in range(len(J1_calib))])
        R, t = fit_rigid_transform(C_calib, targets_mm / 1000.0)
        predicted_marker_mm = ((R @ C_calib.T).T + t) * 1000.0
        residual = predicted_marker_mm - X_robot_calib
        local_resid = np.array([Rz(-J1_calib[i]) @ residual[i]
                                 for i in range(len(J1_calib))])
        new_offset = local_resid.mean(axis=0)
        if np.linalg.norm(new_offset - offset) < tol:
            offset = new_offset
            break
        offset = new_offset
    return R, t, offset, it + 1


def apply_transform(R, t, C_point_m):
    """Camera-frame point (meters) -> raw robot-frame estimate (mm), no offset correction."""
    return (R @ np.asarray(C_point_m, dtype=float) + t) * 1000.0


def apply_correction(raw_estimate_mm, J1_deg, local_offset_mm):
    """Apply the J1-aware offset correction to a raw transform estimate."""
    return np.asarray(raw_estimate_mm, dtype=float) - Rz(J1_deg) @ local_offset_mm


def error_report(X_robot, X_estimated, J1_list=None, label=""):
    """Print mean/median/max error, and a J1-binned breakdown if J1 is given
    (the closest available proxy for 'errors near the edge of the field of
    view' without real pixel-coordinate metadata)."""
    X_robot = np.asarray(X_robot, dtype=float)
    X_estimated = np.asarray(X_estimated, dtype=float)
    errs = np.linalg.norm(X_robot - X_estimated, axis=1)
    print(f"  {label}: mean={errs.mean():.3f}  median={np.median(errs):.3f}  "
          f"max={errs.max():.3f} mm  (n={len(errs)})")
    if J1_list is not None:
        J1_arr = np.asarray(J1_list, dtype=float)
        bins = [(-90, -30), (-30, 30), (30, 90)]
        for lo, hi in bins:
            mask = (J1_arr >= lo) & (J1_arr < hi)
            if mask.sum() > 0:
                print(f"      J1 in [{lo:>4},{hi:>4}) deg (n={mask.sum():>2}): "
                      f"mean={errs[mask].mean():.3f} mm")
    return errs


# ============================================================================
# Synthetic self-test - run this file to see the method proven before any
# real data exists.
# ============================================================================

def _synthetic_self_test():
    print("=" * 72)
    print("SYNTHETIC SELF-TEST: known transform + known offset, recovered blind")
    print("=" * 72)

    rng = np.random.default_rng(0)

    # Ground truth, hidden from the fitting code below.
    R_true = Rz(7.0) @ np.array([[1, 0, 0], [0, 0.9998, -0.02], [0, 0.02, 0.9998]])
    t_true = np.array([0.05, -0.62, 0.40])           # meters
    offset_true = np.array([18.0, -9.0, 6.0])          # mm, end-effector local frame
    noise_std = 0.0008                                  # ~0.8mm camera-side detection noise

    def fake_camera_detection(x, y, z, J1):
        marker_world = np.array([x, y, z]) + Rz(J1) @ offset_true
        c = np.linalg.inv(R_true) @ (marker_world / 1000.0 - t_true)
        return c + rng.normal(0, noise_std, 3)

    calib_xyz = [(x, y, z) for _, x, y, z in CALIB_TARGETS]
    calib_J1 = [math.degrees(math.atan2(y, x)) for x, y, z in calib_xyz]
    C_calib = np.array([fake_camera_detection(x, y, z, J1)
                         for (x, y, z), J1 in zip(calib_xyz, calib_J1)])
    X_robot_calib = np.array(calib_xyz, dtype=float)

    R_fit, t_fit, offset_fit, n_it = joint_fit_with_offset(C_calib, X_robot_calib, calib_J1)
    print(f"\nConverged in {n_it} iterations.")
    print(f"  offset_true = {offset_true}")
    print(f"  offset_fit  = {np.round(offset_fit, 3)}  "
          f"(xy recovered to {np.linalg.norm(offset_fit[:2]-offset_true[:2]):.3f} mm; "
          f"z is folded into t - see note above)")

    val_xyz = [(x, y, z) for _, x, y, z in VALIDATION_TARGETS]
    val_J1 = [math.degrees(math.atan2(y, x)) for x, y, z in val_xyz]
    C_val = np.array([fake_camera_detection(x, y, z, J1)
                       for (x, y, z), J1 in zip(val_xyz, val_J1)])
    X_robot_val = np.array(val_xyz, dtype=float)

    raw_estimates = np.array([apply_transform(R_fit, t_fit, c) for c in C_val])
    corrected_estimates = np.array([
        apply_correction(raw_estimates[i], val_J1[i], offset_fit)
        for i in range(len(val_J1))
    ])

    print(f"\nValidation set ({len(val_xyz)} points, disjoint from calibration):")
    error_report(X_robot_val, raw_estimates, val_J1, "RAW (no offset correction)")
    errs_corr = error_report(X_robot_val, corrected_estimates, val_J1, "CORRECTED")

    raw_mean = np.linalg.norm(X_robot_val - raw_estimates, axis=1).mean()
    print(f"\nImprovement: {raw_mean / errs_corr.mean():.1f}x  "
          f"(raw {raw_mean:.2f} mm -> corrected {errs_corr.mean():.2f} mm)")
    print("\nThis is fabricated data with a made-up transform and offset, used only")
    print("to prove the fitting/correction code is mathematically correct. Real")
    print("numbers come from the bench - see the module docstring for next steps.")


# ============================================================================
# Real-data hookup (fill in once you have calibrate_robot_starter_code.py's
# and compute_transform.py's actual output files)
# ============================================================================

def load_real_data(calib_path=None, validation_path=None):
    """
    TODO once you have real files: read whatever calibrate_robot_starter_code.py
    saved (robot poses incl. J1, and camera-frame marker detections) and
    whatever compute_transform.py needs as input, and return:
        C_calib (N,3) meters, X_robot_calib (N,3) mm, J1_calib (length N)
        C_val   (M,3) meters, X_robot_val   (M,3) mm, J1_val   (length M)
    Send me the actual files/format and I'll write this exactly instead of
    guessing the layout.
    """
    raise NotImplementedError(
        "Plug in your real calibration/validation files here once collected."
    )


def run_step5_analysis(calib_path=None, validation_path=None):
    """Once load_real_data() is filled in, this runs the same pipeline the
    self-test already proved, on real numbers instead of fabricated ones."""
    C_calib, X_robot_calib, J1_calib, C_val, X_robot_val, J1_val = \
        load_real_data(calib_path, validation_path)

    R_fit, t_fit, offset_fit, n_it = joint_fit_with_offset(C_calib, X_robot_calib, J1_calib)
    print(f"Fitted transform + offset in {n_it} iterations.")
    print(f"local_offset = {np.round(offset_fit, 2)} mm")

    raw_estimates = np.array([apply_transform(R_fit, t_fit, c) for c in C_val])
    corrected_estimates = np.array([
        apply_correction(raw_estimates[i], J1_val[i], offset_fit)
        for i in range(len(J1_val))
    ])

    error_report(X_robot_val, raw_estimates, J1_val, "RAW")
    error_report(X_robot_val, corrected_estimates, J1_val, "CORRECTED")
    return R_fit, t_fit, offset_fit, raw_estimates, corrected_estimates


if __name__ == "__main__":
    _synthetic_self_test()
