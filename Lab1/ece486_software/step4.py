"""
step4.py  -  Lab 3 Step 4, all in one file  (ECE 486)
========================================================================
Validates IK and FK together: for each target (x,y,z), runs IK to get
joint angles, checks the joint-space workspace, moves the robot, applies
FK to predict where it should have landed, and compares against what the
robot actually reports.

ONE FILE, TWO PLACES. Drop this exact file into EITHER folder:
  - the sim repo (next to sim_starter.py)        -> runs in the simulator
  - the real robot folder (next to the DLLs)      -> runs on the real Dobot
It auto-detects which one it's in by trying to import DobotDllType, and
runs the matching pipeline. Nothing else to configure.

Run:
  Sim:   uv run step4.py            (add --headless to skip the viewer)
  Real:  python step4.py
Output: lab3_step4_results.csv either way.
"""

import argparse
import csv
import math

# ── Detect which environment this copy of the file is sitting in ─────────────
try:
    import DobotDllType as dType
    MODE = "real"
except ImportError:
    MODE = "sim"


# ============================================================================
# SHARED MATH  (identical in both modes -- this is the actual Lab 2/3 content)
# ============================================================================

L1, L2, L3 = 135.0, 147.0, 59.7   # link lengths, mm (Lab 2)


def forward_kinematics(J1_deg, J2_deg, pos6_deg):
    """Lab 2 Step 2. (J1, J2, pos6) -> (x, y, z) in mm."""
    J1 = math.radians(J1_deg)
    J2 = math.radians(J2_deg)
    pos6 = math.radians(pos6_deg)
    r = L1 * math.sin(J2) + L2 * math.cos(pos6) + L3
    x = r * math.cos(J1)
    y = r * math.sin(J1)
    z = L1 * math.cos(J2) - L2 * math.sin(pos6)
    return (x, y, z)


def inverse_kinematics_full(x, y, z):
    """
    Lab 3 Step 2. (x, y, z) -> dict with J1, J2, pos6, and BOTH J3 conventions.

    phi      = this lab's J3 (angle between the links, from the law of cosines)
    J3_lab2  = Lab 2's J3 convention (J2 - pos6); use THIS one when calling
               joint_workspace_check below. phi and J3_lab2 differ by a
               constant 90 degrees -- mixing them up silently gives wrong
               results, so both are returned explicitly rather than just one.
    """
    J1 = math.degrees(math.atan2(y, x))

    r = math.hypot(x, y)
    r_prime = r - L3
    z_prime = z

    D = math.hypot(r_prime, z_prime)
    reachable = (abs(L1 - L2) <= D <= (L1 + L2))
    if not reachable:
        return {'J1': J1, 'J2': None, 'pos6': None, 'phi': None,
                'J3_lab2': None, 'D': D, 'reachable': False}

    cos_phi = max(-1.0, min(1.0, (L1**2 + L2**2 - D**2) / (2*L1*L2)))
    phi = math.degrees(math.acos(cos_phi))

    cos_beta = max(-1.0, min(1.0, (L1**2 + D**2 - L2**2) / (2*L1*D)))
    beta = math.degrees(math.acos(cos_beta))

    alpha = math.degrees(math.atan2(z_prime, r_prime))

    theta_upper = alpha + beta
    J2 = 90 - theta_upper
    pos6 = 180 - phi - theta_upper
    J3_lab2 = J2 - pos6

    return {'J1': J1, 'J2': J2, 'pos6': pos6, 'phi': phi,
            'J3_lab2': J3_lab2, 'D': D, 'reachable': True}


def is_point_safe(x, y, z):
    """Lab 1 Cartesian workspace check."""
    r = math.hypot(x, y)
    return (-120 <= z <= 0) and (140 <= r <= 260) and (x >= 0)


def joint_workspace_check(J1_deg, J2_deg, J3_lab2_deg):
    """Lab 2 Step 5: (J1,J2,J3) -> FK -> Cartesian workspace check."""
    pos6 = J2_deg - J3_lab2_deg
    x, y, z = forward_kinematics(J1_deg, J2_deg, pos6)
    return is_point_safe(x, y, z), (x, y, z)


# Verified Step 4 test set -- see STEP4_TARGETS comment for the rationale.
# 3 interior points, 2 near the outer radius limit, 2 near the inner radius
# limit (mechanically hardest, forces pos6 near/past 90 deg), 1 at the z
# floor, 1 near the z ceiling, 1 on the x=0 boundary, and 2 combined-extreme
# points where two constraints bind at once. Each was built from an exact
# (r, J1, z) so the radius/depth land precisely on the intended boundary,
# then confirmed in-workspace, IK-reachable, and round-trips through FK to
# floating-point precision.
STEP4_TARGETS = [
    # The 5 original targets that ran clean on the real robot.
    ("Interior, centered",           200.0,   0.0,  -50),
    ("Interior, off-axis (+J1)",     181.26,  84.52, -60),
    ("Interior, off-axis (-J1)",     181.26, -84.52, -60),
    ("Outer radius, mid depth",      254.82,  40.36, -50),
    ("Outer radius, shallow",        257.37,  18.00,  -8),
    # 7 replacements, all with pos6 < 78 deg and J2 < 75 deg -- the range
    # confirmed safe by the first real-robot run. The original inner-radius
    # targets (r~140mm) required pos6 > 90 deg which is beyond the Dobot's
    # physical forearm range; the z-floor targets required J2 > 78 deg,
    # also at or beyond the hardware limit. Both faulted the robot.
    ("Deep z, forward",              220.0,    0.0,  -80),   # J2=62.9 pos6=74.2
    ("Deep z, rotated",              200.84,  75.18, -80),   # J2=62.3 pos6=76.1
    ("Shallow z, variety",           230.0,   60.0,  -20),   # J2=42.8 pos6=54.1
    ("High J1 (J1=70), mid depth",    68.40, 187.94, -50),   # J2=47.5 pos6=73.9
    ("Mid z, off-axis",              200.0,  100.0,  -60),   # J2=55.2 pos6=68.8
    ("Deeper, near-outer radius",    210.0,    0.0,  -63),   # J2=54.5 pos6=74.1
    ("Outer radius, deep",           245.0,   50.0,  -65),   # J2=61.9 pos6=61.0
]

# Movement success threshold (mm). If the actual pose is further from the
# FK-predicted pose than this after a move_joint_angles call, the robot did
# not reach its target -- it faulted, hit a joint limit, or stopped early.
# A single such failure can leave the robot stuck, causing all subsequent
# GetPose calls to return the same wrong position. Detecting it early and
# going home to recover prevents the cascade that silently invalidates
# every remaining data point.
MOVE_TOLERANCE_MM = 5.0


# ============================================================================
# ROBOT INTERFACE  (the only part that differs between sim and real)
# ============================================================================

class Robot:
    """Thin uniform wrapper so the Step 4 pipeline below never has to know
    whether it's talking to the simulator or the real Dobot."""

    def __init__(self, args):
        self.mode = MODE
        if self.mode == "sim":
            from sim_starter import create_sim_api, initialize_robot as sim_init
            from dobot_sim_api import (move_joint_angles as sim_move_joint,
                                        get_pose as sim_get_pose,
                                        move_to_home as sim_home,
                                        stop_pump as sim_stop)
            self._api = create_sim_api(seed=args.seed, headless=args.headless)
            sim_init(self._api)
            self._move_joint = sim_move_joint
            self._get_pose = sim_get_pose
            self._home = sim_home
            self._stop = sim_stop
            print(f"[step4] running in SIMULATOR mode")
        else:
            self._api = dType.load()
            self._home_pos = [200, 100, 50]
            self._connect_real_robot()
            print(f"[step4] running in REAL ROBOT mode")

    # ---- real-robot-only setup (mirrors ece486_starter_code.py) ----
    def _connect_real_robot(self):
        api = self._api
        com_port = dType.SearchDobot(api)
        print(com_port)
        if "COM" not in com_port[0]:
            print("Error: robot not found. Exiting.")
            exit()
        state = dType.DobotConnect.DobotConnect_NoError
        for i in range(len(com_port)):
            state_full = dType.ConnectDobot(api, com_port[i], 115200)
            state = state_full[0]
            if state == dType.DobotConnect.DobotConnect_NoError:
                print("Connected!")
                break
        if state != dType.DobotConnect.DobotConnect_NoError:
            print("Cannot connect. Exiting.")
            exit()
        dType.SetQueuedCmdStopExec(api)
        dType.SetQueuedCmdClear(api)
        dType.SetPTPCommonParams(api, 50, 50, isQueued=1)
        dType.SetHOMEParams(api, *self._home_pos, 0, isQueued=1)
        execCmd = dType.SetHOMECmd(api, temp=0, isQueued=1)[0]
        dType.SetQueuedCmdStartExec(api)
        while execCmd > dType.GetQueuedCmdCurrentIndex(api)[0]:
            dType.dSleep(25)
        print("Robot ready.")

    # ---- uniform interface used by the pipeline below ----
    def move_joint(self, J1, J2, pos6):
        if self.mode == "sim":
            self._move_joint(self._api, J1, J2, pos6, 0.0)
        else:
            execCmd = dType.SetPTPCmd(
                self._api, dType.PTPMode.PTPMOVJANGLEMode,
                J1, J2, pos6, 0, isQueued=0)[0]
            while execCmd > dType.GetQueuedCmdCurrentIndex(self._api)[0]:
                dType.dSleep(25)

    def get_pose_xyz(self):
        if self.mode == "sim":
            pose = self._get_pose(self._api)
        else:
            pose = dType.GetPose(self._api)   # [x,y,z,r,J1,J2,pos6,J4]
        return float(pose[0]), float(pose[1]), float(pose[2])

    def go_home(self):
        if self.mode == "sim":
            self._home(self._api)
        else:
            x, y, z = self._home_pos
            execCmd = dType.SetPTPCmd(
                self._api, dType.PTPMode.PTPMOVLXYZMode, x, y, z, 0, isQueued=0)[0]
            while execCmd > dType.GetQueuedCmdCurrentIndex(self._api)[0]:
                dType.dSleep(25)

    def cleanup(self):
        if self.mode == "sim":
            self._stop(self._api)
            if self._api.viewer is not None:
                self._api.viewer.close()
            self._api.env.close()
        # real robot: nothing extra needed to clean up


# ============================================================================
# STEP 4 PIPELINE  (mode-agnostic -- written once, runs the same way in both)
# ============================================================================

def run_step4(robot, targets, out_path="lab3_step4_results.csv"):
    rows = []
    n_ok, n_rejected_ws, n_unreachable = 0, 0, 0

    print("\n" + "=" * 78)
    print(f"LAB 3 STEP 4  --  IK + FK validation  [{robot.mode.upper()} mode]")
    print("=" * 78)

    for label, x, y, z in targets:
        print(f"\n--- {label}: target ({x:.2f}, {y:.2f}, {z:.2f}) ---")

        ik = inverse_kinematics_full(x, y, z)
        if not ik['reachable']:
            print(f"  UNREACHABLE (D={ik['D']:.2f} mm) -- skipped, robot not moved.")
            n_unreachable += 1
            continue

        J1, J2, pos6, J3_lab2 = ik['J1'], ik['J2'], ik['pos6'], ik['J3_lab2']
        print(f"  IK -> J1={J1:.3f}  J2={J2:.3f}  pos6={pos6:.3f}  "
              f"(phi={ik['phi']:.3f}, J3_lab2={J3_lab2:.3f})")

        ws_ok, ws_xyz = joint_workspace_check(J1, J2, J3_lab2)
        if not ws_ok:
            print(f"  REJECTED by joint_workspace_check at predicted "
                  f"({ws_xyz[0]:.2f},{ws_xyz[1]:.2f},{ws_xyz[2]:.2f}) -- robot not moved.")
            n_rejected_ws += 1
            continue

        pred_x, pred_y, pred_z = forward_kinematics(J1, J2, pos6)

        robot.move_joint(J1, J2, pos6)
        act_x, act_y, act_z = robot.get_pose_xyz()

        # Stuck detection: if the robot faulted or hit a joint limit, the
        # actual position will be far from the FK-predicted position.
        # GetPose in that case keeps returning wherever the robot stopped,
        # and every subsequent move also silently fails. Catch it here so
        # one bad point doesn't corrupt all remaining data.
        move_err = math.sqrt((pred_x-act_x)**2 + (pred_y-act_y)**2 + (pred_z-act_z)**2)
        if move_err > MOVE_TOLERANCE_MM:
            print(f"  MOVE FAILED (predicted {pred_x:.2f},{pred_y:.2f},{pred_z:.2f} "
                  f"but got {act_x:.2f},{act_y:.2f},{act_z:.2f}, err={move_err:.2f} mm)")
            print(f"  This likely means the robot hit a joint limit. Going home to recover.")
            robot.go_home()
            n_rejected_ws += 1   # reuse the counter -- report as 'failed moves'
            continue

        err_vs_predicted = math.sqrt((pred_x-act_x)**2 + (pred_y-act_y)**2 + (pred_z-act_z)**2)
        err_vs_target    = math.sqrt((x-act_x)**2 + (y-act_y)**2 + (z-act_z)**2)

        print(f"  Predicted (FK of IK output): ({pred_x:.4f}, {pred_y:.4f}, {pred_z:.4f})")
        print(f"  Actual (robot pose):         ({act_x:.4f}, {act_y:.4f}, {act_z:.4f})")
        print(f"  error vs predicted = {err_vs_predicted:.4e} mm   "
              f"error vs ORIGINAL target = {err_vs_target:.4e} mm")

        rows.append([label, x, y, z, J1, J2, pos6, J3_lab2,
                     pred_x, pred_y, pred_z, act_x, act_y, act_z,
                     err_vs_predicted, err_vs_target])
        n_ok += 1
        robot.go_home()

    with open(out_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["label", "target_x", "target_y", "target_z", "J1", "J2", "pos6", "J3_lab2",
                    "pred_x", "pred_y", "pred_z", "act_x", "act_y", "act_z",
                    "err_vs_predicted_mm", "err_vs_target_mm"])
        w.writerows(rows)

    print("\n" + "=" * 78)
    print("SUMMARY")
    print("=" * 78)
    print(f"  Targets total:        {len(targets)}")
    print(f"  Successfully tested:  {n_ok}")
    print(f"  Unreachable (IK):     {n_unreachable}")
    print(f"  Rejected (workspace/fault): {n_rejected_ws}")
    if rows:
        errs_pred = [r[14] for r in rows]
        errs_tgt  = [r[15] for r in rows]
        print(f"  err vs predicted:       mean={sum(errs_pred)/len(errs_pred):.3e}  "
              f"max={max(errs_pred):.3e} mm")
        print(f"  err vs ORIGINAL target: mean={sum(errs_tgt)/len(errs_tgt):.3e}  "
              f"max={max(errs_tgt):.3e} mm")
    print(f"\nResults saved to {out_path}")
    return rows


# ============================================================================
# MAIN
# ============================================================================

def main():
    parser = argparse.ArgumentParser(description="Lab 3 Step 4 (auto-detects sim vs real robot).")
    parser.add_argument("--seed", type=int, default=0, help="sim only")
    parser.add_argument("--headless", action="store_true", help="sim only")
    parser.add_argument("--out", type=str, default="lab3_step4_results.csv")
    args = parser.parse_args()

    robot = Robot(args)
    try:
        run_step4(robot, STEP4_TARGETS, args.out)
        robot.go_home()
    finally:
        robot.cleanup()


if __name__ == "__main__":
    main()
