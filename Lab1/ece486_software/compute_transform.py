import os
import numpy as np


def find_rigid_transform(X_c, X_r):
    """Computes optimal rotation (R) and translation (T) from camera frame to world frame."""
    if X_c.ndim != 2 or X_r.ndim != 2:
        raise ValueError(f"Expected 2D point arrays, got shapes {X_c.shape} and {X_r.shape}")
    if X_c.shape != X_r.shape:
        raise ValueError(f"Camera and robot point arrays must have the same shape, got {X_c.shape} and {X_r.shape}")
    if X_c.shape[0] < 3:
        raise ValueError(f"Need at least 3 matching point pairs, got {X_c.shape[0]}")
    if X_c.shape[1] != 3:
        raise ValueError(f"Expected 3D points, got shape {X_c.shape}")

    centroid_c = np.mean(X_c, axis=0)
    centroid_r = np.mean(X_r, axis=0)

    X_c_centered = X_c - centroid_c
    X_r_centered = X_r - centroid_r

    H = X_c_centered.T @ X_r_centered
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    T = centroid_r - R @ centroid_c
    return R, T


script_dir = os.path.dirname(os.path.abspath(__file__))
camera_path = os.path.join(script_dir, "camera_points.npy")
robot_path = os.path.join(script_dir, "robot_points.npy")

# Load collected data from the same folder as this script
X_c = np.load(camera_path)
X_r = np.load(robot_path)
print(f"Loaded camera points from {camera_path}: shape {X_c.shape}")
print(f"Loaded robot points from {robot_path}: shape {X_r.shape}")

# Compute transformation
R, T = find_rigid_transform(X_c, X_r)

print("Computed Rotation Matrix (R):")
print(R)

print("\nComputed Translation Vector (T):")
print(T)

# Save transformation in the same folder as this script
np.save(os.path.join(script_dir, "R.npy"), R)
np.save(os.path.join(script_dir, "T.npy"), T)
print("Transformation saved!")
