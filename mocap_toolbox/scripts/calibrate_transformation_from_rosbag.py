#!/usr/bin/env python3
import numpy as np
import rosbag
import math

BAG_FILE = "/home/rosmatch/2025-11-14-11-36-16.bag"  # <- dein Bagfile hier eintragen

MAP_TOPIC   = "/mur620c/mir_pose_stamped_simple"
MOCAP_TOPIC = "/qualisys/mur620c/pose"

TIME_TOL = 0.03  # max. Zeitdifferenz für Matching (in s)
def quat_from_yaw(yaw):
    half = yaw / 2.0
    qx = 0.0
    qy = 0.0
    qz = math.sin(half)
    qw = math.cos(half)
    return qx, qy, qz, qw


def load_poses_from_bag(bag_path):
    times_map, poses_map = [], []
    times_mocap, poses_mocap = [], []

    bag = rosbag.Bag(bag_path)
    for topic, msg, _ in bag.read_messages(topics=[MAP_TOPIC, MOCAP_TOPIC]):

        if topic == MAP_TOPIC:
            ts = msg.header.stamp.to_sec()
            x = msg.pose.position.x
            y = msg.pose.position.y
            times_map.append(ts)
            poses_map.append([x, y])

        elif topic == MOCAP_TOPIC:
            ts = msg.header.stamp.to_sec()
            x = msg.pose.position.x
            y = msg.pose.position.y
            times_mocap.append(ts)
            poses_mocap.append([x, y])

    bag.close()

    return (np.array(times_map),   np.array(poses_map),
            np.array(times_mocap), np.array(poses_mocap))


def match_map_to_mocap(times_map, poses_map, times_mocap, poses_mocap, max_dt):
    order = np.argsort(times_mocap)
    t_m = times_mocap[order]
    p_m = poses_mocap[order]

    matched_map = []
    matched_mocap = []

    for t_ref, p_ref in zip(times_map, poses_map):
        idx = np.searchsorted(t_m, t_ref)

        candidates = []
        if idx > 0:
            candidates.append(idx - 1)
        if idx < len(t_m):
            candidates.append(idx)

        best_idx = None
        best_dt = None
        for c in candidates:
            dt = abs(t_m[c] - t_ref)
            if best_dt is None or dt < best_dt:
                best_dt = dt
                best_idx = c

        if best_dt is not None and best_dt <= max_dt:
            matched_map.append(p_ref)
            matched_mocap.append(p_m[best_idx])

    return np.array(matched_map), np.array(matched_mocap)


def estimate_se2(map_pts, mocap_pts):
    """
    Schätzt SE(2)-Trafo:
       map ≈ R * mocap + t

    map_pts, mocap_pts: (N,2)
    """
    assert map_pts.shape == mocap_pts.shape
    if map_pts.shape[0] < 2:
        raise ValueError("Need at least two point pairs.")

    # P: mocap (Quelle), Q: map (Ziel)
    P = mocap_pts
    Q = map_pts

    cP = P.mean(axis=0)
    cQ = Q.mean(axis=0)

    PA = P - cP   # Quelle zentriert
    QB = Q - cQ   # Ziel zentriert

    # Kovarianz H = Sum q_i r_i^T  -> QB.T @ PA
    H = QB.T @ PA

    U, S, Vt = np.linalg.svd(H)

    R = U @ Vt
    # Reflection verhindern
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt

    t = cQ - R @ cP
    return R, t


def main():
    t_map, p_map, t_mocap, p_mocap = load_poses_from_bag(BAG_FILE)
    print(f"map-Posen:   {len(p_map)}")
    print(f"mocap-Posen: {len(p_mocap)}")

    matched_map, matched_mocap = match_map_to_mocap(
        t_map, p_map, t_mocap, p_mocap, TIME_TOL
    )
    print(f"gematchte Paare: {matched_map.shape[0]}")

    if matched_map.shape[0] < 3:
        raise RuntimeError("Zu wenige Paare – Toleranz / Aufnahme prüfen.")

    # SE(2): map ≈ R * mocap + t
    R, t = estimate_se2(matched_map, matched_mocap)
    yaw = math.atan2(R[1, 0], R[0, 0])

    roll = 0.0
    pitch = 0.0

    print("\n=== Geschätzte Transformation (map <- mocap) ===")
    print(f"Translation (x, y): {t[0]:.4f}, {t[1]:.4f}  [im map-Frame]")
    print(f"Euler (roll, pitch, yaw) [rad]: {roll:.6f}, {pitch:.6f}, {yaw:.6f}")
    print(f"Euler (roll, pitch, yaw) [deg]: "
          f"{math.degrees(roll):.3f}, {math.degrees(pitch):.3f}, {math.degrees(yaw):.3f}")

    qx, qy, qz, qw = quat_from_yaw(yaw)
    print("\nStatic tf für Launch:")
    print("rosrun tf2_ros static_transform_publisher "
          f"{t[0]:.4f} {t[1]:.4f} 0.0 {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f} map mocap")

    # Validierung
    p_mocap_hat = (R @ matched_mocap.T).T + t
    diffs = matched_map - p_mocap_hat
    dists = np.linalg.norm(diffs, axis=1)

    rmse = np.sqrt(np.mean(dists**2))
    mean_err = np.mean(dists)
    std_err = np.std(dists)
    max_err = np.max(dists)

    print("\n=== Fehler zwischen map-Trajektorie und transformierter mocap-Trajektorie ===")
    print(f"Anzahl Punkte:    {len(dists)}")
    print(f"RMSE [m]:         {rmse:.4f}")
    print(f"Mean Error [m]:   {mean_err:.4f}")
    print(f"Std Dev [m]:      {std_err:.4f}")
    print(f"Max Error [m]:    {max_err:.4f}")


if __name__ == "__main__":
    main()