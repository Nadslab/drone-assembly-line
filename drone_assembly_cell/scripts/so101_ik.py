#!/usr/bin/env python3
"""
Forward and inverse kinematics for the SO-101 arm.

All transforms are taken verbatim from so101.urdf.xacro.
The FK chain is:
  base_link
    → [shoulder_pan]  → shoulder_link
    → [shoulder_lift] → upper_arm_link
    → [elbow_flex]    → lower_arm_link
    → [wrist_flex]    → wrist_link
    → [wrist_roll]    → gripper_link
    → [fixed frame]   → gripper_frame_link   ← TCP

IK is solved numerically with scipy.optimize.minimize (SLSQP).
The cost minimises 3-D position error plus a soft penalty that keeps the
gripper Z-axis pointing toward −world-Z (tip-down orientation).
"""

import math
import numpy as np
from scipy.optimize import minimize


# ── 4×4 homogeneous helpers ───────────────────────────────────────────────

def _Rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1,0,0,0],[0,c,-s,0],[0,s,c,0],[0,0,0,1]], float)

def _Ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c,0,s,0],[0,1,0,0],[-s,0,c,0],[0,0,0,1]], float)

def _Rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c,-s,0,0],[s,c,0,0],[0,0,1,0],[0,0,0,1]], float)

def _Txyz(x, y, z):
    T = np.eye(4)
    T[0,3], T[1,3], T[2,3] = x, y, z
    return T

def _rpy(roll, pitch, yaw) -> np.ndarray:
    """URDF convention: Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    return _Rz(yaw) @ _Ry(pitch) @ _Rx(roll)

def _rot_z(q) -> np.ndarray:
    """4×4 rotation about local Z by q radians."""
    return _Rz(q)


# ── Pre-built fixed transforms (joint offsets from URDF) ─────────────────

_PI = math.pi

# Each entry: (translation xyz, rpy tuple) → fixed part of joint transform
_T_pan   = _Txyz(0.0388353,  0.0,        0.0624)  @ _rpy(_PI, 0,       -_PI)
_T_lift  = _Txyz(-0.0303992, -0.0182778, -0.0542) @ _rpy(-_PI/2, -_PI/2, 0)
_T_elbow = _Txyz(-0.11257,   -0.028,      0)      @ _rpy(0,       0,      _PI/2)
_T_wflex = _Txyz(-0.1349,     0.0052,     0)      @ _rpy(0,       0,     -_PI/2)
_T_wroll = _Txyz(0,          -0.0611,     0.0181) @ _rpy(_PI/2,   0.0486795, _PI)
_T_grip  = _Txyz(-0.0079,    -0.000218,  -0.0981274) @ _rpy(0, _PI, 0)  # fixed frame


# ── Forward kinematics ────────────────────────────────────────────────────

def fk(q: list[float]) -> np.ndarray:
    """
    Forward kinematics for SO-101 (5-DOF arm chain, gripper excluded).

    Parameters
    ----------
    q : [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll]

    Returns
    -------
    4×4 numpy array — pose of gripper_frame_link in base_link frame.
    """
    T = (_T_pan   @ _rot_z(q[0])
       @ _T_lift  @ _rot_z(q[1])
       @ _T_elbow @ _rot_z(q[2])
       @ _T_wflex @ _rot_z(q[3])
       @ _T_wroll @ _rot_z(q[4])
       @ _T_grip)
    return T


def tcp_position(q: list[float]) -> np.ndarray:
    """Return TCP (x, y, z) in base_link frame for joint angles q."""
    return fk(q)[:3, 3]


# ── Inverse kinematics ────────────────────────────────────────────────────

_LIMITS = [
    (-1.91986, 1.91986),   # shoulder_pan
    (-1.74533, 1.74533),   # shoulder_lift
    (-1.69,    1.69),      # elbow_flex
    (-1.65806, 1.65806),   # wrist_flex
    (-2.74385, 2.84121),   # wrist_roll
]

_ORIENT_WEIGHT = 0.5  # weight of gripper-down penalty vs position error


def ik(target_xyz: list[float],
       prefer_tip_down: bool = True,
       q0: list[float] | None = None,
       tol: float = 1e-4) -> tuple[list[float], float]:
    """
    Numerical IK for SO-101.

    Parameters
    ----------
    target_xyz      : desired TCP position in base_link frame [x, y, z]
    prefer_tip_down : if True, add a soft penalty so the gripper faces down
    q0              : initial joint guess (defaults to all-zero)
    tol             : position tolerance in metres

    Returns
    -------
    (q, residual) where q is a list of 5 joint angles and residual is the
    final position error in metres.  Raises RuntimeError if IK fails.
    """
    target = np.array(target_xyz, float)
    q0 = list(q0) if q0 is not None else [0.0] * 5

    def cost(q):
        T = fk(q)
        pos_err = np.linalg.norm(T[:3, 3] - target)
        if prefer_tip_down:
            # gripper_frame Z column should point along -world-Z
            gz = T[2, 2]              # world-Z component of gripper Z axis
            orient_err = (gz + 1.0) ** 2   # 0 when gz == -1 (pointing down)
            return pos_err**2 + _ORIENT_WEIGHT * orient_err
        return pos_err**2

    res = minimize(cost, q0, method='SLSQP', bounds=_LIMITS,
                   options={'ftol': 1e-8, 'maxiter': 2000})

    T_sol = fk(res.x)
    residual = float(np.linalg.norm(T_sol[:3, 3] - target))

    if residual > tol:
        raise RuntimeError(
            f'IK did not converge: target={target_xyz}, '
            f'got={T_sol[:3, 3].tolist()}, residual={residual:.4f} m')

    return list(res.x), residual


def ik_multi_seed(target_xyz: list[float],
                  prefer_tip_down: bool = True,
                  tol: float = 5e-3) -> list[float]:
    """
    Try IK from several seeds, return the best solution.
    More robust than single-seed IK for unreachable or near-singular configs.
    """
    seeds = [
        # neutral / home-ish
        [ 0.0,  0.0,  0.0,  0.0, 0.0],
        [ 0.0, -0.5,  0.5,  1.0, 0.0],
        # bin_A region (negative pan, arm swings left/+Y)
        [-1.5, -0.4,  0.3,  1.66, 2.6],
        [-1.5, -0.4,  0.8,  1.2, -0.3],
        [-1.2, -0.3,  0.5,  1.3,  0.0],
        # bin_C region (positive pan, arm swings right/-Y)
        [ 1.5,  0.1, -0.2,  1.66, 0.4],
        [ 1.5,  0.0,  0.0,  1.66, 0.0],
        [ 1.5,  0.2,  0.2,  1.2,  0.0],
    ]
    best_q, best_r = None, float('inf')
    for seed in seeds:
        try:
            q, r = ik(target_xyz, prefer_tip_down=prefer_tip_down, q0=seed, tol=tol)
            if r < best_r:
                best_q, best_r = q, r
        except RuntimeError:
            pass
    if best_q is None:
        raise RuntimeError(f'IK failed for all seeds: target={target_xyz}')
    return best_q


# ── Quick self-test ───────────────────────────────────────────────────────

if __name__ == '__main__':
    import sys
    q_zero = [0.0, 0.0, 0.0, 0.0, 0.0]
    pos_zero = tcp_position(q_zero)
    print(f'TCP at zero config: {pos_zero.tolist()}')

    # Test round-trip (arm frame with yaw=pi/2 spawn; home = [0,0,0,0,0] directly)
    test_targets = [
        [ 0.05,  0.15,  0.10],   # bin_A approach
        [ 0.05,  0.15,  0.018],  # bin_A grasp
        [ 0.05, -0.20,  0.10],   # bin_C approach
        [ 0.05, -0.20,  0.018],  # bin_C grasp
    ]
    for tgt in test_targets:
        try:
            q_sol = ik_multi_seed(tgt)
            pos_sol = tcp_position(q_sol)
            err = np.linalg.norm(np.array(pos_sol) - np.array(tgt))
            print(f'target={tgt}  →  q={[f"{v:.3f}" for v in q_sol]}  err={err:.4f} m')
        except RuntimeError as e:
            print(f'FAILED: {e}', file=sys.stderr)
