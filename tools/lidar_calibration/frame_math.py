#!/usr/bin/env python3
"""Quaternion and angle helpers using the same Rz(yaw)Ry(pitch)Rx(roll) convention as tf2."""

from __future__ import annotations

from typing import Tuple

import numpy as np


def normalize_quaternions(quaternions: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Normalize (x, y, z, w) quaternions and return normalized values plus a validity mask."""
    q = np.asarray(quaternions, dtype=float)
    if q.ndim != 2 or q.shape[1] != 4:
        raise ValueError("quaternions must have shape (N, 4)")
    norms = np.linalg.norm(q, axis=1)
    valid = np.isfinite(q).all(axis=1) & np.isfinite(norms) & (norms > 1.0e-9)
    normalized = np.zeros_like(q)
    normalized[valid] = q[valid] / norms[valid, None]
    return normalized, valid


def quaternion_to_rotation_matrices(quaternions: np.ndarray) -> np.ndarray:
    """Convert normalized (x, y, z, w) quaternions to body-to-world matrices."""
    q, valid = normalize_quaternions(quaternions)
    if not np.all(valid):
        raise ValueError("invalid quaternion passed to quaternion_to_rotation_matrices")

    x, y, z, w = q.T
    matrices = np.empty((len(q), 3, 3), dtype=float)
    matrices[:, 0, 0] = 1.0 - 2.0 * (y * y + z * z)
    matrices[:, 0, 1] = 2.0 * (x * y - z * w)
    matrices[:, 0, 2] = 2.0 * (x * z + y * w)
    matrices[:, 1, 0] = 2.0 * (x * y + z * w)
    matrices[:, 1, 1] = 1.0 - 2.0 * (x * x + z * z)
    matrices[:, 1, 2] = 2.0 * (y * z - x * w)
    matrices[:, 2, 0] = 2.0 * (x * z - y * w)
    matrices[:, 2, 1] = 2.0 * (y * z + x * w)
    matrices[:, 2, 2] = 1.0 - 2.0 * (x * x + y * y)
    return matrices


def quaternion_to_rpy(quaternions: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return tf2-compatible roll, pitch and yaw arrays in radians."""
    q, valid = normalize_quaternions(quaternions)
    if not np.all(valid):
        raise ValueError("invalid quaternion passed to quaternion_to_rpy")

    x, y, z, w = q.T
    sin_roll_cos_pitch = 2.0 * (w * x + y * z)
    cos_roll_cos_pitch = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(sin_roll_cos_pitch, cos_roll_cos_pitch)

    sin_pitch = np.clip(2.0 * (w * y - z * x), -1.0, 1.0)
    pitch = np.arcsin(sin_pitch)

    sin_yaw_cos_pitch = 2.0 * (w * z + x * y)
    cos_yaw_cos_pitch = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(sin_yaw_cos_pitch, cos_yaw_cos_pitch)
    return roll, pitch, yaw


def circular_mean(angles: np.ndarray) -> float:
    """Circular mean in radians."""
    values = np.asarray(angles, dtype=float)
    return float(np.arctan2(np.mean(np.sin(values)), np.mean(np.cos(values))))


def angular_difference(angles: np.ndarray, reference: float) -> np.ndarray:
    """Signed wrapped angle difference in [-pi, pi]."""
    values = np.asarray(angles, dtype=float)
    return np.arctan2(np.sin(values - reference), np.cos(values - reference))


def roll_pitch_from_mean_up(rotation_matrices: np.ndarray) -> Tuple[float, float, np.ndarray]:
    """
    Estimate fixed roll/pitch from the world-up direction expressed in body coordinates.

    R maps body vectors to world. Therefore R.T @ [0, 0, 1] is world-up in body.
    Under R = Rz(yaw) Ry(pitch) Rx(roll):
      pitch = -asin(up_body.x)
      roll  = atan2(up_body.y, up_body.z)
    """
    matrices = np.asarray(rotation_matrices, dtype=float)
    if matrices.ndim != 3 or matrices.shape[1:] != (3, 3):
        raise ValueError("rotation_matrices must have shape (N, 3, 3)")

    up_samples = matrices[:, 2, :]
    mean_up = np.mean(up_samples, axis=0)
    norm = float(np.linalg.norm(mean_up))
    if not np.isfinite(norm) or norm < 1.0e-9:
        raise ValueError("mean body-frame up direction is invalid")
    mean_up /= norm
    pitch = -float(np.arcsin(np.clip(mean_up[0], -1.0, 1.0)))
    roll = float(np.arctan2(mean_up[1], mean_up[2]))
    return roll, pitch, mean_up

