#!/usr/bin/env python3
"""Open3D-backed offline point-cloud cleanup and mesh conversion helpers."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
from typing import Callable, Iterable

import numpy as np


ProgressCallback = Callable[[str, int, int], None]


@dataclass(frozen=True)
class Pose:
    translation: np.ndarray
    rotation: np.ndarray

    def transform(self, points: np.ndarray) -> np.ndarray:
        return np.asarray(points, dtype=np.float64) @ self.rotation.T + self.translation


def _finite_xyz(points: np.ndarray, *, minimum_points: int = 1) -> np.ndarray:
    result = np.asarray(points, dtype=np.float64)
    if result.ndim != 2 or result.shape[1] != 3:
        raise ValueError("点云必须是 N×3 XYZ 数组")
    if len(result) < minimum_points:
        raise ValueError(f"点云至少需要 {minimum_points} 个点")
    if not np.isfinite(result).all():
        raise ValueError("点云包含非有限坐标")
    return result


def cluster_filter_points(
    points: np.ndarray,
    o3d,
    *,
    eps: float,
    min_points: int,
    min_cluster_points: int,
    remove_noise: bool = True,
):
    """Remove DBSCAN noise and clusters smaller than ``min_cluster_points``."""
    points = _finite_xyz(points)
    eps = float(eps)
    min_points = int(min_points)
    min_cluster_points = int(min_cluster_points)
    if not math.isfinite(eps) or eps <= 0:
        raise ValueError("聚类距离 eps 必须大于 0")
    if min_points < 1 or min_points > 10000:
        raise ValueError("DBSCAN min_points 必须在 [1, 10000] 范围内")
    if min_cluster_points < 1:
        raise ValueError("最小保留聚类点数必须大于 0")

    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)
    labels = np.asarray(
        cloud.cluster_dbscan(eps=eps, min_points=min_points, print_progress=False),
        dtype=np.int32,
    )
    if len(labels) != len(points):
        raise RuntimeError("Open3D DBSCAN 返回了错误的标签数量")

    valid = labels >= 0
    cluster_count = int(labels[valid].max()) + 1 if np.any(valid) else 0
    sizes = np.bincount(labels[valid], minlength=cluster_count) if cluster_count else np.zeros(0, dtype=int)
    keep = np.zeros(len(points), dtype=bool)
    if cluster_count:
        keep = valid & (sizes[np.maximum(labels, 0)] >= min_cluster_points)
    if not remove_noise:
        keep |= labels < 0

    kept = np.ascontiguousarray(points[keep], dtype=np.float32)
    removed = np.ascontiguousarray(points[~keep], dtype=np.float32)
    return kept, removed, {
        "clusters": cluster_count,
        "noise_points": int(np.count_nonzero(labels < 0)),
        "kept_clusters": int(np.count_nonzero(sizes >= min_cluster_points)),
        "kept_points": len(kept),
        "removed_points": len(removed),
    }


_POSE_PATTERN = re.compile(r"^\s*se3\s*\(([^)]*)\)\s*$", re.IGNORECASE)


def parse_pose_line(line: str) -> Pose:
    match = _POSE_PATTERN.match(line)
    fields = match.group(1).split(",") if match else line.split()
    try:
        values = [float(value.strip()) for value in fields]
    except ValueError as exc:
        raise ValueError(f"无法解析位姿：{line.rstrip()}") from exc
    if len(values) != 7 or not np.isfinite(values).all():
        raise ValueError("位姿必须包含 x y z qx qy qz qw 七个有限数值")
    x, y, z, qx, qy, qz, qw = values
    quaternion = np.array([qw, qx, qy, qz], dtype=np.float64)
    norm = float(np.linalg.norm(quaternion))
    if norm <= 1e-12:
        raise ValueError("位姿四元数长度为零")
    qw, qx, qy, qz = quaternion / norm
    rotation = np.array(
        [
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ],
        dtype=np.float64,
    )
    return Pose(np.array([x, y, z], dtype=np.float64), rotation)


def _traverse_voxels(origin: np.ndarray, endpoint: np.ndarray, resolution: float):
    delta = endpoint - origin
    distance = float(np.linalg.norm(delta))
    if distance <= max(1e-9, resolution * 0.5):
        return
    direction = delta / distance
    current = np.floor(origin / resolution).astype(np.int64)
    target = np.floor(endpoint / resolution).astype(np.int64)
    if np.array_equal(current, target):
        return
    step = np.sign(direction).astype(np.int64)
    t_max = np.full(3, np.inf, dtype=np.float64)
    t_delta = np.full(3, np.inf, dtype=np.float64)
    for axis in range(3):
        if step[axis] == 0:
            continue
        boundary = (current[axis] + (1 if step[axis] > 0 else 0)) * resolution
        t_max[axis] = (boundary - origin[axis]) / direction[axis]
        t_delta[axis] = resolution / abs(direction[axis])

    max_steps = int(math.ceil(distance / resolution)) + 6
    for _ in range(max_steps):
        axis = int(np.argmin(t_max))
        next_distance = float(t_max[axis])
        current[axis] += step[axis]
        t_max[axis] += t_delta[axis]
        if next_distance > distance - resolution or np.array_equal(current, target):
            break
        yield int(current[0]), int(current[1]), int(current[2])


def raycast_dynamic_filter(
    poses: Iterable[Pose],
    frames: Iterable[np.ndarray],
    *,
    voxel_resolution: float,
    pass_through_threshold: int,
    ray_stride: int = 1,
    max_range: float = 0.0,
    progress: ProgressCallback | None = None,
):
    """HWS-compatible DDA pass-through filtering for per-frame mapping data."""
    poses = list(poses)
    frames = [_finite_xyz(frame) for frame in frames]
    if not poses or len(poses) != len(frames):
        raise ValueError("poses 与 frame_N.pcd 数量必须相同且非空")
    voxel_resolution = float(voxel_resolution)
    pass_through_threshold = int(pass_through_threshold)
    ray_stride = int(ray_stride)
    max_range = float(max_range)
    if not math.isfinite(voxel_resolution) or voxel_resolution <= 0:
        raise ValueError("射线体素分辨率必须大于 0")
    if pass_through_threshold < 0:
        raise ValueError("穿透阈值不能为负")
    if ray_stride < 1:
        raise ValueError("射线步长必须大于 0")
    if not math.isfinite(max_range) or max_range < 0:
        raise ValueError("最大射线距离不能为负")

    transformed_frames = [pose.transform(frame) for pose, frame in zip(poses, frames)]
    merged = np.concatenate(transformed_frames, axis=0)
    voxel_keys = np.floor(merged / voxel_resolution).astype(np.int64)
    unique_keys = np.unique(voxel_keys, axis=0)
    voxel_to_index = {tuple(key): index for index, key in enumerate(unique_keys)}
    pass_counts = np.zeros(len(unique_keys), dtype=np.int32)

    total_rays = sum((len(frame) + ray_stride - 1) // ray_stride for frame in frames)
    processed = 0
    for frame_index, (pose, endpoints) in enumerate(zip(poses, transformed_frames)):
        origin = pose.translation
        local_points = frames[frame_index]
        for point_index in range(0, len(endpoints), ray_stride):
            if max_range > 0 and np.linalg.norm(local_points[point_index]) > max_range:
                continue
            for key in _traverse_voxels(origin, endpoints[point_index], voxel_resolution):
                index = voxel_to_index.get(key)
                if index is not None:
                    pass_counts[index] += 1
            processed += 1
        if progress:
            progress("raycasting", processed, total_rays)

    point_voxel_indices = np.fromiter(
        (voxel_to_index[tuple(key)] for key in voxel_keys),
        dtype=np.int64,
        count=len(voxel_keys),
    )
    keep = pass_counts[point_voxel_indices] <= pass_through_threshold
    kept = np.ascontiguousarray(merged[keep], dtype=np.float32)
    removed = np.ascontiguousarray(merged[~keep], dtype=np.float32)
    return kept, removed, {
        "frames": len(frames),
        "input_points": len(merged),
        "rays": processed,
        "occupied_voxels": len(unique_keys),
        "removed_voxels": int(np.count_nonzero(pass_counts > pass_through_threshold)),
        "kept_points": len(kept),
        "removed_points": len(removed),
    }


def load_hws_mapping_dataset(directory: str | Path, o3d):
    directory = Path(directory).expanduser().resolve()
    if not directory.is_dir():
        raise ValueError(f"建图目录不存在：{directory}")
    poses_path = directory / "poses.txt"
    if not poses_path.is_file():
        raise ValueError(f"缺少 poses.txt：{poses_path}")
    poses = []
    for line_number, line in enumerate(poses_path.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        try:
            poses.append(parse_pose_line(line))
        except ValueError as exc:
            raise ValueError(f"poses.txt 第 {line_number} 行无效：{exc}") from exc
    frames = []
    for index in range(len(poses)):
        frame_path = directory / f"frame_{index}.pcd"
        if not frame_path.is_file():
            raise ValueError(f"缺少帧文件：{frame_path.name}")
        cloud = o3d.io.read_point_cloud(str(frame_path))
        points = np.asarray(cloud.points, dtype=np.float64)
        frames.append(_finite_xyz(points))
    return directory, poses, frames


def postprocess_points(
    points: np.ndarray,
    o3d,
    *,
    voxel_size: float = 0.05,
    statistical_neighbors: int = 20,
    statistical_std_ratio: float = 2.0,
    cluster_eps: float = 0.0,
    cluster_min_points: int = 10,
    min_cluster_points: int = 0,
):
    points = _finite_xyz(points)
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)
    if voxel_size > 0:
        cloud = cloud.voxel_down_sample(float(voxel_size))
    before_statistical = len(cloud.points)
    if statistical_neighbors >= 2:
        cloud, _ = cloud.remove_statistical_outlier(
            nb_neighbors=int(statistical_neighbors), std_ratio=float(statistical_std_ratio)
        )
    result = np.asarray(cloud.points, dtype=np.float64)
    stats = {
        "after_voxel_points": before_statistical,
        "after_statistical_points": len(result),
    }
    if cluster_eps > 0 and min_cluster_points > 0:
        result, removed, cluster_stats = cluster_filter_points(
            result,
            o3d,
            eps=cluster_eps,
            min_points=cluster_min_points,
            min_cluster_points=min_cluster_points,
            remove_noise=True,
        )
        stats.update({f"cluster_{key}": value for key, value in cluster_stats.items()})
        stats["cluster_removed_points"] = len(removed)
    return np.ascontiguousarray(result, dtype=np.float32), stats


_MESH_PRESETS = {
    "visual": {
        "preset": "visual",
        "method": "poisson",
        "voxel_size": 0.03,
        "statistical_neighbors": 20,
        "statistical_std_ratio": 2.0,
        "normal_radius": 0.15,
        "normal_max_neighbors": 40,
        "consistent_neighbors": 30,
        "consistent_max_points": 150000,
        "poisson_depth": 9,
        "poisson_scale": 1.1,
        "density_quantile": 0.02,
        "crop_to_input": True,
        "target_triangles": 500000,
        "smooth_iterations": 0,
        "ball_radii": [0.04, 0.08, 0.16],
    },
    "collision": {
        "preset": "collision",
        "method": "poisson",
        "voxel_size": 0.06,
        "statistical_neighbors": 20,
        "statistical_std_ratio": 2.0,
        "normal_radius": 0.30,
        "normal_max_neighbors": 40,
        "consistent_neighbors": 20,
        "consistent_max_points": 150000,
        "poisson_depth": 8,
        "poisson_scale": 1.1,
        "density_quantile": 0.03,
        "crop_to_input": True,
        "target_triangles": 80000,
        "smooth_iterations": 1,
        "ball_radii": [0.08, 0.16, 0.32],
    },
}


def validate_mesh_options(options: dict | None):
    supplied = dict(options or {})
    preset = supplied.get("preset", "visual")
    if preset not in _MESH_PRESETS:
        raise ValueError("网格预设必须是 visual 或 collision")
    result = {**_MESH_PRESETS[preset], **supplied, "preset": preset}
    if result["method"] not in ("poisson", "ball_pivoting"):
        raise ValueError("重建算法必须是 poisson 或 ball_pivoting")
    for name in ("voxel_size", "normal_radius", "poisson_scale"):
        result[name] = float(result[name])
        if not math.isfinite(result[name]) or result[name] <= 0:
            raise ValueError(f"{name} 必须大于 0")
    result["statistical_neighbors"] = int(result["statistical_neighbors"])
    result["normal_max_neighbors"] = int(result["normal_max_neighbors"])
    result["consistent_neighbors"] = int(result["consistent_neighbors"])
    result["consistent_max_points"] = int(result["consistent_max_points"])
    result["poisson_depth"] = int(result["poisson_depth"])
    result["target_triangles"] = int(result["target_triangles"])
    result["smooth_iterations"] = int(result["smooth_iterations"])
    result["statistical_std_ratio"] = float(result["statistical_std_ratio"])
    result["density_quantile"] = float(result["density_quantile"])
    result["crop_to_input"] = bool(result["crop_to_input"])
    if not 5 <= result["poisson_depth"] <= 13:
        raise ValueError("poisson_depth 必须在 [5, 13] 范围内")
    if not 0 <= result["density_quantile"] < 0.5:
        raise ValueError("density_quantile 必须在 [0, 0.5) 范围内")
    if result["target_triangles"] < 1000:
        raise ValueError("目标三角形数量不能小于 1000")
    if not 0 <= result["smooth_iterations"] <= 50:
        raise ValueError("平滑迭代次数必须在 [0, 50] 范围内")
    radii = [float(value) for value in result.get("ball_radii", [])]
    if not radii or any(not math.isfinite(value) or value <= 0 for value in radii):
        raise ValueError("Ball Pivoting 半径必须是正数列表")
    result["ball_radii"] = sorted(radii)
    return result


def reconstruct_mesh(points: np.ndarray, o3d, options: dict | None = None):
    points = _finite_xyz(points, minimum_points=100)
    config = validate_mesh_options(options)
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)
    cloud = cloud.voxel_down_sample(config["voxel_size"])
    if len(cloud.points) < 100:
        raise ValueError("降采样后点数不足 100，无法稳定重建网格")
    cloud, _ = cloud.remove_statistical_outlier(
        nb_neighbors=config["statistical_neighbors"],
        std_ratio=config["statistical_std_ratio"],
    )
    cloud.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(
            radius=config["normal_radius"], max_nn=config["normal_max_neighbors"]
        )
    )
    if len(cloud.points) <= config["consistent_max_points"] and len(cloud.points) > config["consistent_neighbors"]:
        cloud.orient_normals_consistent_tangent_plane(config["consistent_neighbors"])
        normal_orientation = "consistent_tangent_plane"
    else:
        cloud.orient_normals_to_align_with_direction(np.array([0.0, 0.0, 1.0]))
        normal_orientation = "positive_z_hemisphere"

    if config["method"] == "poisson":
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            cloud,
            depth=config["poisson_depth"],
            scale=config["poisson_scale"],
            linear_fit=False,
        )
        densities = np.asarray(densities)
        if config["density_quantile"] > 0 and len(densities):
            mesh.remove_vertices_by_mask(
                densities < np.quantile(densities, config["density_quantile"])
            )
    else:
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
            cloud,
            o3d.utility.DoubleVector(config["ball_radii"]),
        )

    if config["crop_to_input"]:
        mesh = mesh.crop(cloud.get_axis_aligned_bounding_box())
    mesh.remove_duplicated_vertices()
    mesh.remove_duplicated_triangles()
    mesh.remove_degenerate_triangles()
    mesh.remove_unreferenced_vertices()
    if len(mesh.triangles) > config["target_triangles"]:
        mesh = mesh.simplify_quadric_decimation(config["target_triangles"])
    if config["smooth_iterations"]:
        mesh = mesh.filter_smooth_taubin(number_of_iterations=config["smooth_iterations"])
    mesh.remove_degenerate_triangles()
    mesh.remove_unreferenced_vertices()
    mesh.compute_vertex_normals()
    if len(mesh.triangles) == 0:
        raise RuntimeError("表面重建没有生成有效三角形")
    stats = {
        "input_points": len(points),
        "processed_points": len(cloud.points),
        "vertices": len(mesh.vertices),
        "triangles": len(mesh.triangles),
        "preset": config["preset"],
        "method": config["method"],
        "normal_orientation": normal_orientation,
    }
    return mesh, stats, config


def convert_step_to_points(
    step_path: str | Path,
    o3d,
    *,
    worker_path: str | Path,
    point_count: int,
    unit_scale: float = 0.001,
    linear_deflection: float = 5.0,
    angular_deflection: float = 0.35,
    freecad_command: str = "freecadcmd",
    timeout_seconds: int = 7200,
    memory_limit_gb: float = 6.0,
):
    step_path = Path(step_path).expanduser().resolve()
    if not step_path.is_file() or step_path.suffix.lower() not in (".stp", ".step"):
        raise ValueError(f"STEP 文件不存在或后缀无效：{step_path}")
    point_count = int(point_count)
    unit_scale = float(unit_scale)
    if not 1000 <= point_count <= 10_000_000:
        raise ValueError("采样点数必须在 [1000, 10000000] 范围内")
    if not math.isfinite(unit_scale) or unit_scale <= 0:
        raise ValueError("单位缩放必须大于 0")
    if linear_deflection <= 0 or angular_deflection <= 0:
        raise ValueError("STEP 三角化偏差必须大于 0")
    memory_limit_gb = float(memory_limit_gb)
    if not math.isfinite(memory_limit_gb) or memory_limit_gb < 0:
        raise ValueError("FreeCAD 内存上限不能为负")

    with tempfile.TemporaryDirectory(prefix="cloudlab_step_") as temporary_directory:
        stl_path = Path(temporary_directory) / "step_mesh.stl"
        command = [freecad_command, str(Path(worker_path).resolve())]
        if memory_limit_gb > 0:
            prlimit = shutil.which("prlimit")
            if prlimit:
                limit_bytes = int(memory_limit_gb * 1024**3)
                command = [prlimit, f"--as={limit_bytes}", "--", *command]
        environment = os.environ.copy()
        environment.update(
            {
                "CLOUDLAB_STEP_INPUT": str(step_path),
                "CLOUDLAB_STEP_OUTPUT": str(stl_path),
                "CLOUDLAB_STEP_LINEAR_DEFLECTION": str(float(linear_deflection)),
                "CLOUDLAB_STEP_ANGULAR_DEFLECTION": str(float(angular_deflection)),
            }
        )
        try:
            result = subprocess.run(
                command,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                env=environment,
                timeout=int(timeout_seconds),
                check=False,
            )
        except subprocess.TimeoutExpired as exc:
            raise RuntimeError(f"FreeCAD STEP 转换超过 {timeout_seconds} 秒") from exc
        if result.returncode != 0 or not stl_path.is_file():
            tail = "\n".join(result.stdout.splitlines()[-20:])
            raise RuntimeError(f"FreeCAD STEP 三角化失败（exit={result.returncode}）：\n{tail}")
        mesh = o3d.io.read_triangle_mesh(str(stl_path))
        if len(mesh.triangles) == 0:
            raise RuntimeError("STEP 三角化结果为空")
        cloud = mesh.sample_points_uniformly(number_of_points=point_count)
        points = np.asarray(cloud.points, dtype=np.float64) * unit_scale
        return np.ascontiguousarray(points, dtype=np.float32), {
            "source": str(step_path),
            "points": len(points),
            "mesh_triangles": len(mesh.triangles),
            "unit_scale": unit_scale,
            "freecad_memory_limit_gb": memory_limit_gb,
        }


def stats_header(stats: dict) -> str:
    return json.dumps(stats, ensure_ascii=True, separators=(",", ":"))
