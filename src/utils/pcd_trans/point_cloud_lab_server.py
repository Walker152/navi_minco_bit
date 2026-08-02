#!/usr/bin/env python3
"""Optional localhost Open3D companion for CloudLab Studio.

The HTML editor remains usable without this process. Start this server only when
binary-compressed PCD parsing or Open3D statistical filtering is required.
"""

import argparse
import json
import math
import os
import tempfile
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote

import numpy as np


MAX_BODY_BYTES = 1024 * 1024 * 1024
_OPEN3D = None
_OPEN3D_ERROR = None


def decode_xyz_f32(payload):
    """Decode little-endian tightly packed XYZ float32 request data."""
    if len(payload) % 12:
        raise ValueError("XYZ float32 payload 字节数必须对应 3 的倍数坐标")
    points = np.frombuffer(payload, dtype="<f4").reshape((-1, 3))
    if not np.isfinite(points).all():
        raise ValueError("点云包含非有限坐标")
    return np.asarray(points, dtype=np.float32)


def validate_statistical_params(neighbors, std_ratio):
    neighbors = int(neighbors)
    std_ratio = float(std_ratio)
    if neighbors < 2 or neighbors > 500:
        raise ValueError("neighbors 必须在 [2, 500] 范围内")
    if not math.isfinite(std_ratio) or std_ratio <= 0 or std_ratio > 20:
        raise ValueError("std_ratio 必须在 (0, 20] 范围内")
    return neighbors, std_ratio


def safe_pcd_suffix(filename):
    return ".pcd" if Path(filename).suffix.lower() != ".pcd" else ".pcd"


def load_open3d():
    global _OPEN3D, _OPEN3D_ERROR
    if _OPEN3D is not None:
        return _OPEN3D
    if _OPEN3D_ERROR is not None:
        raise RuntimeError(_OPEN3D_ERROR)
    errors = []
    for module_name in ("open3d", "open3d.cuda.pybind", "open3d.cpu.pybind"):
        try:
            module = __import__(module_name, fromlist=["geometry", "io", "utility"])
            if all(hasattr(module, attr) for attr in ("geometry", "io", "utility")):
                _OPEN3D = module
                return module
        except Exception as exc:  # Import failures differ across Open3D builds.
            errors.append(f"{module_name}: {exc}")
    _OPEN3D_ERROR = "Open3D 不可用；" + " | ".join(errors)
    raise RuntimeError(_OPEN3D_ERROR)


def open3d_capabilities():
    try:
        module = load_open3d()
        return {"open3d": True, "backend": module.__name__}
    except RuntimeError as exc:
        return {"open3d": False, "error": str(exc)}


def points_to_cloud(points):
    o3d = load_open3d()
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(np.asarray(points, dtype=np.float64))
    return cloud


def parse_pcd_bytes(payload, filename="cloud.pcd"):
    o3d = load_open3d()
    temp_path = None
    try:
        with tempfile.NamedTemporaryFile(suffix=safe_pcd_suffix(filename), delete=False) as handle:
            handle.write(payload)
            temp_path = handle.name
        cloud = o3d.io.read_point_cloud(temp_path)
        points = np.asarray(cloud.points, dtype=np.float32)
        if points.size == 0:
            raise ValueError("Open3D 未从 PCD 中读取到有效点")
        if not np.isfinite(points).all():
            raise ValueError("PCD 包含非有限坐标")
        return np.ascontiguousarray(points, dtype="<f4").tobytes()
    finally:
        if temp_path:
            try:
                os.unlink(temp_path)
            except FileNotFoundError:
                pass


def statistical_filter_bytes(payload, neighbors, std_ratio):
    neighbors, std_ratio = validate_statistical_params(neighbors, std_ratio)
    points = decode_xyz_f32(payload)
    cloud = points_to_cloud(points)
    filtered, _ = cloud.remove_statistical_outlier(
        nb_neighbors=neighbors,
        std_ratio=std_ratio,
    )
    result = np.asarray(filtered.points, dtype=np.float32)
    return np.ascontiguousarray(result, dtype="<f4").tobytes()


def normal_labels_bytes(payload, radius, max_neighbors, ground_max, slope_max, wall_min):
    points = decode_xyz_f32(payload)
    radius = float(radius)
    max_neighbors = int(max_neighbors)
    if not math.isfinite(radius) or radius <= 0:
        raise ValueError("normal radius 必须大于 0")
    if max_neighbors < 3 or max_neighbors > 500:
        raise ValueError("normal max_neighbors 必须在 [3, 500] 范围内")
    cloud = points_to_cloud(points)
    o3d = load_open3d()
    cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=max_neighbors))
    normals = np.asarray(cloud.normals)
    cosines = np.clip(np.abs(normals[:, 2]), 0.0, 1.0)
    angles = np.degrees(np.arccos(cosines))
    labels = np.full(points.shape[0], 4, dtype=np.uint8)
    labels[angles <= float(ground_max)] = 1
    labels[(angles > float(ground_max)) & (angles <= float(slope_max))] = 2
    labels[angles >= float(wall_min)] = 3
    return labels.tobytes()


class CloudLabRequestHandler(BaseHTTPRequestHandler):
    server_version = "CloudLabOpen3D/1.0"

    def log_message(self, format_string, *args):
        print("[CloudLab]", format_string % args)

    def cors_headers(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header(
            "Access-Control-Allow-Headers",
            "Content-Type, X-Filename, X-Nb-Neighbors, X-Std-Ratio, "
            "X-Normal-Radius, X-Max-Neighbors, X-Ground-Max, X-Slope-Max, X-Wall-Min",
        )

    def send_json(self, status, value):
        payload = json.dumps(value, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.cors_headers()
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def send_binary(self, payload):
        self.send_response(200)
        self.cors_headers()
        self.send_header("Content-Type", "application/octet-stream")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def read_body(self):
        length = int(self.headers.get("Content-Length", "0"))
        if length <= 0:
            raise ValueError("请求体为空")
        if length > MAX_BODY_BYTES:
            raise ValueError("请求体超过 1 GiB 安全限制")
        payload = self.rfile.read(length)
        if len(payload) != length:
            raise ValueError("请求体读取不完整")
        return payload

    def do_OPTIONS(self):
        self.send_response(204)
        self.cors_headers()
        self.send_header("Content-Length", "0")
        self.end_headers()

    def do_GET(self):
        if self.path != "/health":
            self.send_json(404, {"error": "not found"})
            return
        capabilities = open3d_capabilities()
        self.send_json(200, {"service": "cloudlab-open3d", **capabilities})

    def do_POST(self):
        try:
            payload = self.read_body()
            if self.path == "/parse-pcd":
                filename = unquote(self.headers.get("X-Filename", "cloud.pcd"))
                result = parse_pcd_bytes(payload, filename)
            elif self.path == "/statistical-outlier":
                result = statistical_filter_bytes(
                    payload,
                    self.headers.get("X-Nb-Neighbors", "20"),
                    self.headers.get("X-Std-Ratio", "2.0"),
                )
            elif self.path == "/normal-labels":
                result = normal_labels_bytes(
                    payload,
                    self.headers.get("X-Normal-Radius", "0.3"),
                    self.headers.get("X-Max-Neighbors", "30"),
                    self.headers.get("X-Ground-Max", "8"),
                    self.headers.get("X-Slope-Max", "38"),
                    self.headers.get("X-Wall-Min", "70"),
                )
            else:
                self.send_json(404, {"error": "not found"})
                return
            self.send_binary(result)
        except (ValueError, RuntimeError) as exc:
            self.send_json(400, {"error": str(exc)})
        except Exception as exc:
            self.send_json(500, {"error": f"后端处理失败: {exc}"})


def main():
    parser = argparse.ArgumentParser(description="CloudLab Studio optional Open3D localhost service")
    parser.add_argument("--host", default="127.0.0.1", help="Keep 127.0.0.1 unless remote access is explicitly required")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    server = ThreadingHTTPServer((args.host, args.port), CloudLabRequestHandler)
    capabilities = open3d_capabilities()
    print(f"CloudLab Open3D service: http://{args.host}:{args.port}")
    print(json.dumps(capabilities, ensure_ascii=False))
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
