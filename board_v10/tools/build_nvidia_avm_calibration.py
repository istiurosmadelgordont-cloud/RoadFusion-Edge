"""Build a ground-plane AVM map from a PhysicalAI-AV clip's camera calibration.

The downloaded calibration belongs to one clip and one camera rig. The output
must only be paired with video from that clip; this script does not infer a
calibration from uncalibrated video.
"""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation


CAMERAS = {
    "front": "camera_front_wide_120fov",
    "rear": "camera_rear_tele_30fov",
    "left": "camera_cross_left_120fov",
    "right": "camera_cross_right_120fov",
    "rear_left": "camera_rear_left_70fov",
    "rear_right": "camera_rear_right_70fov",
    "front_tele": "camera_front_tele_30fov",
}


def calibration_table(root: Path, name: str, chunk: int, clip: str):
    path = root / name / f"{name}.chunk_{chunk:04d}.parquet"
    return pd.read_parquet(path).loc[clip]


def build(group: Path, output: Path, pixels_per_metre: float):
    manifest = json.loads((group / "additional_views" / "manifest.json").read_text())
    root = group / "additional_views" / "calibration"
    clip, chunk = manifest["clip_id"], int(manifest["chunk"])
    intrinsics = calibration_table(root, "camera_intrinsics.offline", chunk, clip)
    extrinsics = calibration_table(root, "sensor_extrinsics.offline", chunk, clip)
    dimensions = calibration_table(root, "vehicle_dimensions", chunk, clip)

    side = 256
    ys, xs = np.mgrid[:side, :side]
    ground = np.stack(
        ((side / 2 - ys) / (pixels_per_metre * side / 640)
         + dimensions.rear_axle_to_bbox_center,
         -(xs - side / 2) / (pixels_per_metre * side / 640),
         np.zeros_like(xs)), axis=-1
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    storage = cv2.FileStorage(str(output), cv2.FileStorage_WRITE)
    if not storage.isOpened():
        raise OSError(f"Cannot write {output}")
    storage.write("version", 2)
    storage.write("pixels_per_metre", pixels_per_metre)
    storage.write("vehicle_width", float(dimensions.width))
    storage.write("vehicle_length", float(dimensions.length))

    try:
        for name, camera in CAMERAS.items():
            params = json.loads(intrinsics.loc[camera, "model_parameters"])
            resolution = params["resolution"]
            width, height = int(resolution[0]), int(resolution[1])
            pose = extrinsics.loc[camera]
            rotation = Rotation.from_quat(pose[["qx", "qy", "qz", "qw"]]).as_matrix()
            rays = (ground - pose[["x", "y", "z"]].to_numpy()) @ rotation
            ray_radius = np.linalg.norm(rays[..., :2], axis=-1)
            theta = np.arctan2(ray_radius, rays[..., 2])

            principal = np.asarray(params["principal_point"], dtype=float)
            corners = np.array([[0, 0], [width - 1, 0],
                                [0, height - 1], [width - 1, height - 1]])
            max_radius = np.linalg.norm(corners - principal, axis=1).max()
            # Outside the image domain the fitted polynomial can turn back.
            # np.interp requires strictly increasing angles for the inverse.
            radii = np.linspace(0, max_radius, 20001)
            angles = np.polynomial.polynomial.polyval(
                radii, params["pixeldist_to_angle_poly"]
            )
            if not np.all(np.diff(angles) > 0):
                raise ValueError(f"Non-monotonic in-image calibration: {camera}")
            radius = np.interp(theta, angles, radii)
            pixel = np.asarray(params["principal_point"]) + (
                radius[..., None] * rays[..., :2]
                / np.maximum(ray_radius[..., None], 1e-8)
            )
            u = (pixel[..., 0] / (width - 1)).astype(np.float32)
            v = (pixel[..., 1] / (height - 1)).astype(np.float32)
            valid = ((theta <= angles[-1]) & (theta < params["max_angle"]) & (u >= 0) & (u <= 1)
                     & (v >= 0) & (v <= 1))
            ego_mask = cv2.imdecode(
                np.frombuffer(intrinsics.loc[camera, "ego_mask_image_png"], np.uint8),
                cv2.IMREAD_GRAYSCALE,
            )
            if ego_mask is None:
                raise ValueError(f"Missing ego mask for {camera}")
            mask_sample = cv2.remap(
                ego_mask, pixel[..., 0].astype(np.float32),
                pixel[..., 1].astype(np.float32), cv2.INTER_NEAREST,
                borderMode=cv2.BORDER_CONSTANT, borderValue=255,
            )
            valid &= mask_sample == 0

            storage.startWriteStruct(name, cv2.FileNode_MAP)
            storage.write("map_x", u)
            storage.write("map_y", v)
            storage.write("mask", valid.astype(np.uint8))
            storage.endWriteStruct()
            print(f"{name}: {np.count_nonzero(valid) / valid.size:.1%} of BEV pixels")
    finally:
        storage.release()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("group", type=Path, help="Downloaded ADAS material group")
    parser.add_argument("output", type=Path, help="Scene avm.yml path")
    parser.add_argument("--pixels-per-metre", type=float, default=23.0)
    args = parser.parse_args()
    build(args.group, args.output, args.pixels_per_metre)
