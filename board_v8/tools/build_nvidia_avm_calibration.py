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


def bivariate(coefficients, horizontal, vertical):
    coefficients = np.asarray(coefficients, dtype=float)
    if not len(coefficients) or not np.isfinite(coefficients).all():
        raise ValueError('Invalid windshield polynomial coefficients')
    order = int((np.sqrt(8 * len(coefficients) + 1) - 3) / 2)
    if (order + 1) * (order + 2) // 2 != len(coefficients):
        raise ValueError('Invalid windshield polynomial coefficient count')
    result = np.zeros_like(horizontal)
    offset = 0
    for j in range(order + 1):
        count = order - j + 1
        result += np.polynomial.polynomial.polyval(horizontal, coefficients[offset:offset+count]) * vertical**j
        offset += count
    return result


def external_rays(rays, parameters):
    """World-to-lens windshield distortion, using the reference polynomial.

    Model and coefficient order: https://nvidia.github.io/ncore/data/sensor_models.html
    Pixel coordinates here use pixel centers, as OpenCV remap does.
    """
    if parameters is None:
        return rays, np.ones(rays.shape[:-1], bool)
    if parameters.get('external_distortion_type', 'bivariate-windshield') != 'bivariate-windshield':
        raise ValueError('Unsupported external camera distortion model')
    unit = rays / np.maximum(np.linalg.norm(rays, axis=-1, keepdims=True), 1e-12)
    horizontal, vertical = np.arcsin(np.clip(unit[..., 0], -1, 1)), np.arcsin(np.clip(unit[..., 1], -1, 1))
    kind = parameters['reference_poly']
    if kind == 'FORWARD':
        h = bivariate(parameters['horizontal_poly'], horizontal, vertical)
        v = bivariate(parameters['vertical_poly'], horizontal, vertical)
    elif kind == 'BACKWARD':
        # Invert the exact backward mapping rather than trusting an approximate
        # forward polynomial. Newton iterations stay in spherical-angle space.
        h = bivariate(parameters['horizontal_poly'], horizontal, vertical)
        v = bivariate(parameters['vertical_poly'], horizontal, vertical)
        epsilon = 1e-6
        for _ in range(12):
            ph = bivariate(parameters['horizontal_poly_inverse'], h, v)
            pv = bivariate(parameters['vertical_poly_inverse'], h, v)
            a = (bivariate(parameters['horizontal_poly_inverse'], h+epsilon, v)-ph)/epsilon
            b = (bivariate(parameters['horizontal_poly_inverse'], h, v+epsilon)-ph)/epsilon
            c = (bivariate(parameters['vertical_poly_inverse'], h+epsilon, v)-pv)/epsilon
            d = (bivariate(parameters['vertical_poly_inverse'], h, v+epsilon)-pv)/epsilon
            determinant = a*d-b*c
            if np.any(abs(determinant) < 1e-8):
                raise ValueError('Non-invertible windshield polynomial')
            dh = (d*(ph-horizontal)-b*(pv-vertical))/determinant
            dv = (a*(pv-vertical)-c*(ph-horizontal))/determinant
            h -= dh
            v -= dv
        if np.any(abs(bivariate(parameters['horizontal_poly_inverse'], h, v)-horizontal) > 1e-6) or np.any(abs(bivariate(parameters['vertical_poly_inverse'], h, v)-vertical) > 1e-6):
            raise ValueError('Windshield polynomial inversion did not converge')
    else:
        raise ValueError('Unsupported windshield reference polynomial')
    x, y = np.sin(h), np.sin(v)
    square_z = 1-x*x-y*y
    valid = (unit[..., 2] > 0) & (square_z >= 0) & np.isfinite(square_z)
    return np.stack([x, y, np.sqrt(np.maximum(square_z, 0))], axis=-1), valid


def project_ftheta(rays, parameters):
    rays, valid = external_rays(rays, parameters.get('external_distortion_parameters'))
    valid &= np.linalg.norm(rays, axis=-1) > 1e-12
    ray_radius = np.linalg.norm(rays[..., :2], axis=-1)
    theta = np.arctan2(ray_radius, rays[..., 2])
    width, height = parameters['resolution']
    principal = np.asarray(parameters['principal_point'], dtype=float)
    c, d, e = parameters.get('linear_cde', [1., 0., 0.])
    linear = np.array([[c, d], [e, 1.]])
    if not np.isfinite(linear).all() or abs(np.linalg.det(linear)) < 1e-8:
        raise ValueError('Invalid FTheta linear transform')
    kind = parameters['reference_poly']
    if kind == 'ANGLE_TO_PIXELDIST':
        radius = np.polynomial.polynomial.polyval(theta, parameters['angle_to_pixeldist_poly'])
    elif kind == 'PIXELDIST_TO_ANGLE':
        corners = np.array([[0, 0], [width-1, 0], [0, height-1], [width-1, height-1]])
        max_radius = np.linalg.norm((corners-principal) @ np.linalg.inv(linear).T, axis=1).max()
        radii = np.linspace(0, max_radius, 20001)
        angles = np.polynomial.polynomial.polyval(radii, parameters['pixeldist_to_angle_poly'])
        if not np.all(np.diff(angles) > 0):
            raise ValueError('Non-monotonic in-image FTheta calibration')
        radius = np.interp(theta, angles, radii)
        valid &= (theta >= angles[0]) & (theta <= angles[-1])
    else:
        raise ValueError('Unsupported FTheta reference polynomial')
    sensor_pixel = radius[..., None] * rays[..., :2] / np.maximum(ray_radius[..., None], 1e-12)
    pixel = principal + sensor_pixel @ linear.T
    valid &= (theta < parameters['max_angle']) & (radius >= 0) & np.isfinite(pixel).all(axis=-1)
    return pixel, valid


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
    storage.write("clip_id", clip)
    storage.write("projection_model", "ftheta-reference-linear-windshield-v1")
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
            pixel, angular_valid = project_ftheta(rays, params)
            u = (pixel[..., 0] / (width - 1)).astype(np.float32)
            v = (pixel[..., 1] / (height - 1)).astype(np.float32)
            valid = (angular_valid & (u >= 0) & (u <= 1)
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
