"""Camera field-of-view checks and pinhole measurement modeling.

Part G supplies visibility, projection, and pixel covariance (docs/HUONG_DAN_KY_THUAT.md §2).
The platform differentiates projection using a chain-rule Jacobian.
"""

from __future__ import annotations

from typing import Any
from typing import Sequence

import numpy as np

from fusion_lab.workspace_support import get_tracking_params

Matrix = np.matrix | np.ndarray

MIN_DEPTH = 1e-6


def _sensor_position(x: Matrix, sensor: Any) -> np.ndarray:
    """Transform the state position into sensor coordinates: p_s = R p + t."""
    position = np.asarray(x, dtype=float).reshape(-1)[:3]
    transform = np.asarray(sensor.veh_to_sens, dtype=float)
    return transform[:3, :3] @ position + transform[:3, 3]


def _is_camera(sensor: Any) -> bool:
    return getattr(sensor, "name", None) == "camera" or hasattr(sensor, "f_i")


def is_in_field_of_view(x: Matrix, sensor: Any) -> bool:
    """Return True if state x is visible within the sensor horizontal field of view.

    Args:
        x: State vector (6x1) with position in vehicle frame.
        sensor: Lidar or camera adapter with ``veh_to_sens`` and ``fov``
            (radians).

    Returns:
        True if sensor coordinates are finite and the horizontal angle is within
        ``sensor.fov``. A camera additionally requires depth > 1e-6.
    """
    p_s = _sensor_position(x, sensor)
    if not np.isfinite(p_s).all():
        return False
    # Points behind or on the image plane have no valid pinhole projection.
    if _is_camera(sensor) and p_s[0] <= getattr(sensor, "depth_epsilon", MIN_DEPTH):
        return False
    # Camera bounds come from intrinsics and image width: atan((c_i - [width, 0]) / f_i).
    low, high = sensor.fov
    angle = np.arctan2(p_s[1], p_s[0])
    return bool(low <= angle <= high)


def camera_measurement_prediction(x: Matrix, sensor: Any) -> Matrix:
    """Predict image-plane measurement h(x) using the pinhole camera model.

    Args:
        x: State vector.
        sensor: Camera with intrinsics ``f_i, f_j, c_i, c_j``.

    Returns:
        2x1 predicted pixel coordinates as ``np.matrix``.

    Raises:
        ValueError: With coordinate context if sensor coordinates are nonfinite
            or depth is at most 1e-6.
    """
    p_s = _sensor_position(x, sensor)
    depth_epsilon = getattr(sensor, "depth_epsilon", MIN_DEPTH)
    if not np.isfinite(p_s).all() or p_s[0] <= depth_epsilon:
        raise ValueError(
            "Camera projection needs finite coordinates and positive depth "
            f"> {depth_epsilon:g}; sensor position={p_s.tolist()}"
        )
    depth, left, up = p_s
    # Waymo camera axes are forward, left, up: left/up decrease the pixel indices.
    u = sensor.c_i - sensor.f_i * left / depth
    v = sensor.c_j - sensor.f_j * up / depth
    return np.asmatrix([[u], [v]], dtype=float)


def build_camera_measurement(z: Sequence[float], sensor: Any) -> dict[str, Any]:
    """Build camera measurement vector z and covariance R from pixel coordinates.

    Args:
        z: Sequence ``[u, v]`` pixel coordinates.
        sensor: Camera sensor object.

    Returns:
        Dict with keys ``z``, ``R``, ``sensor``.
    """
    params = get_tracking_params()
    pixels = np.asarray(z, dtype=float).reshape(-1)[:2]
    z_mat = np.asmatrix(pixels).T
    R = np.asmatrix(np.diag([params.sigma_cam_i**2, params.sigma_cam_j**2]))
    return {"z": z_mat, "R": R, "sensor": sensor}
