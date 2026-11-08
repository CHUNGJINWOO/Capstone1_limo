"""
filtering.py
Ground segmentation, vertical windowing, and obstacle extraction
for Orbbec DaBai Depth Camera on AgileX LIMO Pro.
"""
import math
import numpy as np


def rotation(q):
    a = np.asarray([q.x, q.y, q.z, q.w], dtype=float)
    norm = np.linalg.norm(a)
    if not np.isfinite(a).all() or norm < 1e-9:
        raise ValueError('Invalid camera TF quaternion')
    x, y, z, w = a / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]
    ])


def filter_points(points, transform, minimum_range, maximum_range,
                  minimum_height, maximum_height, self_radius):
    camera = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    distances = np.linalg.norm(camera, axis=1)
    valid = np.isfinite(camera).all(axis=1) & (distances >= minimum_range) & (distances <= maximum_range)
    clear = camera[valid]
    t = transform.translation
    offset = np.array([t.x, t.y, t.z])
    if not np.isfinite(offset).all():
        raise ValueError('Invalid camera TF translation')
    body = clear @ rotation(transform.rotation).T + offset
    mask = (
        (body[:, 2] >= minimum_height) &
        (body[:, 2] <= maximum_height) &
        (np.hypot(body[:, 0], body[:, 1]) > self_radius)
    )
    return clear[mask], clear, {
        'input_points': len(camera),
        'valid_points': len(clear),
        'obstacle_points': int(mask.sum())
    }


def stamp_age(now, stamp):
    age = now - stamp
    return age if math.isfinite(age) else float('inf')
