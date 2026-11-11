#!/usr/bin/env python3
"""
generate_keepout_mask.py:
Generates Keepout filter mask PGM and YAML for Nav2 Costmap Filter.
Enforces table underpass exclusion zones (keepout zones) as an optional/comparative layer.
"""
from pathlib import Path
import numpy as np
from PIL import Image

BASE_DIR = Path('/home/cjinw/school/2026-2/Capstone1_limo/experiments/lounge_failure_investigation_20261107')
WORLDS_DIR = BASE_DIR / 'worlds'

def create_keepout_mask():
    res = 0.05 # 5cm per pixel
    origin_x = -6.0
    origin_y = -4.5
    width_m = 12.0
    height_m = 9.0

    cols = int(round(width_m / res)) # 240
    rows = int(round(height_m / res)) # 180

    # Nav2 Costmap Filter keepout mask:
    # 0 = free space (cost = 0)
    # 100 = keepout / lethal obstacle (cost = 254)
    # In standard PGM representation (0..255):
    # Occupied/Forbidden = 0 (black), Free = 254 (white)
    mask = np.full((rows, cols), 254, dtype=np.uint8)

    tables = [
        (-2.5, 1.8, 1.40, 0.80), # Table 1
        (0.5, 1.8, 1.40, 0.80),  # Table 2
        (-2.5, 0.0, 1.40, 0.80), # Table 3
        (1.5, -3.0, 0.80, 1.40), # VIP Table
    ]

    for (cx, cy, w, h) in tables:
        x_min = cx - w/2.0
        x_max = cx + w/2.0
        y_min = cy - h/2.0
        y_max = cy + h/2.0

        c_min = max(0, int(round((x_min - origin_x) / res)))
        c_max = min(cols, int(round((x_max - origin_x) / res)))
        # Note: image row 0 is top, y increases downwards in image array, upwards in map
        r_max = rows - max(0, int(round((y_min - origin_y) / res)))
        r_min = rows - min(rows, int(round((y_max - origin_y) / res)))

        mask[r_min:r_max, c_min:c_max] = 0 # Lethal keepout

    img = Image.fromarray(mask, mode='L')
    pgm_path = WORLDS_DIR / 'lounge_keepout_mask.pgm'
    img.save(pgm_path)
    print(f"[OK] Generated keepout mask image -> {pgm_path}")

    yaml_content = f"""image: lounge_keepout_mask.pgm
mode: trinary
resolution: {res}
origin: [{origin_x:.3f}, {origin_y:.3f}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
"""
    yaml_path = WORLDS_DIR / 'lounge_keepout_mask.yaml'
    with open(yaml_path, 'w') as f:
        f.write(yaml_content)
    print(f"[OK] Generated keepout mask YAML -> {yaml_path}")

if __name__ == '__main__':
    create_keepout_mask()
