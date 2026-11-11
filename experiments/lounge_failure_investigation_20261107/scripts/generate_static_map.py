#!/usr/bin/env python3
"""
generate_static_map.py:
Generates the baseline static 2D occupancy grid map (PGM and YAML)
for the 12m x 9m Lounge World architecture.
Includes permanent structural walls (outer perimeter and solid divider walls)
while leaving tables and glass partition for dynamic obstacle layer evaluation.
"""
from pathlib import Path
import numpy as np
from PIL import Image

BASE_DIR = Path('/home/cjinw/school/2026-2/Capstone1_limo/experiments/lounge_failure_investigation_20261107')
MAPS_DIR = BASE_DIR / 'maps'
MAPS_DIR.mkdir(parents=True, exist_ok=True)

def create_static_map():
    res = 0.05 # 5cm per pixel
    origin_x = -6.1
    origin_y = -4.6
    width_m = 12.2
    height_m = 9.2

    cols = int(round(width_m / res)) # 244
    rows = int(round(height_m / res)) # 184

    # 254 = Free Space (white)
    # 0 = Occupied / Wall (black)
    grid = np.full((rows, cols), 254, dtype=np.uint8)

    def draw_box(cx, cy, w, h):
        x_min = cx - w/2.0
        x_max = cx + w/2.0
        y_min = cy - h/2.0
        y_max = cy + h/2.0

        c_min = max(0, int(round((x_min - origin_x) / res)))
        c_max = min(cols, int(round((x_max - origin_x) / res)))
        r_max = rows - max(0, int(round((y_min - origin_y) / res)))
        r_min = rows - min(rows, int(round((y_max - origin_y) / res)))
        grid[r_min:r_max, c_min:c_max] = 0

    # 1. Outer Boundary Walls (Thickness 0.2m)
    draw_box(0.0, 4.5, 12.2, 0.2)  # North Wall
    draw_box(0.0, -4.5, 12.2, 0.2) # South Wall
    draw_box(-6.0, 0.0, 0.2, 9.2)  # West Wall
    draw_box(6.0, 0.0, 0.2, 9.2)   # East Wall

    # 2. Interior Solid Partition Wall (West portion of Y = -1.5m)
    draw_box(-4.25, -1.5, 3.5, 0.2)

    # 3. Door Post Pillar
    draw_box(-1.35, -1.5, 0.10, 0.20)

    # 4. Narrow Corridor Wall (X = 2.6m, Y in [1.5, 4.5])
    draw_box(2.6, 3.0, 0.15, 3.0)

    img = Image.fromarray(grid, mode='L')
    pgm_path = MAPS_DIR / 'lounge_map.pgm'
    img.save(pgm_path)
    print(f"[OK] Saved lounge static map image -> {pgm_path}")

    yaml_content = f"""image: lounge_map.pgm
mode: trinary
resolution: {res}
origin: [{origin_x:.3f}, {origin_y:.3f}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
"""
    yaml_path = MAPS_DIR / 'lounge_map.yaml'
    with open(yaml_path, 'w') as f:
        f.write(yaml_content)
    print(f"[OK] Saved lounge static map YAML -> {yaml_path}")

if __name__ == '__main__':
    create_static_map()
