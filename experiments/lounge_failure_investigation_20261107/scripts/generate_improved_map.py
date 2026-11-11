#!/usr/bin/env python3
"""
generate_improved_map.py:
Generates the improved static 2D occupancy grid map (PGM and YAML)
which includes:
1. Virtual keepout zone for Glass Partition Wall (Y = -1.5m, X in [-1.3, 5.9])
2. Virtual keepout zone for Table 1 Underpass (X = -2.5m, Y = 1.8m)
3. Virtual keepout zone for Table 2 Underpass
"""
from pathlib import Path
import numpy as np
from PIL import Image

BASE_DIR = Path('/home/cjinw/school/2026-2/Capstone1_limo/experiments/lounge_failure_investigation_20261107')
MAPS_DIR = BASE_DIR / 'maps'
MAPS_DIR.mkdir(parents=True, exist_ok=True)

def create_improved_map():
    res = 0.05 # 5cm per pixel
    origin_x = -6.1
    origin_y = -4.6
    width_m = 12.2
    height_m = 9.2

    cols = int(round(width_m / res)) # 244
    rows = int(round(height_m / res)) # 184

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

    # 2. Interior Solid Partition Wall
    draw_box(-4.25, -1.5, 3.5, 0.2)
    draw_box(-1.35, -1.5, 0.10, 0.20)
    draw_box(2.6, 3.0, 0.15, 3.0)

    # 3. IMPROVEMENT: Glass Wall Keepout Virtual Wall (Y = -1.5m, X from -1.3m to 5.9m)
    draw_box(2.3, -1.5, 7.2, 0.15)

    # 4. IMPROVEMENT: Table Underpass Keepout Zones
    # Table 1: cx=-2.5, cy=1.8, size 1.4m x 0.9m
    draw_box(-2.5, 1.8, 1.4, 0.9)
    # Table 2: cx=0.5, cy=1.8, size 1.4m x 0.9m
    draw_box(0.5, 1.8, 1.4, 0.9)

    img = Image.fromarray(grid, mode='L')
    pgm_path = MAPS_DIR / 'lounge_map_improved.pgm'
    img.save(pgm_path)
    print(f"[OK] Saved improved static map image -> {pgm_path}")

    yaml_content = f"""image: lounge_map_improved.pgm
mode: trinary
resolution: {res}
origin: [{origin_x:.3f}, {origin_y:.3f}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
"""
    yaml_path = MAPS_DIR / 'lounge_map_improved.yaml'
    with open(yaml_path, 'w') as f:
        f.write(yaml_content)
    print(f"[OK] Saved improved static map YAML -> {yaml_path}")

if __name__ == '__main__':
    create_improved_map()
