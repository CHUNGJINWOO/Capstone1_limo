import time
import numpy as np
from PIL import Image

t0 = time.time()
# Simulate a 160x160 occupancy grid
grid_arr = np.full((160, 160), -1, dtype=np.int8)
# some free space
grid_arr[40:120, 40:120] = 0
# some obstacles
grid_arr[40, 40:120] = 100
grid_arr[120, 40:120] = 100

map_rgb = np.full((160, 160, 3), [20, 29, 46], dtype=np.uint8)
map_rgb[grid_arr == 0] = [235, 240, 245]
map_rgb[(grid_arr > 0) & (grid_arr < 100)] = [245, 158, 11]
map_rgb[grid_arr == 100] = [15, 23, 42]

img = Image.fromarray(map_rgb).transpose(Image.FLIP_TOP_BOTTOM)
img_resized = img.resize((760, 700), Image.NEAREST)
t1 = time.time()
print(f"NumPy map rendering time: {(t1 - t0)*1000:.2f} ms")
