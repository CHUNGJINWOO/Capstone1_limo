"""OccupancyGrid -> PNG (free white, occupied black, unknown grey; image row 0 = top = +y) and map statistics."""
import hashlib, io
import numpy as np


def grid_stats(msg):
    a = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
    free = int(((a >= 0) & (a < 25)).sum()); occ = int((a >= 65).sum())
    return a, {'known_cells': free + occ, 'free_cells': free, 'occupied_cells': occ, 'unknown_cells': int((a < 0).sum())}, hashlib.sha1(a.tobytes()).hexdigest()


def grid_png(a):
    img = np.full(a.shape, 128, np.uint8); img[(a >= 0) & (a < 25)] = 255; img[a >= 65] = 0
    img = np.flipud(img)
    from PIL import Image
    buf = io.BytesIO(); Image.fromarray(img).save(buf, format='PNG', optimize=True); return buf.getvalue()
