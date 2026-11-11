"""Geometry-preserving fixed-beam-count resampling of a sensor_msgs/LaserScan (pure python, no ROS imports).

Why: Karto (slam_toolbox) derives the expected number of readings once, from the first scan it sees, as
round((angle_max - angle_min) / angle_increment) + 1, and rejects every later scan whose ranges length differs.
The YDLidar driver on the LIMO (fixed_resolution: false) publishes 242..250 readings per revolution.

Rules (what this function guarantees):
  * output grid: N_out beams from angle_min_out to angle_max_out, increment (max-min)/(N_out-1), so Karto's expected
    count is exactly N_out for every output scan;
  * every output beam copies the NEAREST source beam (angular error <= half a source increment, ~0.45 deg); ranges are
    never interpolated or averaged, so no depth edge is smeared and no range value is invented;
  * invalid source readings (0.0 / inf / nan / <= range_min) are copied as they are; an output beam whose angle lies
    outside the source coverage by more than half a source increment becomes NaN (invalid), never a distance;
  * intensities follow the same mapping; header (stamp, frame_id), range_min/max and scan_time are preserved,
    time_increment is recomputed for the new grid.
"""
import math


def normalize(angle_min, angle_increment, ranges, intensities, scan_time,
              n_out=244, angle_min_out=-1.9198621771937625, angle_max_out=1.9198621771937625):
    """Return dict(angle_min, angle_max, angle_increment, time_increment, ranges, intensities, src_index, max_angle_err)."""
    n_src = len(ranges)
    inc_out = (angle_max_out - angle_min_out) / (n_out - 1)
    out_r, out_i, src_idx = [], [], []
    max_err = 0.0
    have_int = len(intensities) == n_src and n_src > 0
    for k in range(n_out):
        a = angle_min_out + k * inc_out
        j = int(round((a - angle_min) / angle_increment)) if n_src else -1
        if 0 <= j < n_src:
            err = abs(a - (angle_min + j * angle_increment))
            max_err = max(max_err, err)
            out_r.append(ranges[j]); out_i.append(intensities[j] if have_int else 0.0); src_idx.append(j)
        else:
            out_r.append(float('nan')); out_i.append(0.0); src_idx.append(-1)
    time_inc = (inc_out / (2.0 * math.pi)) * scan_time if scan_time > 0 else 0.0
    return dict(angle_min=angle_min_out, angle_max=angle_min_out + (n_out - 1) * inc_out, angle_increment=inc_out,
                time_increment=time_inc, ranges=out_r, intensities=out_i if have_int else [], src_index=src_idx, max_angle_err=max_err)
