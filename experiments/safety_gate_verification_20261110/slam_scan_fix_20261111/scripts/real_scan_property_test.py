import os, sys, math, collections, numpy as np, rosbag2_py
sys.path.insert(0, os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'variants', 'scan_normalizer_ws', 'src', 'limo_scan_normalizer')))
from limo_scan_normalizer.core import normalize
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
tot = collections.Counter(); worst = 0.0; worst_ratio = 0.0; bad = []
for bag in sys.argv[1:]:
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    while r.has_next():
        topic, data, t = r.read_next()
        if topic != '/scan': continue
        m = deserialize_message(data, get_message(types[topic]))
        rg, it = list(m.ranges), list(m.intensities)
        o = normalize(m.angle_min, m.angle_increment, rg, it, m.scan_time)
        n_in = len(rg); tot['scans'] += 1; tot['N_in=%d' % n_in] += 1
        ok = True
        ok &= len(o['ranges']) == 244 and int(round((o['angle_max'] - o['angle_min']) / o['angle_increment'])) + 1 == 244
        ok &= abs(o['angle_max'] - m.angle_max) < 1e-5 and abs(o['angle_min'] - m.angle_min) < 1e-5
        ok &= all(j >= 0 for j in o['src_index'])
        ok &= all(o['ranges'][k] == rg[j] for k, j in enumerate(o['src_index']))                  # pure copies
        ok &= all(o['intensities'][k] == it[j] for k, j in enumerate(o['src_index']))
        zin = sum(1 for x in rg if x <= m.range_min); zout = sum(1 for x in o['ranges'] if x <= m.range_min)
        ok &= not any(math.isnan(x) or math.isinf(x) for x in o['ranges'])
        worst = max(worst, o['max_angle_err']); worst_ratio = max(worst_ratio, o['max_angle_err'] / (0.5 * m.angle_increment))
        # distinct source beams used / dropped
        used = len(set(o['src_index'])); tot['src_beams_dropped'] += n_in - used; tot['out_beams_duplicated'] += 244 - used
        tot['invalid_in_share_x1e4'] += int(1e4 * zin / n_in); tot['invalid_out_share_x1e4'] += int(1e4 * zout / 244)
        if not ok: bad.append((bag.split('/')[-2], t))
print('scans checked:', tot['scans'], '| failures:', len(bad))
print('input lengths:', {k: v for k, v in tot.items() if k.startswith('N_in')})
print('max angular error: %.5f rad = %.3f deg (limit: half a source increment, worst ratio to the limit %.2f)' % (worst, math.degrees(worst), worst_ratio))
print('avg per scan: source beams dropped %.2f, output beams duplicated from a neighbour %.2f' % (tot['src_beams_dropped'] / tot['scans'], tot['out_beams_duplicated'] / tot['scans']))
print('avg invalid(<=range_min) share: input %.2f%%, output %.2f%%' % (tot['invalid_in_share_x1e4'] / tot['scans'] / 100, tot['invalid_out_share_x1e4'] / tot['scans'] / 100))
