import math, random, sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from limo_scan_normalizer.core import normalize

A0, A1 = -1.9198621771937625, 1.9198621771937625

def make(n, fn=None, invalid_every=7):
    inc = (A1 - A0) / (n - 1)
    rr = []
    for j in range(n):
        a = A0 + j * inc
        rr.append(0.0 if invalid_every and j % invalid_every == 0 else (fn(a) if fn else 1.0 + 0.5 * math.sin(3 * a)))
    return inc, rr

def karto_expected(r):
    return int(round((r['angle_max'] - r['angle_min']) / r['angle_increment'])) + 1

def test_identity_when_grid_matches():
    inc, rr = make(244); r = normalize(A0, inc, rr, [], 0.1)
    assert r['ranges'] == rr and r['src_index'] == list(range(244)) and r['max_angle_err'] < 1e-9

def test_karto_expected_count_is_constant_for_all_input_sizes():
    for n in range(236, 252):
        inc, rr = make(n); r = normalize(A0, inc, rr, [], 0.1)
        assert len(r['ranges']) == 244 and karto_expected(r) == 244, n
        assert abs(r['angle_max'] - A1) < 1e-9 and abs(r['angle_min'] - A0) < 1e-12

def test_nearest_beam_mapping_and_angle_error_bound():
    for n in (240, 242, 245, 250):
        inc, rr = make(n); r = normalize(A0, inc, rr, [], 0.1)
        assert r['max_angle_err'] <= 0.5 * inc + 1e-9, (n, r['max_angle_err'], 0.5 * inc)
        for k, j in enumerate(r['src_index']):
            assert j >= 0 and r['ranges'][k] == rr[j]          # a copy of a real source beam, never a blend

def test_no_invented_ranges_and_invalid_stays_invalid():
    inc, rr = make(245, invalid_every=5); r = normalize(A0, inc, rr, [], 0.1)
    assert set(r['ranges']) <= set(rr)
    assert not any(math.isnan(x) or math.isinf(x) for x in r['ranges'])      # nothing introduced
    for k, j in enumerate(r['src_index']):
        assert (r['ranges'][k] == 0.0) == (rr[j] == 0.0)

def test_outside_source_coverage_becomes_nan_not_a_distance():
    inc, rr = make(100)                     # narrower source: 100 beams but claim only the middle third
    r = normalize(-0.5, 0.01, [1.0] * 100, [], 0.1)       # covers [-0.5, 0.49]
    assert any(math.isnan(x) for x in r['ranges']) and all(x == 1.0 or math.isnan(x) for x in r['ranges'])

def test_intensities_follow_ranges_and_time_increment():
    inc, rr = make(245); it = [float(j) for j in range(245)]
    r = normalize(A0, inc, rr, it, 0.1)
    assert [it[j] for j in r['src_index']] == r['intensities']
    assert abs(r['time_increment'] - (r['angle_increment'] / (2 * math.pi)) * 0.1) < 1e-12
    assert normalize(A0, inc, rr, [], 0.1)['intensities'] == []

if __name__ == '__main__':
    for name, fn in list(globals().items()):
        if name.startswith('test_'):
            fn(); print('PASS', name)
