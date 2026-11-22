#!/usr/bin/env python3
"""Registry authoring API (no ROS, no robot): POST /map-registry/areas and the authoring fields of POST /map-registry/maps.

Covers: area creation rules, area link + lineage on upload (validated BEFORE storing anything), the unknown-pixel threshold
safety check shared with scripts/register_saved_map.py (reject by default, derive a corrected yaml copy on request,
originals + hashes recorded), and that a rejected request stores nothing.
"""
import base64
import hashlib
import json
import os
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'server')]
from fleet_server.auth import Principal  # noqa: E402
from fleet_server.map_registry import MapRegistry  # noqa: E402

P = '/api/v1/map-registry'


def pgm(trinary=True, w=24, h=20):
    px = bytearray()
    for r in range(h):
        for c in range(w):
            if r < 2 or r >= h - 2 or c < 2 or c >= w - 2:
                px.append(0)
            elif r == 7 and c % 3 == 0:
                px.append(205 if trinary else 100)
            else:
                px.append(254 if trinary else 240)
    for i in range(8, 14):                       # an unknown block, like a real saved map
        px[10 * w + i] = 205 if trinary else 120
    return f'P5\n{w} {h}\n255\n'.encode() + bytes(px)


def yml(free='0.196', occ='0.65', negate=0):
    return (f'image: map.pgm\nmode: trinary\nresolution: 0.05\norigin: [-1.0, -1.0, 0.0]\nnegate: {negate}\n'
            f'occupied_thresh: {occ}\nfree_thresh: {free}\n').encode()


def b64(b):
    return base64.b64encode(b).decode()


def sha(b):
    return hashlib.sha256(b).hexdigest()


class AuthoringTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.reg = MapRegistry(os.path.join(self.tmp.name, 'r.sqlite'), os.path.join(self.tmp.name, 'store'))
        self.op, self.viewer = Principal('op', 'operator'), Principal('v', 'viewer')
        self.reg.create_facility('fac', '배재대학교', actor='t')
        self.reg.create_building('bld-h', 'fac', 'H관', actor='t')
        self.reg.create_floor('fl-h1', 'bld-h', '1F', elevation_m=0.0, actor='t')
        self.reg.create_floor('fl-h2', 'bld-h', '2F', elevation_m=3.0, actor='t')
        self.n = 0

    def tearDown(self):
        self.reg.close()
        self.tmp.cleanup()

    def post(self, path, body, who=None, idem=None):
        hdr = {'Idempotency-Key': idem} if idem else None
        st, b, _ = self.reg.dispatch_write('POST', P + path, who or self.op, body=body, headers=hdr)
        return st, b

    def upload(self, yaml_b=None, pgm_b=None, **extra):
        self.n += 1
        body = dict(floor_id='fl-h1', label=f'm{self.n}', yaml_b64=b64(yaml_b or yml()), pgm_b64=b64(pgm_b or pgm()), **extra)
        return self.post('/maps', body)

    def n_maps(self):
        return self.reg.one('SELECT COUNT(*) AS n FROM map_versions')['n']

    # ---------------------------------------------------------------- areas
    def test_area_create_permissions_idempotency_conflicts(self):
        body = dict(area_id='h107', floor_id='fl-h1', name='H107 강의실', coverage='PARTIAL_FLOOR')
        self.assertEqual(self.post('/areas', body, self.viewer)[0], 403)
        st, b = self.post('/areas', body, idem='a-1')
        self.assertEqual((st, b['area']['name'], b['area']['coverage']), (201, 'H107 강의실', 'PARTIAL_FLOOR'))
        st, b = self.post('/areas', body, idem='a-1')
        self.assertTrue(b.get('idempotent_replay'))
        self.assertEqual(self.post('/areas', dict(body, name='다른 이름'))[0], 409)           # same id, different attributes
        self.assertEqual(self.post('/areas', dict(body, area_id='x', floor_id='nope'))[0], 404)
        self.assertEqual(self.post('/areas', dict(body, area_id='y', coverage='WHOLE'))[0], 400)
        self.assertEqual(self.post('/areas', dict(body, area_id='bad id!'))[0], 400)
        tree = self.reg.list_facilities_tree()
        areas = [a['area_id'] for f in tree for b2 in f['buildings'] for fl in b2['floors'] for a in fl.get('areas', [])]
        self.assertEqual(areas, ['h107'])

    # ---------------------------------------------------------------- upload + area + lineage
    def test_upload_with_area_and_lineage_and_evidence(self):
        self.post('/areas', dict(area_id='h107', floor_id='fl-h1', name='H107 강의실'))
        st, b = self.upload(area_id='h107', lineage=dict(mapping_mode='NEW_MAP'), note='single position')
        self.assertEqual(st, 201, b)
        mvid = b['map_version']['map_version_id']
        self.assertEqual(b['area_id'], 'h107')
        mv = self.reg.get_map_version(mvid)
        self.assertEqual(mv['area']['area_id'], 'h107')
        self.assertEqual(mv['lineage']['mapping_mode'], 'NEW_MAP')
        self.assertEqual(mv['verification_status'], 'UNVERIFIED')
        ev = mv['verification_evidence']
        self.assertEqual(ev['method'], 'web_upload')
        self.assertEqual(ev['uploader'], 'op')
        self.assertEqual((ev['location']['building'], ev['location']['floor'], ev['location']['area']), ('H관', '1F', 'H107 강의실'))
        self.assertFalse(ev['location']['auto_detected'])
        self.assertIn('known_area_m2', ev['grid_counts_pgm'])
        self.assertIsNone(ev['yaml_adjustment'])

    def test_invalid_authoring_fields_store_nothing(self):
        self.post('/areas', dict(area_id='h107', floor_id='fl-h1', name='H107'))
        self.post('/areas', dict(area_id='h201', floor_id='fl-h2', name='H201'))
        before = self.n_maps()
        self.assertEqual(self.upload(area_id='h201')[0], 409)                                  # area on another floor
        self.assertEqual(self.upload(area_id='nope')[0], 404)
        self.assertEqual(self.upload(lineage=dict(mapping_mode='CONTINUE_MAP'))[0], 400)       # needs a parent
        self.assertEqual(self.upload(lineage=dict(mapping_mode='NEW_MAP', parent_map_version_id='mv-x'))[0], 400)
        self.assertEqual(self.upload(lineage=dict(mapping_mode='TELEPORT'))[0], 400)
        self.assertEqual(self.upload(threshold_policy='ignore')[0], 400)
        self.assertEqual(self.upload(note='x' * 501)[0], 400)
        self.assertEqual(self.n_maps(), before)                                                # nothing was stored

    def test_continue_map_with_parent(self):
        st, b = self.upload(lineage=dict(mapping_mode='NEW_MAP'))
        parent = b['map_version']['map_version_id']
        st, b = self.upload(pgm_b=pgm(w=26), lineage=dict(mapping_mode='CONTINUE_MAP', parent_map_version_id=parent))
        self.assertEqual(st, 201, b)
        self.assertEqual(b['lineage']['parent_map_version_id'], parent)

    def test_retry_with_same_idempotency_key_replays(self):
        self.post('/areas', dict(area_id='h107', floor_id='fl-h1', name='H107'))
        body = dict(floor_id='fl-h1', label='same', yaml_b64=b64(yml()), pgm_b64=b64(pgm()), area_id='h107',
                    lineage=dict(mapping_mode='IMPORTED', session_id='s1'))
        st1, b1 = self.post('/maps', body, idem='up-1')
        st2, b2 = self.post('/maps', body, idem='up-1')
        self.assertEqual((st1, st2), (201, 201))
        self.assertTrue(b2['idempotent_replay'])
        self.assertEqual(self.n_maps(), 1)

    # ---------------------------------------------------------------- threshold safety check
    def test_unsafe_free_thresh_rejected_by_default_and_nothing_stored(self):
        before = self.n_maps()
        st, b = self.upload(yaml_b=yml(free='0.25'))                                           # what map_saver_cli writes
        self.assertEqual(st, 422)
        d = b['error']['detail']
        self.assertEqual(d['code'], 'unknown_pixels_misclassified')
        self.assertTrue(d['fixable_by_free_thresh'])
        self.assertEqual(d['derived_free_thresh'], 0.196)
        self.assertEqual(self.n_maps(), before)

    def test_derive_policy_registers_corrected_copy_and_records_both_hashes(self):
        y, g = yml(free='0.25'), pgm()
        st, b = self.upload(yaml_b=y, pgm_b=g, threshold_policy='derive', expected_checksums=dict(yaml_sha256=sha(y), pgm_sha256=sha(g)))
        self.assertEqual(st, 201, b)
        adj = b['yaml_adjustment']
        self.assertEqual((adj['free_thresh_from'], adj['free_thresh_to']), (0.25, 0.196))
        self.assertEqual(adj['original_yaml_sha256'], sha(y))
        self.assertNotEqual(adj['derived_yaml_sha256'], sha(y))
        mv = self.reg.get_map_version(b['map_version']['map_version_id'])
        self.assertAlmostEqual(mv['free_thresh'], 0.196)
        self.assertEqual(mv['verification_evidence']['artifacts_sha256']['map/map.yaml'], sha(y))      # original hash kept
        self.assertEqual(mv['verification_evidence']['yaml_adjustment']['reason'][:5], 'saved')
        self.assertTrue(mv['verification_evidence']['client_checksums_provided'])

    def test_derive_still_verifies_client_checksum_of_the_original(self):
        y = yml(free='0.25')
        before = self.n_maps()
        st, b = self.upload(yaml_b=y, threshold_policy='derive', expected_checksums=dict(yaml_sha256='0' * 64))
        self.assertIn(st, (400, 409, 422))
        self.assertEqual(b['error']['code'], 'checksum_mismatch')
        self.assertEqual(self.n_maps(), before)

    def test_negate_one_is_not_fixable_even_with_derive(self):
        st, b = self.upload(yaml_b=yml(free='0.25', negate=1), threshold_policy='derive')
        self.assertEqual(st, 422)
        self.assertFalse(b['error']['detail']['fixable_by_free_thresh'])

    def test_non_trinary_pgm_is_not_judged(self):
        st, b = self.upload(yaml_b=yml(free='0.25'), pgm_b=pgm(trinary=False))
        self.assertEqual(st, 201, b)
        self.assertIsNone(b['yaml_adjustment'])

    def test_safe_yaml_needs_no_policy(self):
        st, b = self.upload(yaml_b=yml(free='0.196'))
        self.assertEqual(st, 201, b)
        self.assertIsNone(b['yaml_adjustment'])

    def test_registering_never_activates_a_floor_map(self):
        st, b = self.upload()
        self.assertEqual(st, 201)
        self.assertIsNone(self.reg.one('SELECT * FROM floor_active_maps WHERE floor_id=?', ('fl-h1',)))


if __name__ == '__main__':
    unittest.main()
