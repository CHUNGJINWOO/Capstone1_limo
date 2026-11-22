"""Area level (operator-declared sub-floor) of the Map Registry and the C103 registration script."""
import os
import sqlite3
import subprocess
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..'))
sys.path.insert(0, os.path.join(ROOT, 'server'))

from fleet_server.map_registry import ConflictError, MapRegistry, ValidationError  # noqa: E402

YAML = b'image: map.pgm\nresolution: 0.05\norigin: [-1.0, -2.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n'


def pgm(w=8, h=6):
    body = bytes([254] * (w * h // 2) + [205] * (w * h // 2 - 2) + [0, 0])
    return b'P5\n%d %d\n255\n' % (w, h) + body


class AreaTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.reg = MapRegistry(os.path.join(self.tmp.name, 'r.db'), os.path.join(self.tmp.name, 'store'))
        self.reg.create_facility('fac', 'Fac')
        self.reg.create_building('b', 'fac', 'B')
        self.reg.create_floor('f1', 'b', '1F')
        self.reg.create_floor('f2', 'b', '2F')

    def tearDown(self):
        self.reg.close()
        self.tmp.cleanup()

    def _map(self, floor='f1', label='m1'):
        return self.reg.register_map_bundle(floor_id=floor, label=label, yaml_bytes=YAML, pgm_bytes=pgm())['map_version']['map_version_id']

    def test_area_create_idempotent_and_conflict(self):
        a = self.reg.create_area('lounge', 'f1', 'Lounge')
        self.assertEqual(a['coverage'], 'PARTIAL_FLOOR')
        self.assertEqual(self.reg.create_area('lounge', 'f1', 'Lounge')['created_at'], a['created_at'])
        with self.assertRaises(ConflictError):
            self.reg.create_area('lounge', 'f1', 'Other name')
        with self.assertRaises(ValidationError):
            self.reg.create_area('x', 'f1', 'X', coverage='WHOLE_BUILDING')

    def test_link_is_write_once_and_same_floor(self):
        self.reg.create_area('lounge', 'f1', 'Lounge')
        self.reg.create_area('hall', 'f1', 'Hall')
        self.reg.create_area('up', 'f2', 'Up')
        mv = self._map()
        self.reg.link_map_version_area(mv, 'lounge')
        self.reg.link_map_version_area(mv, 'lounge')  # idempotent
        with self.assertRaises(ConflictError):
            self.reg.link_map_version_area(mv, 'hall')
        with self.assertRaises(ConflictError):
            self.reg.link_map_version_area(mv, 'up')
        with self.assertRaises(sqlite3.DatabaseError):
            self.reg.x('DELETE FROM map_version_areas WHERE map_version_id=?', (mv,))

    def test_tree_and_hydrate_expose_area(self):
        self.reg.create_area('lounge', 'f1', 'Lounge')
        mv = self._map()
        self.reg.link_map_version_area(mv, 'lounge')
        floor = self.reg.list_facilities_tree()[0]['buildings'][0]['floors'][0]
        self.assertEqual(floor['areas'][0]['area_id'], 'lounge')
        self.assertEqual(floor['areas'][0]['map_version_ids'], [mv])
        self.assertEqual(self.reg.get_map_version(mv)['area']['area_id'], 'lounge')
        self.assertEqual(self.reg.list_map_versions(floor_id='f1')[0]['area']['name'], 'Lounge')
        unlinked = self._map(label='m2')
        self.assertIsNone(self.reg.get_map_version(unlinked)['area'])

    def test_existing_db_gets_area_tables(self):
        path = os.path.join(self.tmp.name, 'r.db')
        self.reg.close()
        reg2 = MapRegistry(path, os.path.join(self.tmp.name, 'store'))
        try:
            self.assertEqual(reg2.list_facilities_tree()[0]['buildings'][0]['floors'][0]['areas'], [])
        finally:
            reg2.close()
        self.reg = MapRegistry(path, os.path.join(self.tmp.name, 'store'))


class C103ScriptTests(unittest.TestCase):
    RUN = os.path.abspath(os.path.join(ROOT, '..', 'local_data', 'raw_results', 'limo_runs', '20261119_auto_map_depth'))

    @unittest.skipUnless(os.path.isdir(os.path.join(RUN, 'map')), 'C103 run artifacts not present')
    def test_registration_is_idempotent_and_preserves_unknown(self):
        script = os.path.join(ROOT, 'scripts', 'register_c103_map.py')
        with tempfile.TemporaryDirectory() as d:
            out1 = subprocess.run([sys.executable, script, '--data-dir', d], capture_output=True, text=True, check=True).stdout
            out2 = subprocess.run([sys.executable, script, '--data-dir', d], capture_output=True, text=True, check=True).stdout
            self.assertIn('"created": true', out1)
            self.assertIn('"created": false', out2)
            self.assertIn('"unknown": 87564', out1)
            reg = MapRegistry(os.path.join(d, 'map_registry.db'), os.path.join(d, 'map_store'))
            try:
                mv = reg.list_map_versions(floor_id='paichai_c_1f')[0]
                self.assertEqual(mv['verification_status'], 'UNVERIFIED')
                self.assertEqual(mv['area']['area_id'], 'paichai_c_1f_c103_lounge')
                self.assertFalse(mv['verification_evidence']['sensors']['slam_raster']['depth'])
                self.assertTrue(mv['sha256']['posegraph_sha256'])
            finally:
                reg.close()


if __name__ == '__main__':
    unittest.main()
