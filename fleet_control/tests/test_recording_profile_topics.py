"""The fixed recorder profile must carry /scan_slam and the separated RMF BuildingMap topic, and must never name /map twice-typed."""
import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

MODULE = Path(__file__).resolve().parents[1] / 'agent/limo_robot_agent/limo_robot_agent/recording_session.py'
spec = importlib.util.spec_from_file_location('profile_recording_session', MODULE)
m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)


class Tests(unittest.TestCase):
    def test_profile_extra_topics(self):
        with tempfile.TemporaryDirectory(prefix='profile-topics-') as t:
            root = Path(t); (root / 'store').mkdir()
            rec = root / 'r.py'; rec.write_text('pass'); ver = root / 'v.py'; ver.write_text('pass')
            s = m.RecordingSession(root / 'store', 'session_1', rec, ver)
            with mock.patch.object(m.subprocess, 'Popen') as popen:
                s.start()
            cmd = popen.call_args[0][0]
            extras = [cmd[i + 1] for i, a in enumerate(cmd) if a == '--extra']
            self.assertEqual(extras, ['/scan_slam', '/rmf/building_map'])
            self.assertNotIn('/map', extras)       # /map (OccupancyGrid) comes from the recorder's CORE_TOPICS, not from here
            s.close()

    def test_building_map_topic_is_not_map(self):
        self.assertEqual(m.RMF_BUILDING_MAP_TOPIC, '/rmf/building_map')


if __name__ == '__main__':
    unittest.main()
