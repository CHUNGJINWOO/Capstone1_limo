import importlib.util
import json
from pathlib import Path
import sqlite3
import tempfile
import unittest
from unittest.mock import patch

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'verify_bag_copy.py'
spec = importlib.util.spec_from_file_location('verify_bag_copy', SCRIPT)
copy = importlib.util.module_from_spec(spec)
spec.loader.exec_module(copy)


class CopyTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='bag-copy-test-')
        self.root = Path(self.tmp.name)
        self.run = self.root / 'run'
        self.bag = self.run / 'bag'
        self.bag.mkdir(parents=True)
        self.dest = self.root / 'destination'
        self.dest.mkdir()
        (self.run / 'exit.json').write_text(json.dumps({'reason': 'operator_signal_SIGINT', 'recorder_returncode': 0}))
        (self.bag / 'metadata.yaml').write_text('synthetic metadata')
        with sqlite3.connect(self.bag / 'bag_0.db3') as db:
            db.execute('create table sample (value text)')
            db.execute("insert into sample values ('synthetic')")
        self.ros = patch.object(copy.subprocess, 'run', return_value=type('Result', (), {'returncode': 0})())
        self.ros.start()
        self.addCleanup(self.ros.stop)
        self.addCleanup(self.tmp.cleanup)

    def test_copy_repeat_and_preserve_source(self):
        before = {p.name: copy.digest(p) for p in self.bag.iterdir()}
        result = copy.copy_verified(self.run, self.dest, 's1')
        self.assertEqual(result, copy.copy_verified(self.run, self.dest, 's1'))
        self.assertEqual(before, {p.name: copy.digest(p) for p in self.bag.iterdir()})
        self.assertEqual(before, {x['name']: x['sha256'] for x in result['files']})

    def test_partial_prefix_resumes(self):
        target = self.dest / 's1'; target.mkdir()
        (target / '.copy-session.json').write_text(json.dumps({'session_id': 's1', 'source': str(self.run.resolve())}))
        source = (self.bag / 'bag_0.db3').read_bytes()
        (target / 'bag_0.db3').write_bytes(source[:100])
        copy.copy_verified(self.run, self.dest, 's1')
        self.assertEqual((target / 'bag_0.db3').read_bytes(), source)

    def test_incorrect_partial_refused(self):
        target = self.dest / 's1'; target.mkdir()
        (target / '.copy-session.json').write_text(json.dumps({'session_id': 's1', 'source': str(self.run.resolve())}))
        (target / 'bag_0.db3').write_bytes(b'bad')
        with self.assertRaisesRegex(ValueError, 'prefix conflict'):
            copy.copy_verified(self.run, self.dest, 's1')
        self.assertEqual((target / 'bag_0.db3').read_bytes(), b'bad')

    def test_failed_exit_rejected(self):
        (self.run / 'exit.json').write_text(json.dumps({'reason': 'recorder_failed', 'recorder_returncode': 1}))
        with self.assertRaisesRegex(ValueError, 'exit evidence'):
            copy.copy_verified(self.run, self.dest, 's1')

    def test_corrupt_database_rejected(self):
        (self.bag / 'bag_0.db3').write_bytes(b'not sqlite')
        with self.assertRaises(sqlite3.DatabaseError):
            copy.copy_verified(self.run, self.dest, 's1')

    def test_foreign_destination_and_bad_id_rejected(self):
        (self.dest / 's1').mkdir()
        with self.assertRaisesRegex(ValueError, 'not this copy session'):
            copy.copy_verified(self.run, self.dest, 's1')
        with self.assertRaisesRegex(ValueError, 'invalid session'):
            copy.copy_verified(self.run, self.dest, '../oops')

    def test_symlink_rejected(self):
        (self.bag / 'linked.db3').symlink_to(self.bag / 'bag_0.db3')
        with self.assertRaisesRegex(ValueError, 'regular files'):
            copy.copy_verified(self.run, self.dest, 's1')

    def test_ros_info_failure_rejected(self):
        with patch.object(copy.subprocess, 'run', return_value=type('Result', (), {'returncode': 1})()):
            with self.assertRaisesRegex(ValueError, 'bag info failed'):
                copy.copy_verified(self.run, self.dest, 's1')


if __name__ == '__main__':
    unittest.main()
