import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'agent/limo_robot_agent'))
from limo_robot_agent.bag_worker import BagWorker


class BagWorkerTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.worker = BagWorker.__new__(BagWorker)
        self.worker.root = Path(self.temp.name)
        self.worker.path = self.worker.root / 'journal.json'
        self.worker.status = 'IDLE'
        self.worker.stop = threading.Event()
        self.worker.journal = {}
        self.worker.agent = SimpleNamespace(robot_id='r1', connected=True, boot_id='b1')
        self.calls = []
        self.worker.request = self.request
        self.worker.bag = SimpleNamespace(identity={'pid': 100}, process=SimpleNamespace(poll=lambda: None),
                                           start=lambda *args: None, renew=lambda: None, close=lambda: None,
                                           stop=lambda *args: True)
        self.worker.robot = SimpleNamespace(get_subscriptions_info_by_topic=lambda _: [
            SimpleNamespace(node_name='fleet_bag_session1', node_namespace='/')])

    def request(self, action, **kwargs):
        self.calls.append((action, kwargs))
        return {'accepted': True}

    def command(self, **change):
        return dict(dict(robot_id='r1', kind='bag', action='START', epoch=1,
                         session_id='session1', command_id='command1',
                         expires_at=time.time()+10), **change)

    def test_start_journal_prevents_duplicate(self):
        self.worker.execute(self.command())
        self.assertEqual(self.worker.status, 'RECORDING')
        self.worker.execute(self.command())
        self.assertEqual([a for a, _ in self.calls].count('accept'), 1)
        self.assertTrue(self.worker.path.is_file())

    def test_wrong_robot_epoch_session_fenced(self):
        self.worker.execute(self.command(robot_id='r2'))
        self.assertEqual(self.calls, [])
        self.worker.journal.update(active_session='session1', epoch=1)
        self.worker.execute(self.command(action='STOP', session_id='other'))
        self.worker.execute(self.command(action='STOP', epoch=2))
        self.assertEqual(self.calls, [])

    def test_uncertain_restart_does_not_execute(self):
        self.worker.status = 'UNKNOWN'
        self.worker.execute(self.command())
        self.assertEqual(self.calls, [])

    def test_failed_stop_remains_unknown(self):
        self.worker.journal.update(active_session='session1', epoch=1)
        self.worker.bag.stop = lambda *_: False
        self.worker.execute(self.command(action='STOP'))
        self.assertEqual(self.worker.status, 'UNKNOWN')
        self.assertEqual(self.calls[-1][1]['outcome'], 'UNKNOWN')


if __name__ == '__main__':
    unittest.main()
