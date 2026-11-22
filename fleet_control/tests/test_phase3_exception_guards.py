"""Mock-only exceptions must not reach real / unlocked paths; locked-mock recovery needs Nav2 terminal evidence.

* STALL_PENDING continuation (safety.py `locked_mock_proof`): only Agent mock + simulated + locked arbiter AND Gate motor
  path LOCKED; refused for real agents, SIM_GATED/ARMED motor paths and mock agents without the locked arbiter.
* `status_continuous` replaces message age, but a real agent still gets no Safety Context without Gate attestation.
* Locked-mock recovery (ownership.py): refused without terminal evidence or with unmet idle checks; accepted with the
  Nav2 action result or the goal's terminal state in the status array; real agents stay refused.
* VelocityArbiter refuses non-mock robots; agent_node refuses manual_locked_arbiter outside isolated mock.
"""
import copy, json, os, subprocess, sys, time, unittest
from pathlib import Path
from types import SimpleNamespace
from test_r3_r4_server_safety import Tests as Fixture
from test_gate_r7_evidence import Tests as Evidence

FC = Path(__file__).resolve().parents[1]
ADMIN = SimpleNamespace(name='admin', role='admin')


class Base(unittest.TestCase):
    def setUp(self):
        self.b = Fixture(); self.b.setUp(); self.f = self.b.f; self.db = self.b.db
        self.f.robots['r']['agent'].update(mode='mock', simulated=True, locked_velocity_arbiter=True)
        m, _ = self.b.create(); cmd = self.f._deliver('r')['commands'][0]; self.cmd, self.mid = cmd, m['mission_id']
        self.f.ack_command('r', cmd['command_id'], 'ACCEPTED', agent_boot_id='boot1')
        self.uuid = 'a' * 32
        e = Evidence(); e.setUp(); e.m.update(boot_id='gate1', gate_boot_id='gate1', fault_epoch=1)
        gate = e.evaluate()
        gate.update(status='STALL_PENDING', movement_permitted=False, movement_block_reasons=['GATE_NOT_NORMAL:STALL_PENDING'],
                    motor_path={'status': 'LOCKED'})
        nav = {'source': 'navigate_to_pose/_action/status', 'known': True, 'publishers': 1, 'active_count': 1,
               'received_status_array': True, 'age_s': 30.0, 'status_continuous': True,
               'active_goal_uuids': [self.uuid], 'foreign_active_goal_uuids': []}
        self.s = copy.deepcopy(self.b.state); self.s['safety_gate'] = gate
        self.s['execution'] = dict(active=dict(command_id=cmd['command_id'], mission_id=m['mission_id'], goal_uuid=self.uuid,
                                               goal_handle_available=True), nav2_goals=nav)
        self.f.robots['r']['state'] = self.s; self.f.robots['r']['state_ts'] = time.time()
        with self.f.lock, self.db.transaction():
            self.f.check_execution('r', self.s['execution'])

    def tearDown(self):
        self.b.tearDown()


class StallPendingContinuation(Base):
    def test_locked_mock_continues_only_while_running(self):
        self.assertIsNone(self.f.safety_context('r'))
        self.assertIsNotNone(self.f.safety_context('r', active_continuation=True))

    def test_not_for_real_unlocked_or_plain_mock(self):
        cases = [('agent', {'mode': 'real', 'simulated': False}), ('agent', {'locked_velocity_arbiter': False}),
                 ('gate', {'motor_path': {'status': 'SIM_GATED'}}), ('gate', {'motor_path': {'status': 'ARMED'}}),
                 ('gate', {'motor_path': None})]
        for where, change in cases:
            with self.subTest(change=change):
                agent = dict(self.f.robots['r']['agent']); gate = dict(self.s['safety_gate'])
                (self.f.robots['r']['agent'] if where == 'agent' else self.s['safety_gate']).update(change)
                self.assertIsNone(self.f.safety_context('r', active_continuation=True))
                self.f.robots['r']['agent'] = agent; self.s['safety_gate'] = gate

    def test_status_continuous_does_not_replace_gate_attestation_for_real(self):
        self.s['safety_gate'].update(status='NORMAL', movement_permitted=False, movement_block_reasons=['GATE_ATTESTATION_UNAVAILABLE'])
        self.f.robots['r']['agent'].update(mode='real', simulated=False, locked_velocity_arbiter=False)
        self.assertIsNone(self.f.safety_context('r'))
        self.assertIsNone(self.f.safety_context('r', active_continuation=True))


class LockedMockRecovery(Base):
    def make_idle(self, **over):
        st = self.s
        st['safety_gate'].update(status='NORMAL', cmd_blocked=False, motor_path={'status': 'LOCKED'})
        st['execution'] = {'active': None, 'nav2_goals': {'known': True, 'publishers': 1, 'active_count': 0, 'foreign_active_goal_uuids': [],
                                                          'status_continuous': True},
                           'nav_idle': {'ok': True}, 'journal_terminals': [], 'nav_terminal_observed': {}}
        st['motion_watchdog'] = {'required': True, 'state': 'MOTION_AUTHORIZED', 'agent_boot_matches': True}
        st['velocity_arbiter'] = {'selected_linear': 0.0, 'selected_turn': 0.0}
        st['mapping_worker'] = {'enabled': True, 'status': 'IDLE', 'processes_started': False}
        for k, v in over.items():
            st[k] = v
        self.f.robots['r']['state_ts'] = time.time()
        self.db.x("UPDATE missions SET status='RECONCILIATION_REQUIRED' WHERE mission_id=?", (self.mid,))
        self.db.x("INSERT OR REPLACE INTO meta(k,v) VALUES(?,?)", ('reconciliation_block:r', 'test'))
        self.f._refresh_block('r')

    def test_refused_without_terminal_evidence(self):
        self.make_idle()
        ok, why, ev = self.f.recovery_evidence('r')
        self.assertEqual((ok, why), (False, 'terminal_evidence_missing'))
        with self.assertRaises(Exception):
            self.f.recover_robot_block('r', ADMIN, 'checked on site, nothing moves')

    def test_refused_when_watchdog_or_arbiter_not_idle(self):
        for over in ({'motion_watchdog': {'required': True, 'state': 'FAILSAFE', 'agent_boot_matches': True}},
                     {'velocity_arbiter': {'selected_linear': 0.05, 'selected_turn': 0.0}},
                     {'mapping_worker': {'enabled': True, 'status': 'UNKNOWN', 'processes_started': True}}):
            with self.subTest(over=list(over)):
                self.make_idle(**over)
                self.s['execution']['nav_terminal_observed'] = {self.uuid: 'CANCELED'}
                self.assertEqual(self.f.recovery_evidence('r')[1], 'idle_not_proven')

    def test_status_array_terminal_recovers_with_audit(self):
        self.make_idle()
        self.s['execution']['nav_terminal_observed'] = {self.uuid: 'CANCELED'}
        ok, why, ev = self.f.recovery_evidence('r')
        self.assertTrue(ok, (why, ev))
        self.f.recover_robot_block('r', ADMIN, 'Nav2 reported the goal canceled; robot stopped')
        self.assertEqual(self.db.one('SELECT status FROM missions WHERE mission_id=?', (self.mid,))['status'], 'CANCELED')
        self.assertFalse(self.f.robots['r']['reconciliation_required'])
        kinds = [e['kind'] for e in self.f.events(limit=10)]
        self.assertIn('robot_block_recovered', kinds)

    def test_action_result_journal_recovers_mission(self):
        self.make_idle()
        self.s['execution']['journal_terminals'] = [{'command_id': self.cmd['command_id'], 'goal_uuid': self.uuid,
                                                     'terminal_source': 'nav2_action_result', 'terminal_status': 'SUCCEEDED'}]
        self.f.recover_mission(self.mid, ADMIN, 'Nav2 action result SUCCEEDED; verified stopped')
        self.assertEqual(self.db.one('SELECT status FROM missions WHERE mission_id=?', (self.mid,))['status'], 'SUCCEEDED')

    def test_journal_uuid_mismatch_refused(self):
        self.make_idle()
        self.s['execution']['journal_terminals'] = [{'command_id': self.cmd['command_id'], 'goal_uuid': 'b' * 32,
                                                     'terminal_source': 'nav2_action_result', 'terminal_status': 'SUCCEEDED'}]
        self.assertEqual(self.f.recovery_evidence('r')[1], 'terminal_evidence_missing')

    def test_real_agent_still_refused(self):
        self.make_idle()
        self.f.robots['r']['agent'].update(mode='real', simulated=False, locked_velocity_arbiter=False)
        self.s['execution']['nav_terminal_observed'] = {self.uuid: 'CANCELED'}
        ok, why, _ = self.f.recovery_evidence('r')
        self.assertFalse(ok)


class ConstructionGuards(unittest.TestCase):
    def test_arbiter_refuses_non_mock(self):
        sys.path.insert(0, str(FC / 'agent' / 'limo_robot_agent'))
        try:
            from limo_robot_agent.velocity_arbiter import VelocityArbiter
        except Exception as exc:
            self.skipTest(f'ROS python unavailable: {exc}')
        for mode, sim in (('real', False), ('sim', True), ('mock', False)):
            with self.assertRaises(ValueError):
                VelocityArbiter(SimpleNamespace(mode=mode, simulated=sim), None, None)

    def test_agent_node_refuses_locked_arbiter_outside_isolated_mock(self):
        env = dict(os.environ, PYTHONPATH=str(FC / 'agent' / 'limo_robot_agent') + os.pathsep + os.environ.get('PYTHONPATH', ''),
                   ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID='229')
        env.pop('ROS_DISCOVERY_SERVER', None)
        for args in (['mode:=real', 'simulated:=false'], ['mode:=sim', 'simulated:=true'],
                     ['mode:=mock', 'simulated:=true', 'require_motion_watchdog:=false'],
                     ['mode:=mock', 'simulated:=true', 'server_url:=http://127.0.0.1:18190']):
            with self.subTest(args=args):
                cmd = [sys.executable, '-m', 'limo_robot_agent.agent_node', '--ros-args', '-p', 'manual_web_bench:=true',
                       '-p', 'manual_locked_arbiter:=true', '-p', 'require_motion_watchdog:=true',
                       '-p', 'server_url:=http://127.0.0.1:28000'] + [x for a in args for x in ('-p', a)]
                try:
                    p = subprocess.run(cmd, env=env, capture_output=True, text=True, timeout=30)
                except FileNotFoundError as exc:
                    self.skipTest(str(exc))
                if 'No module named' in p.stderr:
                    self.skipTest('ROS python unavailable')
                self.assertNotEqual(p.returncode, 0)
                self.assertIn('manual_locked_arbiter requires isolated mock', p.stderr + p.stdout)


if __name__ == '__main__':
    unittest.main(verbosity=1)
