"""Characterization (Phase 3.5, 2026-11-21): the 'locked velocity graph' contract is implemented three times. This test pins what each
implementation CURRENTLY returns for the same graphs, so a refactor cannot change it silently. A difference between columns is not
asserted to be right or wrong here; see fleet_control/docs/REFACTOR_PHASE35_20261121.md (R3).

  VelocityArbiter.graph_locked      locked-arbiter chain: Arbiter -> /fleet/nav_cmd_vel -> Watchdog -> /cmd_vel_raw -> Gate
  ManualWebBench._graph_blocker     LEGACY direct mode only (isolated_arbiter False): the bench itself writes /cmd_vel_raw
  GateResetWorker._graph_locked     fallback used only when the Agent has no Arbiter (not used by any current configuration)

2026-11-21 R3 behaviour change (separate step, documented): the GateResetWorker fallback now refuses every graph (False). Before, it
returned True for rogue_cmd_vel_raw, no_watchdog_writer, foreign_nav_cmd_vel, no_smoother, foreign_smoother, canonical and
legacy_bench_writer (original expectations: local_data/raw_results/refactor35_claude_20261121/pre_change_originals/).
"""
import sys
import types
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'agent' / 'limo_robot_agent'))
from limo_robot_agent import velocity_arbiter as va, manual_web_bench as mwb, gate_reset_worker as grw  # noqa: E402

E = lambda n, ns='/': types.SimpleNamespace(node_name=n, node_namespace=ns, endpoint_gid=[1] * 24)
AGENT = 'fleet_robot_agent'
BASE = {('pub', '/cmd_vel_locked'): [E('stall_detector_node')], ('pub', '/stall_detector/status_json'): [E('stall_detector_node')],
        ('sub', '/cmd_vel_raw'): [E('stall_detector_node')], ('pub', '/cmd_vel_raw'): [E('fleet_motion_watchdog')],
        ('sub', '/fleet/nav_cmd_vel'): [E('fleet_motion_watchdog')], ('pub', '/fleet/nav_cmd_vel'): [E(AGENT)],
        ('pub', '/fleet/cmd_vel_nav_smoothed'): [E('velocity_smoother')]}
#           case                  arbiter  bench (legacy)               reset-worker fallback
EXPECTED = {'canonical':          (True,  'velocity_source_conflict', False),
            'cmd_vel_publisher':  (False, 'motor_topic_has_publisher', False),
            'cmd_vel_subscriber': (False, 'velocity_source_conflict', False),
            'rogue_cmd_vel_raw':  (False, 'velocity_source_conflict', False),
            'no_watchdog_writer': (False, None, False),
            'second_gate_status': (False, 'gate_graph_unconfirmed', False),
            'foreign_nav_cmd_vel': (False, 'velocity_source_conflict', False),
            'no_smoother':        (True,  'velocity_source_conflict', False),
            'foreign_smoother':   (False, 'velocity_source_conflict', False),
            'legacy_bench_writer': (False, None, False)}
CASES = {'canonical': {}, 'cmd_vel_publisher': {('pub', '/cmd_vel'): [E('rogue')]}, 'cmd_vel_subscriber': {('sub', '/cmd_vel'): [E('bridge')]},
         'rogue_cmd_vel_raw': {('pub', '/cmd_vel_raw'): [E('fleet_motion_watchdog'), E('rogue')]}, 'no_watchdog_writer': {('pub', '/cmd_vel_raw'): []},
         'second_gate_status': {('pub', '/stall_detector/status_json'): [E('stall_detector_node'), E('x')]},
         'foreign_nav_cmd_vel': {('pub', '/fleet/nav_cmd_vel'): [E(AGENT), E('teleop')]}, 'no_smoother': {('pub', '/fleet/cmd_vel_nav_smoothed'): []},
         'foreign_smoother': {('pub', '/fleet/cmd_vel_nav_smoothed'): [E('teleop')]},
         'legacy_bench_writer': {('pub', '/cmd_vel_raw'): [E(AGENT)]}}        # legacy mode: the bench is the single raw writer


def robot(g):
    r = types.SimpleNamespace()
    r.get_publishers_info_by_topic = lambda t: list(g.get(('pub', t), []))
    r.get_subscriptions_info_by_topic = lambda t: list(g.get(('sub', t), []))
    r.count_publishers = lambda t: len(g.get(('pub', t), []))
    r.get_name = lambda: AGENT; r.get_namespace = lambda: '/'
    return r


class LockedGraphContract(unittest.TestCase):
    def test_current_verdicts(self):
        for name, delta in CASES.items():
            g = dict(BASE); g.update(delta); r = robot(g)
            a = va.VelocityArbiter.__new__(va.VelocityArbiter); a.robot = r
            b = mwb.ManualWebBench.__new__(mwb.ManualWebBench); b.robot = r
            b.publisher = object() if name == 'legacy_bench_writer' else None    # legacy mode: the bench owns its raw publisher
            w = grw.GateResetWorker.__new__(grw.GateResetWorker); w.robot = r
            with self.subTest(case=name):
                self.assertEqual((a.graph_locked(), b._graph_blocker(), w._graph_locked()), EXPECTED[name])


if __name__ == '__main__':
    unittest.main(verbosity=2)
