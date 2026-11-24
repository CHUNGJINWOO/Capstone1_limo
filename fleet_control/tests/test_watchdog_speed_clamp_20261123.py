"""Watchdog speed clamp (G2, 2026-11-23): real rclpy MotionWatchdog node, callbacks driven directly (no graph traffic), reusing the R2 test scaffolding.
Not a real-robot result."""
import json, math, os, tempfile, threading, time, unittest
from types import SimpleNamespace
import test_motion_watchdog as base
from test_motion_watchdog import AUTHORIZED, LOCKED, FAILSAFE, Twist, String
from limo_robot_agent.speed_clamp import WatchdogSpeedClamp, SAFE_FALLBACK


def limit_msg(boot='b1', binding=('sb', 5, 3), scopes=None, seq=1):
    return String(data=json.dumps({'agent_boot_id': boot, 'binding': list(binding), 'seq': seq, 'scopes': scopes or {}}))


def scope(lin, ang, gen=1, rid=None):
    return {'max_linear_mps': lin, 'max_angular_rps': ang, 'generation': gen, 'request_id': rid or f'r{gen}'}


def twist(x, z=0.0):
    t = Twist(); t.linear.x, t.angular.z = x, z; return t


class ClampTests(base.WatchdogTests):
    def setUp(self):
        super().setUp()
        self.w.clamp = WatchdogSpeedClamp()
        self.in_pub, self.sl_pubs = 'velocity_smoother', ['fleet_robot_agent']
        self.w.input_nodes = self.w.input_nodes + ('manual_mapping_teleop',)       # the real launch's allow-list
        orig = self.w.get_publishers_info_by_topic
        def pubs(topic):
            if topic == '/fleet/speed_limit':
                return [SimpleNamespace(node_name=n, endpoint_gid=b'\x01' * 16) for n in self.sl_pubs]
            if topic == self.w.input_topic:
                return [SimpleNamespace(node_name=self.in_pub, endpoint_gid=b'\x02' * 16)]
            return orig(topic)
        self.w.get_publishers_info_by_topic = pubs
        self.w._update_graph(time.monotonic())
        self.w.out = SimpleNamespace(publish=lambda m: self.out.append((m.linear.x, m.angular.z)), topic_name='/cmd_vel_raw')

    def set_input(self, name):
        self.in_pub = name; self.w._update_graph(time.monotonic())

    def sync(self, scopes=None, **kw):
        self.w.on_speed_limit(limit_msg(scopes=scopes, **kw))

    def authorize(self):
        self.w.self_stall = 10.0
        self.status(); self.hb(); self.sync(); self.w.tick(); time.sleep(0.25); self.hb(); self.sync(); self.w.tick()
        self.assertEqual(self.w.state, AUTHORIZED, self.w.reason)

    def feed(self, x, z=0.0):
        self.out.clear(); self.w.on_cmd(twist(x, z)); return self.out[-1] if self.out else None


for _n in dir(base.WatchdogTests):
    if _n.startswith('test_'):
        setattr(ClampTests, _n, None)                    # the 66 R2 tests stay in test_motion_watchdog (clamp OFF); only the scaffolding is reused


class Clamp(ClampTests):
    def test_disabled_by_default_forwards_unchanged(self):
        self.w.clamp = None; base_w = self.w; self.status(); self.hb(); base_w.tick(); time.sleep(0.25); self.hb(); base_w.tick()
        self.assertEqual(base_w.state, AUTHORIZED); self.assertEqual(self.feed(0.5, 0.9), (0.5, 0.9))        # R2 behaviour: forwarded unchanged

    def test_static_ceilings_each_axis_both_directions_and_zero(self):
        self.authorize()
        self.assertEqual(self.feed(0.5, 0.9), (0.15, 0.5)); self.assertEqual(self.feed(-0.5, -0.9), (-0.10, -0.5))
        self.assertEqual(self.feed(0.05, -0.1), (0.05, -0.1)); self.assertEqual(self.feed(0.0, 0.0), (0.0, 0.0))
        self.assertEqual(self.feed(0.5, 0.0), (0.15, 0.0)); self.assertEqual(self.feed(0.0, 0.9), (0.0, 0.5))
        self.set_input('manual_mapping_teleop'); self.assertEqual(self.w.input_scope, 'manual'); self.assertEqual(self.feed(0.5, 0.9), (0.10, 0.3)); self.assertEqual(self.feed(-0.5, 0.0), (-0.10, 0.0))
        self.set_input('fleet_robot_agent'); self.assertIsNone(self.w.input_scope); self.assertEqual(self.feed(0.5, 0.9), (0.15, 0.5))                         # the Agent's own (Arbiter) output: static only

    def test_approved_limit_lowers_the_matching_scope_only(self):
        self.authorize(); self.sync({'auto': scope(0.08, 0.25)})
        self.assertEqual(self.feed(0.5, 0.9), (0.08, 0.25)); self.assertEqual(self.feed(-0.5, -0.9), (-0.08, -0.25))
        self.set_input('manual_mapping_teleop'); self.assertEqual(self.feed(0.5, 0.9), (0.10, 0.3))

    def test_zero_priority_in_every_non_authorized_state(self):
        self.authorize(); self.sync({'auto': scope(0.08, 0.25)})
        self.w.to(FAILSAFE, 'TEST'); self.out.clear(); self.w.on_cmd(twist(0.5, 0.5)); self.assertEqual(self.out, [])
        self.assertTrue(all(o == (0.0, 0.0) for o in self.out)); self.w.to(LOCKED, 'TEST'); self.w.on_cmd(twist(0.5, 0.5)); self.assertEqual([o for o in self.out if o != (0.0, 0.0)], [])
        self.w.request_shutdown(); self.w.on_cmd(twist(0.5, 0.5)); self.assertEqual([o for o in self.out if o != (0.0, 0.0)], [])

    def test_nonfinite_input_is_zero_not_clamped_to_a_value(self):
        self.authorize()
        for bad in (math.nan, math.inf, -math.inf):
            with self.subTest(bad):
                self.out.clear(); self.w.on_cmd(twist(bad, 0.0)); self.assertEqual(self.out, [(0.0, 0.0)])

    def test_not_synced_blocks_authorization_and_trips_when_sync_is_lost(self):
        self.status(); self.hb(); self.w.tick(); time.sleep(0.25); self.hb(); self.w.tick()
        self.assertNotEqual(self.w.state, AUTHORIZED); self.assertEqual(self.w.reason, 'SPEED_LIMIT_NOT_SYNCED')
        self.authorize(); self.w.clamp.sync_t = time.monotonic() - 2.0; self.hb(); self.w.tick()
        self.assertNotEqual(self.w.state, AUTHORIZED); self.assertEqual(self.w.last_fault, 'SPEED_LIMIT_NOT_SYNCED')
        self.out.clear(); self.w.on_cmd(twist(0.1)); self.assertEqual([o for o in self.out if o != (0.0, 0.0)], [])

    def test_sync_needs_the_allowed_single_publisher_and_the_current_agent_boot(self):
        self.authorize(); self.sl_pubs = ['fleet_robot_agent', 'rogue']; self.hb(); self.w.tick(); self.assertNotEqual(self.w.state, AUTHORIZED); self.assertEqual(self.w.last_fault, 'SPEED_LIMIT_NOT_SYNCED')
        self.sl_pubs = ['fleet_robot_agent']; self.w._update_graph(time.monotonic()); self.w.clamp.sync_t = None; self.assertFalse(self.w.clamp.synced('b1'))
        self.w.clamp.accept({'agent_boot_id': 'other', 'binding': ['sb', 5, 3], 'scopes': {}}, 'b1', True); self.assertFalse(self.w.clamp.synced('b1'))

    def test_acceptance_rules(self):
        c = WatchdogSpeedClamp(); acc = lambda scopes, b=('sb', 5, 3), boot='b1', pub=True: c.accept({'agent_boot_id': boot, 'binding': list(b), 'scopes': scopes}, 'b1', pub)
        def acc2(scopes, **kw):                     # a RAISE needs the same message twice, >= 0.15 s apart
            first = acc(scopes, **kw); time.sleep(0.16); return first, acc(scopes, **kw)
        self.assertEqual(acc({'auto': scope(0.2, 0.3)}), 0); self.assertEqual(acc({'auto': scope(0.1, 0.6)}), 0)
        for bad in (math.nan, math.inf, True, '0.1', 0, -0.1, 0.005):
            self.assertEqual(acc({'auto': scope(bad, 0.3)}), 0, bad)
        self.assertEqual(acc({'manual': scope(0.12, 0.3)}), 0)                               # manual ceiling is 0.10
        self.assertEqual(acc({'auto': scope(0.1, 0.3, gen=0)}), 0); self.assertEqual(acc({'auto': scope(0.1, 0.3)}, boot='x'), 0); self.assertEqual(acc({'auto': scope(0.1, 0.3)}, pub=False), 0)
        self.assertEqual(acc({'nope': scope(0.1, 0.3)}), 0); self.assertEqual(c.limits('auto'), (0.15, 0.10, 0.5))
        self.assertEqual(acc({'auto': scope(0.10, 0.3, gen=3)}), 1); self.assertEqual(c.limits('auto')[0], 0.10)
        self.assertEqual(acc({'auto': scope(0.14, 0.4, gen=3)}), 0, 'raise with the same generation and binding'); self.assertEqual(acc({'auto': scope(0.14, 0.4, gen=2)}), 0)
        self.assertEqual(acc({'auto': scope(0.06, 0.2, gen=2)}), 1, 'lowering is always accepted'); self.assertEqual(c.limits('auto')[0], 0.06)
        self.assertEqual(acc({'auto': scope(0.12, 0.4, gen=4)}), 0, 'a raise is not taken from one message'); self.assertEqual(c.limits('auto')[0], 0.06)
        self.assertEqual(acc2({'auto': scope(0.12, 0.4, gen=4)})[1], 1, 'raise with a higher generation, seen twice'); self.assertEqual(c.limits('auto')[0], 0.12)
        self.assertEqual(acc2({'auto': scope(0.15, 0.5, gen=1)}, b=('sb', 6, 3))[1], 1, 'a new binding (new lease) may be higher: it is a new validated request, seen twice')
        self.assertEqual(c.limits('auto')[0], 0.15)
        self.assertEqual(acc({'auto': scope(0.12, 0.4, gen=9, rid='z')}, b=('sb', 6, 3), pub=False), 0, 'no publisher, no change'); time.sleep(0.16)
        self.assertEqual(acc({'auto': scope(0.12, 0.4, gen=9, rid='z')}, b=('sb', 6, 3), pub=False), 0)

    def test_a_second_writer_appearing_between_messages_cannot_raise_a_limit(self):
        c = WatchdogSpeedClamp(); m = lambda g: {'agent_boot_id': 'b1', 'binding': ['sb', 5, 3], 'scopes': {'auto': scope(0.06, 0.2, gen=1)}}
        c.accept(m(1), 'b1', True)
        evil = {'agent_boot_id': 'b1', 'binding': ['sb', 5, 3], 'scopes': {'auto': scope(0.15, 0.5, gen=99, rid='evil')}}
        self.assertEqual(c.accept(evil, 'b1', True), 0)                      # first message while the graph still looked clean: only recorded
        time.sleep(0.16); self.assertEqual(c.accept(evil, 'b1', False), 0)    # the graph now shows a second writer: refused
        self.assertEqual(c.limits('auto')[0], 0.06)

    def test_a_raise_is_judged_against_the_graph_at_message_time(self):
        self.authorize(); self.sync({'auto': scope(0.06, 0.2, gen=1)}); self.assertEqual(self.feed(0.5), (0.06, 0.0))
        self.sl_pubs = ['fleet_robot_agent', 'rogue']                          # no tick has run since: the cached flag is still True
        self.sync({'auto': scope(0.15, 0.5, gen=99, rid='evil')}); time.sleep(0.16); self.sync({'auto': scope(0.15, 0.5, gen=99, rid='evil')})
        self.assertEqual(self.w.clamp.limits('auto')[0], 0.06); self.assertFalse(self.w.sl_pub_ok)

    def test_restart_restores_the_lower_limit_as_unconfirmed(self):
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, 'wd.json'); c = WatchdogSpeedClamp(state_path=p)
            c.accept({'agent_boot_id': 'b1', 'binding': ['sb', 5, 3], 'scopes': {'auto': scope(0.06, 0.2, gen=7)}}, 'b1', True)
            c2 = WatchdogSpeedClamp(state_path=p)                                              # Watchdog restart
            self.assertEqual(c2.limits('auto')[0], 0.06); self.assertTrue(c2.status('b1')['applied']['auto']['stale']); self.assertTrue(c2.status('b1')['restored_from_file_unconfirmed'])
            self.assertEqual(c2.accept({'agent_boot_id': 'b1', 'binding': ['sb', 5, 3], 'scopes': {'auto': scope(0.12, 0.4, gen=7, rid='x')}}, 'b1', True), 0, 'same generation cannot raise a restored limit')
            self.assertEqual(c2.limits('auto')[0], 0.06)
            m = {'agent_boot_id': 'b1', 'binding': ['sb', 5, 3], 'scopes': {'auto': scope(0.12, 0.4, gen=8)}}
            c2.accept(m, 'b1', True); time.sleep(0.16); self.assertEqual(c2.accept(m, 'b1', True), 1)
            self.assertEqual(c2.limits('auto')[0], 0.12)

    def test_unreadable_or_out_of_range_state_is_the_low_fallback(self):
        for content in ('garbage', json.dumps({'applied': {'auto': {'linear_mps': 3.0, 'angular_rps': 0.2}}}), json.dumps({'applied': {'auto': {'linear_mps': float('nan'), 'angular_rps': 0.2}}})):
            with self.subTest(content[:12]), tempfile.TemporaryDirectory() as d:
                p = os.path.join(d, 'wd.json'); open(p, 'w').write(content); c = WatchdogSpeedClamp(state_path=p)
                self.assertEqual(c.fault, 'PERSISTED_STATE_UNREADABLE'); self.assertEqual(c.limits('auto')[0], SAFE_FALLBACK['linear_mps']); self.assertEqual(c.limits('manual')[2], SAFE_FALLBACK['angular_rps'])

    def test_no_file_means_static_ceilings_and_a_stale_message_keeps_the_value(self):
        c = WatchdogSpeedClamp(state_path='/nonexistent/dir/x.json'); self.assertEqual(c.limits('auto'), (0.15, 0.10, 0.5))
        c.accept({'agent_boot_id': 'b1', 'binding': ['sb', 5, 3], 'scopes': {'auto': scope(0.06, 0.2)}}, 'b1', True)
        c.applied['auto']['seen_t'] = time.monotonic() - 100; self.assertTrue(c.status('b1')['applied']['auto']['stale']); self.assertEqual(c.limits('auto')[0], 0.06)     # stale, not released

    def test_status_block_reports_what_the_agent_needs_for_confirmation(self):
        self.authorize(); self.sync({'auto': scope(0.08, 0.25, gen=2, rid='abc')}); st = self.w.clamp.status('b1')
        self.assertTrue(st['enabled'] and st['synced']); a = st['applied']['auto']
        self.assertEqual((a['generation'], a['request_id'], a['linear_mps'], a['angular_rps'], a['stale']), (2, 'abc', 0.08, 0.25, False))
        self.assertEqual(st['effective']['auto']['forward_mps'], 0.08)

    def test_concurrent_limit_changes_and_commands_never_exceed_the_ceilings(self):
        self.authorize(); stop = threading.Event(); errors = []; seen = []
        self.w.out = SimpleNamespace(publish=lambda m: seen.append((m.linear.x, m.angular.z)), topic_name='/cmd_vel_raw')
        def setter():
            g = 1
            while not stop.is_set():
                g += 1; self.sync({'auto': scope(0.03 + (g % 5) * 0.02, 0.1 + (g % 4) * 0.1, gen=g)}); time.sleep(0.001)
        def feeder():
            try:
                while not stop.is_set(): self.w.on_cmd(twist(0.4, 0.9)); self.w.on_cmd(twist(-0.4, -0.9))
            except Exception as e: errors.append(e)
        ts = [threading.Thread(target=setter), threading.Thread(target=feeder), threading.Thread(target=feeder)]
        [t.start() for t in ts]; time.sleep(1.0); stop.set(); [t.join() for t in ts]
        self.assertEqual(errors, []); self.assertTrue(len(seen) > 100)
        self.assertTrue(all(-0.10 <= v <= 0.15 and abs(w) <= 0.5 for v, w in seen))


if __name__ == '__main__':
    unittest.main()
