"""Agent-side speed limits: limiter rules and the Arbiter's final clamp (G2, 2026-11-23). Pure Python (stub ROS), no robot.
The Arbiter test runs the real VelocityArbiter.update() with its source-selection blockers stubbed to 'allowed' (they are covered by
test_velocity_arbiter_* and the stack probe) and checks what leaves the Arbiter toward the Watchdog."""
import json, math, os, sys, tempfile, time, unittest
from pathlib import Path
from types import SimpleNamespace
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'agent' / 'limo_robot_agent'), str(ROOT / 'tests')]
import test_agent_safety_v1 as _stub      # installs the stub rclpy / geometry_msgs used by the other Agent unit tests
from limo_robot_agent import speed_limits as sl
from limo_robot_agent.speed_limits import SpeedLimiter


def session(epoch=5, boot='sb1', agent_boot='ab1', contract_gen=3, expires=None, req=None, connected=True):
    return {'connected': connected, 'reconciliation_required': False, 'server_boot_id': boot, 'agent_boot_id': agent_boot,
            'lease': {'epoch': epoch, 'expires_at': (expires or time.time() + 60)}, 'protocol_version': 2, 'safety_contract_version': 1,
            'contract_generation': contract_gen, 'speed_limit_request': req}


def request(gen=1, scope='auto', lin=0.12, ang=0.35, epoch=5, boot='sb1', agent_boot='ab1', rid=None):
    return {'generation': gen, 'request_id': rid or f'r{gen}', 'scope': scope, 'max_linear_mps': lin, 'max_angular_rps': ang,
            'lease_epoch': epoch, 'server_boot_id': boot, 'agent_boot_id': agent_boot}


class Limiter(unittest.TestCase):
    def new(self, path=None):
        l = SpeedLimiter(path); l.enforced_by = 'ARBITER'; return l

    def apply(self, l, sess_epoch=None, **kw):
        s = session(epoch=sess_epoch or kw.get('epoch', 5), req=request(**kw)); l.process(s); return l.last_result

    def test_default_is_the_existing_ceilings(self):
        l = self.new()
        self.assertEqual(l.limits('manual'), (0.10, 0.0, 0.30)); self.assertEqual(l.limits('auto'), (0.15, 0.10, 0.50))

    def test_apply_lowers_only_the_requested_scope(self):
        l = self.new(); r = self.apply(l, lin=0.08, ang=0.2)
        self.assertEqual(r['status'], 'APPLIED'); self.assertEqual(l.limits('auto'), (0.08, 0.08, 0.2)); self.assertEqual(l.limits('manual'), (0.10, 0.0, 0.30))

    def test_clamp_each_axis_both_directions_and_preserves_zero(self):
        l = self.new(); self.apply(l, lin=0.08, ang=0.2)
        self.assertEqual(l.clamp('auto', 0.15, 0.5), (0.08, 0.2)); self.assertEqual(l.clamp('auto', -0.10, -0.5), (-0.08, -0.2))
        self.assertEqual(l.clamp('auto', 0.05, -0.1), (0.05, -0.1))            # inside the limit: unchanged
        self.assertEqual(l.clamp('auto', 0.0, 0.0), (0.0, 0.0))               # STOP / zero is never changed
        self.assertEqual(l.clamp('auto', 0.15, 0.0), (0.08, 0.0)); self.assertEqual(l.clamp('auto', 0.0, 0.5), (0.0, 0.2))
        self.assertEqual(l.clamp('manual', -0.05, 0.0), (0.0, 0.0))            # manual stays forward-only
        for bad in (math.nan, math.inf, -math.inf, True, None, '0.1'):
            self.assertEqual(l.clamp('auto', bad, 0.1), (0.0, 0.0)); self.assertEqual(l.clamp('auto', 0.1, bad), (0.0, 0.0))
        self.assertEqual(l.clamp('nope', 0.1, 0.1), (0.0, 0.0))

    def test_reverse_keeps_its_own_lower_limit(self):
        l = self.new(); self.assertEqual(l.clamp('auto', -0.5, 0.0), (-0.10, 0.0))      # existing -0.10 reverse limit, not the 0.15 forward limit
        self.apply(l, lin=0.05, ang=0.2); self.assertEqual(l.clamp('auto', -0.5, 0.0), (-0.05, 0.0))

    def test_rejections_leave_the_setting_unchanged(self):
        cases = {'exceeds': dict(lin=0.2), 'exceeds_ang': dict(ang=0.6), 'zero': dict(lin=0.0), 'negative': dict(lin=-0.05), 'nan': dict(lin=math.nan),
                 'inf': dict(ang=math.inf), 'bool': dict(lin=True), 'str': dict(lin='0.1'), 'below_min': dict(lin=0.005), 'scope': dict(scope='x'),
                 'gen0': dict(gen=0), 'gen_str': dict(gen='1'), 'epoch': dict(epoch=6, sess_epoch=5), 'boot': dict(boot='other'), 'agent_boot': dict(agent_boot='x')}
        for name, kw in cases.items():
            with self.subTest(name):
                l = self.new(); r = self.apply(l, **kw)
                self.assertEqual(r['status'], 'REJECTED', r); self.assertEqual(l.limits('auto'), (0.15, 0.10, 0.50))

    def test_manual_ceiling_is_enforced_separately(self):
        l = self.new(); self.assertEqual(self.apply(l, scope='manual', lin=0.12)['reason'], 'EXCEEDS_CEILING')
        self.assertEqual(self.apply(l, scope='manual', gen=2, lin=0.06, ang=0.2)['status'], 'APPLIED'); self.assertEqual(l.limits('manual'), (0.06, 0.0, 0.2))

    def test_stale_generation_duplicate_and_new_binding(self):
        l = self.new(); self.apply(l, gen=3, lin=0.1, ang=0.3)
        self.assertEqual(self.apply(l, gen=2, lin=0.05, ang=0.2)['reason'], 'STALE_GENERATION'); self.assertEqual(l.limits('auto')[0], 0.1)
        self.assertEqual(self.apply(l, gen=3, lin=0.05, ang=0.2, rid='other')['reason'], 'STALE_GENERATION')
        s = session(req=request(gen=3, lin=0.1, ang=0.3)); l.process(s)                              # the same delivery again: handled once, idempotent
        l.last_result = {'generation': 4, 'request_id': 'r4', 'status': 'APPLIED'}
        self.assertEqual(self.apply(l, gen=1, epoch=6, sess_epoch=5, lin=0.05, ang=0.2)['status'], 'REJECTED')       # lease epoch of the request is not the session's
        s2 = session(epoch=6, req=request(gen=1, epoch=6, lin=0.05, ang=0.2, rid='n1')); l.process(s2)   # new lease epoch: generation restarts
        self.assertEqual(l.last_result['status'], 'APPLIED'); self.assertEqual(l.limits('auto')[0], 0.05)

    def test_lease_loss_keeps_clamping_and_never_raises(self):
        l = self.new(); self.apply(l, lin=0.06, ang=0.2); self.assertFalse(l.stale)
        for label, s in (('lease epoch changed', session(epoch=9)), ('lease expired', session(expires=time.time() - 1)), ('disconnected', session(connected=False)),
                         ('server boot', session(boot='sb2')), ('contract generation', session(contract_gen=4))):
            with self.subTest(label):
                l = self.new(); self.apply(l, lin=0.06, ang=0.2); l.process(s)
                self.assertTrue(l.stale); self.assertEqual(l.limits('auto')[0], 0.06)           # stale but still the lower limit
        l = self.new(); self.apply(l, lin=0.06, ang=0.2); l.process(session(epoch=9))
        self.assertEqual(self.apply(l, gen=1, epoch=9, lin=0.12, ang=0.4, rid='n2')['status'], 'APPLIED')     # only a NEW valid request raises it again
        self.assertFalse(l.stale); self.assertEqual(l.limits('auto')[0], 0.12)

    def test_no_enforcement_point_rejects(self):
        l = SpeedLimiter(); r = self.apply(l); self.assertEqual(r['reason'], 'NO_ENFORCEMENT_POINT'); self.assertEqual(l.limits('auto')[0], 0.15)

    def test_restart_keeps_the_lower_limit_as_stale(self):
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, 'sl.json'); l = self.new(p); self.apply(l, lin=0.06, ang=0.2)
            l2 = self.new(p); self.assertTrue(l2.stale); self.assertEqual(l2.limits('auto')[0], 0.06); self.assertEqual(l2.stale_reason, 'AGENT_RESTARTED_NEEDS_NEW_REQUEST')
            self.assertEqual(self.apply(l2, gen=1, lin=0.1, ang=0.3)['status'], 'APPLIED'); self.assertFalse(l2.stale)

    def test_unreadable_or_out_of_range_state_falls_back_to_the_low_value(self):
        for content in ('not json', json.dumps({'applied': {'auto': {'linear_mps': 5.0, 'angular_rps': 0.2}}}), json.dumps({'applied': {'auto': {'linear_mps': float('nan'), 'angular_rps': 0.2}}})):
            with self.subTest(content[:20]), tempfile.TemporaryDirectory() as d:
                p = os.path.join(d, 'sl.json'); open(p, 'w').write(content); l = self.new(p)
                self.assertEqual(l.fault, 'PERSISTED_STATE_UNREADABLE'); self.assertEqual(l.limits('auto')[0], sl.SAFE_FALLBACK['linear_mps'])
                self.assertEqual(l.limits('manual')[2], sl.SAFE_FALLBACK['angular_rps'])

    def test_snapshot_fields(self):
        l = self.new(); self.apply(l, lin=0.08, ang=0.2); s = l.snapshot()
        self.assertEqual((s['enforced_by'], s['generation'], s['stale']), ('ARBITER', 1, False))
        self.assertEqual(s['ceiling']['auto'], {'linear_mps': 0.15, 'angular_rps': 0.5, 'reverse_mps': 0.1}); self.assertEqual(s['effective']['auto']['linear_mps'], 0.08)
        self.assertEqual(s['last_result']['status'], 'APPLIED'); self.assertEqual(s['applied']['auto']['generation'], 1)


class ArbiterClamp(unittest.TestCase):
    def build(self, source):
        from limo_robot_agent import velocity_arbiter as va
        published = []
        robot = SimpleNamespace(mode='mock', simulated=True, lock=__import__('threading').RLock(),
                                create_publisher=lambda *a, **k: SimpleNamespace(publish=lambda m: published.append((m.linear.x, m.angular.z))),
                                create_subscription=lambda *a, **k: None, create_timer=lambda *a, **k: SimpleNamespace(cancel=lambda: None), active=None)
        sess = session()
        agent = SimpleNamespace(session_snapshot=lambda: sess)
        bench = SimpleNamespace(snapshot=lambda: {'deadman': True, 'expires': time.monotonic() + 5, 'requested_linear': self.bench_v, 'requested_turn': self.bench_w})
        self.bench_v, self.bench_w = 0.10, 0.30
        a = va.VelocityArbiter(robot, agent, bench)
        a.common_blocker = a.manual_blocker = a.manual_generation_blocker = lambda *x: None
        a.nav_blocker = lambda *x: None
        a.mode = lambda: source
        a.update(); published.clear()              # the first update starts generation 1 (it clears nav samples)
        return a, published, sess

    def test_manual_and_auto_pass_through_the_ceilings_unchanged_without_a_request(self):
        a, out, _ = self.build('MANUAL'); a.update(); self.assertEqual(out[-1], (0.10, 0.30))
        a, out, _ = self.build('NAVIGATION'); a.nav_sample = (time.monotonic(), 0.15, 0.5); a.update(); self.assertEqual(out[-1], (0.15, 0.5))
        a.nav_sample = (time.monotonic(), -0.10, -0.5); a.update(); self.assertEqual(out[-1], (-0.10, -0.5))

    def test_requested_limits_clamp_what_leaves_the_arbiter(self):
        a, out, sess = self.build('MANUAL')
        sess['speed_limit_request'] = request(gen=1, scope='manual', lin=0.05, ang=0.15); a.update()
        self.assertEqual(a.limiter.last_result['status'], 'APPLIED'); self.assertEqual(out[-1], (0.05, 0.15))
        self.bench_v, self.bench_w = 0.0, 0.0; a.update(); self.assertEqual(out[-1], (0.0, 0.0))            # zero preserved
        a, out, sess = self.build('NAVIGATION'); sess['speed_limit_request'] = request(gen=1, scope='auto', lin=0.08, ang=0.25)
        a.nav_sample = (time.monotonic(), 0.15, 0.5); a.update(); self.assertEqual(out[-1], (0.08, 0.25))
        a.nav_sample = (time.monotonic(), -0.10, -0.5); a.update(); self.assertEqual(out[-1], (-0.08, -0.25))     # both directions
        a.nav_sample = (time.monotonic(), 0.0, 0.0); a.update(); self.assertEqual(out[-1], (0.0, 0.0))
        self.assertEqual(a.snapshot()['speed_limits']['effective']['auto']['linear_mps'], 0.08)

    def test_a_manual_request_does_not_touch_auto_and_vice_versa(self):
        a, out, sess = self.build('NAVIGATION'); sess['speed_limit_request'] = request(gen=1, scope='manual', lin=0.05, ang=0.1)
        a.nav_sample = (time.monotonic(), 0.15, 0.5); a.update(); self.assertEqual(out[-1], (0.15, 0.5))

    def test_blocked_conditions_still_publish_zero(self):
        a, out, sess = self.build('MANUAL'); a.common_blocker = lambda *x: 'GATE_NOT_NORMAL'; a.update(); self.assertEqual(out[-1], (0.0, 0.0))

    def test_a_failing_limiter_never_lets_an_unclamped_command_through(self):
        a, out, sess = self.build('MANUAL'); a.limiter.clamp = lambda *x: (_ for _ in ()).throw(RuntimeError('x')); a.update(); self.assertEqual(out[-1], (0.0, 0.0))

    def test_mode_change_keeps_the_lower_limit_and_does_not_reuse_it_across_scopes(self):
        a, out, sess = self.build('NAVIGATION'); sess['speed_limit_request'] = request(gen=1, scope='auto', lin=0.06, ang=0.2)
        a.nav_sample = (time.monotonic(), 0.15, 0.5); a.update(); self.assertEqual(out[-1], (0.06, 0.2))
        a.mode = lambda: 'MANUAL'; a.update(); self.assertEqual(out[-1], (0.10, 0.30))            # manual scope unaffected
        a.mode = lambda: 'NAVIGATION'; a.update(); a.nav_sample = (time.monotonic(), 0.15, 0.5); a.update(); self.assertEqual(out[-1], (0.06, 0.2))
        sess['lease'] = {'epoch': 8, 'expires_at': time.time() + 60}; sess['speed_limit_request'] = None; a.update()      # a new generation clears samples
        a.nav_sample = (time.monotonic(), 0.15, 0.5); a.update()
        self.assertTrue(a.limiter.stale); self.assertEqual(out[-1], (0.06, 0.2))                      # lease changed: stale, still clamped, not raised


if __name__ == '__main__':
    unittest.main()
