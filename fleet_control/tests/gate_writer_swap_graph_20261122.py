"""SIMULATION ONLY (2026-11-22): the Gate's /cmd_vel_raw input-identity rule in the graph state seen in Gazebo run r4 (f03 -> f04).

Gate log of r4: after the Watchdog was SIGKILLed and restarted, the Gate's DDS graph showed ONLY the dead writer's endpoint (its bound
GID 010f025b0f94..) for ~20 s while the new Watchdog was already sending; then the new endpoint appeared, node name not yet resolved,
and the Gate latched NOT_EXCLUSIVE. The isolated rig (gate_resume_stale_cmd_isolated.py --kinds swap) does not reproduce that ordering
(both endpoints visible at once), so this check drives the Gate's own input_graph() with that graph state and a writer status stream.

Gates: Gate r12 (experiments/.../candidate_gate_r12, read in place, unmodified) and a candidate (--gate, e.g. r13 with
input_writer_status_topic). Unit-level model of one Gate function, not a DDS or real-robot measurement.
usage: python3 gate_writer_swap_graph_20261122.py [--gate <candidate stall_detector_node.py>]
"""
import argparse, importlib.util, json, sys, time
from pathlib import Path
from types import SimpleNamespace as NS

REPO = Path(__file__).resolve().parents[2]
R12 = REPO / 'experiments/safety_chain_fix_20261113/candidate_gate_r12/nodes/stall_detector_node.py'
OLD, NEW = '010f025b0f94' + '00' * 12, '010f025b0e12' + '00' * 12


def load(path):
    spec = importlib.util.spec_from_file_location(f'gate_{abs(hash(str(path)))}', path)
    m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m); return m


def gate_class(mod):
    return next(v for v in vars(mod).values() if isinstance(v, type) and hasattr(v, 'input_graph') and hasattr(v, 'input_faults'))


def fake(cls, with_status):
    log = NS(info=lambda *a: None, error=lambda *a: None, warn=lambda *a: None)
    g = NS(in_allowed=['fleet_motion_watchdog'], in_bound=None, in_cand=None, in_settle=0.0, in_sticky=None, in_ok_since=None,
           in_reason='NO_PUBLISHER', in_n={'binds': 0}, in_pubs=[], in_invalid_t=None, in_hold=1.0, wr_topic='/fleet/motion_watchdog/status' if with_status else '',
           wr=None, in_bound_boot=None, get_logger=lambda: log, graph=[])
    g.get_publishers_info_by_topic = lambda topic: [NS(endpoint_gid=bytes.fromhex(x), node_name=n, node_namespace='/') for x, n in g.graph]
    g.input_graph = lambda: cls.input_graph(g); g.input_faults = lambda t=0.0: cls.input_faults(g, t)
    return g


def run(path, with_status):
    g = fake(gate_class(load(path)), with_status)
    steps = []
    def step(label, graph, wr):
        g.graph = graph; g.wr = None if wr is None else (wr[0], wr[1], time.monotonic())
        ok = g.input_graph(); ok = g.input_graph()        # settle 0 -> second evaluation uses the bound writer
        steps.append({'step': label, 'accepts_input': ok, 'reason': g.in_reason, 'sticky': g.in_sticky, 'faults': g.input_faults()})
    step('1 old writer running', [(OLD, 'fleet_motion_watchdog')], ('boot_old', OLD))
    step('2 old SIGKILLed, new started; Gate graph shows only the dead endpoint', [(OLD, 'fleet_motion_watchdog')], ('boot_new', NEW))
    step('3 new endpoint discovered, name unresolved', [(NEW, '_NODE_NAME_UNKNOWN_'), (OLD, 'fleet_motion_watchdog')], ('boot_new', NEW))
    return steps


if __name__ == '__main__':
    ap = argparse.ArgumentParser(); ap.add_argument('--gate'); ap.add_argument('--out')
    a = ap.parse_args()
    res = {'r12': run(R12, False)}
    if a.gate: res['candidate'] = run(Path(a.gate), True)
    for k, steps in res.items():
        for s in steps: print(k, json.dumps(s))
    verdict = {k: ('ACCEPTS_UNVERIFIED_NEW_WRITER' if v[1]['accepts_input'] else 'ZERO_UNTIL_RESET') for k, v in res.items()}
    print('VERDICT', json.dumps(verdict))
    if a.out: Path(a.out).write_text(json.dumps({'steps': res, 'verdict': verdict}, indent=1))
    sys.exit(0)
