#!/usr/bin/env python3
"""Open-RMF Fleet Adapter for LIMO ("pcu"). RMF -> this adapter -> Mission Manager (fleet_server) -> Robot Agent -> Nav2.
This process never publishes /cmd_vel and never talks to Nav2; every motion request is a Mission Manager mission (auth, idempotency, ownership, recovery)."""
import argparse, datetime, json, math, os, signal, sys, threading, time
import numpy as np
import rmf_adapter as adpt
import rmf_adapter.battery as battery
import rmf_adapter.geometry as geometry
import rmf_adapter.graph as graph
import rmf_adapter.plan as plan
import rmf_adapter.vehicletraits as traits
from .coords import Transform
from .mission_client import MissionClient
from .runner import MissionRunner, LINK_OK


def build_graph(cfg, vt):
    g = graph.Graph(); idx = {}
    for w in cfg['waypoints']:
        g.add_waypoint(cfg['map_name'], [float(w['rmf'][0]), float(w['rmf'][1])]); i = g.num_waypoints - 1; idx[w['name']] = i   # do not keep the returned reference (dangling in this binding)
        if w.get('charger'): g.get_waypoint(i).set_charger(True)
        g.add_key(w['name'], i)
    for a, b in cfg['lanes']: g.add_bidir_lane(idx[a], idx[b])
    return g, idx


class Cb:
    """Receives runner events for one path and forwards them to RMF."""
    def __init__(self, handle, estimator, finished, offset, speed, idxs):
        self.h, self.est, self.fin, self.off, self.speed, self.idxs = handle, estimator, finished, offset, speed, idxs

    def progress(self, gen, idx, remaining_m):
        try: self.est(self.off + self.idxs[min(idx, len(self.idxs) - 1)], datetime.timedelta(seconds=remaining_m / self.speed))
        except Exception as e: self.h.log(f'estimator failed: {e}')

    def done(self, gen, ok, reason):
        if ok:
            self.h.log(f'path generation {gen} finished'); self.fin()
        else:
            self.h.log(f'path generation {gen} FAILED: {reason}')
            self.h.on_path_failed(reason)


class LimoCommandHandle(adpt.RobotCommandHandle):
    def __init__(self, runner, tf, cfg, log):
        super().__init__(); self.runner, self.tf, self.cfg, self.log = runner, tf, cfg, log
        self.updater = None; self.failures = 0

    def current_nav2_pose(self):
        r = (self.runner.snapshot().get('robot') or {}).get('state') or {}
        p = r.get('pose') or {}
        if p.get('status') == 'OK' and p.get('frame_id') == 'map': return p['x'], p['y'], p.get('yaw_rad', 0.0)
        return None

    def follow_new_path(self, waypoints, next_arrival_estimator, path_finished_callback):
        cur = self.current_nav2_pose(); goals = []; idxs = []; first = 0
        for i, w in enumerate(waypoints):
            x, y, yaw = self.tf.rmf_to_nav2(float(w.position[0]), float(w.position[1]), float(w.position[2]))
            if not goals and cur and math.hypot(x - cur[0], y - cur[1]) < self.cfg['skip_start_waypoint_m']: first = i + 1; continue
            if goals and math.hypot(x - goals[-1][0], y - goals[-1][1]) < 0.05:           # RMF repeats a position for wait/dock events: one Nav2 goal is enough
                goals[-1] = (x, y, yaw); idxs[-1] = i - first; continue
            goals.append((x, y, yaw)); idxs.append(i - first)
        if not goals:
            self.log('path has nothing to drive'); path_finished_callback(); return
        self.log(f'RMF path: {len(waypoints)} waypoints -> {len(goals)} Mission Manager goals')
        self.runner.follow_path(goals, Cb(self, next_arrival_estimator, path_finished_callback, first, self.cfg['arrival_speed_mps'], idxs))

    def stop(self): self.runner.stop()
    def dock(self, dock_name, docking_finished_callback): docking_finished_callback()

    def on_path_failed(self, reason):
        self.failures += 1
        if self.updater:
            self.updater.log_error(f'Mission Manager path failed: {reason}')
            self.updater.override_status('error')


def main(argv=None):
    ap = argparse.ArgumentParser(); ap.add_argument('--config', required=True); ap.add_argument('--server-url', required=True); ap.add_argument('--token-file', required=True)
    ap.add_argument('--state-dir', required=True); ap.add_argument('--rmf-api-uri', default=None, help='optional ws://host:port: where the RMF library broadcasts its task/fleet state JSON (an RMF API server or a sink)'); ap.add_argument('--dry-graph', action='store_true', help='only build the graph and exit')
    a = ap.parse_args(argv)
    cfg = json.load(open(a.config)); tf = Transform.from_config(cfg['transform_rmf_from_nav2'])
    def log(m): print(f'{time.strftime("%H:%M:%S")} {m}', flush=True)
    t = cfg['traits']
    vt = traits.VehicleTraits(linear=traits.Limits(*t['linear']), angular=traits.Limits(*t['angular']), profile=traits.Profile(geometry.make_final_convex_circle(t['footprint_radius']), geometry.make_final_convex_circle(t['vicinity_radius'])))
    vt.differential.reversible = False
    g, idx = build_graph(cfg, vt)
    log(f'graph: {g.num_waypoints} waypoints, {g.num_lanes} lanes, keys={sorted(g.keys)}')
    if a.dry_graph: return 0
    os.makedirs(a.state_dir, exist_ok=True)
    runner = MissionRunner(MissionClient(a.server_url, a.token_file), cfg['mission_robot_id'], os.path.join(a.state_dir, f'rmf-{cfg["robot_name"]}.json'), log=log)
    adpt.init_rclcpp()
    adapter = adpt.Adapter.make('limo_fleet_adapter')
    if not adapter: log('could not create adapter node'); return 2
    fleet = adapter.add_fleet(cfg['fleet_name'], vt, g, a.rmf_api_uri)

    def consider(_desc):
        c = adpt.fleet_update_handle.Confirmation(); snap = runner.snapshot(); fresh_ok, why = runner.check_available_now()
        if snap['link'] == LINK_OK and fresh_ok: c.accept()
        else: c.add_errors([json.dumps({'reason': f"robot not available for RMF tasks: link={snap['link']} {why or snap.get('last_error', '')}"})])
        return c
    fleet.consider_patrol_requests(consider); fleet.consider_composed_requests(consider)
    bs = battery.BatterySystem.make(12.0, 6.0, 2.0); ms = battery.MechanicalSystem.make(5.0, 0.2, 0.2)
    fleet.set_task_planner_params(bs, battery.SimpleMotionPowerSink(bs, ms), battery.SimpleDevicePowerSink(bs, battery.PowerSystem.make(10.0)), battery.SimpleDevicePowerSink(bs, battery.PowerSystem.make(1.0)), 0.2, 1.0, False)

    runner.start()
    log(f'waiting for the robot pose from the Mission Manager ({a.server_url}) ...')
    cmd = LimoCommandHandle(runner, tf, cfg, log)
    pose = None
    while pose is None:
        pose = cmd.current_nav2_pose(); time.sleep(0.5)
    rx, ry, ryaw = tf.nav2_to_rmf(*pose)
    starts = plan.compute_plan_starts(g, cfg['map_name'], np.array([rx, ry, ryaw]), adapter.now(), cfg['initial_waypoint_snap_m'], 1.0, 1e-8)
    if not starts: log(f'robot at RMF ({rx:.2f},{ry:.2f}) is not on/near the nav graph: not registering (move it near a waypoint)'); return 3
    prof = traits.Profile(geometry.make_final_convex_circle(t['footprint_radius']))
    fleet.add_robot(cmd, cfg['robot_name'], prof, starts, lambda u: setattr(cmd, 'updater', u))
    adapter.start(); log(f'RMF fleet "{cfg["fleet_name"]}" robot "{cfg["robot_name"]}" registered; start waypoint index {starts[0].waypoint}')

    stop = threading.Event(); signal.signal(signal.SIGINT, lambda *_: stop.set()); signal.signal(signal.SIGTERM, lambda *_: stop.set())
    status = None
    while not stop.is_set():
        u = cmd.updater; snap = runner.snapshot()
        if u:
            want = 'offline' if snap['link'] in ('SERVER_UNREACHABLE', 'ROBOT_OFFLINE') else ('error' if snap['link'] == 'RECONCILIATION_REQUIRED' else None)
            if want != status:
                u.override_status(want); status = want; log(f'RMF robot status override: {want} (link={snap["link"]})')
            p = cmd.current_nav2_pose()
            if p and want is None:
                x, y, yaw = tf.nav2_to_rmf(*p); pos = np.array([x, y, yaw])
                st = plan.compute_plan_starts(g, cfg['map_name'], pos, adapter.now(), cfg['merge_waypoint_distance_m'], 1.0, 1e-8)
                if st: u.update_position(st)
                else: u.update_lost_position(cfg['map_name'], pos)
            bat = (((snap.get('robot') or {}).get('state') or {}).get('battery') or {}).get('percent')
            if isinstance(bat, (int, float)): u.update_battery_soc(max(0.0, min(1.0, bat / 100.0)))
        stop.wait(1.0 / cfg['position_report_hz'])
    log('stopping: canceling any active Mission Manager mission'); runner.stop(); time.sleep(1.5); runner.close(); adapter.stop(); sys.stdout.flush(); os._exit(0)   # C++ executor teardown aborts otherwise


if __name__ == '__main__': sys.exit(main())
