#!/usr/bin/env python3
"""Pre-start DDS slot check (read-only toward the running stack). Run it BEFORE starting a ROS node that must receive data.

Why: on the real LIMO (2026-11-20) a new participant that was handed participant ID 23 never received any message from the running stack
(discovery worked, data did not); IDs 26..100 were fine. The cause of that slot is not established (docs/DDS_NEW_PARTICIPANT_ROOTCAUSE_20261120.md,
docs/DDS_STARTUP_DIAGNOSTIC_20261120.md). This script does not fix it; it detects it before a real node is started on that slot.

What it does
 1. finds the lowest free participant ID (both RTPS unicast UDP ports free on 127.0.0.1) = the slot a new node would get,
 2. starts a canary participant with that ID forced (temporary FastDDS profile, canary process only) that SUBSCRIBES to the required topics,
 3. SLOT_OK only if every required topic delivers a first message within --first-s AND the counters advance in BOTH halves of --fresh-s (needs publish period < fresh-s/2),
 4. otherwise SLOT_DEAD; it then tries the next free IDs (up to --alternatives) and reports the first healthy one as recommended_participant_id.
The canary ends by itself (rclpy shutdown) - it is never killed from outside, because abnormal exits are a suspect for the bad slot.
Exit codes: 0 SLOT_OK, 3 SLOT_DEAD (healthy alternative may be listed), 4 NO_HEALTHY_SLOT_FOUND, 2 usage/environment error.
usage: dds_slot_precheck.py [--topics /scan,/odom] [--first-s 8] [--fresh-s 2] [--alternatives 3] [--json]
Topic types are looked up from the live graph; any 'name:pkg/msg/Type' pair may be given for topics that do not exist yet in the graph.
"""
import argparse, json, os, socket, subprocess, sys, tempfile, time

DEFAULT_TOPICS = '/scan,/odom'   # /tf is NOT used: on the real robot a healthy slot sometimes got few or no /tf samples (bursty), a false-positive source


def ports(domain, pid):
    base = 7400 + 250 * domain
    return base + 10 + 2 * pid, base + 11 + 2 * pid


def slot_free(domain, pid):
    socks = []
    try:
        for p in ports(domain, pid):
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.bind(('127.0.0.1', p))
            socks.append(s)
        return True
    except OSError:
        return False
    finally:
        for s in socks:
            s.close()


def free_slots(domain, count, start=0, limit=120):
    out = []
    for pid in range(start, limit):
        if slot_free(domain, pid):
            out.append(pid)
            if len(out) == count:
                break
    return out


def canary_main(topics, first_s, fresh_s):
    """Runs inside the canary process. Prints one JSON line."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, HistoryPolicy
    from rosidl_runtime_py.utilities import get_message
    rclpy.init()
    n = Node('dds_slot_canary_%d' % (int(time.time() * 1000) % 100000))
    known = {'/scan': 'sensor_msgs/msg/LaserScan', '/odom': 'nav_msgs/msg/Odometry', '/tf': 'tf2_msgs/msg/TFMessage', '/map': 'nav_msgs/msg/OccupancyGrid'}
    st = {t: {'count': 0, 't_first': None, 'discovered': False} for t, _ in topics}
    subs = {}
    t0 = time.monotonic()

    def cb(t):
        def f(_):
            s = st[t]
            s['count'] += 1
            if s['t_first'] is None:
                s['t_first'] = round(time.monotonic() - t0, 2)
        return f
    deadline = t0 + first_s
    while time.monotonic() < deadline and not all(s['t_first'] is not None for s in st.values()):
        for t, typ in topics:
            infos = n.get_publishers_info_by_topic(t)
            if infos:
                st[t]['discovered'] = True
            if infos and t not in subs:
                q = infos[0].qos_profile
                subs[t] = n.create_subscription(get_message(typ or known[t]), t, cb(t), QoSProfile(depth=max(1, q.depth or 1), reliability=q.reliability, durability=q.durability, history=HistoryPolicy.KEEP_LAST))
        rclpy.spin_once(n, timeout_sec=0.1)
    # freshness: data must keep flowing in BOTH halves of the window (a publisher that stopped midway must not pass)
    m0 = {t: s['count'] for t, s in st.items()}
    for half in (1, 2):
        end = time.monotonic() + fresh_s / 2.0
        while time.monotonic() < end:
            rclpy.spin_once(n, timeout_sec=0.1)
        if half == 1:
            m1 = {t: s['count'] for t, s in st.items()}
    for t, s in st.items():
        s['advanced'] = s['count'] > m1[t] > m0[t]
    print(json.dumps(st), flush=True)
    rclpy.shutdown()


def run_canary(pid, topics, first_s, fresh_s):
    xml = tempfile.NamedTemporaryFile('w', suffix='.xml', delete=False)
    xml.write('<?xml version="1.0"?><profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"><participant profile_name="f" is_default_profile="true">'
              '<rtps><participantID>%d</participantID></rtps></participant></profiles>' % pid)
    xml.close()
    env = dict(os.environ, FASTRTPS_DEFAULT_PROFILES_FILE=xml.name)
    spec = ','.join(t + (':' + typ if typ else '') for t, typ in topics)
    try:
        # generous outer limit only as a last resort for a hung interpreter; the canary normally exits on its own
        p = subprocess.run([sys.executable, os.path.abspath(__file__), '--canary', spec, '--first-s', str(first_s), '--fresh-s', str(fresh_s)],
                           env=env, capture_output=True, text=True, timeout=first_s + fresh_s + 40)
    except subprocess.TimeoutExpired:
        return {'error': 'canary did not finish'}
    finally:
        os.unlink(xml.name)
    lines = [l for l in p.stdout.strip().splitlines() if l.startswith('{')]
    if not lines:
        return {'error': 'canary produced no result: ' + p.stderr.strip()[-200:]}
    return json.loads(lines[-1])


def verdict(res):
    if 'error' in res:
        return False
    return all(s['t_first'] is not None and s['advanced'] for s in res.values())


def parse_topics(s):
    out = []
    for item in s.split(','):
        if not item:
            continue
        name, _, typ = item.partition(':')
        out.append((name, typ or None))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topics', default=DEFAULT_TOPICS)
    ap.add_argument('--first-s', type=float, default=8.0)
    ap.add_argument('--fresh-s', type=float, default=2.0)
    ap.add_argument('--alternatives', type=int, default=3)
    ap.add_argument('--canary')
    ap.add_argument('--json', action='store_true')
    a = ap.parse_args()
    topics = parse_topics(a.canary or a.topics)
    if a.canary:
        return canary_main(topics, a.first_s, a.fresh_s)
    try:
        domain = int(os.environ['ROS_DOMAIN_ID'])
    except (KeyError, ValueError):
        print('ROS_DOMAIN_ID must be set to the stack domain', file=sys.stderr)
        return 2
    slots = free_slots(domain, 1 + a.alternatives)
    report = {'domain': domain, 'topics': [t for t, _ in topics], 'lowest_free_id': slots[0] if slots else None, 'checked': [], 'recommended_participant_id': None}
    for pid in slots:
        res = run_canary(pid, topics, a.first_s, a.fresh_s)
        ok = verdict(res)
        report['checked'].append({'id': pid, 'ports': list(ports(domain, pid)), 'healthy': ok, 'detail': res})
        if ok:
            report['recommended_participant_id'] = pid
            break
    first = report['checked'][0] if report['checked'] else None
    if first and first['healthy']:
        report['verdict'], code = 'SLOT_OK', 0
    elif report['recommended_participant_id'] is not None:
        report['verdict'], code = 'SLOT_DEAD', 3
    else:
        report['verdict'], code = ('NO_HEALTHY_SLOT_FOUND' if report['checked'] else 'NO_FREE_SLOT'), 4
    print(json.dumps(report, indent=None if not a.json else 1))
    if not a.json:
        print('%s: lowest free ID %s%s' % (report['verdict'], report['lowest_free_id'], ('; healthy alternative ID %s' % report['recommended_participant_id']) if code == 3 else ''), file=sys.stderr)
    return code


if __name__ == '__main__':
    sys.exit(main())
