#!/usr/bin/env python3
"""Run-time scenario control for the integrated simulation (Gazebo Fortress via `ign` CLI; simulation only).
usage: scenario_ctl.py {spawn-box|spawn-person|move-person|stop-person|remove} ...   (needs IGN_PARTITION of the running simulation)
World coordinates (Gazebo `lounge_world`), not Nav2 map coordinates.
"""
import os, subprocess, sys, tempfile

HERE = os.path.dirname(os.path.abspath(__file__)); MODELS = os.path.join(HERE, '..', 'models', 'scenario')
WORLD = os.environ.get('INT_WORLD_NAME', 'lounge_world')


def _ign(args, timeout=15):
    r = subprocess.run(['ign'] + args, capture_output=True, text=True, timeout=timeout); return r.returncode, (r.stdout + r.stderr).strip()


def spawn(sdf_path, name, x, y, z=0.0, yaw=0.0):
    import math
    req = (f'sdf_filename: "{sdf_path}" name: "{name}" allow_renaming: false '
           f'pose {{ position {{ x: {x} y: {y} z: {z} }} orientation {{ z: {math.sin(yaw/2):.6f} w: {math.cos(yaw/2):.6f} }} }}')
    rc, out = _ign(['service', '-s', f'/world/{WORLD}/create', '--reqtype', 'ignition.msgs.EntityFactory', '--reptype', 'ignition.msgs.Boolean', '--timeout', '8000', '--req', req])
    return rc == 0 and 'true' in out, out


def remove(name):
    rc, out = _ign(['service', '-s', f'/world/{WORLD}/remove', '--reqtype', 'ignition.msgs.Entity', '--reptype', 'ignition.msgs.Boolean', '--timeout', '8000', '--req', f'name: "{name}" type: 2'])
    return rc == 0 and 'true' in out, out


def person_sdf():
    mesh = os.environ.get('INT_PERSON_MESH')
    if not mesh or not os.path.isfile(mesh): raise SystemExit('INT_PERSON_MESH must name a person mesh file (.dae); not stored in the repository')
    text = open(os.path.join(MODELS, 'person_walker.sdf.in')).read().replace('@PERSON_MESH@', mesh)
    fd, p = tempfile.mkstemp(suffix='.sdf', prefix='person_walker_'); os.write(fd, text.encode()); os.close(fd); return p


def chair_sdf():
    mesh = os.environ.get('INT_CHAIR_MESH')
    if not mesh or not os.path.isfile(mesh): raise SystemExit('INT_CHAIR_MESH must name a chair mesh (.obj); not stored in the repository')
    text = open(os.path.join(MODELS, 'chair.sdf.in')).read().replace('@CHAIR_MESH@', mesh)
    fd, p = tempfile.mkstemp(suffix='.sdf', prefix='scenario_chair_'); os.write(fd, text.encode()); os.close(fd); return p


def person_velocity(vx, vy=0.0, wz=0.0):
    rc, out = _ign(['topic', '-t', '/model/person_walker/cmd_vel', '-m', 'ignition.msgs.Twist', '-p', f'linear: {{x: {vx}, y: {vy}}} angular: {{z: {wz}}}'])
    return rc == 0, out


if __name__ == '__main__':
    c = sys.argv[1]; a = sys.argv[2:]
    if c == 'spawn-box': print(spawn(os.path.join(MODELS, 'obstacle_box.sdf'), a[0] if a else 'obstacle_box', float(a[1]), float(a[2])))
    elif c == 'spawn-person': print(spawn(person_sdf(), 'person_walker', float(a[0]), float(a[1]), 0.0, float(a[2]) if len(a) > 2 else 0.0))
    elif c == 'spawn-chair': print(spawn(chair_sdf(), 'scenario_chair', float(a[0]), float(a[1]), 0.0, float(a[2]) if len(a) > 2 else 0.0))
    elif c == 'move-person': print(person_velocity(float(a[0]), float(a[1]) if len(a) > 1 else 0.0))
    elif c == 'stop-person': print(person_velocity(0.0))
    elif c == 'remove': print(remove(a[0]))
    else: raise SystemExit(__doc__)
