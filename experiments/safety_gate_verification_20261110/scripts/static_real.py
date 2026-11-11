import importlib.util, sys, os
from launch import LaunchContext
from launch.utilities import perform_substitutions, normalize_to_list_of_substitutions
from launch.actions import GroupAction, DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node, SetRemap
from launch.conditions import IfCondition, UnlessCondition

path = sys.argv[1]
spec = importlib.util.spec_from_file_location('L', path); m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
ld = m.generate_launch_description()
ents = ld.entities

def ctx_with(**kw):
    c = LaunchContext()
    for e in ents:
        if isinstance(e, DeclareLaunchArgument):
            c.launch_configurations[e.name] = e.default_value[0].perform(c) if e.default_value else ''
    c.launch_configurations.update(kw)
    return c

def ev(c, subs):
    return perform_substitutions(c, normalize_to_list_of_substitutions(subs))

def run(**kw):
    c = ctx_with(**kw)
    print(f'--- args={kw}')
    gates = []
    for e in ents:
        if isinstance(e, Node):
            exe = ev(c, e._Node__node_executable)
            cond = e.condition.evaluate(c) if e.condition is not None else True
            if 'stall_detector' in exe:
                rem = [(ev(c, a), ev(c, b)) for a, b in (e._Node__remappings or [])]
                gates.append((cond, rem))
        if isinstance(e, GroupAction):
            sr = [(ev(c, a.src), ev(c, a.dst)) for a in e._GroupAction__actions if isinstance(a, SetRemap)] if hasattr(e, '_GroupAction__actions') else []
            print('  nav2 SetRemaps:', sr)
    print('  stall gate nodes active:', [(cond, rem) for cond, rem in gates if cond])
    n_active = sum(1 for cond, _ in gates if cond)
    print('  active gate count =', n_active)
    return n_active, gates

for kw in ({'allow_motor_drive': 'false'}, {'allow_motor_drive': 'true'}, {'allow_motor_drive': 'TRUE'}, {'allow_motor_drive': ''},
            {'allow_motor_drive': 'true', 'enable_stall_detector': 'false'}, {'allow_motor_drive': 'false', 'enable_stall_detector': 'false'}):
    run(**kw)
