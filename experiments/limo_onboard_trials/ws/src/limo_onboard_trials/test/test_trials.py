"""파일 경로·시험 조건·SLAM/AMCL 분리를 검사한다. 주행 성능 검사는 아니다."""
import copy
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
import xml.etree.ElementTree as ET

import pytest
import yaml
from launch import LaunchContext
from limo_onboard_trials.trial_config import configure, validate_map

SHARE = Path(__file__).resolve().parents[1]


def load_launch(filename, monkeypatch, args):
    spec = importlib.util.spec_from_file_location('trial_launch_test', SHARE/'launch'/filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda name: str(SHARE))
    monkeypatch.setattr(module, 'LaunchConfiguration', lambda name:
                        NS(perform=lambda context: args.get(name, '')))
    monkeypatch.setattr(module, 'Node', lambda **kwargs: dict(kind='node', **kwargs))
    monkeypatch.setattr(module, 'PythonLaunchDescriptionSource', lambda filename: filename, raising=False)
    monkeypatch.setattr(module, 'IncludeLaunchDescription', lambda filename, **kwargs:
                        dict(kind='include', filename=filename, **kwargs), raising=False)
    return module


@pytest.mark.parametrize('profile', ['usb', 'bt', 'rpp_bt'])
def test_profiles_leave_usb_sensors_costmaps_and_tolerances_unchanged(profile):
    source, rewrites = configure(SHARE, profile, '15')
    baseline = yaml.safe_load((SHARE/'config/nav2_usb.yaml').read_text())
    actual = yaml.safe_load(Path(source).read_text())
    expected = copy.deepcopy(baseline)
    if profile == 'rpp_bt':
        plugin = actual['controller_server']['ros__parameters']['FollowPath']
        assert plugin['plugin'].endswith('RegulatedPurePursuitController')
        assert plugin['use_collision_detection'] is True
        expected['controller_server']['ros__parameters']['FollowPath'] = plugin
    if profile != 'usb':
        expected['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = '__BT_XML_PATH__'
    assert actual == expected
    assert bool(rewrites) == (profile != 'usb')


@pytest.mark.parametrize('backup', ['15', '30'])
def test_bt_distance_and_no_spin(backup):
    _, rewrites = configure(SHARE, 'bt', backup)
    bt = ET.parse(rewrites['default_nav_to_pose_bt_xml']).getroot()
    assert bt.find('.//Spin') is None
    assert float(bt.find('.//BackUp').attrib['backup_dist']) == int(backup)/100
    assert bt.find('.//Wait') is not None


@pytest.mark.parametrize('profile,backup,horizon', [
    ('unknown','15',''), ('usb','30',''), ('bt','15','.6'),
    ('rpp_bt','15','nan'), ('rpp_bt','15','0'), ('bt','99','')])
def test_invalid_or_ambiguous_combinations_fail(profile, backup, horizon):
    with pytest.raises(ValueError):
        configure(SHARE, profile, backup, horizon)


def test_rpp_runtime_rewrite_has_portable_bt_and_horizon():
    source, rewrites = configure(SHARE, 'rpp_bt', '15', '.6')
    from nav2_common.launch import RewrittenYaml
    output = Path(RewrittenYaml(source_file=source, root_key='',
                  param_rewrites=rewrites, convert_types=True).perform(LaunchContext()))
    try:
        actual = yaml.safe_load(output.read_text())
        assert actual['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] == str(SHARE/'config/recovery_backup15.xml')
        assert actual['controller_server']['ros__parameters']['FollowPath']['max_allowed_time_to_collision_up_to_carrot'] == .6
    finally:
        output.unlink()


def test_map_resolves_image_relative_to_yaml_and_rejects_missing(tmp_path):
    filename = tmp_path/'map.yaml'
    filename.write_text('image: map.pgm\n')
    with pytest.raises(ValueError):
        validate_map(filename)
    (tmp_path/'map.pgm').touch()
    assert validate_map(filename) == str(filename)


def test_navigation_launch_has_localization_bringup_without_separate_slam(monkeypatch, tmp_path):
    filename = tmp_path/'map.yaml'
    filename.write_text('image: map.pgm\n'); (tmp_path/'map.pgm').touch()
    args = {'profile':'rpp_bt','backup':'15','mode':'navigation','map':str(filename),
            'collision_time':'','use_sim_time':'false'}
    module = load_launch('navigation_trial.launch.py', monkeypatch, args)
    actions = module.start(LaunchContext())
    assert len(actions) == 1 and actions[0]['filename'].endswith('bringup_launch.py')
    assert dict(actions[0]['launch_arguments'])['map'] == str(filename)


def test_mapping_launch_has_only_slam_and_navigation(monkeypatch):
    args = {'profile':'usb','backup':'15','mode':'mapping','map':'',
            'collision_time':'','use_sim_time':'false'}
    module = load_launch('navigation_trial.launch.py', monkeypatch, args)
    actions = module.start(LaunchContext())
    assert len(actions) == 2 and actions[0]['package'] == 'slam_toolbox'
    assert actions[1]['filename'].endswith('navigation_launch.py')
    assert 'map' not in dict(actions[1]['launch_arguments'])
    args['map'] = 'something.yaml'
    with pytest.raises(ValueError):
        module.start(LaunchContext())


@pytest.mark.parametrize('target,package', [('centroid','explore_lite'), ('middle','limo_mapping_explore')])
def test_exploration_selects_existing_or_separate_middle_node(monkeypatch, target, package):
    module = load_launch('exploration_trial.launch.py', monkeypatch,
                         {'target':target,'use_sim_time':'false'})
    actions = module.start(LaunchContext())
    assert len(actions) == 1 and actions[0]['package'] == package
    assert actions[0]['parameters'][1]['use_sim_time'] is False
