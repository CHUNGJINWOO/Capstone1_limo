"""Check SLAM startup configuration, without starting mapping processes."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS

import pytest


@pytest.fixture
def launch_module(monkeypatch):
    path = Path(__file__).resolve().parents[1] / 'launch/auto_mapping.launch.py'
    spec = importlib.util.spec_from_file_location('auto_mapping_launch_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    monkeypatch.setattr(module, 'IncludeLaunchDescription', lambda *a, **kw: kw)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda name:
                        str(path.parents[1]) if name == 'limo_auto_mapping' else '/unused')
    def run(args):
        monkeypatch.setattr(module, 'LaunchConfiguration', lambda name:
                            NS(perform=lambda _: args.get(name, '')))
        return module.start(None)
    return run


def test_new_mapping_has_no_seed_and_waits_for_action(launch_module):
    nodes = launch_module({'use_sim_time': 'false'})
    assert len(nodes) == 3
    slam = nodes[0]['parameters'][0]
    assert slam['mode'] == 'mapping' and not slam['use_sim_time']
    assert 'map_file_name' not in slam
    assert nodes[2]['executable'] == 'mapping_server'


def test_resume_passes_pose_and_graph_without_static_map_server(launch_module, tmp_path):
    prefix = tmp_path / 'seed'
    for ext in ('.posegraph', '.data'):
        Path(str(prefix)+ext).write_text('test fixture, not a real graph')
    nodes = launch_module({'posegraph': str(prefix), 'start_pose': '[1, 2, 0.5]'})
    slam = nodes[0]['parameters'][0]
    assert slam['mode'] == 'mapping'
    assert slam['map_file_name'] == str(prefix)
    assert slam['map_start_pose'] == [1.0, 2.0, 0.5]
    assert slam['map_start_at_dock'] is False


@pytest.mark.parametrize('pose', ['', '[0,0]', '[0,0,.nan]', 'invalid'])
def test_resume_rejects_missing_or_invalid_pose(launch_module, tmp_path, pose):
    prefix = tmp_path / 'seed'
    for ext in ('.posegraph', '.data'):
        Path(str(prefix)+ext).touch()
    with pytest.raises(RuntimeError):
        launch_module({'posegraph': str(prefix), 'start_pose': pose})


def test_resume_requires_both_files_and_pose_requires_graph(launch_module, tmp_path):
    prefix = tmp_path / 'seed'
    Path(str(prefix) + '.posegraph').touch()
    with pytest.raises(RuntimeError):
        launch_module({'posegraph': str(prefix), 'start_pose': '[0,0,0]'})
    with pytest.raises(RuntimeError):
        launch_module({'start_pose': '[0,0,0]'})
