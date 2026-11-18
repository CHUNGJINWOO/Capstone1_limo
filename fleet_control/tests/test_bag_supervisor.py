"""Single Linux process regression for Agent SIGKILL and unrelated recorder isolation."""
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import pytest


def wait_for(read, timeout=8):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        try:
            value = read()
            if value:
                return value
        except (OSError, ValueError):
            pass
        time.sleep(.05)
    raise AssertionError('timed out waiting for process evidence')


@pytest.mark.skipif(sys.platform != 'linux', reason='Linux process ownership')
def test_agent_sigkill_closes_only_its_recorder(tmp_path):
    agent_dir = Path(__file__).resolve().parents[1] / 'agent/limo_robot_agent'
    fake_ros = tmp_path / 'ros2'
    fake_ros.write_text('#!' + sys.executable + '''
import signal, sys, time
signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
while True:
    time.sleep(.1)
''')
    fake_ros.chmod(0o700)
    env = dict(os.environ, PATH=str(tmp_path) + os.pathsep + os.environ['PATH'], PYTHONPATH=str(agent_dir))
    unrelated = subprocess.Popen([fake_ros], start_new_session=True)
    agent = subprocess.Popen([sys.executable, '-c',
                              'from limo_robot_agent.bag_supervisor import SupervisedBag;'
                              'from pathlib import Path; import sys,time;'
                              'b=SupervisedBag(Path(sys.argv[1])); b.start("testsession",["/map"]); time.sleep(30)',
                              str(tmp_path)], env=env)
    try:
        journal = tmp_path / 'sessions/testsession/journal.json'
        read = lambda: json.loads(journal.read_text())
        spawned = wait_for(lambda: (v := read())['status'] == 'SPAWNED' and v)
        owned = spawned['process']['pid']
        assert owned != unrelated.pid
        assert Path(f'/proc/{owned}').exists()
        agent.kill()
        agent.wait(timeout=2)
        ended = wait_for(lambda: (v := read())['status'] in ('UNKNOWN', 'FAILED') and v)
        assert ended['reason'] == 'agent_pipe_closed'
        assert ended['status'] == 'UNKNOWN'
        assert not Path(f'/proc/{owned}').exists()
        assert unrelated.poll() is None
    finally:
        if agent.poll() is None:
            agent.kill()
        agent.wait(timeout=2)
        unrelated.terminate()
        unrelated.wait(timeout=2)
