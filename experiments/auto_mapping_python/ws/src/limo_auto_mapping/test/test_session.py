"""Supervisor failure-path checks with injected backend, no robot or Nav2."""
import threading
from types import SimpleNamespace as NS
import pytest
from limo_mapping_interfaces.action import AutoMapping
from limo_auto_mapping.server import MappingServer, SessionCancelled
from test_policy import grid, world


class Handle:
    def __init__(self, retry=True):
        self.request = NS(max_duration_sec=60.0, allow_margin_retry=retry,
                          output_prefix='/tmp/unused_mapping_test')
        self.is_cancel_requested = False
        self.state = None

    def abort(self):
        self.state = 'aborted'

    def succeed(self):
        self.state = 'succeeded'

    def canceled(self):
        self.state = 'canceled'


class Backend:
    """Methods absent here deliberately remain untested ROS service integrations."""
    def __init__(self):
        self.lock = threading.Lock()
        self.busy = True
        self.healthy = True
        raw = grid([[100]*3, [100, 0, 100], [100]*3])
        self.samples = {'map': (raw, 0)}
        self.position = world(raw, 4)
        self.events = []
        self.passes = 0

    def feedback(self, phase, retries):
        self.events.append(phase)

    def until(self, predicate, timeout):
        assert predicate()

    def ready(self):
        return True

    def nav_busy(self):
        return False

    def explore(self, retries):
        self.passes += 1
        self.events.append('explore_' + str(retries))
        return 'exploration_complete'

    def retry_margin(self):
        self.events.append('reduced')
        return True

    def restore(self):
        self.events.append('restored')

    def stop_explorer(self):
        self.events.append('stopped')

    def robot_pose(self, frame):
        return self.position

    def save(self, prefix):
        self.events.append('saved')


def test_success_restores_before_save_and_cleans_up():
    backend, handle = Backend(), Handle()
    result = MappingServer.execute(backend, handle)
    assert isinstance(result, AutoMapping.Result)
    assert result.success and result.margin_retried and handle.state == 'succeeded'
    assert backend.passes == 2
    assert backend.events.index('restored') < backend.events.index('saved')
    assert backend.events[-2:] == ['stopped', 'restored']
    assert not backend.busy


def test_cancel_during_reduced_pass_restores_without_save():
    backend, handle = Backend(), Handle()
    handle.is_cancel_requested = True
    def explore(retries):
        if retries:
            raise SessionCancelled('cancel during reduced pass')
        return 'exploration_complete'
    backend.explore = explore
    result = MappingServer.execute(backend, handle)
    assert handle.state == 'canceled' and not result.success
    assert result.margin_retried and 'saved' not in backend.events
    assert backend.events[-2:] == ['stopped', 'restored']


def test_restore_failure_latches_server_and_never_reports_success():
    backend, handle = Backend(), Handle()
    def fail():
        raise RuntimeError('parameter service unavailable')
    backend.restore = fail
    result = MappingServer.execute(backend, handle)
    assert handle.state == 'aborted' and not result.success and not backend.healthy
    assert 'saved' not in backend.events


def test_exhaustion_is_partial_even_with_no_raw_frontiers():
    backend, handle = Backend(), Handle(retry=False)
    backend.explore = lambda _: 'frontiers_exhausted'
    result = MappingServer.execute(backend, handle)
    assert not result.success and not result.exploration_complete
    assert handle.state == 'aborted' and 'saved' in backend.events


def test_remaining_frontier_does_not_claim_full_coverage():
    backend, handle = Backend(), Handle(retry=False)
    raw = grid([[100]*3, [100, 0, -1], [100]*3])
    backend.samples['map'] = (raw, 0)
    result = MappingServer.execute(backend, handle)
    assert not result.success and handle.state == 'aborted'
    assert 'saved' in backend.events
