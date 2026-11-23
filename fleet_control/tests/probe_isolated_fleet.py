"""Drive the browser's operation API and verify owned artifacts on localhost."""
import argparse
import json
from pathlib import Path
import time
from urllib.request import Request, urlopen
import uuid


def run(args):
    root = Path(args.run_dir)
    token_file = root / 'secrets/operator.token'
    until = time.monotonic() + 10
    while not token_file.is_file() and time.monotonic() < until:
        time.sleep(.1)
    token = token_file.read_text().strip()
    base = f'http://127.0.0.1:{args.port}/api/v1'
    robot = '/robots/locked_bench'

    def call(path, data=None, binary=False):
        req = Request(base + path, None if data is None else json.dumps(data).encode(),
                      headers={'Authorization': 'Bearer ' + token, 'Content-Type': 'application/json'})
        with urlopen(req, timeout=3) as response:
            payload = response.read()
        return payload if binary else json.loads(payload)

    def wait(predicate, deadline=25):
        until = time.monotonic() + deadline
        while time.monotonic() < until:
            value = predicate()
            if value:
                return value
            time.sleep(.2)
        raise TimeoutError('expected evidence timed out')

    def live():
        try:
            state = call(robot)
        except OSError:
            return None
        return state if state['online'] and state.get('reconciled') else None

    wait(live)
    lease = call(robot + '/lock', dict(action='acquire', ttl_s=60))
    epoch = lease['epoch']

    def operation(kind, action, expected):
        call(robot + '/lock', dict(action='renew', ttl_s=60))
        path = robot + '/' + kind
        session = call(path)
        command = call(path, dict(action=action, session_id=session.get('session_id'),
                                  idempotency_key=uuid.uuid4().hex, epoch=epoch))

        def done():
            current = call(path)
            c = current.get('command') or {}
            if c.get('command_id') != command['command_id']:
                return None
            if current['state'] in ('UNKNOWN', 'FAILED') or c['status'] in ('FAILED', 'UNKNOWN'):
                raise AssertionError(current)
            return current if c['status'] == 'COMPLETED' and current['state'] == expected else None

        result = wait(done)
        print(kind, action, result['state'], result['result']['evidence'], flush=True)
        return result

    def velocity(mode=None):
        state = call(robot)['state']
        gate = state['safety_gate']
        assert gate['motor_path']['status'] == 'LOCKED', gate
        selected = state.get('velocity_arbiter') or {}
        assert selected.get('motor_output') is False, selected
        # G2: v5 Arbiter vocabulary. 'mode' is always MOCK_LOCKED; the selection is `source` (effective, NONE when the output is zero)
        # and `selected_source` (MANUAL / MAPPING / NAVIGATION). main's AUTO = MAPPING or NAVIGATION, IDLE = zero output.
        if mode is None:
            return selected
        ok = {'AUTO': selected.get('selected_source') == 'NAV2' and selected.get('ready') is True,
              'MANUAL': selected.get('selected_source') == 'MANUAL',
              'IDLE': selected.get('source') in ('NONE', 'MANUAL_ZERO')}[mode]
        return selected if ok else None

    def manual_cycle():
        path = robot + '/manual-bench'
        start = call(path, dict(action='start', epoch=epoch))
        observed = False
        try:
            for seq in range(50):
                result = call(path, dict(action='sample', input_format='normalized_axes_v1', epoch=epoch, session_id=start['session_id'],
                                         seq=seq, linear=.05, turn=.12, deadman=True))
                state = velocity('MANUAL')
                if state and state['selected_linear'] > 0:
                    observed = True
                    break
                time.sleep(.08)
            assert observed, 'manual source never selected'
            # Let the last command expire without a stop request or another sample.
            zero = wait(lambda: (state := velocity('IDLE')) and state['selected_linear'] == 0
                        and state['selected_turn'] == 0, deadline=4)
            assert zero
        finally:
            call(path, dict(action='stop', epoch=epoch, session_id=start['session_id']))
        print('velocity MANUAL selected and timed out with motor path LOCKED', flush=True)

    try:
        mapping = operation('mapping', 'START', 'MAPPING')
        bag = operation('bag', 'START', 'RECORDING')
        if args.check_velocity:
            wait(lambda: velocity('AUTO'), deadline=10)
            print('velocity AUTO with motor path LOCKED', flush=True)
        assert call(robot + '/map.png', binary=True).startswith(b'\x89PNG')
        for kind in ('rgb', 'depth'):
            assert call(robot + '/camera/' + kind + '.jpg', binary=True).startswith(b'\xff\xd8\xff')
        operation('mapping', 'PAUSE', 'PAUSED')
        saved = operation('mapping', 'SAVE', 'PAUSED')
        save_dir = root / 'agent/mapping_worker' / mapping['session_id'] / ('save-' + saved['command']['command_id'])      # G2: v5 MappingWorker layout
        assert all((save_dir / f).stat().st_size for f in ('map.yaml', 'map.pgm', 'graph.posegraph', 'graph.data'))
        operation('mapping', 'RESUME', 'MAPPING')
        operation('bag', 'STOP', 'STOPPED')
        assert (root / 'agent/bag_worker/bags' / bag['session_id'] / 'metadata.yaml').is_file()
        operation('mapping', 'STOP', 'STOPPED')
        if args.check_velocity:
            manual_cycle()
    finally:
        call(robot + '/lock', dict(action='release'))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dir')
    parser.add_argument('port', type=int)
    parser.add_argument('--check-velocity', action='store_true')
    run(parser.parse_args())
