"""Linux process ownership and durable JSON records for isolated workers."""
import ctypes
import json
import os
import resource
import signal
import subprocess
import sys
import tempfile


def write_json(path, value):
    fd, name = tempfile.mkstemp(dir=path.parent, prefix='.' + path.name)
    try:
        with os.fdopen(fd, 'w') as f:
            json.dump(value, f)
            f.flush()
            os.fsync(f.fileno())
        os.replace(name, path)
        fd = os.open(path.parent, os.O_DIRECTORY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)
    finally:
        if os.path.exists(name):
            os.unlink(name)


def spawn_owned(command, *, file_limit=0, **kwargs):
    # A fresh interpreter avoids running preexec_fn in a threaded ROS process.
    return subprocess.Popen(
        [sys.executable, '-m', __name__, str(os.getpid()), str(file_limit), *map(str, command)],
        start_new_session=True, **kwargs)


def stop_owned(process, grace=2):
    for sig, timeout in ((signal.SIGINT, grace), (signal.SIGTERM, 1), (signal.SIGKILL, 2)):
        if process.poll() is not None:
            return process.returncode
        process.send_signal(sig)
        try:
            return process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            continue
    raise TimeoutError('owned process exit not observed')


if __name__ == '__main__':
    # PR_SET_PDEATHSIG and then recheck PPID to close the fork-to-prctl race.
    if ctypes.CDLL(None, use_errno=True).prctl(1, signal.SIGKILL, 0, 0, 0):
        raise OSError(ctypes.get_errno(), 'PR_SET_PDEATHSIG')
    if os.getppid() != int(sys.argv[1]):
        sys.exit(125)
    if int(sys.argv[2]):
        limit = int(sys.argv[2])
        resource.setrlimit(resource.RLIMIT_FSIZE, (limit, limit))
    os.execvp(sys.argv[3], sys.argv[3:])
