"""Run-folder discovery shared by the passive ROS and USB recorders."""
from datetime import datetime
import fcntl
from pathlib import Path

LOCK_NAME = '.ros_capture.lock'


def create_run(parent=None):
    """Create a run below the invocation directory and hold its active lock."""
    parent = Path.cwd() if parent is None else Path(parent).expanduser()
    folder = parent.resolve() / datetime.now().strftime('%Y-%m-%d-%H-%M-%S-%f')
    folder.mkdir(parents=True, exist_ok=False)
    lock = (folder / LOCK_NAME).open('w')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    return folder, lock


def is_active(folder):
    try:
        with (folder / LOCK_NAME).open('rb') as lock:
            try:
                fcntl.flock(lock, fcntl.LOCK_SH | fcntl.LOCK_NB)
            except BlockingIOError:
                return True
    except FileNotFoundError:
        pass
    return False


def usb_run(output=None, cwd=None):
    """Use an explicit run, or exactly one active run in/under the current folder."""
    if output is not None:
        folder = Path(output).expanduser().resolve(strict=True)
        if not folder.is_dir():
            raise ValueError('--output must be an existing recording directory')
        return folder
    parent = Path.cwd() if cwd is None else Path(cwd)
    candidates = [parent] + [p for p in parent.iterdir() if p.is_dir() and not p.is_symlink()]
    active = [p.resolve() for p in candidates if is_active(p)]
    if not active:
        raise ValueError('No active ROS capture in ' + str(parent) +
                         '. Start bench_capture.py from this same directory first, '
                         'and leave it running.')
    if len(active) != 1:
        raise ValueError('Multiple active ROS captures; select one with --output: ' +
                         ', '.join(str(p) for p in active))
    return active[0]
