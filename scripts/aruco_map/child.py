"""A command in a session of its own, its output in a log file, stopped by process group.

The same helper as scripts/testing/lab2/lab2_run.py's Child, kept here so that
the ArUco map tools import nothing of Lab 2. play_launch gives its nodes process
groups of their own in the same session; stop() signals every group of the
session, escalating SIGINT -> SIGTERM -> SIGKILL, so no node is left orphaned.
"""

import os
import signal
import subprocess
import time
from pathlib import Path
from typing import List, Optional

REPO = Path(__file__).resolve().parents[2]


def session_pgids(sid: int) -> List[int]:
    """The process groups with a live process in session `sid` (from /proc)."""
    out = set()
    for p in Path('/proc').iterdir():
        if not p.name.isdigit():
            continue
        try:
            stat = (p / 'stat').read_text()
        except OSError:
            continue
        fields = stat[stat.rindex(')') + 2:].split()
        state, pgid, session = fields[0], int(fields[2]), int(fields[3])
        if session == sid and state != 'Z':
            out.add(pgid)
    return sorted(out)


class Child:
    """A command in a session of its own, its output in a log file."""

    def __init__(self, name: str, cmd: List[str], log: Path, env=None, cwd=REPO):
        self.name, self.cmd, self.log, self.env, self.cwd = name, list(cmd), Path(log), env, cwd
        self.proc: Optional[subprocess.Popen] = None

    def start(self):
        self.log.parent.mkdir(parents=True, exist_ok=True)
        f = open(self.log, 'w')
        f.write(' '.join(self.cmd) + '\n')
        f.flush()
        self.proc = subprocess.Popen(self.cmd, cwd=self.cwd, env=self.env, stdout=f,
                                     stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                     start_new_session=True)
        f.close()
        return self

    @property
    def pid(self) -> Optional[int]:
        return self.proc.pid if self.proc else None

    def alive(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def remaining(self) -> List[int]:
        """Process groups of this child's session still alive."""
        if self.proc is None:
            return []
        self.proc.poll()
        return session_pgids(self.proc.pid)

    def stop(self, steps=((signal.SIGINT, 20.0), (signal.SIGTERM, 5.0), (signal.SIGKILL, 3.0)),
             say=print) -> bool:
        """Signal every process group of the session, escalating; True once none is left."""
        if self.proc is None:
            return True
        prev_wait = 0.0
        for sig, wait in steps:
            groups = self.remaining()
            if not groups:
                return True
            if sig != steps[0][0]:
                say(f'  {self.name}: still running after {prev_wait:.0f} s, sending {sig.name}')
            # the leader's group first: play_launch shuts its nodes down on SIGINT itself
            for g in ([self.proc.pid] if sig == steps[0][0] and self.proc.pid in groups else groups):
                try:
                    os.killpg(g, sig)
                except (ProcessLookupError, PermissionError):
                    pass
            end = time.monotonic() + wait
            while time.monotonic() < end and self.remaining():
                time.sleep(0.2)
            prev_wait = wait
        return not self.remaining()
