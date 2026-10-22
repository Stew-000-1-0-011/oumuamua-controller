"""PTY bash shells with per-tab replay buffer and on-disk logging (ROS-free)."""
from __future__ import annotations

import errno
import fcntl
import os
import pty
import re
import select
import signal
import struct
import termios
import time
from datetime import datetime
from pathlib import Path
from typing import Callable

NAME_RE = re.compile(r"[A-Za-z0-9_-]{1,32}")
ROWS, COLS = 24, 80
_FLUSH_CTRL = {"\x03", "\x04", "\x1a"}


def _stamp() -> str:
    t = time.time()
    return time.strftime("%H:%M:%S", time.localtime(t)) + ".%03d" % int((t % 1) * 1000)


class _Tab:
    def __init__(self, pid: int, fd: int, logdir: Path, logfile: Path):
        self.pid = pid
        self.fd = fd
        self.logdir = logdir
        self.logfile = logfile
        self.log = open(logfile, "a", encoding="utf-8", buffering=1)
        self.replay = bytearray()
        self.in_buf = ""
        self.out_buf = b""


class ShellManager:
    def __init__(self, log_root: Path, on_output: Callable[[str, bytes], None],
                 on_closed: Callable[[str, str], None], replay_lines: int = 2000,
                 replay_bytes: int = 256 * 1024, env: dict | None = None):
        self.log_root = Path(log_root)
        self.on_output = on_output
        self.on_closed = on_closed
        self.replay_lines = replay_lines
        self.replay_bytes = replay_bytes
        self.env = env
        self._tabs: dict[str, _Tab] = {}

    # ---- helpers -------------------------------------------------------
    def _logline(self, tab: _Tab, prefix: str, text: str) -> None:
        try:
            tab.log.write(f"{_stamp()} {prefix}{text}\n")
        except (OSError, ValueError):
            pass

    def _trim(self, tab: _Tab) -> None:
        buf = tab.replay
        if len(buf) > self.replay_bytes:
            cut = len(buf) - self.replay_bytes
            nl = buf.find(b"\n", cut)
            del buf[: (nl + 1) if nl >= 0 else cut]
        extra = buf.count(b"\n") - self.replay_lines
        if extra > 0:
            pos = -1
            for _ in range(extra):
                pos = buf.find(b"\n", pos + 1)
            del buf[: pos + 1]

    def _finish(self, name: str, tab: _Tab) -> None:
        if tab.in_buf:
            self._logline(tab, "> ", tab.in_buf)
        if tab.out_buf:
            self._logline(tab, "  ", tab.out_buf.decode("utf-8", "replace").rstrip("\r"))
        try:
            tab.log.close()
        except OSError:
            pass
        try:
            os.close(tab.fd)
        except OSError:
            pass

    def _reap(self, pid: int, wait_s: float) -> bool:
        end = time.monotonic() + wait_s
        while True:
            try:
                p, _ = os.waitpid(pid, os.WNOHANG)
            except ChildProcessError:
                return True
            if p:
                return True
            if time.monotonic() >= end:
                return False
            time.sleep(0.01)

    # ---- API -----------------------------------------------------------
    def open(self, tab: str) -> tuple[bool, str]:
        if not isinstance(tab, str) or not NAME_RE.fullmatch(tab):
            return False, "invalid tab name"
        if tab in self._tabs:
            return False, "tab exists"
        logdir = self.log_root / tab
        try:
            logdir.mkdir(parents=True, exist_ok=True)
        except OSError as e:
            return False, f"log dir: {e}"
        logfile = logdir / (datetime.now().strftime("%Y%m%d-%H%M%S") + ".log")
        if self.env is None:
            env = dict(os.environ)
            argv = ["bash", "-i"]
        else:
            env = dict(self.env)
            argv = ["bash", "--norc", "--noprofile", "-i"]
        env["TERM"] = "xterm-256color"
        pid, fd = pty.fork()
        if pid == 0:
            try:
                fcntl.ioctl(0, termios.TIOCSWINSZ, struct.pack("HHHH", ROWS, COLS, 0, 0))
                os.execvpe("bash", argv, env)
            finally:
                os._exit(127)
        fcntl.ioctl(fd, termios.TIOCSWINSZ, struct.pack("HHHH", ROWS, COLS, 0, 0))
        fcntl.fcntl(fd, fcntl.F_SETFL, fcntl.fcntl(fd, fcntl.F_GETFL) | os.O_NONBLOCK)
        self._tabs[tab] = _Tab(pid, fd, logdir, logfile)
        return True, ""

    def write(self, tab: str, data: bytes) -> tuple[bool, str]:
        t = self._tabs.get(tab)
        if t is None:
            return False, "no such tab"
        # log input
        for ch in _render_input_chunks(data):
            if ch == "\n":
                self._logline(t, "> ", t.in_buf)
                t.in_buf = ""
            else:
                t.in_buf += ch
                if ch in ("^C", "^D", "^Z"):
                    self._logline(t, "> ", t.in_buf)
                    t.in_buf = ""
        view = memoryview(data)
        deadline = time.monotonic() + 0.5
        while len(view):
            try:
                n = os.write(t.fd, view)
                view = view[n:]
            except BlockingIOError:
                if time.monotonic() > deadline:
                    return False, "shell busy"
                select.select([], [t.fd], [], 0.05)
            except OSError as e:
                return False, f"write failed: {e.strerror}"
        return True, ""

    def close(self, tab: str) -> tuple[bool, str]:
        t = self._tabs.pop(tab, None)
        if t is None:
            return False, "no such tab"
        try:
            os.kill(t.pid, signal.SIGHUP)
        except ProcessLookupError:
            pass
        if not self._reap(t.pid, 0.5):
            for kill in (lambda: os.killpg(t.pid, signal.SIGKILL),
                         lambda: os.kill(t.pid, signal.SIGKILL)):
                try:
                    kill()
                except (ProcessLookupError, PermissionError):
                    pass
            self._reap(t.pid, 2.0)
        self._finish(tab, t)
        return True, ""

    def rename(self, tab: str, new: str) -> tuple[bool, str]:
        t = self._tabs.get(tab)
        if t is None:
            return False, "no such tab"
        if not isinstance(new, str) or not NAME_RE.fullmatch(new):
            return False, "invalid tab name"
        if new in self._tabs:
            return False, "tab exists"
        newdir = self.log_root / new
        try:
            newdir.mkdir(parents=True, exist_ok=True)
            newfile = newdir / t.logfile.name
            if newfile.exists():
                newfile = newdir / (datetime.now().strftime("%Y%m%d-%H%M%S") + "-r.log")
            t.log.flush()
            t.log.close()
            os.replace(t.logfile, newfile)
            t.log = open(newfile, "a", encoding="utf-8", buffering=1)
            try:
                t.logdir.rmdir()
            except OSError:
                pass
            t.logdir, t.logfile = newdir, newfile
        except OSError as e:
            return False, f"rename failed: {e}"
        del self._tabs[tab]
        self._tabs[new] = t
        return True, ""

    def replay(self, tab: str) -> bytes:
        t = self._tabs.get(tab)
        return bytes(t.replay) if t else b""

    def tabs(self) -> list[str]:
        return list(self._tabs)

    def cwd(self, tab: str) -> str:
        t = self._tabs.get(tab)
        if t is None:
            return ""
        try:
            return os.readlink(f"/proc/{t.pid}/cwd")
        except OSError:
            return ""

    def poll(self, timeout_s: float) -> None:
        if not self._tabs:
            if timeout_s > 0:
                time.sleep(min(timeout_s, 0.05))
            return
        by_fd = {t.fd: name for name, t in self._tabs.items()}
        try:
            ready, _, _ = select.select(list(by_fd), [], [], max(0.0, timeout_s))
        except (OSError, ValueError):
            return
        for fd in ready:
            name = by_fd[fd]
            t = self._tabs.get(name)
            if t is None:
                continue
            try:
                data = os.read(fd, 65536)
            except BlockingIOError:
                continue
            except OSError as e:
                if e.errno not in (errno.EIO, errno.EBADF):
                    pass
                data = b""
            if not data:
                self._exited(name, t)
                continue
            self._output(name, t, data)

    def _output(self, name: str, t: _Tab, data: bytes) -> None:
        t.replay += data
        self._trim(t)
        buf = t.out_buf + data
        *lines, t.out_buf = buf.split(b"\n")
        for ln in lines:
            self._logline(t, "  ", ln.decode("utf-8", "replace").rstrip("\r"))
        try:
            self.on_output(name, data)
        except Exception:
            pass

    def _exited(self, name: str, t: _Tab) -> None:
        del self._tabs[name]
        if not self._reap(t.pid, 0.3):
            try:
                os.kill(t.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            self._reap(t.pid, 1.0)
        self._finish(name, t)
        try:
            self.on_closed(name, "exited")
        except Exception:
            pass

    def close_all(self) -> None:
        for name in list(self._tabs):
            self.close(name)


def _render_input_chunks(data: bytes):
    """Yield display tokens: printable chars, '^X' for control bytes, '\\n' for newline."""
    for ch in data.decode("utf-8", "replace"):
        o = ord(ch)
        if ch == "\n":
            yield "\n"
        elif o == 0x7F:
            yield "^?"
        elif o < 0x20:
            yield "^" + chr(o + 64)
        else:
            yield ch
