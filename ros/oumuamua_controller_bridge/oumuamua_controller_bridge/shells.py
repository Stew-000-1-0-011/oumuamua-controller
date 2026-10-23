"""PTY bash shells with per-tab replay buffer and on-disk logging (ROS-free)."""
from __future__ import annotations

import errno
import fcntl
import os
import pty
import re
import resource
import select
import signal
import struct
import termios
import threading
import time
import traceback
import functools
from datetime import datetime
from pathlib import Path
from typing import Callable

NAME_RE = re.compile(r"[A-Za-z0-9_-]{1,32}")
ROWS, COLS = 24, 80
_FLUSH_CTRL = {"\x03", "\x04", "\x1a"}
MAX_PENDING_INPUT = 64 * 1024


def _locked(fn):
    @functools.wraps(fn)
    def wrapper(self, *a, **k):
        with self._lock:
            return fn(self, *a, **k)
    return wrapper


def _stamp() -> str:
    t = time.time()
    return time.strftime("%H:%M:%S", time.localtime(t)) + ".%03d" % int((t % 1) * 1000)


class _Tab:
    def __init__(self, pid: int, fd: int, logdir: Path, logfile: Path, log):
        self.pid = pid
        self.fd = fd
        self.logdir = logdir
        self.logfile = logfile
        self.log = log
        self.replay = bytearray()
        self.in_buf = ""
        self.out_buf = b""
        self.wbuf = bytearray()  # input the pty has not accepted yet (flushed by poll)
        self.dropped = False  # output was not forwarded (flood): resync once the link drains


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
        self._lock = threading.RLock()
        self._zombies: list[list] = []  # [pid, kill_at, give_up_at, killed]: reaped by poll()

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

    def _later_reap(self, pid: int, kill_after_s: float, give_up_s: float = 3.0) -> None:
        """Reap `pid` from poll() without waiting; SIGKILL it if it lingers."""
        now = time.monotonic()
        self._zombies.append([pid, now + kill_after_s, now + give_up_s, False])

    def _reap_step(self) -> None:
        if not self._zombies:
            return
        now = time.monotonic()
        keep = []
        for z in self._zombies:
            pid, kill_at, give_up, killed = z
            try:
                p, _ = os.waitpid(pid, os.WNOHANG)
            except ChildProcessError:
                continue
            if p:
                continue
            if now >= give_up:
                continue
            if now >= kill_at and not killed:
                for kill in (lambda: os.killpg(pid, signal.SIGKILL),
                             lambda: os.kill(pid, signal.SIGKILL)):
                    try:
                        kill()
                    except (ProcessLookupError, PermissionError):
                        pass
                z[3] = True
            keep.append(z)
        self._zombies = keep

    # ---- API -----------------------------------------------------------
    @_locked
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
        try:
            log = open(logfile, "a", encoding="utf-8", buffering=1)
        except OSError as e:
            return False, f"log file: {e}"
        try:
            maxfd = min(resource.getrlimit(resource.RLIMIT_NOFILE)[0], 1 << 16)
        except (ValueError, OSError):
            maxfd = 4096
        try:
            pid, fd = pty.fork()
        except OSError as e:
            log.close()
            return False, f"fork failed: {e}"
        if pid == 0:
            try:
                os.closerange(3, maxfd)
                fcntl.ioctl(0, termios.TIOCSWINSZ, struct.pack("HHHH", ROWS, COLS, 0, 0))
                os.execvpe("bash", argv, env)
            finally:
                os._exit(127)
        try:
            fcntl.ioctl(fd, termios.TIOCSWINSZ, struct.pack("HHHH", ROWS, COLS, 0, 0))
            fcntl.fcntl(fd, fcntl.F_SETFL, fcntl.fcntl(fd, fcntl.F_GETFL) | os.O_NONBLOCK)
        except OSError as e:
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            self._reap(pid, 1.0)
            os.close(fd)
            log.close()
            return False, f"pty setup failed: {e}"
        self._tabs[tab] = _Tab(pid, fd, logdir, logfile, log)
        return True, ""

    @_locked
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
        if t.wbuf:
            if len(t.wbuf) + len(data) > MAX_PENDING_INPUT:
                return False, "shell busy"
            t.wbuf += data
            return True, ""
        try:
            n = os.write(t.fd, data)
        except BlockingIOError:
            n = 0
        except OSError as e:
            return False, f"write failed: {e.strerror}"
        if n < len(data):
            rest = data[n:]
            if len(rest) > MAX_PENDING_INPUT:
                return False, "shell busy"
            t.wbuf += rest  # poll() writes it when the pty accepts more
        return True, ""

    def _flush_input(self, t: _Tab) -> None:
        try:
            n = os.write(t.fd, bytes(t.wbuf))
        except BlockingIOError:
            return
        except OSError:
            t.wbuf.clear()
            return
        del t.wbuf[:n]

    @_locked
    def close(self, tab: str) -> tuple[bool, str]:
        t = self._tabs.pop(tab, None)
        if t is None:
            return False, "no such tab"
        try:
            os.kill(t.pid, signal.SIGHUP)
        except ProcessLookupError:
            pass
        self._later_reap(t.pid, 0.5)  # SIGKILL from poll() if SIGHUP is not enough
        self._reap_step()
        self._finish(tab, t)
        return True, ""

    @_locked
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

    @_locked
    def replay(self, tab: str) -> bytes:
        t = self._tabs.get(tab)
        return bytes(t.replay) if t else b""

    @_locked
    def tabs(self) -> list[str]:
        return list(self._tabs)

    @_locked
    def cwd(self, tab: str) -> str:
        t = self._tabs.get(tab)
        if t is None:
            return ""
        try:
            return os.readlink(f"/proc/{t.pid}/cwd")
        except OSError:
            return ""

    @_locked
    def poll(self, timeout_s: float) -> None:
        self._reap_step()
        if not self._tabs:
            if timeout_s > 0:
                time.sleep(min(timeout_s, 0.05))
            return
        by_fd = {t.fd: name for name, t in self._tabs.items()}
        p = select.poll()
        for fd, name in by_fd.items():
            p.register(fd, select.POLLIN | (select.POLLOUT if self._tabs[name].wbuf else 0))
        try:
            ready = p.poll(max(0.0, timeout_s) * 1000)
        except OSError:
            return
        for fd, ev in ready:
            name = by_fd[fd]
            t = self._tabs.get(name)
            if t is None:
                continue
            if ev & select.POLLOUT and t.wbuf:
                self._flush_input(t)
            if not ev & ~select.POLLOUT:
                continue
            try:
                data = os.read(fd, 65536)
            except BlockingIOError:
                continue
            except OSError:
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
            traceback.print_exc()

    def _exited(self, name: str, t: _Tab) -> None:
        if self._tabs.pop(name, None) is None:
            return
        self._later_reap(t.pid, 0.3, 2.0)
        self._finish(name, t)
        try:
            self.on_closed(name, "exited")
        except Exception:
            traceback.print_exc()

    @_locked
    def close_all(self) -> None:
        for name in list(self._tabs):
            self.close(name)
        end = time.monotonic() + 3.5
        while self._zombies and time.monotonic() < end:
            self._reap_step()
            time.sleep(0.01)


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
