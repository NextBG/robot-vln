"""Runs robot drivers as separate, individually restartable child processes.

Each module is its own `ros2 launch` / `ros2 run` process group, so one driver can be restarted
(e.g. after a battery swap drops the S1 / Mid-360 / USB devices) without touching the others.
Modules that exit on their own are restarted with back-off when `keep_alive` is set.
"""
import ctypes
import os
import shlex
import signal
import subprocess
import threading
import time

_PR_SET_PDEATHSIG = 1


def _die_with_parent():
    os.setsid()
    # a backgrounded launcher has SIGINT ignored; ros2 launch needs it for a clean shutdown
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    signal.signal(signal.SIGTERM, signal.SIG_DFL)
    try:
        ctypes.CDLL('libc.so.6', use_errno=True).prctl(_PR_SET_PDEATHSIG, signal.SIGTERM)
    except OSError:
        pass


class Module:
    def __init__(self, name, label, cmd, autostart=True, keep_alive=True):
        self.name, self.label, self.cmd = name, label, cmd
        self.autostart, self.keep_alive = autostart, keep_alive
        self.proc = None
        self.wanted = autostart
        self.started = 0.0
        self.spawns = 0
        self.last_exit = None
        self.busy = False
        self.backoff = 2.0
        self.next_try = 0.0


class Supervisor:
    def __init__(self, logger, log_dir):
        self.log = logger
        self.log_dir = log_dir
        os.makedirs(log_dir, exist_ok=True)
        self.modules = {}
        self.lock = threading.Lock()
        self.running = True
        threading.Thread(target=self._watch, daemon=True).start()

    def add(self, name, label, cmd, autostart=True, keep_alive=True):
        self.modules[name] = Module(name, label, cmd, autostart, keep_alive)

    # ------------------------------------------------------------------ process control
    def _spawn(self, m):
        logf = open(os.path.join(self.log_dir, f'{m.name}.log'), 'ab')
        logf.write(f'\n===== start {time.ctime()}: {m.cmd}\n'.encode())
        logf.flush()
        m.proc = subprocess.Popen(m.cmd if isinstance(m.cmd, list) else shlex.split(m.cmd),
                                  stdout=logf, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                  preexec_fn=_die_with_parent)
        logf.close()
        m.started = time.time()
        self.log.info(f'[{m.name}] started pid {m.proc.pid}')

    def _kill(self, m, timeout=6.0):
        p = m.proc
        if p is None or p.poll() is not None:
            m.proc = None
            return
        for sig, wait in ((signal.SIGINT, timeout), (signal.SIGTERM, 3.0), (signal.SIGKILL, 2.0)):
            try:
                os.killpg(p.pid, sig)
            except ProcessLookupError:
                break
            try:
                p.wait(wait)
                break
            except subprocess.TimeoutExpired:
                continue
        # ros2 launch children may outlive the launcher; make sure the group is gone
        try:
            os.killpg(p.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        m.last_exit = p.poll()
        m.proc = None
        self.log.info(f'[{m.name}] stopped (exit {m.last_exit})')

    def _restart_blocking(self, m):
        # never spawn here: PDEATHSIG is tied to the spawning *thread*, so only _watch spawns
        try:
            self._kill(m)
            m.backoff = 2.0
            m.next_try = time.time() + 1.0
        finally:
            m.busy = False

    def restart(self, name):
        names = list(self.modules) if name == 'all' else [name]
        out = []
        for n in names:
            m = self.modules.get(n)
            if m is None or m.busy or (name == 'all' and not m.wanted):
                continue
            m.wanted = True
            m.busy = True
            threading.Thread(target=self._restart_blocking, args=(m,), daemon=True).start()
            out.append(n)
        return out

    def start(self, name):
        m = self.modules.get(name)
        if m is None:
            return False
        m.wanted = True
        if m.proc is None and not m.busy:
            m.next_try = 0.0
        return True

    def stop(self, name):
        m = self.modules.get(name)
        if m is None or m.busy:
            return False
        m.wanted = False
        m.busy = True

        def run():
            try:
                self._kill(m)
            finally:
                m.busy = False
        threading.Thread(target=run, daemon=True).start()
        return True

    def shutdown(self):
        self.running = False
        threads = [threading.Thread(target=self._kill, args=(m,)) for m in self.modules.values()]
        for t in threads:
            t.start()
        for t in threads:
            t.join(12)

    def _watch(self):
        while self.running:
            now = time.time()
            for m in list(self.modules.values()):
                if m.busy:
                    continue
                if m.proc is not None and m.proc.poll() is not None:
                    m.last_exit = m.proc.returncode
                    self.log.warning(f'[{m.name}] exited with {m.last_exit}')
                    m.proc = None
                    m.next_try = now + m.backoff
                    m.backoff = min(30.0, m.backoff * 2)
                    if not m.keep_alive:
                        m.wanted = False
                if m.proc is None and m.wanted and now >= m.next_try:
                    try:
                        self._spawn(m)
                        m.spawns += 1
                    except OSError as e:
                        self.log.error(f'[{m.name}] spawn failed: {e}')
                        m.next_try = now + 10.0
                if m.proc is not None and now - m.started > 60:
                    m.backoff = 2.0
            time.sleep(0.5)

    def status(self):
        now = time.time()
        return {n: {'label': m.label, 'running': m.proc is not None and m.proc.poll() is None,
                    'wanted': m.wanted, 'busy': m.busy, 'restarts': max(0, m.spawns - 1),
                    'uptime': round(now - m.started) if m.proc is not None else None,
                    'last_exit': m.last_exit}
                for n, m in self.modules.items()}
