#!/usr/bin/env python3
"""
Combined static-file server + local control-plane for the Stable-Bot GUI.

Replaces `python3 -m http.server 8080 --directory ...` — serves the same
HTML/JS, but also exposes POST endpoints the browser can hit when the
ROS stack is dead (i.e., when rosbridge itself is unreachable, so the
usual /control_cmd path doesn't work):

  POST /reset        → runs reset_stewart_stack.py (sync, ~3-5 s)
  POST /launch       → spawns `ros2 launch stewart_bringup stewart_gui_launch.py`
                        detached, with stdout/stderr to
                        ~/ros2_ws/src/stewart_bringup/logs/last_launch.log
  POST /stop_launch  → SIGINTs the launch subprocess we spawned
  GET  /launch_status → {"running": bool, "pid": N or null, "log_tail": "..."}

Everything listens on localhost only. No ROS sourcing is required to run
this (it shells out to /opt/ros/kilted/bin/ros2 via the reset script).

Usage:
  python3 gui_server.py
  python3 gui_server.py --port 8080 --web-dir ~/ros2_ws/src/stewart_bringup/web
"""
import argparse
import http.server
import json
import os
import signal
import socketserver
import subprocess
import sys
import threading
import time

def _find_stewart_bringup_dir():
    for cand in ('~/ros2_ws/src/stewart_bringup',
                 '~/ros2_ws/src/stable_bot/stewart_bringup'):
        p = os.path.expanduser(cand)
        if os.path.isfile(os.path.join(p, 'package.xml')):
            return p
    return os.path.expanduser('~/ros2_ws/src/stewart_bringup')
_BRINGUP_DIR = _find_stewart_bringup_dir()
DEFAULT_WEB_DIR = os.path.join(_BRINGUP_DIR, 'web')
RESET_SCRIPT = os.path.join(_BRINGUP_DIR, 'scripts/reset_stewart_stack.py')
LAUNCH_LOG = os.path.join(_BRINGUP_DIR, 'logs/last_launch.log')

# Global launch subprocess state (guarded by a lock)
_launch_lock = threading.Lock()
_launch_proc = None


def _rosbridge_already_running():
    """True if anything is listening on TCP 9090 (rosbridge default port).
    Catches stacks started outside this gui_server (notably systemd on Pi)."""
    import socket
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(0.3)
    try:
        s.connect(('127.0.0.1', 9090))
        return True
    except Exception:
        return False
    finally:
        s.close()


def _run_launch():
    """Spawn `ros2 launch stewart_bringup stewart_gui_launch.py` detached.
    Output goes to LAUNCH_LOG. Returns the Popen or raises."""
    global _launch_proc
    with _launch_lock:
        if _launch_proc is not None and _launch_proc.poll() is None:
            return _launch_proc   # already running
        if _rosbridge_already_running():
            raise RuntimeError(
                "rosbridge already listening on :9090 (probably started by "
                "systemd). Refusing to spawn a duplicate stack — use "
                "`sudo systemctl restart stable_bot.service` instead.")
        os.makedirs(os.path.dirname(LAUNCH_LOG), exist_ok=True)
        log_f = open(LAUNCH_LOG, 'w')
        # Use bash -c so we can source ROS before launching. PATH may not
        # include ros2 if the user didn't source in the env that started
        # this HTTP server.
        cmd = (
            'source /opt/ros/kilted/setup.bash && '
            'source ~/ros2_ws/install/local_setup.bash && '
            'exec ros2 launch stewart_bringup stewart_gui_launch.py'
        )
        _launch_proc = subprocess.Popen(
            ['bash', '-c', cmd],
            stdout=log_f, stderr=subprocess.STDOUT,
            start_new_session=True,  # so Ctrl-C on this server doesn't kill it
        )
        return _launch_proc


def _stop_launch():
    global _launch_proc
    with _launch_lock:
        if _launch_proc is None or _launch_proc.poll() is not None:
            return False, "no launch process tracked"
        try:
            os.killpg(os.getpgid(_launch_proc.pid), signal.SIGINT)
            try:
                _launch_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(_launch_proc.pid), signal.SIGKILL)
                _launch_proc.wait(timeout=2)
            pid = _launch_proc.pid
            _launch_proc = None
            return True, f"stopped pid {pid}"
        except Exception as e:
            return False, f"stop failed: {e}"


def _launch_status():
    pid = None
    running = False
    with _launch_lock:
        if _launch_proc is not None and _launch_proc.poll() is None:
            running = True
            pid = _launch_proc.pid
    log_tail = ""
    try:
        with open(LAUNCH_LOG, 'r') as f:
            log_tail = f.read()[-4000:]
    except Exception:
        pass
    return {'running': running, 'pid': pid, 'log_tail': log_tail}


class Handler(http.server.SimpleHTTPRequestHandler):
    web_dir = DEFAULT_WEB_DIR

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=self.web_dir, **kwargs)

    def _send_json(self, obj, status=200):
        data = json.dumps(obj).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'POST, GET, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()

    def do_GET(self):
        if self.path == '/launch_status':
            self._send_json(_launch_status())
            return
        return super().do_GET()

    def do_POST(self):
        if self.path == '/reset':
            try:
                r = subprocess.run(
                    ['python3', RESET_SCRIPT],
                    capture_output=True, text=True, timeout=90)
                self._send_json({
                    'rc': r.returncode,
                    'stdout': r.stdout[-4000:],
                    'stderr': r.stderr[-1000:],
                })
            except subprocess.TimeoutExpired:
                self._send_json({'rc': -1, 'stdout': '',
                                 'stderr': 'reset timed out after 30 s'}, 500)
            except Exception as e:
                self._send_json({'rc': -2, 'stdout': '',
                                 'stderr': f'error: {e}'}, 500)
            return
        if self.path == '/launch':
            try:
                p = _run_launch()
                self._send_json({'pid': p.pid, 'log': LAUNCH_LOG})
            except Exception as e:
                self._send_json({'error': str(e)}, 500)
            return
        if self.path == '/stop_launch':
            ok, msg = _stop_launch()
            self._send_json({'ok': ok, 'message': msg})
            return
        self.send_error(404)

    # Quieter logging
    def log_message(self, fmt, *args):
        sys.stderr.write(f"[{self.log_date_time_string()}] {fmt % args}\n")


class ReusingServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--port', type=int, default=8080)
    p.add_argument('--host', default='127.0.0.1',
                   help='bind address (default localhost only — do NOT expose '
                        'this to the network, the endpoints can run arbitrary '
                        'commands)')
    p.add_argument('--web-dir', default=DEFAULT_WEB_DIR)
    args = p.parse_args()

    Handler.web_dir = args.web_dir
    srv = ReusingServer((args.host, args.port), Handler)
    print(f"serving {args.web_dir} on http://{args.host}:{args.port}")
    print(f"endpoints: GET /launch_status, POST /reset, POST /launch, POST /stop_launch")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nshutting down")
    finally:
        srv.server_close()


if __name__ == '__main__':
    main()
