#!/usr/bin/env python3
"""profile_system.py — whole-stack resource profiler for the stable_bot Pi.

Answers the one question the demo/bench digests DON'T: "where is the CPU
actually going, per ROS node?" The digests capture see->move *latency* and
*aggregate* host stats (system_stats.jsonl), but never per-node attribution.
This samples the running stack for a window and produces a ranked,
git-pushable report so the cost of each node is 100% understood from data.

Run on the Pi (as the `sorak` user — the py-spy step elevates with sudo):

    python3 ~/stable_bot_repo/stewart_bringup/scripts/profile_system.py \
        --duration 25 --label demo2 --pyspy --push

For the most useful read, run it TWICE and compare:
    * once on the idle booted stack   (--label idle)
    * once during a demo2 / bench run (--label demo2)
and pass --compare <old profile.json> on the second run to get a per-node
delta column (this is how you prove the depth fix — or any change — helped).

Outputs to  tuning_data/system_profiles/<UTC>_<label>/ :
    profile.json   full machine-readable record (schema_version 1)
    profile.md     ranked human summary — the grep-at-a-glance artifact

Pure stdlib + /proc + vcgencmd: no numpy / rclpy / psutil dependency, so it
runs even if the ROS overlay isn't sourced (only --topics needs `ros2`).
Output lives under tuning_data/ on purpose: pi_deploy.sh's allowlist
(SAFE_RE) auto-commits that path, so a fresh profile never blocks a deploy.
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import re
import shutil
import subprocess
import sys
import threading
import time
from collections import defaultdict
from datetime import datetime, timezone

SCHEMA_VERSION = 1

# Pi 5 firmware throttle bits (vcgencmd get_throttled). Same decode the
# digests/gui banner use — kept inline so this script is self-contained.
THROTTLE_BITS = [
    (0x1,     'under_voltage_now'),
    (0x2,     'arm_freq_capped_now'),
    (0x4,     'throttled_now'),
    (0x8,     'soft_temp_limit_now'),
    (0x10000, 'under_voltage_since_boot'),
    (0x20000, 'arm_freq_capped_since_boot'),
    (0x40000, 'throttled_since_boot'),
    (0x80000, 'soft_temp_limit_since_boot'),
]

# Critical-path topics measured when --topics is given. Anything not
# currently publishing simply reports rate 0.0 — harmless.
DEFAULT_TOPICS = [
    '/oak/latency_ms', '/oak/ball/v0/rgb_pixel', '/ball_xy_mono',
    '/ball_state', '/platform/imu/data', '/ball_track/diagnostic',
    '/status', '/oak/health',
]


# --------------------------------------------------------------------------
# /proc sampling — no external deps
# --------------------------------------------------------------------------

def _clk_tck() -> float:
    try:
        return float(os.sysconf('SC_CLK_TCK'))
    except Exception:
        return 100.0


def _ncpu() -> int:
    return os.cpu_count() or 4


def _read_total_cpu():
    """Aggregate + per-core jiffie counters from /proc/stat.

    Returns {'cpu': (idle, iowait, total), 'cpu0': (...), ...}.
    """
    out = {}
    try:
        with open('/proc/stat') as f:
            for line in f:
                if not line.startswith('cpu'):
                    break
                parts = line.split()
                key = parts[0]
                vals = [int(x) for x in parts[1:]]
                # user nice system idle iowait irq softirq steal ...
                idle = vals[3]
                iowait = vals[4] if len(vals) > 4 else 0
                total = sum(vals)
                out[key] = (idle, iowait, total)
    except Exception:
        pass
    return out


def _proc_cmdline(pid):
    try:
        with open(f'/proc/{pid}/cmdline', 'rb') as f:
            raw = f.read()
        return [a.decode('utf-8', 'replace') for a in raw.split(b'\x00') if a]
    except Exception:
        return []


def _proc_comm(pid):
    try:
        with open(f'/proc/{pid}/comm') as f:
            return f.read().strip()
    except Exception:
        return '?'


def _node_label(pid):
    """Human node name for a pid: prefer the ROS __node:= remap, then the
    install lib/<pkg>/<exe> basename, then known specials, then argv0."""
    argv = _proc_cmdline(pid)
    if not argv:
        return f'[{_proc_comm(pid)}]'          # kernel thread / no cmdline
    joined = ' '.join(argv)
    m = re.search(r'__node:=([\w/]+)', joined)
    if m:
        return m.group(1).lstrip('/')
    # ros2 launch-spawned node executable: .../lib/<pkg>/<exe>
    for a in argv:
        m = re.search(r'/lib/[\w]+/([\w]+)$', a)
        if m:
            return m.group(1)
    low = joined.lower()
    if 'gui_server.py' in low:
        return 'gui_server'
    if 'rosbridge' in low:
        return 'rosbridge'
    if 'ros2' in low and ' bag' in low + ' ':
        return 'ros2 bag record'
    if '_ros2_daemon' in low or 'ros2 daemon' in low:
        return 'ros2 daemon'
    # Fall back to argv0 basename, plus a .py hint if it's an interpreter.
    base = os.path.basename(argv[0])
    if base.startswith('python') and len(argv) > 1:
        for a in argv[1:]:
            if a.endswith('.py'):
                return os.path.basename(a)
    return base


def _proc_cpu_ticks(pid):
    """(utime+stime) jiffies from /proc/<pid>/stat, robust to spaces/parens
    in the comm field (everything after the last ')')."""
    try:
        with open(f'/proc/{pid}/stat') as f:
            data = f.read()
        after = data[data.rfind(')') + 2:].split()
        # after[0]=state(field3); utime=field14 -> idx11, stime=field15 -> idx12
        return int(after[11]) + int(after[12])
    except Exception:
        return None


def _proc_rss_mb(pid):
    try:
        with open(f'/proc/{pid}/status') as f:
            for line in f:
                if line.startswith('VmRSS:'):
                    return int(line.split()[1]) / 1024.0   # kB -> MB
    except Exception:
        pass
    return 0.0


def _proc_threads(pid):
    try:
        return len(os.listdir(f'/proc/{pid}/task'))
    except Exception:
        return 0


def _proc_is_python(pid):
    argv = _proc_cmdline(pid)
    return bool(argv) and os.path.basename(argv[0]).startswith('python')


def _all_pids():
    return [int(p) for p in os.listdir('/proc') if p.isdigit()]


def _snapshot_proc_ticks():
    """pid -> utime+stime ticks, for every readable process, right now."""
    snap = {}
    for pid in _all_pids():
        t = _proc_cpu_ticks(pid)
        if t is not None:
            snap[pid] = t
    return snap


# --------------------------------------------------------------------------
# vcgencmd / firmware
# --------------------------------------------------------------------------

def _vcgencmd(*args):
    try:
        r = subprocess.run(['vcgencmd', *args],
                           capture_output=True, text=True, timeout=3)
        return r.stdout.strip()
    except Exception:
        return ''


def _read_throttled():
    s = _vcgencmd('get_throttled')          # throttled=0x0
    m = re.search(r'0x[0-9a-fA-F]+', s)
    if not m:
        return None, {}
    val = int(m.group(0), 16)
    flags = {name: bool(val & bit) for bit, name in THROTTLE_BITS}
    return val, flags


def _read_arm_mhz():
    s = _vcgencmd('measure_clock', 'arm')   # frequency(0)=2400000000
    m = re.search(r'=(\d+)', s)
    return (int(m.group(1)) / 1e6) if m else None


def _read_temp_c():
    s = _vcgencmd('measure_temp')           # temp=58.0'C
    m = re.search(r'=([\d.]+)', s)
    if m:
        return float(m.group(1))
    try:
        with open('/sys/class/thermal/thermal_zone0/temp') as f:
            return int(f.read().strip()) / 1000.0
    except Exception:
        return None


def _read_mem():
    info = {}
    try:
        with open('/proc/meminfo') as f:
            for line in f:
                k, _, v = line.partition(':')
                info[k] = int(v.split()[0])    # kB
    except Exception:
        return {}
    total = info.get('MemTotal', 0) / 1024.0
    avail = info.get('MemAvailable', 0) / 1024.0
    return {'total_mb': round(total, 1),
            'used_mb': round(total - avail, 1),
            'avail_mb': round(avail, 1)}


def _depth_subsystem_state():
    """Best-effort: was OAK depth ENABLED/DISABLED at boot? (Confirms the
    depth-map fix is actually in effect.) Reads the latest journal banner."""
    try:
        r = subprocess.run(
            ['bash', '-c',
             "journalctl -u stable_bot.service --no-pager 2>/dev/null "
             "| grep 'Depth subsystem' | tail -1"],
            capture_output=True, text=True, timeout=8)
        line = r.stdout.strip()
        if 'ENABLED' in line:
            return 'ENABLED'
        if 'DISABLED' in line:
            return 'DISABLED'
    except Exception:
        pass
    return 'unknown'


# --------------------------------------------------------------------------
# py-spy drill-down
# --------------------------------------------------------------------------

def _find_pyspy():
    for cand in (shutil.which('py-spy'),
                 os.path.expanduser('~/.local/bin/py-spy'),
                 '/usr/local/bin/py-spy', '/usr/bin/py-spy'):
        if cand and os.path.isfile(cand):
            return cand
    return None


def _sudo_ok():
    """True if sudo runs non-interactively (so we won't hang on a prompt)."""
    try:
        r = subprocess.run(['sudo', '-n', 'true'],
                           capture_output=True, timeout=5)
        return r.returncode == 0
    except Exception:
        return False


def _pyspy_record(pyspy, pid, duration, rate=100):
    """Launch `py-spy record --format raw` (folded stacks) on pid, returning
    the Popen + temp path. --nonblocking so we don't pause a control node."""
    tmp = f'/tmp/pyspy_{pid}_{int(time.time())}.txt'
    cmd = ['sudo', '-n', pyspy, 'record', '--pid', str(pid),
           '--duration', str(int(duration)), '--rate', str(rate),
           '--format', 'raw', '--nonblocking', '--subprocesses',
           '--output', tmp]
    try:
        p = subprocess.Popen(cmd, stdout=subprocess.DEVNULL,
                             stderr=subprocess.PIPE)
        return p, tmp
    except Exception:
        return None, tmp


def _parse_folded(path, top_n=12):
    """Aggregate folded stacks into top own-time leaf functions.

    Each line is `frame;frame;...;leaf  <count>`. The leaf is the on-CPU
    function for those samples, so summing per-leaf gives %Own (what the
    interactive `py-spy top` Own column shows)."""
    own = defaultdict(int)
    total = 0
    try:
        with open(path) as f:
            for line in f:
                line = line.rstrip('\n')
                if not line:
                    continue
                stack, _, cnt = line.rpartition(' ')
                if not cnt.isdigit():
                    continue
                c = int(cnt)
                total += c
                leaf = stack.split(';')[-1].strip()
                if leaf:
                    own[leaf] += c
    except Exception:
        return {'total_samples': 0, 'top': []}
    ranked = sorted(own.items(), key=lambda kv: kv[1], reverse=True)[:top_n]
    top = [{'function': fn,
            'own_pct': round(100.0 * c / total, 1) if total else 0.0,
            'samples': c} for fn, c in ranked]
    return {'total_samples': total, 'top': top}


# --------------------------------------------------------------------------
# topic Hz (optional; needs `ros2` sourced)
# --------------------------------------------------------------------------

def _topic_hz(topic, window_s):
    """Run `ros2 topic hz` for window_s, terminate, parse 'average rate'."""
    try:
        p = subprocess.Popen(
            ['ros2', 'topic', 'hz', topic, '--window', '50'],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
    except Exception:
        return None
    time.sleep(window_s)
    try:
        p.terminate()
        out, _ = p.communicate(timeout=5)
    except Exception:
        try:
            p.kill()
        except Exception:
            pass
        out = ''
    m = re.findall(r'average rate:\s*([\d.]+)', out or '')
    return float(m[-1]) if m else 0.0


def _measure_topics(topics, window_s):
    results = {}
    threads = []

    def worker(t):
        results[t] = _topic_hz(t, window_s)

    for t in topics:
        th = threading.Thread(target=worker, args=(t,))
        th.start()
        threads.append(th)
    for th in threads:
        th.join()
    return results


# --------------------------------------------------------------------------
# main
# --------------------------------------------------------------------------

def _repo_dir():
    # <repo>/stewart_bringup/scripts/profile_system.py -> <repo>
    here = os.path.abspath(__file__)
    return os.path.dirname(os.path.dirname(os.path.dirname(here)))


def _git_sha(repo):
    try:
        r = subprocess.run(['git', '-C', repo, 'rev-parse', '--short', 'HEAD'],
                           capture_output=True, text=True, timeout=10)
        return r.stdout.strip() or 'unknown'
    except Exception:
        return 'unknown'


def _collect(duration, ncpu, clk, sample_system_every=1.0):
    """Run the attribution window: snapshot proc ticks + /proc/stat at the
    ends, sampling the system aggregate ~1 Hz in between."""
    sys_series = []
    procs0 = _snapshot_proc_ticks()
    cpu0 = _read_total_cpu()
    t0 = time.monotonic()

    deadline = t0 + duration
    while time.monotonic() < deadline:
        time.sleep(min(sample_system_every, max(0.0, deadline - time.monotonic())))
        val, flags = _read_throttled()
        sys_series.append({
            'load1': os.getloadavg()[0],
            'arm_mhz': _read_arm_mhz(),
            'temp_c': _read_temp_c(),
            'throttled_hex': (hex(val) if val is not None else None),
            'throttled_flags': flags,
        })

    procs1 = _snapshot_proc_ticks()
    cpu1 = _read_total_cpu()
    t1 = time.monotonic()
    dt = max(1e-6, t1 - t0)

    # Per-process CPU over the window (only pids present at both ends).
    rows = []
    for pid, ticks1 in procs1.items():
        if pid not in procs0:
            continue
        d_ticks = ticks1 - procs0[pid]
        if d_ticks <= 0:
            continue
        cores = d_ticks / clk / dt          # cores used (top %CPU / 100)
        rows.append({
            'pid': pid,
            'node': _node_label(pid),
            'pct_core': round(cores * 100.0, 1),    # top-style (>100 = multi-core)
            'pct_machine': round(cores / ncpu * 100.0, 1),
            'rss_mb': round(_proc_rss_mb(pid), 1),
            'threads': _proc_threads(pid),
            'is_python': _proc_is_python(pid),
        })
    rows.sort(key=lambda r: r['pct_core'], reverse=True)

    # System busy / iowait from the aggregate /proc/stat delta.
    def busy_iowait(a, b):
        if 'cpu' not in a or 'cpu' not in b:
            return None, None
        i0, w0, t_0 = a['cpu']
        i1, w1, t_1 = b['cpu']
        dtot = (t_1 - t_0) or 1
        busy = (dtot - (i1 - i0) - (w1 - w0)) / dtot * 100.0
        iowait = (w1 - w0) / dtot * 100.0
        return round(busy, 1), round(iowait, 1)

    busy, iowait = busy_iowait(cpu0, cpu1)
    return rows, sys_series, {'dt_s': round(dt, 1),
                              'busy_pct': busy, 'iowait_pct': iowait}


def _series_stat(series, key):
    vals = [s[key] for s in series if s.get(key) is not None]
    if not vals:
        return None
    return {'min': round(min(vals), 1),
            'mean': round(sum(vals) / len(vals), 1),
            'max': round(max(vals), 1)}


def _build_report(args, repo):
    ncpu = _ncpu()
    clk = _clk_tck()
    pyspy = _find_pyspy() if args.pyspy else None

    # If --pyspy: a quick 2 s pre-rank to pick the hottest PYTHON nodes, then
    # start py-spy records that overlap the main window (same load).
    pyspy_procs = []
    if args.pyspy:
        if pyspy is None:
            print('  [pyspy] py-spy not found — skipping drill-down '
                  '(pipx install py-spy)', file=sys.stderr)
        elif not _sudo_ok():
            print('  [pyspy] sudo needs a password — run `sudo -v` first, '
                  'then re-run. Skipping drill-down.', file=sys.stderr)
            pyspy = None
        else:
            pre0 = _snapshot_proc_ticks()
            time.sleep(2.0)
            pre1 = _snapshot_proc_ticks()
            ranked = []
            for pid, t1 in pre1.items():
                if pid in pre0 and _proc_is_python(pid):
                    d = t1 - pre0[pid]
                    if d > 0:
                        ranked.append((d, pid))
            ranked.sort(reverse=True)
            dur = min(args.duration, 20)
            for _, pid in ranked[:args.pyspy_top]:
                p, tmp = _pyspy_record(pyspy, pid, dur)
                if p is not None:
                    pyspy_procs.append({'pid': pid, 'node': _node_label(pid),
                                        'proc': p, 'tmp': tmp})

    # Optional topic Hz, measured concurrently with the main window.
    topic_result_holder = {}
    topic_thread = None
    if args.topics:
        if shutil.which('ros2') is None:
            print('  [topics] `ros2` not on PATH (source the overlay) — '
                  'skipping topic Hz.', file=sys.stderr)
        else:
            tlist = args.topics_list.split(',') if args.topics_list else DEFAULT_TOPICS

            def _tw():
                topic_result_holder['hz'] = _measure_topics(
                    tlist, min(args.duration, 8))
            topic_thread = threading.Thread(target=_tw)
            topic_thread.start()

    print(f'  sampling for {args.duration}s ...', file=sys.stderr)
    rows, sys_series, agg = _collect(args.duration, ncpu, clk)

    # Join py-spy records + parse.
    pyspy_out = []
    for pp in pyspy_procs:
        try:
            pp['proc'].wait(timeout=args.duration + 30)
        except Exception:
            try:
                pp['proc'].kill()
            except Exception:
                pass
        parsed = _parse_folded(pp['tmp'])
        pyspy_out.append({'node': pp['node'], 'pid': pp['pid'], **parsed})
        if not args.keep_raw:
            try:
                os.remove(pp['tmp'])
            except Exception:
                pass
    if topic_thread is not None:
        topic_thread.join()

    tracked_core = round(sum(r['pct_core'] for r in rows) / 100.0, 2)
    last_flags = sys_series[-1]['throttled_flags'] if sys_series else {}
    report = {
        'schema_version': SCHEMA_VERSION,
        'timestamp_utc': datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'),
        'label': args.label,
        'git_sha': _git_sha(repo),
        'depth_subsystem': _depth_subsystem_state(),
        'host': {
            'ncpu': ncpu, 'duration_s': agg['dt_s'],
            'busy_pct': agg['busy_pct'], 'iowait_pct': agg['iowait_pct'],
            'load1': _series_stat(sys_series, 'load1'),
            'arm_mhz': _series_stat(sys_series, 'arm_mhz'),
            'temp_c': _series_stat(sys_series, 'temp_c'),
            'throttled_flags_final': last_flags,
            'mem': _read_mem(),
            'tracked_cores_used': tracked_core,
            'verdict': _verdict(agg, last_flags),
        },
        'processes': rows,
        'pyspy': pyspy_out,
        'topics_hz': topic_result_holder.get('hz', {}),
    }
    return report


def _verdict(agg, flags):
    if flags.get('throttled_now') or flags.get('under_voltage_now'):
        return 'THROTTLED (firmware capping — fix power/heat before trusting numbers)'
    busy = agg.get('busy_pct') or 0
    iowait = agg.get('iowait_pct') or 0
    if iowait > 15 and iowait > busy * 0.5:
        return 'I/O-BOUND (disk — likely the bag recorder on the SD card)'
    if busy > 85:
        return 'COMPUTE-BOUND (cores saturated at full clock)'
    return 'headroom OK'


# --------------------------------------------------------------------------
# rendering + persistence
# --------------------------------------------------------------------------

def _render_md(report, compare=None):
    h = report['host']
    L = []
    L.append(f"# System profile — {report['label']}  ({report['timestamp_utc']})")
    L.append('')
    L.append(f"- code git sha: `{report['git_sha']}`  ·  "
             f"OAK depth subsystem: **{report['depth_subsystem']}**")
    L.append(f"- window: {h['duration_s']}s  ·  cores: {h['ncpu']}  ·  "
             f"**{h['verdict']}**")

    def fmt(s):
        return f"{s['min']}/{s['mean']}/{s['max']}" if s else 'n/a'

    L.append(f"- CPU busy: **{h['busy_pct']}%**  ·  iowait: {h['iowait_pct']}%  "
             f"·  load1 (min/mean/max): {fmt(h['load1'])}")
    L.append(f"- ARM MHz (min/mean/max): {fmt(h['arm_mhz'])}  ·  "
             f"temp °C: {fmt(h['temp_c'])}")
    flags_on = [k for k, v in (h.get('throttled_flags_final') or {}).items() if v]
    L.append(f"- throttle flags: {', '.join(flags_on) if flags_on else 'none (0x0)'}")
    mem = h.get('mem') or {}
    L.append(f"- mem used/total: {mem.get('used_mb', '?')}/"
             f"{mem.get('total_mb', '?')} MB")
    L.append(f"- **tracked stack ≈ {h['tracked_cores_used']} of {h['ncpu']} cores**")
    L.append('')

    # Per-node table.
    cmp_map = {}
    if compare:
        for p in compare.get('processes', []):
            cmp_map[p['node']] = p['pct_core']
    hdr = '| # | node | %core | %machine | RSS MB | thr |'
    sep = '|---|------|------:|---------:|-------:|----:|'
    if compare:
        hdr = hdr[:-1] + ' Δ%core |'
        sep = sep[:-1] + '-------:|'
    L.append(hdr)
    L.append(sep)
    for i, r in enumerate(report['processes'][:30], 1):
        if r['pct_core'] < 0.3:
            break
        row = (f"| {i} | {'🐍 ' if r['is_python'] else ''}{r['node']} "
               f"| {r['pct_core']} | {r['pct_machine']} | {r['rss_mb']} "
               f"| {r['threads']} |")
        if compare:
            old = cmp_map.get(r['node'])
            d = (f"{r['pct_core'] - old:+.1f}" if old is not None else 'new')
            row = row[:-1] + f" {d} |"
        L.append(row)
    L.append('')

    # py-spy drill-down.
    if report.get('pyspy'):
        L.append('## py-spy own-time (hottest Python nodes)')
        L.append('')
        for blk in report['pyspy']:
            L.append(f"### {blk['node']} (pid {blk['pid']}, "
                     f"{blk.get('total_samples', 0)} samples)")
            if not blk.get('top'):
                L.append('_(no samples — non-Python, exited, or sudo/ptrace blocked)_')
                L.append('')
                continue
            L.append('| own% | function |')
            L.append('|-----:|----------|')
            for fn in blk['top']:
                L.append(f"| {fn['own_pct']} | `{fn['function']}` |")
            L.append('')

    # Topic Hz.
    if report.get('topics_hz'):
        L.append('## topic rates (Hz)')
        L.append('')
        L.append('| topic | Hz |')
        L.append('|-------|---:|')
        for t, hz in sorted(report['topics_hz'].items()):
            L.append(f"| `{t}` | {hz} |")
        L.append('')

    L.append('---')
    L.append('_Generated by profile_system.py. Re-run during a demo2/bench '
             'and at idle; pass --compare <old profile.json> for deltas._')
    return '\n'.join(L) + '\n'


def _push(repo, out_dir):
    rel = os.path.relpath(out_dir, repo)
    files = [os.path.join(rel, 'profile.json'), os.path.join(rel, 'profile.md')]
    log = []

    def run(cmd):
        r = subprocess.run(cmd, cwd=repo, capture_output=True, text=True,
                           timeout=120)
        log.append(('OK' if r.returncode == 0 else 'FAILED') + ': ' + ' '.join(cmd))
        if r.returncode != 0:
            log.append(r.stderr.strip())
        return r.returncode == 0

    if not os.path.isdir(os.path.join(repo, '.git')):
        return False, f'{repo} is not a git repo'
    if not run(['git', 'add', '-f', *files]):
        return False, '\n'.join(log)
    if subprocess.run(['git', 'diff', '--cached', '--quiet'],
                      cwd=repo).returncode == 0:
        log.append('(nothing to commit)')
        return True, '\n'.join(log)
    msg = f'System profile {os.path.basename(out_dir)}\n'
    if not run(['git', 'commit', '-m', msg]):
        return False, '\n'.join(log)
    if not run(['git', 'pull', '--rebase']):
        return False, '\n'.join(log)
    if not run(['git', 'push']):
        return False, '\n'.join(log)
    log.append('✓ pushed to origin')
    return True, '\n'.join(log)


def main():
    # Make output locale-proof: the report + help carry a few non-ASCII
    # glyphs (Δ, ≈, ✓). A non-UTF-8 console (e.g. Windows cp1252) would
    # otherwise crash on write rather than just mojibake.
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding='utf-8', errors='replace')
        except Exception:
            pass

    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--duration', type=int, default=20,
                    help='sampling window in seconds (default 20)')
    ap.add_argument('--label', default='profile',
                    help="tag for the run dir, e.g. idle / demo2 / bench")
    ap.add_argument('--pyspy', action='store_true',
                    help='py-spy own-time drill-down on the hottest Python nodes (needs sudo)')
    ap.add_argument('--pyspy-top', type=int, default=4,
                    help='how many top Python nodes to drill into (default 4)')
    ap.add_argument('--keep-raw', action='store_true',
                    help='keep the raw py-spy folded-stack files in /tmp')
    ap.add_argument('--topics', action='store_true',
                    help='also measure critical-path topic Hz (needs ros2 sourced)')
    ap.add_argument('--topics-list', default='',
                    help='comma-separated topic override for --topics')
    ap.add_argument('--push', action='store_true',
                    help='git add+commit+pull --rebase+push the profile artifacts')
    ap.add_argument('--compare', default='',
                    help='path to an earlier profile.json for a per-node Δ column')
    args = ap.parse_args()

    repo = _repo_dir()
    report = _build_report(args, repo)

    compare = None
    if args.compare:
        try:
            with open(args.compare) as f:
                compare = json.load(f)
        except Exception as e:
            print(f'  [compare] could not read {args.compare}: {e}',
                  file=sys.stderr)

    out_dir = os.path.join(repo, 'tuning_data', 'system_profiles',
                           f"{report['timestamp_utc']}_{args.label}")
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, 'profile.json'), 'w', encoding='utf-8') as f:
        json.dump(report, f, indent=2)
    md = _render_md(report, compare)
    with open(os.path.join(out_dir, 'profile.md'), 'w', encoding='utf-8') as f:
        f.write(md)

    print(md)
    print(f'  wrote {out_dir}', file=sys.stderr)

    if args.push:
        ok, log = _push(repo, out_dir)
        print(log, file=sys.stderr)
        sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
