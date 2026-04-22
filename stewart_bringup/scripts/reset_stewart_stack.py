#!/usr/bin/env python3
"""
Hard-reset the Stewart-platform ROS stack.

Use this when the GUI shows "(connected but no /status message received yet)"
and the GUI's own recovery buttons hang — typically after a previous launch
was Ctrl-C'd mid-action and left a zombie stewart_control_node process
holding can0 and a dead rclpy executor.

What it does, in order:
  1. SIGKILL anything matching stewart_control_node, rosbridge, xsens_mti_node,
     rosapi.
  2. Stop the ros2 CLI daemon (and `pkill -9 _ros2_daemon` if `stop` hangs).
  3. Wait briefly so the kernel can reclaim sockets and ports.
  4. Start a fresh ros2 daemon.
  5. Verify nothing matching the kill patterns is still alive.
  6. Print the launch command for you to run.

Optional flags:
  --launch     Also `ros2 launch stewart_bringup stewart_gui_launch.py` at the end.
  --verify     After --launch, poll /status for up to 20 s to confirm the
               control node is actually publishing.
  --no-daemon  Skip the daemon stop/start (just kill processes).
  --nuclear    Also kill any python3 process mentioning rclpy / ros2 /
               stewart. Use if plain --launch leaves children behind.

No sudo needed. Doesn't touch the CAN interface — if can0 is wedged at the
gs_usb driver level, you still need to physically replug the USB cable
(or usbipd detach+attach from Windows).
"""
import argparse
import os
import subprocess
import sys
import time

# Force unbuffered stdout so the user sees progress even if the script is
# killed mid-run — previous debugging was confused by lost buffered output.
try:
    sys.stdout.reconfigure(line_buffering=True)
except Exception:
    pass

ROS2_BIN = '/opt/ros/kilted/bin/ros2'

KILL_PATTERNS = [
    'stewart_control_node',
    'rosbridge_websocket',
    'rosbridge',
    'xsens_mti_node',
    'rosapi_node',
    'rosapi',
    'ros2 launch stewart',          # the launch wrapper itself
]

# Extra patterns used only by --nuclear
NUCLEAR_EXTRA = [
    '_ros2_daemon',
    'stewart_bringup',
]


def run(argv, check=False, timeout=5):
    """Run a command, capture output, return (returncode, stdout, stderr)."""
    try:
        r = subprocess.run(argv, capture_output=True, text=True,
                           timeout=timeout)
        return r.returncode, r.stdout, r.stderr
    except subprocess.TimeoutExpired:
        return -1, '', f"timeout after {timeout}s"
    except FileNotFoundError as e:
        return -2, '', str(e)


def header(msg):
    print(f"\n=== {msg} ===")


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--launch', action='store_true',
                   help='also start the GUI launch bundle at the end')
    p.add_argument('--verify', action='store_true',
                   help='after --launch, poll /status for 20 s to confirm '
                        'the stack is actually publishing')
    p.add_argument('--no-daemon', action='store_true',
                   help='skip the ros2 daemon stop/start')
    p.add_argument('--nuclear', action='store_true',
                   help='also kill the ros2 daemon process and any python3 '
                        'that matches known Stewart-stack patterns')
    args = p.parse_args()

    header("step 0: free port 9090 (rosbridge)")
    # Any stale process (typically a zombie rosbridge from a previous
    # failed launch) will cause the new rosbridge to get stuck retrying
    # 'Address already in use'. fuser -k sends SIGKILL to whatever holds
    # the port. Exit-codes > 0 just mean nothing was listening.
    rc, out, err = run(['bash', '-c', 'fuser -k 9090/tcp 2>&1 || true'],
                       timeout=3)
    if rc == 0 and out.strip():
        print(f"  freed :9090 (killed pids: {out.strip()})")
    else:
        print("  :9090 was free")

    header("step 1: SIGKILL stewart-related processes")
    patterns = list(KILL_PATTERNS)
    if args.nuclear:
        patterns += NUCLEAR_EXTRA
    for pat in patterns:
        rc, out, err = run(['pkill', '-9', '-f', pat])
        # pkill returns 1 when nothing matched (not an error for us).
        if rc == 0:
            print(f"  killed: {pat}")
        elif rc == 1:
            print(f"  none running: {pat}")
        else:
            print(f"  pkill {pat} rc={rc}: {err.strip()}")
    if args.nuclear:
        import signal as _sig
        self_pid = os.getpid()
        ppid = os.getppid()
        print(f"  nuclear: self_pid={self_pid} ppid={ppid}",
              flush=True)
        print(f"  nuclear: scanning /proc...", flush=True)
        candidates = []
        try:
            proc_entries = os.listdir('/proc')
        except Exception as e:
            print(f"  nuclear: /proc scan failed: {e}", flush=True)
            proc_entries = []
        for entry in proc_entries:
            if not entry.isdigit():
                continue
            pid = int(entry)
            if pid in (self_pid, ppid):
                continue
            try:
                with open(f'/proc/{pid}/cmdline', 'rb') as f:
                    cmdline_raw = f.read()
            except Exception:
                continue
            cmdline = cmdline_raw.replace(b'\x00', b' ').decode(
                'utf-8', errors='replace')
            if 'reset_stewart_stack' in cmdline:
                continue
            if 'python3' not in cmdline:
                continue
            if ('stewart_bringup' not in cmdline
                    and 'rclpy' not in cmdline):
                continue
            candidates.append((pid, cmdline.strip()))

        print(f"  nuclear: {len(candidates)} candidate(s) found", flush=True)
        for pid, cmdline in candidates:
            print(f"    will kill pid {pid}: {cmdline[:90]}", flush=True)
        for pid, cmdline in candidates:
            try:
                os.kill(pid, _sig.SIGKILL)
                print(f"    killed pid {pid}", flush=True)
            except Exception as e:
                print(f"    kill {pid} failed: {e}", flush=True)
        print("  nuclear: sweep complete", flush=True)

    if not args.no_daemon:
        header("step 2: stop ros2 daemon")
        rc, out, err = run([ROS2_BIN, 'daemon', 'stop'], timeout=5)
        if rc == 0:
            print("  stopped")
        elif rc == -1:
            print("  `ros2 daemon stop` hung — force-killing")
            run(['pkill', '-9', '-f', '_ros2_daemon'])
        else:
            print(f"  rc={rc}, stdout: {out.strip()} stderr: {err.strip()}")

    header("step 3: pause for kernel cleanup")
    time.sleep(1.0)
    print("  done")

    if not args.no_daemon:
        header("step 4: start fresh ros2 daemon")
        rc, out, err = run([ROS2_BIN, 'daemon', 'start'], timeout=30)
        if rc == 0:
            print("  started")
        else:
            # Daemon is only a CLI introspection cache — the stack runs fine
            # without it. On slow hosts (Pi) cold-start can exceed our timeout.
            # Don't bail; just warn so the rest of the reset still completes.
            print(f"  start WARN rc={rc}: {err.strip() or out.strip()}")
            print("  (continuing anyway — daemon is only for `ros2 topic` "
                  "etc., not for the running stack)")

    header("step 5: verify nothing stale is still running")
    leftover = []
    for pat in KILL_PATTERNS:
        rc, out, err = run(['pgrep', '-af', pat])
        if rc == 0 and out.strip():
            for line in out.strip().split('\n'):
                # pgrep -af includes the matching process name; skip if it's
                # this script itself (which contains the patterns as strings).
                if 'reset_stewart_stack' in line:
                    continue
                leftover.append(line)
    if leftover:
        print("  WARNING — these processes are still alive:")
        for l in leftover:
            print(f"    {l}")
        print("  Try running the script again, or kill manually with `kill -9 <PID>`.")
    else:
        print("  clean. Stack is ready for relaunch.")

    header("done")
    if args.launch:
        if args.verify:
            # Spawn launch as background process so we can poll /status
            # from this same script before handing off / exiting.
            header("step 6: launching stack in background + verifying")
            log_path = os.path.expanduser(
                '~/ros2_ws/src/stewart_bringup/logs/last_launch.log')
            os.makedirs(os.path.dirname(log_path), exist_ok=True)
            log_f = open(log_path, 'w')
            proc = subprocess.Popen(
                ['bash', '-c',
                 f'source /opt/ros/kilted/setup.bash && '
                 f'source ~/ros2_ws/install/local_setup.bash && '
                 f'exec {ROS2_BIN} launch stewart_bringup stewart_gui_launch.py'],
                stdout=log_f, stderr=subprocess.STDOUT,
                start_new_session=True)
            print(f"  launch pid={proc.pid}, log={log_path}")
            print("  polling /status (up to 20 s)...")
            deadline = time.time() + 20.0
            good = False
            while time.time() < deadline:
                time.sleep(1.0)
                # Try to echo /status with a short timeout
                r = subprocess.run(
                    ['bash', '-c',
                     'source /opt/ros/kilted/setup.bash && '
                     'source ~/ros2_ws/install/local_setup.bash && '
                     'timeout 3 ros2 topic echo /status --once'],
                    capture_output=True, text=True, timeout=5)
                if r.returncode == 0 and 'armed' in r.stdout:
                    good = True
                    elapsed = 20.0 - (deadline - time.time())
                    print(f"  /status IS publishing (after {elapsed:.1f} s). "
                          f"Stack is healthy.")
                    break
                else:
                    print(f"  ... no /status yet at "
                          f"t+{20.0 - (deadline - time.time()):.0f}s")
            if not good:
                print("  !! /status never arrived — control node failed.")
                # Clean up the background launch so we don't leave a zombie
                # fighting for port 9090 on the next attempt.
                try:
                    import signal as _sig
                    os.killpg(os.getpgid(proc.pid), _sig.SIGTERM)
                    try:
                        proc.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        os.killpg(os.getpgid(proc.pid), _sig.SIGKILL)
                    print(f"  cleaned up launch pid {proc.pid}")
                except Exception as e:
                    print(f"  failed to clean up launch: {e}")
                # Also free port 9090 in case rosbridge was already bound
                run(['bash', '-c', 'fuser -k 9090/tcp 2>/dev/null || true'],
                    timeout=3)
                print("  !! check log:", log_path)
                print("  !! last 20 lines:")
                try:
                    with open(log_path) as f:
                        for line in f.readlines()[-20:]:
                            print("    " + line.rstrip())
                except Exception:
                    pass
                sys.exit(2)
        else:
            print("Launching the GUI bundle now (Ctrl-C to stop)...\n")
            os.execvp(ROS2_BIN,
                      [ROS2_BIN, 'launch', 'stewart_bringup',
                       'stewart_gui_launch.py'])
    else:
        print("Relaunch the GUI bundle with:")
        print("  ros2 launch stewart_bringup stewart_gui_launch.py")
        print("(or re-run this script with --launch to do it for you)")


if __name__ == '__main__':
    main()
