"""Latency math for the demo digests — pure NumPy, no ROS.

Extracted so the demo digest, `compare_demo_bags.py`, and any future
"ball-tap" analyzer share one verified implementation (mirrors the
`stewart_vision._image_quality` / `_ball_physics` pattern). Being
ROS-free, it is unit-tested on any machine (see
`stewart_bringup/test/test_latency.py`).

The headline tool here is `actuation_latency`: the *effective*
command->motion delay, measured by cross-correlating the commanded
tilt (`/ball_track/diagnostic`) against the measured platform tilt
(the platform IMU's fused orientation, ~240 Hz, or `/platform_rpy`
as a coarser fallback). This is the half of the control loop the
vision-only latency map (`oak_latency_map.md`) could never measure —
see `control_path_latency.md` §3.
"""
from __future__ import annotations

import numpy as np


def quat_to_roll_pitch_deg(quat):
    """ZYX roll/pitch (deg) from an (N,4) array of [qx,qy,qz,qw].
    The platform MTi-630 reports fused orientation, so this gives the
    high-rate 'what the platform actually did' tilt — finer than the
    ~17 Hz /platform_rpy topic. Vectorized; returns (roll, pitch)."""
    q = np.asarray(quat, dtype=np.float64).reshape(-1, 4)
    qx, qy, qz, qw = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = np.degrees(np.arctan2(sinr_cosp, cosr_cosp))
    sinp = np.clip(2.0 * (qw * qy - qz * qx), -1.0, 1.0)
    pitch = np.degrees(np.arcsin(sinp))
    return roll, pitch


def resample_uniform(t_s, y, fs_hz, t0_s, t1_s):
    """Interp (t_s, y) onto a uniform grid [t0,t1] at fs. NaN outside
    the source span. Returns (grid_s, y_resampled)."""
    t_s = np.asarray(t_s, dtype=np.float64)
    y = np.asarray(y, dtype=np.float64)
    n = int(max(2, round((t1_s - t0_s) * fs_hz)))
    grid = t0_s + np.arange(n) / fs_hz
    if t_s.size < 2:
        return grid, np.full(n, np.nan)
    order = np.argsort(t_s)
    ys = np.interp(grid, t_s[order], y[order], left=np.nan, right=np.nan)
    return grid, ys


def xcorr_lag_s(ref_t_s, ref_y, resp_t_s, resp_y,
                fs_hz=200.0, max_lag_s=0.40):
    """Lag L>=0 (s) such that resp(t) best matches ref(t-L): how long
    the RESPONSE (measured platform tilt) trails the COMMAND (cmd
    tilt). The measured lag is the *effective* command->motion latency
    — transport + feeder ZOH + leg slew + the mechanical rise
    (verified against synthetic signals with known delay + rise: a
    pure delay is recovered exactly; a first-order rise adds a constant
    offset equal to its group delay).

    Sign-agnostic (the platform tilt may be sign-flipped vs the cmd
    convention): scores |normalised correlation|, reports the sign in
    the returned correlation. Resamples both series to a common uniform
    grid so different topic rates (cmd ~55 Hz, IMU ~240 Hz) line up.

    Returns (lag_s | None, peak_corr_signed, n_overlap)."""
    if (ref_t_s is None or resp_t_s is None
            or len(ref_t_s) < 4 or len(resp_t_s) < 4):
        return None, 0.0, 0
    t0 = max(float(ref_t_s[0]), float(resp_t_s[0]))
    t1 = min(float(ref_t_s[-1]), float(resp_t_s[-1]))
    if t1 - t0 < 3.0 * max_lag_s:
        return None, 0.0, 0
    _, a = resample_uniform(ref_t_s, ref_y, fs_hz, t0, t1)    # command
    _, b = resample_uniform(resp_t_s, resp_y, fs_hz, t0, t1)  # response
    good = np.isfinite(a) & np.isfinite(b)
    if good.sum() < 8:
        return None, 0.0, 0
    a = a - np.nanmean(a[good])
    b = b - np.nanmean(b[good])
    a[~good] = 0.0
    b[~good] = 0.0
    if np.linalg.norm(a) == 0.0:
        return None, 0.0, 0
    max_lag = int(round(max_lag_s * fs_hz))
    best = (0, 0.0)
    for L in range(0, max_lag + 1):
        aa, bb = (a, b) if L == 0 else (a[:-L], b[L:])
        nb = np.linalg.norm(bb)
        if nb == 0.0:
            continue
        c = float(np.dot(aa, bb) / (np.linalg.norm(aa) * nb))
        if abs(c) > abs(best[1]):
            best = (L, c)
    return best[0] / fs_hz, best[1], int(good.sum())


def dominant_period_s(t_s, y, fs_hz=200.0,
                      min_period_s=0.05, max_period_s=2.0):
    """Estimate the dominant period of a (possibly periodic) signal via
    autocorrelation. Returns period_s, or None if not clearly periodic.

    Why this matters for latency: a saturated/orbital run is a limit
    cycle, and a delayed periodic signal correlates *equally* at lag,
    lag±T, lag±T/2 (with sign flips). So the cross-correlation lag is
    only unique modulo the period — the absolute actuation number from
    an orbital run is ambiguous. We detect the period to FLAG that,
    rather than report false precision. The unambiguous fix is an
    aperiodic excitation (the tap/step test)."""
    if t_s is None or len(t_s) < 16:
        return None
    t0, t1 = float(t_s[0]), float(t_s[-1])
    if t1 - t0 < 4.0 * min_period_s:
        return None
    _, ys = resample_uniform(t_s, y, fs_hz, t0, t1)
    good = np.isfinite(ys)
    if good.sum() < 16:
        return None
    ys = ys - np.nanmean(ys[good])
    ys[~good] = 0.0
    if np.linalg.norm(ys) == 0.0:
        return None
    n = ys.size
    ac = np.correlate(ys, ys, mode='full')[n - 1:]
    if ac[0] == 0.0:
        return None
    ac = ac / ac[0]
    lo = max(1, int(min_period_s * fs_hz))
    hi = min(int(max_period_s * fs_hz), n - 1)
    if hi <= lo + 2:
        return None
    seg = ac[lo:hi]
    thr = max(0.3, 0.5 * float(np.max(seg)))
    # First PROMINENT local maximum = the fundamental period. Global
    # argmax is unreliable here: a long limit cycle has many near-equal
    # autocorrelation peaks at multiples of T (argmax can land on any),
    # and a monotonic ramp from an aperiodic step has no interior peak
    # (so we correctly return None and never call a step "periodic").
    for i in range(1, seg.size - 1):
        if seg[i] >= seg[i - 1] and seg[i] > seg[i + 1] and seg[i] >= thr:
            return (lo + i) / fs_hz
    return None


def actuation_latency(bt_t_ns, cmd_pitch, cmd_roll,
                      imu_t_ns, imu_rp, rpy_t_ns, rpy_data,
                      min_corr=0.3):
    """Command->motion latency by cross-correlating commanded tilt
    against the measured platform tilt. Prefers the high-rate platform
    IMU; falls back to /platform_rpy (~17 Hz) if the IMU orientation
    isn't populated (a zero quaternion -> all-zero rp -> useless).

    `*_t_ns` are int64 ROS bag receive-times (nanoseconds). `imu_rp`
    is an (N,2) [roll,pitch] array (deg); `rpy_data` is (N,3)
    [roll,pitch,yaw]. Returns a dict (source, per-axis lag + corr,
    headline `actuation_ms`) or None when there's no usable signal.
    The headline is the axis with the stronger correlation, so a quiet
    axis can't dominate the number."""
    if bt_t_ns is None or len(bt_t_ns) < 8:
        return None
    source = act_t = act_roll = act_pitch = None
    if (imu_t_ns is not None and len(imu_t_ns) > 8 and imu_rp is not None
            and getattr(imu_rp, 'size', 0) and np.any(np.isfinite(imu_rp))
            and float(np.nanstd(imu_rp)) > 1e-4):
        source = 'platform_imu'
        act_t = imu_t_ns
        act_roll, act_pitch = imu_rp[:, 0], imu_rp[:, 1]
    elif (rpy_t_ns is not None and len(rpy_t_ns) > 8
          and rpy_data is not None and getattr(rpy_data, 'size', 0)):
        source = 'platform_rpy'
        act_t = rpy_t_ns
        act_roll, act_pitch = rpy_data[:, 0], rpy_data[:, 1]
    else:
        return None
    # Common origin (ns->s) to preserve float64 precision on the lag.
    org = float(min(int(bt_t_ns[0]), int(act_t[0])))
    bt_s = (np.asarray(bt_t_ns, dtype=np.float64) - org) * 1e-9
    act_s = (np.asarray(act_t, dtype=np.float64) - org) * 1e-9
    p_lag, p_corr, p_n = xcorr_lag_s(bt_s, cmd_pitch, act_s, act_pitch)
    r_lag, r_corr, r_n = xcorr_lag_s(bt_s, cmd_roll, act_s, act_roll)
    out = {
        'source': source,
        'pitch_lag_ms': None if p_lag is None else round(p_lag * 1e3, 1),
        'pitch_corr': round(p_corr, 3),
        'roll_lag_ms': None if r_lag is None else round(r_lag * 1e3, 1),
        'roll_corr': round(r_corr, 3),
        'n_overlap': int(max(p_n, r_n)),
    }
    cands = [(abs(p_corr), p_lag), (abs(r_corr), r_lag)]
    cands = [(c, l) for c, l in cands if l is not None and c > min_corr]
    out['actuation_ms'] = (round(max(cands)[1] * 1e3, 1)
                           if cands else None)
    # Periodicity / ambiguity flag. If the command is an orbital limit
    # cycle, the cross-correlation lag is only unique modulo its period
    # (and ±half-period with a sign flip), so the absolute number can't
    # be trusted — report the period, mark it ambiguous, and list the
    # aliased candidate lags. The unambiguous measurement is the tap test.
    period = dominant_period_s(bt_s, cmd_pitch)
    out['cmd_period_s'] = round(period, 3) if period else None
    act_lag_s = (out['actuation_ms'] / 1e3) if out['actuation_ms'] else None
    out['ambiguous'] = False
    if period is not None and act_lag_s is not None:
        # A sign-agnostic correlation aliases every HALF period (the
        # in-phase peak AND the anti-phase peak), so the candidate lags
        # are act_lag ± k·(T/2) that land in the search window. >1
        # candidate => the absolute number can't be trusted.
        half = period / 2.0
        cands = sorted({round(act_lag_s + k * half, 3)
                        for k in range(-6, 7)
                        if 0.0 <= act_lag_s + k * half <= 0.40})
        if len(cands) > 1:
            out['ambiguous'] = True
            out['lag_candidates_s'] = cands
    return out


def step_response_metrics(cmd_t_s, cmd_y, resp_t_s, resp_y,
                          fs_hz=400.0, onset_frac=0.1, settle_frac=0.05):
    """Per-stage metrics for a single aperiodic tilt STEP — the clean,
    UNAMBIGUOUS actuation measurement an orbital cross-correlation can't
    give (no period → no aliasing). This is what the Latency Bench panel's
    tilt-step run feeds. `cmd_y` is commanded tilt (deg) over time; `resp_y`
    is the measured response (deg for IMU, turns for a leg encoder) — call
    it once per stage to isolate each leg of the actuation chain
    (cmd→encoder = transport/feeder; encoder→IMU = leg→platform; IMU rise =
    mechanical).

    Returns dict or None (no clean step found):
      step_cmd, step_resp, gain (resp/cmd),
      dead_time_ms  — cmd-onset → resp-onset (pure transport delay),
      xcorr_lag_ms  — best cmd→resp shift (unambiguous on a step),
      rise_10_90_ms — resp 10→90 % of its step,
      settle_ms     — resp enters & stays within ±settle_frac of final,
      overshoot_pct.
    """
    if (cmd_t_s is None or resp_t_s is None
            or len(cmd_t_s) < 4 or len(resp_t_s) < 8):
        return None
    t0 = max(float(cmd_t_s[0]), float(resp_t_s[0]))
    t1 = min(float(cmd_t_s[-1]), float(resp_t_s[-1]))
    if t1 - t0 < 0.2:
        return None
    tg, c = resample_uniform(cmd_t_s, cmd_y, fs_hz, t0, t1)
    _, r = resample_uniform(resp_t_s, resp_y, fs_hz, t0, t1)
    good = np.isfinite(c) & np.isfinite(r)
    if good.sum() < 8:
        return None
    tg, c, r = tg[good], c[good], r[good]
    n = c.size
    nb = max(2, n // 10)               # baseline = first 10 %
    nf = max(2, n // 5)                # final = last 20 %
    c_base, c_final = float(np.mean(c[:nb])), float(np.mean(c[-nf:]))
    r_base, r_final = float(np.mean(r[:nb])), float(np.mean(r[-nf:]))
    c_step = c_final - c_base
    r_step = r_final - r_base
    if abs(c_step) < 1e-3:             # no commanded step → nothing to measure
        return None
    sgn = 1.0 if c_step > 0 else -1.0

    def _onset(y, base, step):
        thr = base + onset_frac * step
        idx = np.where((y - thr) * np.sign(step) >= 0)[0]
        return float(tg[idx[0]]) if idx.size else None

    t_cmd_on = _onset(c, c_base, c_step)
    t_resp_on = _onset(r, r_base, r_step if abs(r_step) > 1e-6 else c_step)
    dead_ms = (None if (t_cmd_on is None or t_resp_on is None)
               else round((t_resp_on - t_cmd_on) * 1e3, 1))

    rise_ms = settle_ms = overshoot = None
    if abs(r_step) > 1e-3:
        def _cross(frac):
            thr = r_base + frac * r_step
            idx = np.where((r - thr) * sgn >= 0)[0]
            return float(tg[idx[0]]) if idx.size else None
        t10, t90 = _cross(0.1), _cross(0.9)
        if t10 is not None and t90 is not None and t90 >= t10:
            rise_ms = round((t90 - t10) * 1e3, 1)
        band = settle_frac * abs(r_step)
        outside = np.where(np.abs(r - r_final) > band)[0]
        if outside.size and outside[-1] + 1 < n and t_cmd_on is not None:
            settle_ms = round((tg[outside[-1] + 1] - t_cmd_on) * 1e3, 1)
        peak = float(np.max(r) if sgn > 0 else np.min(r))
        overshoot = round(
            max(0.0, (peak - r_final) * sgn / abs(r_step)) * 100.0, 1)

    lag_s, _corr, _n = xcorr_lag_s(cmd_t_s, cmd_y, resp_t_s, resp_y,
                                   fs_hz=min(fs_hz, 200.0))
    return {
        'step_cmd': round(c_step, 3),
        'step_resp': round(r_step, 3),
        'gain': round(r_step / c_step, 3),
        'dead_time_ms': dead_ms,
        'xcorr_lag_ms': None if lag_s is None else round(lag_s * 1e3, 1),
        'rise_10_90_ms': rise_ms,
        'settle_ms': settle_ms,
        'overshoot_pct': overshoot,
    }


def step_train_metrics(cmd_t_s, cmd_y, resp_t_s, resp_y,
                       min_step=0.3, pre_s=0.15, settle_frac=0.05):
    """Per-rep metrics for a TRAIN of tilt steps — what the Latency Bench
    actually commands (`reps` up/down cycles). Detects each RISING edge in
    the commanded tilt and measures the response per rep, returning the
    MEDIAN. Robust where `step_response_metrics` (single step) fails:
      - multi-rep: a whole-run baseline→final both land at level → step≈0;
      - sparse diag: the edge time is the transition SAMPLE itself (the
        first publish after the set_pose), not a resampled onset, so a low
        steady-state publish rate doesn't smear the edge.

    `cmd_y` is the commanded tilt on the driven axis (signed). `resp_y` is
    a 1-D response (e.g. the IMU axis with the larger response, or a leg
    encoder) — the digest picks a frame-robust one. Returns medians +
    n_steps, or None if no clean rising edge is found."""
    cmd_t = np.asarray(cmd_t_s, dtype=np.float64)
    cmd = np.asarray(cmd_y, dtype=np.float64)
    rt = np.asarray(resp_t_s, dtype=np.float64)
    r = np.asarray(resp_y, dtype=np.float64)
    if cmd_t.size < 3 or rt.size < 8:
        return None
    rises = np.where(np.diff(cmd) >= min_step)[0] + 1   # rising-edge indices
    falls = np.where(np.diff(cmd) <= -min_step)[0] + 1  # falling-edge indices
    if rises.size == 0:
        return None
    per = []
    for ei in rises:
        t_e = float(cmd_t[ei])
        # End the window at the next FALLING edge so it covers the HELD
        # tilt only — not the return-to-level that follows (which would
        # drag the plateau back to baseline and read step_resp≈0).
        fj = falls[falls > ei]
        t_fall = (float(cmd_t[fj[0]]) if fj.size
                  else min(t_e + 1.5, float(rt[-1])))
        t_fall = min(t_fall, float(rt[-1]))
        cmd_step = float(cmd[ei] - cmd[ei - 1])
        if cmd_step < min_step or t_fall - t_e < 0.1:
            continue
        pre = r[(rt >= t_e - pre_s) & (rt < t_e)]
        win_m = (rt >= t_e) & (rt <= t_fall)
        if pre.size < 1 or win_m.sum() < 4:
            continue
        r_base = float(np.median(pre))
        rw_t, rw = rt[win_m], r[win_m]
        half_t = rw_t[0] + 0.5 * (rw_t[-1] - rw_t[0])
        plateau = float(np.median(rw[rw_t >= half_t]))
        r_step = plateau - r_base
        rec = {'gain': r_step / cmd_step, 'cmd': cmd_step, 'resp': r_step,
               'dead': None, 'rise': None, 'settle': None, 'over': None}
        if abs(r_step) >= 1e-3:
            sgn = 1.0 if r_step >= 0 else -1.0

            def _cross(frac):
                idx = np.where((rw - (r_base + frac * r_step)) * sgn >= 0)[0]
                return float(rw_t[idx[0]]) if idx.size else None

            on = _cross(0.1)
            rec['dead'] = (on - t_e) * 1e3 if on is not None else None
            t10, t90 = _cross(0.1), _cross(0.9)
            if t10 is not None and t90 is not None and t90 >= t10:
                rec['rise'] = (t90 - t10) * 1e3
            outside = np.where(np.abs(rw - plateau) > settle_frac
                               * abs(r_step))[0]
            if outside.size and outside[-1] + 1 < rw.size:
                rec['settle'] = (rw_t[outside[-1] + 1] - t_e) * 1e3
            peak = float(np.max(rw) if sgn > 0 else np.min(rw))
            rec['over'] = max(0.0, (peak - plateau) * sgn / abs(r_step)) * 100.0
        per.append(rec)
    if not per:
        return None

    def _med(key):
        vals = [p[key] for p in per if p[key] is not None]
        return round(float(np.median(vals)), 1) if vals else None

    return {
        'n_steps': len(per),
        'dead_time_ms': _med('dead'),
        'rise_10_90_ms': _med('rise'),
        'settle_ms': _med('settle'),
        'overshoot_pct': _med('over'),
        'gain': _med('gain'),
        'step_cmd': _med('cmd'),
        'step_resp': _med('resp'),
    }
