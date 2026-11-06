#!/usr/bin/env python3
"""Summarise a tracking telemetry run ([telemetry] in config/system.ini).

Reads <run>/frames.csv, ticks.csv and run.txt and reports what the tuning
decisions hang on:

  timing     frame age, dropped frames, tracking-loop jitter
  filter     detection/accept rates, NIS and innovation whiteness: whether
             filter_meas_sigma_px and filter_q_jerk match the data
  motors     dead time, lag and gain of the drives from command vs encoder:
             what filter_actuation_latency_ms should be
  overshoot  per acquisition, from the aim error the control law acted on
  stabilise  ONLY meaningful with --stationary (the bird did not move in the
             window): fits the true focal lengths and the capture-time error
             from how the detections move against the encoders, which is what
             focal_x_px/focal_y_px and readout_ms should be. Phantom target
             speed is reported against the gimbal's rate.

Usage: python3 tools/telemetry_analyse.py <run_dir> [--gimbal N]
                                          [--from S] [--to S] [--stationary]
Times for --from/--to are seconds since the run's first tick.
Needs numpy.
"""

import argparse
import csv
import math
import os
import re
import sys

import numpy as np

try:  # optional: only speeds up the motor fit
    from scipy.signal import lfilter
except Exception:  # noqa: BLE001 - a broken scipy install raises all sorts
    lfilter = None

# Gimbal rates below this are treated as standing still, rad/s.
MOVING_RAD_S = 0.005
# Span the gimbal rate is differentiated over, seconds; matches the filter.
RATE_WINDOW_S = 0.05
# Motor fit: resampling step and the longest stretch fitted, seconds.
MOTOR_STEP_S = 0.004
MOTOR_MAX_SPAN_S = 30.0


def load_csv(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return {}
    out = {}
    for key in rows[0]:
        col = []
        for r in rows:
            v = r[key]
            col.append(float(v) if v not in ("", None) else math.nan)
        out[key] = np.array(col)
    return out


def select(table, mask):
    return {k: v[mask] for k, v in table.items()}


def parse_run(path):
    info = {"gimbals": {}, "config": {}}
    section = None
    in_cfg = False
    with open(path) as f:
        for line in f:
            line = line.rstrip("\n")
            if line.startswith("--- config"):
                in_cfg = True
                continue
            if line.startswith("--- end config"):
                in_cfg = False
                continue
            if in_cfg:
                s = line.strip()
                if s.startswith("[") and s.endswith("]"):
                    section = s[1:-1]
                elif "=" in s and not s.startswith("#") and section:
                    k, v = s.split("=", 1)
                    info["config"].setdefault(section, {})[k.strip()] = v.strip()
                continue
            m = re.match(r"gimbal (\d+) (.*)", line)
            if m:
                toks = m.group(2).split()
                info["gimbals"][int(m.group(1))] = dict(zip(toks[::2], toks[1::2]))
            m = re.match(r"end .*dropped_frames (\d+) dropped_ticks (\d+)", line)
            if m:
                info["dropped"] = (int(m.group(1)), int(m.group(2)))
    return info


def pct(a, q):
    a = a[np.isfinite(a)]
    return float(np.percentile(a, q)) if a.size else math.nan


def stats_ms(a):
    a = a[np.isfinite(a)] * 1e3
    if not a.size:
        return "n/a"
    return (f"mean {a.mean():.1f}  p50 {np.percentile(a, 50):.1f}  "
            f"p95 {np.percentile(a, 95):.1f}  max {a.max():.1f} ms")


def heading(text):
    print(f"\n== {text} " + "=" * max(0, 70 - len(text)))


def effective_pan(ticks, level_tilt):
    """Pan folded by cos(elevation), integrated like push_gimbal_sample."""
    pan = ticks["pan"]
    tilt = ticks["tilt"]
    d = np.remainder(np.diff(pan) + math.pi, 2 * math.pi) - math.pi
    eff = np.empty_like(pan)
    eff[0] = pan[0]
    eff[1:] = pan[0] + np.cumsum(d * np.cos(tilt[1:] - level_tilt))
    return eff


def windowed_rate(t, x, window):
    """Rate at each sample over the preceding `window` seconds (backward)."""
    j = np.searchsorted(t, t - window, side="left")
    j = np.minimum(j, np.arange(t.size) - 1)
    j = np.maximum(j, 0)
    dt = t - t[j]
    with np.errstate(invalid="ignore", divide="ignore"):
        r = (x - x[j]) / dt
    r[dt <= 0] = 0.0
    return r


def centred_rate(t, x, window):
    """Rate centred on each sample, over +-window/2: no lag, for fitting."""
    lo = np.searchsorted(t, t - window / 2, side="left")
    hi = np.searchsorted(t, t + window / 2, side="right") - 1
    lo = np.clip(lo, 0, t.size - 1)
    hi = np.clip(hi, 0, t.size - 1)
    dt = t[hi] - t[lo]
    with np.errstate(invalid="ignore", divide="ignore"):
        r = (x[hi] - x[lo]) / dt
    r[dt <= 0] = 0.0
    return r


def report_timing(frames, ticks):
    heading("Timing")
    dur = ticks["t_s"][-1] - ticks["t_s"][0] if ticks else 0.0
    print(f"run window {dur:.1f} s, {frames['frame_id'].size} frames, {ticks['t_s'].size} ticks")

    ids = frames["frame_id"]
    gaps = np.diff(ids)
    lost = int(np.sum(gaps[gaps > 1] - 1))
    print(f"frames the tracking loop never saw (overwritten in the hand-off): {lost} "
          f"({100 * lost / max(1, ids.size + lost):.1f} %)")
    dt = frames["frame_dt_s"]
    dt = dt[dt > 0]
    if dt.size:
        print(f"camera frame interval: {stats_ms(dt)}  (~{1 / np.median(dt):.1f} fps)")

    have = frames["captured_at_s"] > 0
    print(f"frames with a measured capture time: {100 * have.mean():.0f} %"
          + ("" if have.all() else "  (the rest fall back to filter_pipeline_latency_ms)"))
    if have.any():
        f = select(frames, have)
        print(f"capture -> probe (aggregate pipeline): {stats_ms(f['timestamp_s'] - f['captured_at_s'])}")
        print(f"capture -> filter step:           {stats_ms(f['step_s'] - f['captured_at_s'])}")
        print(f"readout used {1e3 * np.nanmedian(f['readout_s']):.2f} ms, "
              f"exposure {1e3 * np.nanmedian(f['exposure_s']):.2f} ms "
              f"(capture moved back by readout + exposure/2)")

    tt = np.diff(ticks["t_s"])
    print(f"tracking tick interval: {stats_ms(tt)}")


def report_filter(frames, gcfg):
    heading("Filter")
    det = frames["det"] == 1
    est = frames["est"] == 1
    meas = frames["measured"] == 1
    n = frames["det"].size
    print(f"detections on {100 * det.mean():.0f} % of frames; track held on "
          f"{100 * est.mean():.0f} %")
    if det.any():
        rej = det & est & ~meas
        print(f"detections gated out while tracking: {int(rej.sum())} "
              f"({100 * rej.sum() / max(1, (det & est).sum()):.1f} %)")
    seeds = np.sum(np.diff(est.astype(int)) == 1) + (1 if est.size and est[0] else 0)
    print(f"track (re)seeds: {seeds}")

    nis = frames["nis"][meas & np.isfinite(frames["nis"])]
    nis = nis[nis > 0]
    if nis.size:
        print(f"NIS on accepted detections: mean {nis.mean():.2f}, p95 {np.percentile(nis, 95):.2f}"
              "   (2 dof: mean 2.0 and p95 6.0 when the noise model fits)")
        if nis.mean() > 3.0:
            print("  -> innovations are larger than the filter expects: raise "
                  "filter_meas_sigma_px if the detections are jittery (see whiteness), or "
                  "filter_q_jerk if the filter is lagging the target")
        elif nis.mean() < 1.2:
            print("  -> innovations are smaller than the filter expects: "
                  "filter_meas_sigma_px can come down (snappier, less lag)")

    # Innovation whiteness: prior prediction from the previous accepted frame,
    # in the stabilised frame (detections minus the camera shift).
    try:
        fx, fy = float(gcfg["focal_x"]), float(gcfg["focal_y"])
        half_w, half_h = float(gcfg["det_w"]) / 2, float(gcfg["det_h"]) / 2
    except (KeyError, ValueError):
        return
    ego = gcfg.get("ego") == "1"
    sx = fx if ego else 0.0
    sy = fy if ego else 0.0
    f = select(frames, meas)
    if f["det"].size < 20:
        return
    stab_u = f["est_u"] + sx * f["frame_pan"]
    stab_v = f["est_v"] + sy * f["frame_tilt"]
    z_u = (f["det_cx"] - half_w) + sx * f["frame_pan"]
    z_v = (f["det_cy"] - half_h) + sy * f["frame_tilt"]
    t = f["est_capture_s"]
    dt = np.diff(t)
    ok = (dt > 0) & (dt < 0.1)
    pu = stab_u[:-1] + f["vu"][:-1] * dt + 0.5 * f["au"][:-1] * dt * dt
    pv = stab_v[:-1] + f["vv"][:-1] * dt + 0.5 * f["av"][:-1] * dt * dt
    iu_all = z_u[1:] - pu
    iv_all = z_v[1:] - pv
    # Gimbal still: camera-induced image speed below MOVING_RAD_S worth.
    cam = np.hypot(f["cam_vu"], f["cam_vv"])[1:]
    still = cam < MOVING_RAD_S * max(fx, fy)

    def ac1(x):
        x = x - x.mean()
        d = np.dot(x, x)
        return float(np.dot(x[:-1], x[1:]) / d) if d > 0 else math.nan

    nis_m = frames["nis"][meas]
    nis_still = nis_m[1:][still & (nis_m[1:] > 0)]
    for name, m in (("all frames", ok), ("gimbal still", ok & still)):
        iu, iv = iu_all[m], iv_all[m]
        if iu.size < 20:
            print(f"innovations, {name}: too few frames")
            continue
        print(f"innovations, {name}: rms u {np.sqrt(np.mean(iu ** 2)):.2f} px, "
              f"v {np.sqrt(np.mean(iv ** 2)):.2f} px; lag-1 autocorrelation u {ac1(iu):+.2f}, "
              f"v {ac1(iv):+.2f}")
    if nis_still.size > 20:
        print(f"NIS, gimbal still: mean {nis_still.mean():.2f}")
    print("  Judge the noise settings on 'gimbal still': there the innovations are the detector\n"
          "  and the target alone. Lag-1 near 0: white, the filter keeps up. Clearly positive: it\n"
          "  lags the target, raise filter_q_jerk. Clearly negative: it chases jitter, lower\n"
          "  filter_q_jerk or raise filter_meas_sigma_px. Worse while turning than still: the\n"
          "  camera-motion compensation is off, see the last section.")
    sp = np.hypot(frames["vu"][est], frames["vv"][est])
    print(f"filter target speed: p50 {pct(sp, 50):.1f}, p95 {pct(sp, 95):.1f}, "
          f"max {np.nanmax(sp):.1f} px/s")


def fit_motor(t, cmd, pos):
    """First-order lag plus dead time from commanded velocity to encoder rate."""
    if t.size < 50:
        return None
    step = MOTOR_STEP_S
    grid = np.arange(t[0], min(t[-1], t[0] + MOTOR_MAX_SPAN_S), step)
    c = np.interp(grid, t, cmd)
    p = np.interp(grid, t, pos)
    w = centred_rate(grid, p, 0.03)
    best = None
    for delay in np.arange(0.0, 0.151, step):
        k = int(round(delay / step))
        cd = np.concatenate([np.full(k, c[0]), c[: c.size - k]]) if k else c
        for tau in (0.005, 0.01, 0.02, 0.03, 0.04, 0.06, 0.08, 0.1, 0.13, 0.17, 0.22, 0.3):
            a = step / (tau + step)
            if lfilter is not None:
                y = lfilter([a], [1.0, a - 1.0], cd)
            else:
                y = np.empty_like(cd)
                acc = 0.0
                for i, v in enumerate(cd):
                    acc += a * (v - acc)
                    y[i] = acc
            den = np.dot(y, y)
            if den <= 0:
                continue
            gain = np.dot(y, w) / den
            err = np.mean((w - gain * y) ** 2)
            if best is None or err < best[0]:
                best = (err, delay, tau, gain)
    if best is None:
        return None
    var = np.var(w)
    return {"delay": best[1], "tau": best[2], "gain": best[3],
            "r2": 1 - best[0] / var if var > 0 else math.nan}


def report_motors(ticks):
    heading("Motors (command -> encoder)")
    ok = np.isfinite(ticks["cmd_pan"]) & (ticks["driving"] == 1)
    if ok.sum() < 50:
        print("not enough driven ticks")
        return
    t = ticks["t_s"][ok]
    # Fit over the longest stretches only; gaps (home returns) break the model.
    breaks = np.where(np.diff(t) > 0.05)[0]
    segs = np.split(np.arange(t.size), breaks + 1)
    segs = [s for s in segs if s.size > 100]
    if not segs:
        print("no continuous driven stretch longer than 1 s")
        return
    seg = max(segs, key=lambda s: s.size)
    tt = t[seg]
    for axis in ("pan", "tilt"):
        cmd = ticks[f"cmd_{axis}"][ok][seg]
        pos = ticks[axis][ok][seg]
        if np.max(np.abs(cmd)) < 1e-4:
            print(f"{axis}: never commanded in the longest stretch")
            continue
        r = fit_motor(tt, cmd, pos)
        if r is None:
            continue
        print(f"{axis}: dead time {1e3 * r['delay']:.0f} ms, lag {1e3 * r['tau']:.0f} ms, "
              f"gain {r['gain']:.2f} (encoder rate / command), fit R^2 {r['r2']:.2f}")
        small = (np.abs(cmd) > 1e-4) & (np.abs(cmd) < 0.02)
        if small.sum() > 20:
            w = centred_rate(tt, pos, 0.03)
            ratio = np.median(np.abs(w[small]) / np.abs(cmd[small]))
            print(f"      small commands (<0.02 rad/s): median rate/command {ratio:.2f}"
                  + ("  <- stiction: small corrections barely move the axis" if ratio < 0.5 else ""))
        print(f"      -> filter_actuation_latency_ms ~ {1e3 * (r['delay'] + r['tau']):.0f} "
              f"(dead time + lag)")
    print(f"(fitted over {tt[-1] - tt[0]:.1f} s; a gain well below 1 means the drive does not "
          f"reach the commanded speed)")


def report_overshoot(ticks):
    heading("Acquisitions and overshoot")
    aiming = ticks["aiming"] == 1
    idx = np.where(aiming)[0]
    if not idx.size:
        print("never aimed")
        return
    episodes = np.split(idx, np.where(np.diff(idx) > 1)[0] + 1)
    rows = []
    t0 = ticks["t_s"][0]
    for ep in episodes:
        t = ticks["t_s"][ep]
        u = ticks["aim_now_u"][ep]
        v = ticks["aim_now_v"][ep]
        good = np.isfinite(u)
        if good.sum() < 20 or t[-1] - t[0] < 0.5:
            continue
        t, u, v = t[good], u[good], v[good]
        e0 = np.array([u[0], v[0]])
        n0 = np.linalg.norm(e0)
        if n0 < 30:
            continue
        d = e0 / n0
        proj = u * d[0] + v * d[1]
        within = np.where(proj < 0.1 * n0)[0]
        reach = (t[within[0]] - t[0]) if within.size else math.nan
        over = max(0.0, -np.min(proj))
        tail = np.hypot(u, v)[t > t[0] + 1.0]
        rows.append((t[0] - t0, n0, reach, over, 100 * over / n0,
                     float(np.sqrt(np.mean(tail ** 2))) if tail.size else math.nan))
    if not rows:
        print("no acquisition started more than 30 px off target and lasted 0.5 s")
        return
    print(f"{'start s':>8} {'error px':>9} {'to 10% s':>9} {'overshoot px':>13} {'%':>6} "
          f"{'rms after 1 s':>14}")
    for r in rows:
        print(f"{r[0]:8.1f} {r[1]:9.0f} {r[2]:9.2f} {r[3]:13.1f} {r[4]:6.0f} {r[5]:14.1f}")


def report_stabilisation(frames, ticks, gcfg, stationary):
    heading("Camera-motion compensation")
    try:
        fx, fy = float(gcfg["focal_x"]), float(gcfg["focal_y"])
        half_w, half_h = float(gcfg["det_w"]) / 2, float(gcfg["det_h"]) / 2
        level = float(gcfg.get("home_tilt", 0.0))
    except (KeyError, ValueError):
        print("run.txt lacks this gimbal's geometry")
        return
    if gcfg.get("driving") != "1":
        print("gimbal not driven: nothing to compensate")
        return

    tt = ticks["t_s"]
    eff = effective_pan(ticks, level)
    tilt = ticks["tilt"]
    w_pan = centred_rate(tt, eff, RATE_WINDOW_S)
    w_tilt = centred_rate(tt, tilt, RATE_WINDOW_S)

    # Accepted detections only: a gated one is by definition not where the
    # filter's target is. Each track gets its own target position, so a bird
    # that moved between acquisitions does not break the fit.
    est_on = frames["est"] == 1
    seed = np.concatenate([[0], np.cumsum(np.diff(est_on.astype(int)) == 1)])
    keep = (frames["measured"] == 1) & (frames["est_capture_s"] > 0)
    frames = dict(frames, track=seed.astype(float))
    f = select(frames, keep)
    if f["det"].size < 30:
        print("too few detections")
        return
    tc = f["est_capture_s"]
    inside = (tc > tt[0]) & (tc < tt[-1])
    f = select(f, inside)
    tc = tc[inside]
    du = f["det_cx"] - half_w
    dv = f["det_cy"] - half_h
    P = np.interp(tc, tt, eff)
    T = np.interp(tc, tt, tilt)
    WP = np.interp(tc, tt, w_pan)
    WT = np.interp(tc, tt, w_tilt)

    moving = np.hypot(WP, WT) > MOVING_RAD_S
    print(f"detections while the gimbal turned (> {MOVING_RAD_S} rad/s): {int(moving.sum())} "
          f"of {moving.size}")

    est = f["est"] == 1
    track = f["track"].astype(int)
    sp = np.hypot(f["vu"], f["vv"])
    for name, m in (("gimbal still", ~moving & est), ("gimbal turning", moving & est)):
        if m.sum() > 5:
            print(f"filter target speed, {name}: rms {np.sqrt(np.nanmean(sp[m] ** 2)):.1f} px/s")

    if not stationary:
        print("Run with --stationary over a window where the bird held still to fit the focal\n"
              "lengths and the capture-time error. With a moving bird the gimbal follows it, so\n"
              "its motion and the bird's cannot be told apart.")
        return
    if moving.sum() < 20:
        print("the gimbal barely moved in this window: nothing to fit (let it acquire the "
              "stationary bird from off-target a few times)")
        return

    # Stationary target: within one track, du + fx * P(t + d) is constant (and
    # likewise for v). Scan the capture-time error d and a focal scale for the
    # flattest result; a regression would linearise in d, which breaks down
    # for errors of a frame or more.
    readout_ms = 1e3 * np.nanmedian(f["readout_s"])
    ids = np.unique(track)

    def spread(delta, axis_eff, z, focal):
        a = np.interp(tc + delta, tt, axis_eff)
        s_ = z + focal * a
        r = np.empty_like(s_)
        for i in ids:
            m = track == i
            r[m] = s_[m] - np.median(s_[m])
        return r

    results = []
    for axis, z, series, travel, focal in (("pan/u", du, eff, np.ptp(P), fx),
                                            ("tilt/v", dv, tilt, np.ptp(T), fy)):
        if travel < 0.002:
            print(f"{axis}: the axis barely moved ({travel:.4f} rad), skipped")
            continue
        best = None
        for delta in np.arange(-0.4, 0.1001, 0.005):
            for scale in np.arange(0.80, 1.2001, 0.01):
                r = spread(delta, series, z, focal * scale)
                score = np.sqrt(np.mean(np.clip(r, -200, 200) ** 2))
                if best is None or score < best[0]:
                    best = (score, delta, scale)
        base = spread(0.0, series, z, focal)
        base_rms = np.sqrt(np.mean(np.clip(base, -200, 200) ** 2))
        print(f"{axis}: capture-time error {1e3 * best[1]:+.0f} ms, focal {focal * best[2]:.1f} "
              f"(configured {focal:.1f}, x{best[2]:.2f}); target spread {best[0]:.1f} px rms "
              f"(as configured {base_rms:.1f}) over {travel:.3f} rad, {ids.size} tracks")
        results.append((best[1], travel))
    if results:
        d = sum(x * w for x, w in results) / sum(w for _, w in results)
        new = readout_ms - 1e3 * d
        print(f"Capture-time error {1e3 * d:+.0f} ms (axes weighted by travel): the image was taken "
              f"{'later' if d > 0 else 'earlier'} than the filter assumes.")
        if new > 0:
            print(f"  -> readout_ms={new:.1f}  (now {readout_ms:.2f} ms)")
        else:
            print(f"  -> would need readout_ms={new:.1f}, but 0 or less means 'compute from the "
                  "caps'.\n     The error is then not in the readout estimate: suspect the encoder "
                  "timestamps or the PTS.")
    print("Trust these only if the fitted spread is near the detector jitter (it is the bird's\n"
          "own motion plus jitter); a much larger one means the bird moved after all.")


def report_performance(ticks):
    if "loop_us" not in ticks:
        return
    print("\n--- wall-time costs (including blocking/scheduling), microseconds ---")
    print("Nested timings overlap: bus costs are included in leader/follower operations; do not sum them.")

    def summary(label, values, include_zero=False):
        values = values[np.isfinite(values) & (values >= 0 if include_zero else values > 0)]
        if values.size:
            print(f"{label}: n={values.size} p50={pct(values, 50):.3f} "
                  f"p99={pct(values, 99):.3f} max={np.max(values):.3f}")

    def interval(label, start_key, end_key, eligible=None):
        if start_key not in ticks or end_key not in ticks:
            return
        start, end = ticks[start_key], ticks[end_key]
        mask = np.isfinite(start) & np.isfinite(end) & (start > 0) & (end >= start)
        if eligible is not None:
            mask &= eligible
        summary(label, (end[mask] - start[mask]) * 1e6, include_zero=True)

    for key in ("loop_us", "pose_read_us", "observation_wait_us", "filter_us", "control_us",
                "aim_compute_us", "follower_us", "previous_submit_us", "bus_lock_wait_us",
                "sdk_read_us", "sdk_write_us"):
        if key in ticks:
            summary(key, ticks[key])
    if "iteration_us" in ticks:
        summary("loop start spacing", ticks["iteration_us"])
    elif "loop_start_s" in ticks:
        summary("loop start spacing", np.diff(ticks["loop_start_s"]) * 1e6)
    for key in ("sdk_read_calls", "sdk_write_calls", "skipped_observations"):
        if key in ticks:
            values = ticks[key][np.isfinite(ticks[key])]
            print(f"{key}: total={np.sum(values):.0f}")
            summary(key + " per tick (count)", values, include_zero=True)

    interval("leader command call (all attempts)", "command_start_s", "command_done_s")
    interval("aim through leader command return", "aim_s", "command_done_s")
    interval("capture -> probe (aggregate pipeline)", "source_capture_s", "source_probe_s",
             ticks.get("observation_received_s", np.zeros_like(ticks["loop_us"])) > 0)
    interval("probe -> publication", "source_probe_s", "source_publish_s",
             ticks.get("observation_received_s", np.zeros_like(ticks["loop_us"])) > 0)
    interval("publication -> consumption", "source_publish_s", "observation_received_s")
    if "command_start_s" in ticks and "aiming" in ticks:
        aiming = (ticks["aiming"] == 1) & (ticks["command_start_s"] > 0)
        interval("source probe -> aim command return (all attempts)", "source_probe_s",
                 "command_done_s", aiming)
        interval("source capture -> aim command return (all attempts)", "source_capture_s",
                 "command_done_s", aiming)
        if "pan_write_status" in ticks and "tilt_write_status" in ticks:
            ok = (ticks["pan_write_status"] == 1) & (ticks["tilt_write_status"] == 1)
            interval("source capture -> aim command return (SDK writes OK)", "source_capture_s",
                     "command_done_s", aiming & ok)
            print(f"aim commands with failed/unavailable SDK writes: {np.sum(aiming & ~ok)}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("run")
    ap.add_argument("--gimbal", type=int)
    ap.add_argument("--from", dest="t_from", type=float)
    ap.add_argument("--to", dest="t_to", type=float)
    ap.add_argument("--stationary", action="store_true",
                    help="the bird held still in the window: fit focal lengths and capture time")
    args = ap.parse_args()

    info = parse_run(os.path.join(args.run, "run.txt"))
    frames_all = load_csv(os.path.join(args.run, "frames.csv"))
    ticks_all = load_csv(os.path.join(args.run, "ticks.csv"))
    if not ticks_all:
        sys.exit("no ticks recorded")
    if "dropped" in info and any(info["dropped"]):
        print(f"WARNING: telemetry dropped {info['dropped'][0]} frames and "
              f"{info['dropped'][1]} ticks (writer fell behind)")

    gimbals = sorted(set(ticks_all["gimbal"].astype(int)))
    if args.gimbal is not None:
        gimbals = [args.gimbal]
    t_start = np.nanmin(ticks_all["t_s"])
    for g in gimbals:
        ticks = select(ticks_all, ticks_all["gimbal"] == g)
        frames = select(frames_all, frames_all["gimbal"] == g) if frames_all else {}
        lo = t_start + (args.t_from if args.t_from is not None else -1e9)
        hi = t_start + (args.t_to if args.t_to is not None else 1e9)
        ticks = select(ticks, (ticks["t_s"] >= lo) & (ticks["t_s"] <= hi))
        if frames:
            frames = select(frames, (frames["step_s"] >= lo) & (frames["step_s"] <= hi))
        gcfg = info["gimbals"].get(g, {})
        print(f"\n######## gimbal {g}  ({gcfg.get('camera', '?')} camera) ########")
        if not ticks or ticks["t_s"].size < 10:
            print("no ticks in the window")
            continue
        if frames and frames["frame_id"].size:
            report_timing(frames, ticks)
            report_filter(frames, gcfg)
        report_performance(ticks)
        report_motors(ticks)
        report_overshoot(ticks)
        if frames and frames["frame_id"].size:
            report_stabilisation(frames, ticks, gcfg, args.stationary)


if __name__ == "__main__":
    main()
