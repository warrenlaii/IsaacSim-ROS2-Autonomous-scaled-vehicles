#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# CobraFlex ROS Bag Analyzer -- Thesis Handover
# Source lineage: cobraflex_rosbag_analyzer_v27.py
# Handover cleanup: stable module name, portable paths, English-only text.
# Changelog vs v25:
#   - FIX (2026-08-04): real and simulation bags now use separate timebase
#     policies. Only simulation maps headerless/invalid-stamp topics through
#     the odometry record/header relationship. Real /cmd_vel keeps rosbag
#     record time while stamped ZED odometry and IMU keep header.stamp.
#   - FIX: Test 7 angular displacement and odom/gyro agreement are calculated
#     over the actual 5 s yaw-step hold, not over the whole bag (which also
#     contains the preceding straight segment and post-command tail).
#   - ADD: Test 7 reports steady-state yaw rate/tracking and explicitly marks
#     transient delay/tau as qualified when limited by odometry resolution.
# Earlier v20-v25 changes:
#   - FIX: Isaac Sim data completeness is mode-aware. Real-robot-only
#     wheel_speeds, feedback, and ZED pose topics no longer make otherwise
#     complete sim bags appear invalid.
#   - FIX: saturated Test 1 commands report rise time, sustained acceleration,
#     and overshoot as N/A instead of the misleading value 0.000.
#   - FIX: Test 7 step response now reports delay and 10%-to-63% time constant
#     with the odometry sample resolution. The old cross-correlation had its
#     lag sign reversed and therefore clipped every positive delay to zero.
#   - FIX: rotation terminology now distinguishes odometry pose yaw from a
#     true ZED VIO/SLAM channel. Tracking below 100% is not called overshoot.
#   - FIX: voltage drop and actuator effort are N/A when their source data is
#     unavailable; breakaway detection ignores pre-command wheel noise.
#   - FIX: peak acceleration KPIs report magnitudes and roll is labelled as a
#     rate, matching the underlying IMU signals and units.
#   - A2: steady-state command tracking. active_command_window() +
#     steady_state_velocity_from_distance() (slope-fit over the cruise
#     segment) give ramp-free, differentiation-noise-free wheel and chassis
#     cruise speeds. New "Steady-State Command Tracking (A2)" row for linear
#     tests reports chassis/cmd, wheel/cmd, wheel/chassis.
#   - FIX: Test 1 & Test 5 slip ratio no longer use peak-of-differentiated
#     encoder speed (which inflated to 300-400% on real encoder data); they
#     now use the A2 steady-state speeds. Old slip numbers in those two
#     tests should be treated as invalid.
#   - A4: odometry-pose yaw vs gyro cross-check also reports pose/gyro ratio.
#   - FIX (2026-07-14): all stamped sensor topics now use ROS header.stamp
#     as the analysis clock. Headerless topics (for example /cmd_vel) are
#     mapped onto that clock through the odometry record/header pairs.
#     MCAP record time is retained only for publish-arrival diagnostics.
#     This prevents non-realtime Isaac runs (RTF != 1) from inflating wheel
#     distance and fabricating a large longitudinal-slip ratio.
#   - v22->v23: added explicit /joint_states publish-rate row (contract
#     requires >=30Hz); was previously computed internally but never
#     surfaced in the report table.
#   - v24 (2026-07-20, CobraFlex_Work_Log_2026-07-20.md Section 11/13):
#     Test 4's Angular Tracking Ratio now has a PRIMARY steady-state
#     variant fit over a fixed 2-9s window (cmd-start-relative), via new
#     cumulative_unwrapped_angle_series()/rotational_steady_state_from_yaw().
#     The old whole-run integral ratio is kept but relabeled "(legacy,
#     whole-run integral)" -- it mixed bag.times_cmd's full active window
#     against however many bag.odom_yaw samples existed, which could be a
#     shorter/misaligned span if odometry's first messages were missing,
#     producing spurious 33%/39%/48% readings unrelated to true tracking.
#     The 0-2s startup transient is now reported separately instead of
#     being silently averaged into either ratio.
#   - v25 (2026-07-20): low-speed path integration no longer discards every
#     sub-millimetre step; duplicate-pose checks use x/y/yaw and exclude true
#     stationary intervals; Test 4 marks an unrecorded pre-roll transient as
#     N/A instead of printing a negative time window; wheel breakaway uses a
#     threshold below the theoretical W=0.2 wheel-surface speed; pure-rotation
#     distance rows no longer imply that wheel travel should equal chassis-
#     centre travel.
# v19 recap: A1 three-path distance cross-check (/odom_truth parsing,
#   path-vs-endpoint distance, duplicate-sample detection).

import sys
import os
import math
import numpy as np
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QPushButton, QGroupBox, 
                             QFileDialog, QMessageBox, QTableWidget, 
                             QTableWidgetItem, QHeaderView)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont, QColor

import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

# ---------------------------------------------------------
# Advanced Math & DSP Functions
# ---------------------------------------------------------
try:
    from scipy.signal import butter, filtfilt
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False

def zero_phase_filter(data, window=9, cutoff=0.1):
    """Zero-phase filter (no time-lag introduced), used before edge/threshold detection."""
    if not data or len(data) == 0: return []
    if len(data) < window: return data
    if SCIPY_AVAILABLE:
        try:
            b, a = butter(3, cutoff, btype='low')
            return filtfilt(b, a, data).tolist()
        except: pass
    w = np.ones(window) / window
    s = np.convolve(data, w, mode='same').tolist()
    s[:window], s[-window:] = data[:window], data[-window:]
    return s

def safe_trapz(y, x):
    """Compatibility wrapper: NumPy 2.0's trapezoid vs 1.x's trapz."""
    if hasattr(np, 'trapezoid'):
        return float(np.trapezoid(y, x))
    return float(np.trapz(y, x))

def quaternion_to_yaw(q):
    """Extract yaw (rotation about Z) from a geometry_msgs/Quaternion-like object."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def unwrap_delta(prev_yaw, curr_yaw):
    """Shortest signed angular step from prev_yaw to curr_yaw, in radians."""
    delta = curr_yaw - prev_yaw
    while delta > math.pi:
        delta -= 2.0 * math.pi
    while delta < -math.pi:
        delta += 2.0 * math.pi
    return delta

def cumulative_unwrapped_angle(yaw_list):
    """
    Total rotation (rad, signed) accumulated over a sequence of absolute
    yaw samples, using shortest-path unwrapping between consecutive
    samples. This mirrors angular_velocity_sweep.py's live tracking
    method and is immune to publish-rate stalls / gaps in the bag: as
    long as the orientation at each recorded sample is correct, a gap in
    the middle doesn't fabricate or lose rotation the way integrating a
    (possibly stale) angular-velocity signal over a large dt can.
    Only fails if the true rotation between two consecutive SAMPLES
    (not the whole test) exceeds 180 degrees, which requires an
    extremely low sample rate relative to the spin rate.
    """
    if len(yaw_list) < 2:
        return 0.0
    total = 0.0
    for i in range(1, len(yaw_list)):
        total += unwrap_delta(yaw_list[i - 1], yaw_list[i])
    return total

def cumulative_unwrapped_angle_series(times, yaw_list):
    """
    Same unwrapping as cumulative_unwrapped_angle(), but returns
    (times, cumulative_angle) as parallel arrays instead of just the
    final total -- needed to fit a STEADY-STATE yaw rate (slope) over a
    specific time window, the same way steady_state_velocity_from_distance()
    does for linear speed.

    2026-07-20: added because the whole-run Angular Tracking Ratio
    (target_yaw_angle from bag.times_cmd's full active window vs
    actual_yaw_angle from however many bag.odom_yaw samples existed) mixes
    two different, sometimes misaligned time windows -- if odometry's
    first few messages are missing (see CobraFlex_Work_Log_2026-07-20.md
    Section 11), the whole-run actual angle silently comes from a shorter
    span than the whole-run commanded angle, giving readings like 33%,
    39%, 48% that don't reflect the real steady-state tracking. This
    function lets the caller fit slope over an explicit, common window
    instead (see rotational_steady_state_from_yaw()).
    """
    if not times or not yaw_list or len(times) != len(yaw_list) or len(yaw_list) < 2:
        return [], []
    cum = [0.0]
    for i in range(1, len(yaw_list)):
        cum.append(cum[-1] + unwrap_delta(yaw_list[i - 1], yaw_list[i]))
    return list(times), cum

def rotational_steady_state_from_yaw(times, yaw_list, t_start, t_end, min_points=5):
    """
    Estimate steady-state yaw rate (rad/s, signed) as the SLOPE of
    cumulative unwrapped yaw over [t_start, t_end] -- the angular
    equivalent of steady_state_velocity_from_distance(). Unlike that
    function, the window here is passed in explicitly by the caller
    (fixed absolute offsets, e.g. cmd_start+2s to cmd_start+9s) rather
    than being a ramp_frac/tail_frac fraction of the active-command span,
    per CobraFlex_Work_Log_2026-07-20.md Section 11's fixed 2-9s
    steady-state window decision for Test 4.

    Returns slope (rad/s, signed) or None if the window has too few
    points.
    """
    if (not times or not yaw_list or len(times) != len(yaw_list)
            or t_start is None or t_end is None or t_end <= t_start):
        return None
    ts, cum = cumulative_unwrapped_angle_series(times, yaw_list)
    tw, cw = [], []
    for t, c in zip(ts, cum):
        if t_start <= t <= t_end:
            tw.append(t)
            cw.append(c)
    if len(tw) < min_points:
        return None
    return float(np.polyfit(tw, cw, 1)[0])

def gyro_integrated_yaw(times, wz):
    """
    Forward-Euler integration of gyro angular_velocity.z (rad/s) over time
    -> total rotation (rad). Independent cross-check against odometry pose
    yaw from cumulative_unwrapped_angle() above. In Isaac Sim the odometry
    pose is produced by isaac_compute_odometry and is not ZED VIO. On a real
    robot the exact odometry source must be documented before interpreting a
    disagreement as a sensor fault.
    """
    if len(times) < 2:
        return 0.0
    total = 0.0
    for i in range(1, len(times)):
        dt = times[i] - times[i - 1]
        if dt > 0:
            total += wz[i - 1] * dt
    return total


def yaw_step_window(times_cmd, cmd_w):
    """Return (start, end, command_level) for Test 7's largest yaw step."""
    if len(times_cmd) < 2 or len(times_cmd) != len(cmd_w):
        return None
    delta = np.diff(np.abs(np.asarray(cmd_w, dtype=float)))
    if not len(delta):
        return None
    step_idx = int(np.argmax(delta)) + 1
    command_level = float(cmd_w[step_idx])
    if abs(command_level) < 0.01:
        return None
    end_idx = len(cmd_w) - 1
    for idx in range(step_idx + 1, len(cmd_w)):
        if abs(cmd_w[idx]) < 0.5 * abs(command_level):
            end_idx = idx
            break
    start, end = float(times_cmd[step_idx]), float(times_cmd[end_idx])
    return (start, end, command_level) if end > start else None


def integrate_series_window(times, values, start, end):
    """Trapezoidal integral with linearly interpolated window boundaries."""
    if (len(times) < 2 or len(times) != len(values) or end <= start
            or start < times[0] or end > times[-1]):
        return None
    t = np.asarray(times, dtype=float)
    y = np.asarray(values, dtype=float)
    inner = (t > start) & (t < end)
    tw = np.concatenate(([start], t[inner], [end]))
    yw = np.concatenate((
        [float(np.interp(start, t, y))], y[inner], [float(np.interp(end, t, y))]
    ))
    return safe_trapz(yw, tw)


def pose_yaw_change_window(times, yaw, start, end):
    """Signed pose-yaw change over a common Test 7 command window."""
    if (len(times) < 2 or len(times) != len(yaw) or end <= start
            or start < times[0] or end > times[-1]):
        return None
    t = np.asarray(times, dtype=float)
    unwrapped = np.unwrap(np.asarray(yaw, dtype=float))
    return float(np.interp(end, t, unwrapped) - np.interp(start, t, unwrapped))

def median_positive_sample_period(times):
    """Median positive sample interval, or None when it cannot be estimated."""
    if times is None or len(times) < 2:
        return None
    dt = np.diff(np.asarray(times, dtype=float))
    dt = dt[dt > 1e-9]
    return float(np.median(dt)) if len(dt) else None


def estimate_step_response(t_cmd, cmd, t_act, act):
    """
    Estimate Test 7 yaw step delay and a first-order time constant.

    The command step is the largest positive change in |cmd_w|. Delay is
    measured from that command timestamp to the first 10% response crossing;
    tau is the additional time from the 10% crossing to the 63.2% crossing.
    Values at or below one odometry sample are retained numerically and later
    displayed as "< one sample" rather than the false precision 0.000 s.

    Returns a dict containing delay, tau, resolution, and a status string.
    Missing/unobservable values are None, never a fabricated zero.
    """
    result = {
        "delay": None,
        "tau": None,
        "resolution": median_positive_sample_period(t_act),
        "status": "insufficient data",
    }
    try:
        if (len(t_cmd) < 2 or len(cmd) != len(t_cmd)
                or len(t_act) < 3 or len(act) != len(t_act)):
            return result

        t_cmd_arr = np.asarray(t_cmd, dtype=float)
        cmd_arr = np.asarray(cmd, dtype=float)
        t_act_arr = np.asarray(t_act, dtype=float)
        act_arr = np.asarray(act, dtype=float)

        cmd_step = np.diff(np.abs(cmd_arr))
        if not len(cmd_step):
            return result
        step_before = int(np.argmax(cmd_step))
        step_size = float(cmd_step[step_before])
        step_idx = step_before + 1
        command_level = float(cmd_arr[step_idx])
        if step_size < 0.01 or abs(command_level) < 0.01:
            result["status"] = "no sustained yaw step found"
            return result
        t_step = float(t_cmd_arr[step_idx])

        # End of the active yaw command, used only to define a steady window.
        active_end = float(t_cmd_arr[-1])
        for i in range(step_idx + 1, len(cmd_arr)):
            if abs(cmd_arr[i]) < 0.5 * abs(command_level):
                active_end = float(t_cmd_arr[i])
                break
        active_span = active_end - t_step
        if active_span <= 0:
            result["status"] = "yaw step has no active hold interval"
            return result

        before = act_arr[(t_act_arr >= t_step - min(0.5, active_span * 0.25))
                         & (t_act_arr < t_step)]
        baseline = float(np.median(before)) if len(before) else 0.0

        steady = act_arr[(t_act_arr >= t_step + 0.50 * active_span)
                         & (t_act_arr <= t_step + 0.90 * active_span)]
        if not len(steady):
            result["status"] = "no samples in yaw steady-state window"
            return result
        steady_value = float(np.median(steady))
        amplitude = steady_value - baseline
        if abs(amplitude) < 0.01:
            result["status"] = "yaw response amplitude is too small"
            return result

        direction = 1.0 if amplitude > 0 else -1.0

        def first_crossing(fraction):
            threshold = fraction * abs(amplitude)
            for t, value in zip(t_act_arr, act_arr):
                if t >= t_step and direction * (value - baseline) >= threshold:
                    return float(t)
            return None

        t10 = first_crossing(0.10)
        t63 = first_crossing(0.632)
        if t10 is None:
            result["status"] = "10% yaw response crossing not observed"
            return result

        result["delay"] = max(0.0, t10 - t_step)
        result["tau"] = max(0.0, t63 - t10) if t63 is not None else None
        result["status"] = "ok" if t63 is not None else "63% crossing not observed"
        return result
    except Exception as exc:
        result["status"] = f"step-response calculation failed: {exc}"
        return result

def get_steady_state_mean(data, start_ratio=0.5, end_ratio=0.9):
    if not data or len(data) < 10: return 0.0
    return float(np.mean(data[int(len(data)*start_ratio):int(len(data)*end_ratio)]))

def get_active_cmd(cmd_list):
    if not cmd_list: return 0.0
    active = [c for c in cmd_list if abs(c) > 0.01]
    return float(active[np.argmax(np.abs(active))]) if active else 0.0

def compute_path_and_endpoint(xs, ys, ds_threshold=1e-9):
    """
    Given an x/y sample series, return (cumulative_path_distance_list,
    endpoint_displacement).

    - Path distance: sum of consecutive-sample step lengths above a tiny
      numerical epsilon.  The previous 1 mm/sample deadband deleted valid
      motion below 0.06 m/s at 60 Hz, so a run could drift 0.15 m while the
      path integral incorrectly remained 0.000 m.  Sensor-noise assessment
      belongs in the baseline-noise test, not in a fixed per-sample distance
      gate that also removes real low-speed motion.
    - Endpoint displacement: straight-line distance between the FIRST
      and LAST sample only, independent of what happened in between.

    This split was introduced for the A1 investigation. Endpoint displacement is
    immune to frozen/duplicate mid-stream samples (they don't move the
    first or last point), whereas the path integral is not obviously
    affected either (a duplicate sample contributes ds=0 either way).
    Comparing the two pinpoints whether an observed distance difference
    behaves like "the path is genuinely short" (both metrics agree) or
    like an intermediate sampling artifact (they disagree). With the corrected
    physics-step graph, odom, odom_truth, and joint-derived distance should agree.
    """
    if not xs or not ys or len(xs) < 2:
        return [0.0], 0.0
    dist = [0.0]
    for i in range(1, len(xs)):
        dx = xs[i] - xs[i - 1]
        dy = ys[i] - ys[i - 1]
        ds = math.hypot(dx, dy)
        dist.append(dist[-1] + ds if ds > ds_threshold else dist[-1])
    endpoint_disp = math.hypot(xs[-1] - xs[0], ys[-1] - ys[0])
    return dist, endpoint_disp

def count_duplicate_samples(xs, ys, yaws=None, linear_speeds=None,
                            angular_speeds=None, eps=1e-9, yaw_eps=1e-9,
                            motion_v_eps=1e-4, motion_w_eps=1e-4):
    """
    Count consecutive FULL-POSE samples whose x/y/yaw are (near-)identical
    while the reported twist says the robot is moving.

    The old x/y-only check was invalid for Test 4: an ideal in-place turn
    keeps x/y fixed while yaw advances.  Conversely, a correctly stationary
    pre-roll may legitimately publish the same full pose repeatedly.  When
    velocity channels are supplied, stationary intervals are therefore
    excluded from both numerator and denominator.

    This is DISTINCT from a publish-rate stall (a large time gap between
    samples, tracked separately via MAX_INTEGRATION_DT_SEC): a duplicate
    can arrive at a normal or even high message rate while silently
    re-sending the same pose. This is the on_playback_tick / 240 Hz
    physics-tick decoupling artifact noted in the project's findings
    ("~50% duplicate odom samples"). Returns (duplicate_count,
    eligible_motion_steps, duplicate_pct_of_eligible_steps).
    """
    if not xs or not ys or len(xs) < 2:
        return 0, 0, 0.0
    use_yaw = yaws is not None and len(yaws) == len(xs)
    use_motion = (linear_speeds is not None and angular_speeds is not None
                  and len(linear_speeds) == len(xs)
                  and len(angular_speeds) == len(xs))
    dup = 0
    eligible = 0
    for i in range(1, len(xs)):
        if use_motion:
            moving = (
                max(abs(linear_speeds[i - 1]), abs(linear_speeds[i])) > motion_v_eps
                or max(abs(angular_speeds[i - 1]), abs(angular_speeds[i])) > motion_w_eps
            )
            if not moving:
                continue
        eligible += 1
        same_xy = (abs(xs[i] - xs[i - 1]) < eps
                   and abs(ys[i] - ys[i - 1]) < eps)
        same_yaw = (not use_yaw
                    or abs(unwrap_delta(yaws[i - 1], yaws[i])) < yaw_eps)
        if same_xy and same_yaw:
            dup += 1
    pct = (dup / eligible * 100.0) if eligible > 0 else 0.0
    return dup, eligible, pct

def header_stamp_seconds(msg):
    """Return msg.header.stamp in seconds, or None for an unstamped message."""
    try:
        stamp = msg.header.stamp
        value = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        return value if math.isfinite(value) else None
    except (AttributeError, TypeError, ValueError):
        return None

def usable_header_timestamps(header_times):
    """
    True when a topic has a real, monotonic ROS header clock.

    Duplicate stamps are allowed: the legacy PlaybackTick publishers can
    emit the same state twice. What is rejected is a missing/constant clock
    or a clock that moves backwards. A valid zero stamp at simulation start
    is therefore handled correctly.
    """
    if len(header_times) < 2 or any(t is None for t in header_times):
        return False
    values = np.asarray(header_times, dtype=float)
    if not np.all(np.isfinite(values)) or values[-1] - values[0] <= 1e-9:
        return False
    diffs = np.diff(values)
    return bool(np.all(diffs >= -1e-9) and np.any(diffs > 1e-9))

def map_record_times_to_header(record_times, anchor_record, anchor_header):
    """
    Map headerless-topic MCAP record times onto the ROS header clock.

    Piecewise interpolation preserves command transition timing despite
    recorder jitter. Values just outside the odometry anchor interval use
    the whole-run sim/record slope rather than a noisy one-sample slope.
    """
    if not record_times:
        return []
    if len(anchor_record) < 2 or len(anchor_record) != len(anchor_header):
        return list(record_times)

    pairs = sorted((float(r), float(h)) for r, h in zip(anchor_record, anchor_header)
                   if math.isfinite(r) and math.isfinite(h))
    unique = []
    for record_t, header_t in pairs:
        if unique and abs(record_t - unique[-1][0]) <= 1e-12:
            unique[-1] = (record_t, header_t)
        else:
            unique.append((record_t, header_t))
    if len(unique) < 2 or unique[-1][0] - unique[0][0] <= 1e-12:
        return list(record_times)

    record_anchor = np.asarray([p[0] for p in unique], dtype=float)
    header_anchor = np.asarray([p[1] for p in unique], dtype=float)
    query = np.asarray(record_times, dtype=float)
    mapped = np.interp(query, record_anchor, header_anchor)
    global_slope = ((header_anchor[-1] - header_anchor[0]) /
                    (record_anchor[-1] - record_anchor[0]))
    left = query < record_anchor[0]
    right = query > record_anchor[-1]
    mapped[left] = header_anchor[0] + (query[left] - record_anchor[0]) * global_slope
    mapped[right] = header_anchor[-1] + (query[right] - record_anchor[-1]) * global_slope
    return mapped.tolist()

def timing_statistics(analysis_times, record_times, source):
    """Compute simulation/header rate separately from MCAP arrival rate."""
    count = min(len(analysis_times), len(record_times))
    result = {
        "count": count, "source": source, "analysis_span": 0.0,
        "record_span": 0.0, "analysis_rate": None, "record_rate": None,
        "rtf": None, "duplicate_timestamps": 0, "duplicate_pct": 0.0,
    }
    if count < 2:
        return result
    analysis = np.asarray(analysis_times[:count], dtype=float)
    record = np.asarray(record_times[:count], dtype=float)
    analysis_span = float(analysis[-1] - analysis[0])
    record_span = float(record[-1] - record[0])
    steps = count - 1
    duplicate_count = int(np.sum(np.abs(np.diff(analysis)) <= 1e-12))
    result.update({
        "analysis_span": analysis_span,
        "record_span": record_span,
        "analysis_rate": steps / analysis_span if analysis_span > 0 else None,
        "record_rate": steps / record_span if record_span > 0 else None,
        "rtf": analysis_span / record_span if record_span > 0 else None,
        "duplicate_timestamps": duplicate_count,
        "duplicate_pct": duplicate_count / steps * 100.0 if steps else 0.0,
    })
    return result

def active_command_window(times_cmd, cmd, active_thresh=0.05):
    """
    Find the [t_start, t_end] time window during which the command signal
    is sustained at its active (non-zero) level.

    Returns the start/end times of the longest contiguous run of samples
    whose |cmd| exceeds active_thresh. Used to bound the steady-state
    velocity fit so the ramp-up and the post-command decay are excluded.
    Returns (None, None) if there's no sustained active command.
    """
    if not times_cmd or not cmd or len(times_cmd) != len(cmd):
        return None, None
    best_start, best_end, best_len = None, None, 0.0
    run_start_idx = None
    for i, c in enumerate(cmd):
        if abs(c) > active_thresh:
            if run_start_idx is None:
                run_start_idx = i
        else:
            if run_start_idx is not None:
                length = times_cmd[i - 1] - times_cmd[run_start_idx]
                if length > best_len:
                    best_len, best_start, best_end = length, times_cmd[run_start_idx], times_cmd[i - 1]
                run_start_idx = None
    if run_start_idx is not None:  # command still active at end of bag
        length = times_cmd[-1] - times_cmd[run_start_idx]
        if length > best_len:
            best_len, best_start, best_end = length, times_cmd[run_start_idx], times_cmd[-1]
    return best_start, best_end

def steady_state_velocity_from_distance(times, dist, t_start, t_end,
                                        ramp_frac=0.25, tail_frac=0.10,
                                        min_points=5):
    """
    Estimate a steady-state speed (m/s) as the SLOPE of a cumulative-distance
    series over the steady portion of the active-command window.

    Why slope-of-distance rather than mean-of-differentiated-velocity:
    - Differentiating a cumulative distance / encoder-tick series sample by
      sample amplifies quantization + timing jitter into large spurious
      peaks (this is exactly what corrupted the old peak-wheel-speed slip
      ratio -- real-robot encoder diffs produced 300-400% "wheel speeds").
      A linear fit over a window is immune to that: it uses the net distance
      over net time, so per-sample noise averages out.
    - A whole-run average (total_dist / total_duration) is contaminated by
      the acceleration ramp and the deceleration tail, systematically
      under-reporting the true cruising speed. Trimming ramp_frac off the
      front and tail_frac off the back of the active-command window isolates
      the cruise segment.

    Returns slope (m/s, always >= 0) or None if the window is too short /
    has too few points. times and dist must be the same length and aligned.
    """
    if (not times or not dist or len(times) != len(dist)
            or t_start is None or t_end is None or t_end <= t_start):
        return None
    span = t_end - t_start
    w0 = t_start + ramp_frac * span
    w1 = t_end - tail_frac * span
    if w1 <= w0:
        return None
    ts, ds = [], []
    for t, d in zip(times, dist):
        if w0 <= t <= w1:
            ts.append(t)
            ds.append(d)
    if len(ts) < min_points:
        return None
    slope = float(np.polyfit(ts, ds, 1)[0])
    return abs(slope)

try:
    from cobraflex_topics import (
        TOPIC_CMD_VEL, TOPIC_ODOM, TOPIC_IMU, TOPIC_WHEEL_SPEEDS,
        TOPIC_BATTERY, TOPIC_TF, TOPIC_TF_STATIC, TOPIC_JOINT_STATES,
        TOPIC_FEEDBACK, TOPIC_POSE, TOPIC_ROBOT_DESCRIPTION,
        TOPIC_ODOM_TRUTH,
    )
except ImportError:
    # Fallback so this script still runs standalone if cobraflex_topics.py
    # isn't sitting next to it. Keep that file present -- it's shared with
    # the recorder and is what catches topic-name drift between the two.
    TOPIC_CMD_VEL = "/cmd_vel"
    TOPIC_ODOM = "/zed/zed_node/odom"
    TOPIC_IMU = "/zed/zed_node/imu/data"
    TOPIC_WHEEL_SPEEDS = "/cobraflex/wheel_speeds"
    TOPIC_BATTERY = "/cobraflex/battery"
    TOPIC_TF = "/tf"
    TOPIC_TF_STATIC = "/tf_static"
    TOPIC_JOINT_STATES = "/joint_states"
    TOPIC_FEEDBACK = "/cobraflex/feedback"
    TOPIC_POSE = "/zed/zed_node/pose"
    TOPIC_ROBOT_DESCRIPTION = "/robot_description"
    # Sim-side ground-truth odometry (Isaac Sim ActionGraph, separate from
    # /zed/zed_node/odom). Added for the A1 three-path distance cross-check
    # (CobraFlex_Critical_Review_2026-07-12.md): compares ZED path-integrated
    # distance against this topic's path-integrated AND endpoint-displacement
    # distance to isolate which stage of the pipeline carries the sim
    # straight-line undershoot. Real-robot bags simply won't have this topic
    # -- handled as "optional", not a missing-topic warning, further below.
    TOPIC_ODOM_TRUTH = "/odom_truth"

# System Constants
TICKS_PER_METER = 94.0  
# Wheel radius (m). Must match the ActionGraph's differential_controller
# inputs:wheelRadius (and the RL handover spec) so that a wheel-surface
# linear speed derived from /joint_states (rad/s * radius) is directly
# comparable to /cobraflex/wheel_speeds-derived enc_v (m/s) on the real
# robot. If you ever recalibrate the physical wheel radius, update both
# this constant and the Isaac Sim ActionGraph together.
WHEEL_RADIUS_M = 0.03725

# Test-4 wheel-surface breakaway threshold.  With B=0.150 m and W=0.2
# rad/s, the ideal wheel-surface speed is only W*B/2 = 0.015 m/s, so the
# previous 0.020 m/s threshold made that command impossible to detect.
# 0.005 m/s remains comfortably above the near-zero settled signal while
# preserving the low-command measurement.
WHEEL_BREAKAWAY_SPEED_THRESHOLD_MPS = 0.005

# Allow only timestamp-scale rounding at a requested analysis-window edge.
# Larger pre-command coverage gaps are reported as N/A, not silently fitted.
ANALYSIS_WINDOW_TOL_SEC = 0.05

# Guard against publish-rate stalls (Isaac Sim render/physics hiccups, DDS
# hiccups, system load spikes) corrupting velocity-integrated distance.
# /joint_states normally arrives every few ms; if a gap between consecutive
# messages exceeds this, we don't know what actually happened during that
# window (the vehicle may have kept moving, slowed, or the sim may have
# frozen) so we exclude that interval from the distance integral rather
# than silently assuming "constant velocity held for the whole gap", which
# can fabricate a large fictitious distance from a single stale sample.
MAX_INTEGRATION_DT_SEC = 0.15
# Portable default for handover use. Set COBRAFLEX_BAG_DIR to override it.
BAG_DIR_DEFAULT = os.environ.get(
    "COBRAFLEX_BAG_DIR",
    os.path.join(os.path.expanduser("~"), "cobraflex", "test", "test_result")
)

def smooth_data(data, window=9):
    if not data or len(data) == 0: return []
    if len(data) < window: return data
    w = np.ones(window) / window
    s = np.convolve(data, w, mode='same').tolist()
    s[:window], s[-window:] = data[:window], data[-window:]
    return s

def get_steady_state_mean(data, start_ratio=0.5, end_ratio=0.9):
    if not data or len(data) < 10: return 0.0
    start_idx = int(len(data) * start_ratio)
    end_idx = int(len(data) * end_ratio)
    return float(np.mean(data[start_idx:end_idx]))

def get_active_cmd(cmd_list):
    if not cmd_list: return 0.0
    active_cmds = [c for c in cmd_list if abs(c) > 0.01]
    if not active_cmds: return 0.0
    idx = np.argmax(np.abs(active_cmds))
    return active_cmds[idx]

# ==========================================
# Core Data Processing Module
# ==========================================
class BagAnalyzer:
    def __init__(self, bag_path):
        self.bag_path = bag_path
        self.typestore = get_typestore(Stores.ROS2_HUMBLE)
        
        self.times_cmd, self.cmd_v, self.cmd_w = [], [], []
        self.times_odom, self.odom_x, self.odom_y, self.odom_v, self.odom_w = [], [], [], [], []
        self.odom_vy = []  # lateral (twist.linear.y) velocity, for Slip Angle KPI
        self.odom_yaw = []  # per-sample yaw (rad), derived from orientation quaternion
        self.times_enc, self.enc_ticks = [], []
        self.times_imu, self.imu_ax, self.imu_ay, self.imu_wx, self.imu_wy, self.imu_wz = [], [], [], [], [], []
        self.times_bat, self.bat_vol = [], []
        self.times_joint, self.joint_v, self.joint_eff = [], [], []
        self.times_pose, self.pose_x, self.pose_y = [], [], []
        # Sim-side ground-truth odometry (/odom_truth), separate from
        # /zed/zed_node/odom. Optional -- absent on real-robot bags.
        self.times_odomtruth, self.odomtruth_x, self.odomtruth_y = [], [], []
        self.odomtruth_yaw = []
        self.odomtruth_v, self.odomtruth_w = [], []

        # MCAP record timestamps measure when rosbag received each message.
        # They are intentionally kept separate from the topic header clock:
        # in Isaac Sim, record time is wall time and changes with RTF/load.
        self.record_times_cmd, self.record_times_odom = [], []
        self.record_times_enc, self.record_times_imu = [], []
        self.record_times_bat, self.record_times_joint = [], []
        self.record_times_pose, self.record_times_odomtruth = [], []
        self.topic_time_sources = {}
        self.timing_stats = {}
        self.is_sim_bag = False

        self.feedback_warnings = 0
        self.feedback_msg_count = 0
        self.has_urdf = False
        self.has_tf = False
        
        self.odom_dist = [0.0]  
        self.enc_dist = [0.0]   
        self.enc_v = []
        self.joint_v_wheel_ms = []   # /joint_states angular speed converted to wheel-surface m/s
        self.joint_dist = [0.0]      # cumulative distance integrated from joint_v_wheel_ms
        self.joint_stall_count = 0   # number of publish-rate gaps excluded from the integral
        self.joint_stall_total_sec = 0.0
        # Same publish-rate-gap tracking as joint_stall_count above, but for
        # /zed/zed_node/odom. Added because the periodic AnyDesk-correlated
        # freeze artifact hits odom just as often as joint_states, and the
        # straight-line tests (1/2/5/8) depend on odom-derived velocity, not
        # joint_states -- so a freeze there was previously going undetected.
        self.odom_stall_count = 0
        self.odom_stall_total_sec = 0.0

        # A1 three-path distance cross-check state (see
        # CobraFlex_Critical_Review_2026-07-12.md). odomtruth_* mirrors
        # odom_dist/odom_stall_* above but for /odom_truth. endpoint
        # displacements and duplicate-sample counts are filled in after
        # load_data()'s main message loop, once full x/y series exist.
        self.odomtruth_dist = [0.0]
        self.odomtruth_stall_count = 0
        self.odomtruth_stall_total_sec = 0.0
        self.odom_endpoint_disp = 0.0
        self.odomtruth_endpoint_disp = 0.0
        self.odom_duplicate_count = 0
        self.odom_duplicate_steps = 0
        self.odom_duplicate_pct = 0.0
        self.odomtruth_duplicate_count = 0
        self.odomtruth_duplicate_steps = 0
        self.odomtruth_duplicate_pct = 0.0

        # Which signal actually backed the "wheel speed" numbers used in the
        # KPIs below, so the UI/report can be explicit about it instead of
        # silently mixing two different measurement principles:
        #   "encoder"      -> real /cobraflex/wheel_speeds ticks (real robot)
        #   "joint_states" -> derived from /joint_states * WHEEL_RADIUS_M
        #                      (used whenever wheel_speeds wasn't recorded,
        #                      e.g. every Isaac Sim bag)
        #   "none"         -> neither topic had data
        self.wheel_speed_source = "none"
        self.wheel_times = []

        self.load_data()
        
    def load_data(self):
        try:
            with Reader(self.bag_path) as reader:
                for conn, timestamp, rawdata in reader.messages():
                    t = timestamp / 1e9
                    
                    if conn.topic == TOPIC_CMD_VEL:
                        msg = self.typestore.deserialize_cdr(rawdata, conn.msgtype)
                        self.record_times_cmd.append(t)
                        self.times_cmd.append(t)
                        self.cmd_v.append(msg.linear.x)
                        self.cmd_w.append(msg.angular.z)
                    
                    elif conn.topic == TOPIC_ODOM:
                        msg = self.typestore.deserialize_cdr(rawdata, conn.msgtype)
                        self.record_times_odom.append(t)
                        self.times_odom.append(header_stamp_seconds(msg))
                        self.odom_x.append(msg.pose.pose.position.x)
                        self.odom_y.append(msg.pose.pose.position.y)
                        self.odom_v.append(msg.twist.twist.linear.x)
                        self.odom_w.append(msg.twist.twist.angular.z)
                        self.odom_vy.append(msg.twist.twist.linear.y)
                        q = msg.pose.pose.orientation
                        self.odom_yaw.append(quaternion_to_yaw(q))
                        
                    elif conn.topic == TOPIC_ODOM_TRUTH:
                        msg = self.typestore.deserialize_cdr(rawdata, conn.msgtype)
                        self.record_times_odomtruth.append(t)
                        self.times_odomtruth.append(header_stamp_seconds(msg))
                        self.odomtruth_x.append(msg.pose.pose.position.x)
                        self.odomtruth_y.append(msg.pose.pose.position.y)
                        self.odomtruth_yaw.append(
                            quaternion_to_yaw(msg.pose.pose.orientation))
                        self.odomtruth_v.append(msg.twist.twist.linear.x)
                        self.odomtruth_w.append(msg.twist.twist.angular.z)

                    elif conn.topic == TOPIC_POSE:
                        msg = self.typestore.deserialize_cdr(rawdata, conn.msgtype)
                        self.record_times_pose.append(t)
                        self.times_pose.append(header_stamp_seconds(msg))
                        self.pose_x.append(msg.pose.position.x)
                        self.pose_y.append(msg.pose.position.y)
                    
                    elif conn.topic == TOPIC_WHEEL_SPEEDS:
                        msg = self.typestore.deserialize_cdr(rawdata, conn.msgtype)
                        self.record_times_enc.append(t)
                        self.times_enc.append(header_stamp_seconds(msg))
                        self.enc_ticks.append((msg.linear.x + msg.linear.y) / 2.0)
                    
                    elif conn.topic == TOPIC_IMU:
                        msg = self.typestore.deserialize_cdr(rawdata, conn.msgtype)
                        self.record_times_imu.append(t)
                        self.times_imu.append(header_stamp_seconds(msg))
                        self.imu_ax.append(msg.linear_acceleration.x)
                        self.imu_ay.append(msg.linear_acceleration.y)
                        self.imu_wx.append(msg.angular_velocity.x)
                        self.imu_wy.append(msg.angular_velocity.y)
                        self.imu_wz.append(msg.angular_velocity.z)
                        
                    elif conn.topic == TOPIC_BATTERY:
                        msg = self.typestore.deserialize_cdr(rawdata, conn.msgtype)
                        self.record_times_bat.append(t)
                        self.times_bat.append(header_stamp_seconds(msg))
                        if hasattr(msg, 'data'): self.bat_vol.append(msg.data)
                        elif hasattr(msg, 'voltage'): self.bat_vol.append(msg.voltage)
                        
                    elif conn.topic == TOPIC_JOINT_STATES:
                        msg = self.typestore.deserialize_cdr(rawdata, conn.msgtype)
                        self.record_times_joint.append(t)
                        self.times_joint.append(header_stamp_seconds(msg))
                        if hasattr(msg, 'velocity') and len(msg.velocity) > 0:
                            mean_wheel_omega = float(np.mean(np.abs(msg.velocity)))
                            self.joint_v.append(mean_wheel_omega)
                            # rad/s -> wheel-surface m/s, same physical
                            # quantity /cobraflex/wheel_speeds represents
                            # on the real robot.
                            self.joint_v_wheel_ms.append(mean_wheel_omega * WHEEL_RADIUS_M)
                        if hasattr(msg, 'effort') and len(msg.effort) > 0:
                            self.joint_eff.append(float(np.mean(np.abs(msg.effort))))
                            
                    elif conn.topic == TOPIC_FEEDBACK:
                        msg = self.typestore.deserialize_cdr(rawdata, conn.msgtype)
                        self.feedback_msg_count += 1
                        text = str(msg.data).lower() if hasattr(msg, 'data') else ""
                        if "warn" in text or "error" in text or "overload" in text:
                            self.feedback_warnings += 1
                            
                    elif conn.topic == TOPIC_ROBOT_DESCRIPTION:
                        self.has_urdf = True
                    elif conn.topic == TOPIC_TF or conn.topic == TOPIC_TF_STATIC:
                        self.has_tf = True

            if not self.times_odom: 
                raise ValueError("No Odometry data found in bag.")

            # /odom_truth is simulation-only. /joint_states without the real
            # wheel-speed topic is retained as a secondary simulation hint.
            # This classification must happen before clock finalization:
            # record->header mapping is valid for non-realtime simulation but
            # must not be used to erase real sensor publication latency.
            self.is_sim_bag = bool(
                self.times_odomtruth
                or (self.times_joint and not self.times_enc)
            )

            # Establish one analysis clock before any dt, integration, stall,
            # command-window, or delay calculation. Prefer each topic's own
            # valid header stamp. Headerless topics are mapped through the
            # odometry record/header correspondence so all signals share the
            # same simulated-time axis.
            odom_has_header = usable_header_timestamps(self.times_odom)
            if odom_has_header:
                self.topic_time_sources[TOPIC_ODOM] = "header.stamp"
                odom_header_anchor = list(self.times_odom)
            else:
                self.topic_time_sources[TOPIC_ODOM] = "MCAP record time (fallback)"
                self.times_odom = list(self.record_times_odom)
                odom_header_anchor = None

            def finalize_topic_time(topic, candidates, records):
                if usable_header_timestamps(candidates):
                    self.topic_time_sources[topic] = "header.stamp"
                    return list(candidates)
                if self.is_sim_bag and odom_header_anchor is not None:
                    self.topic_time_sources[topic] = "record time mapped to odom header"
                    return map_record_times_to_header(
                        records, self.record_times_odom, odom_header_anchor)
                self.topic_time_sources[topic] = "rosbag record time"
                return list(records)

            # /cmd_vel is geometry_msgs/Twist and has no header by design.
            self.times_cmd = finalize_topic_time(TOPIC_CMD_VEL, [], self.record_times_cmd)
            self.times_enc = finalize_topic_time(TOPIC_WHEEL_SPEEDS, self.times_enc, self.record_times_enc)
            self.times_imu = finalize_topic_time(TOPIC_IMU, self.times_imu, self.record_times_imu)
            self.times_bat = finalize_topic_time(TOPIC_BATTERY, self.times_bat, self.record_times_bat)
            self.times_joint = finalize_topic_time(TOPIC_JOINT_STATES, self.times_joint, self.record_times_joint)
            self.times_pose = finalize_topic_time(TOPIC_POSE, self.times_pose, self.record_times_pose)
            self.times_odomtruth = finalize_topic_time(
                TOPIC_ODOM_TRUTH, self.times_odomtruth, self.record_times_odomtruth)

            self.timing_stats = {
                TOPIC_ODOM: timing_statistics(
                    self.times_odom, self.record_times_odom,
                    self.topic_time_sources[TOPIC_ODOM]),
                TOPIC_ODOM_TRUTH: timing_statistics(
                    self.times_odomtruth, self.record_times_odomtruth,
                    self.topic_time_sources[TOPIC_ODOM_TRUTH]),
                TOPIC_JOINT_STATES: timing_statistics(
                    self.times_joint, self.record_times_joint,
                    self.topic_time_sources[TOPIC_JOINT_STATES]),
                TOPIC_IMU: timing_statistics(
                    self.times_imu, self.record_times_imu,
                    self.topic_time_sources[TOPIC_IMU]),
            }

            # --- Coordinate Normalization & Alignment (V11) ---
            if self.odom_x and self.odom_y:
                o0x, o0y = self.odom_x[0], self.odom_y[0]
                self.odom_x = [x - o0x for x in self.odom_x]
                self.odom_y = [y - o0y for y in self.odom_y]
                
            if self.pose_x and self.pose_y:
                p0x, p0y = self.pose_x[0], self.pose_y[0]
                # Normalize Pose to start at (0,0)
                norm_pose_x = [x - p0x for x in self.pose_x]
                norm_pose_y = [y - p0y for y in self.pose_y]
                
                # Estimate Initial Yaw to align Pose with Odom
                if len(self.odom_x) > 10 and len(norm_pose_x) > 10:
                    odom_theta = math.atan2(self.odom_y[10], self.odom_x[10])
                    pose_theta = math.atan2(norm_pose_y[10], norm_pose_x[10])
                    rot_theta = odom_theta - pose_theta
                    
                    # Apply Rotation Matrix
                    self.pose_x = [x * math.cos(rot_theta) - y * math.sin(rot_theta) for x, y in zip(norm_pose_x, norm_pose_y)]
                    self.pose_y = [x * math.sin(rot_theta) + y * math.cos(rot_theta) for x, y in zip(norm_pose_x, norm_pose_y)]
                else:
                    self.pose_x = norm_pose_x
                    self.pose_y = norm_pose_y

            # Distance Accumulators (ZED /zed/zed_node/odom)
            self.odom_dist, self.odom_endpoint_disp = compute_path_and_endpoint(
                self.odom_x, self.odom_y
            )
            (self.odom_duplicate_count, self.odom_duplicate_steps,
             self.odom_duplicate_pct) = count_duplicate_samples(
                self.odom_x, self.odom_y, self.odom_yaw,
                self.odom_v, self.odom_w
            )

            # Odom state-update gap check uses header/simulation time. MCAP
            # record-time jitter belongs only in timing diagnostics above.
            if len(self.times_odom) > 1:
                for i in range(1, len(self.times_odom)):
                    dt = self.times_odom[i] - self.times_odom[i - 1]
                    if dt > MAX_INTEGRATION_DT_SEC:
                        self.odom_stall_count += 1
                        self.odom_stall_total_sec += dt

            # A1 three-path cross-check: same computation for /odom_truth
            # (sim-side ground truth), if this bag has it. Real-robot bags
            # simply leave these at their zero-initialized defaults.
            if self.times_odomtruth:
                self.odomtruth_dist, self.odomtruth_endpoint_disp = compute_path_and_endpoint(
                    self.odomtruth_x, self.odomtruth_y
                )
                (self.odomtruth_duplicate_count,
                 self.odomtruth_duplicate_steps,
                 self.odomtruth_duplicate_pct) = count_duplicate_samples(                    self.odomtruth_x, self.odomtruth_y, self.odomtruth_yaw,
                    self.odomtruth_v, self.odomtruth_w
                )
                if len(self.times_odomtruth) > 1:
                    for i in range(1, len(self.times_odomtruth)):
                        dt = self.times_odomtruth[i] - self.times_odomtruth[i - 1]
                        if dt > MAX_INTEGRATION_DT_SEC:
                            self.odomtruth_stall_count += 1
                            self.odomtruth_stall_total_sec += dt

            if self.enc_ticks:
                start_tick = self.enc_ticks[0]
                self.enc_dist = [abs(tk - start_tick) / TICKS_PER_METER for tk in self.enc_ticks]

            # Normalize timelines
            t0 = self.times_odom[0]
            self.times_cmd = [t - t0 for t in self.times_cmd]
            self.times_odom = [t - t0 for t in self.times_odom]
            self.times_enc = [t - t0 for t in self.times_enc]
            self.times_imu = [t - t0 for t in self.times_imu]
            self.times_bat = [t - t0 for t in self.times_bat]
            self.times_joint = [t - t0 for t in self.times_joint]
            self.times_pose = [t - t0 for t in self.times_pose]
            self.times_odomtruth = [t - t0 for t in self.times_odomtruth]

            # Encoder-derived velocity (d(enc_dist)/dt). The recorder only
            # gives cumulative wheel distance; differentiate it to get a
            # wheel-surface speed time series so it's comparable against
            # ZED's odom_v for a Longitudinal Slip Ratio KPI (this is in
            # the project's own analysis spec / sample output for Test 1,
            # but wasn't actually being computed anywhere in this file).
            if len(self.enc_dist) > 1 and len(self.times_enc) == len(self.enc_dist):
                for i in range(1, len(self.enc_dist)):
                    dt = self.times_enc[i] - self.times_enc[i - 1]
                    self.enc_v.append((self.enc_dist[i] - self.enc_dist[i - 1]) / dt if dt > 1e-4 else 0.0)

            # Joint-states-derived cumulative distance: integrate the
            # wheel-surface speed (rad/s * WHEEL_RADIUS_M) over time.
            # This is the fallback used whenever /cobraflex/wheel_speeds
            # has no data -- currently every Isaac Sim bag, since that
            # topic only exists on the real robot's motor driver firmware.
            if len(self.joint_v_wheel_ms) > 1 and len(self.times_joint) == len(self.joint_v_wheel_ms):
                for i in range(1, len(self.joint_v_wheel_ms)):
                    dt = self.times_joint[i] - self.times_joint[i - 1]
                    if dt > MAX_INTEGRATION_DT_SEC:
                        # Publish-rate stall: we don't know what velocity
                        # was actually held during this gap, so contribute
                        # 0 distance instead of falsely assuming the last
                        # known velocity persisted for the whole gap.
                        self.joint_stall_count += 1
                        self.joint_stall_total_sec += dt
                        self.joint_dist.append(self.joint_dist[-1])
                        continue
                    # Trapezoidal integration on simulation/header time.
                    # Repeated header stamps contribute dt=0 and therefore
                    # cannot inflate the distance.
                    step = (0.5 * (self.joint_v_wheel_ms[i - 1] +
                                   self.joint_v_wheel_ms[i]) * dt
                            if dt > 1e-9 else 0.0)
                    self.joint_dist.append(self.joint_dist[-1] + step)

            # --- Unified "wheel speed" signal used by the KPIs below ---
            # Prefer the real encoder (/cobraflex/wheel_speeds) whenever it
            # has data (i.e. real-robot bags). Fall back to the
            # /joint_states-derived estimate otherwise (i.e. every Isaac
            # Sim bag, since that topic doesn't exist in the ActionGraph).
            # Never silently mix the two within one bag.
            if self.enc_v:
                self.wheel_speed_source = "encoder"
                self.wheel_v = self.enc_v
                self.wheel_dist = self.enc_dist
                self.wheel_times = self.times_enc[1:]
            elif self.joint_v_wheel_ms:
                self.wheel_speed_source = "joint_states"
                self.wheel_v = self.joint_v_wheel_ms
                self.wheel_dist = self.joint_dist
                self.wheel_times = self.times_joint
            else:
                self.wheel_speed_source = "none"
                self.wheel_v = []
                self.wheel_dist = [0.0]
                self.wheel_times = []

            # Per-core-topic message counts, so the UI can flag a topic
            # that recorded 0 messages instead of silently letting every
            # downstream KPI default to 0.0 and look like a real reading.
            self.core_topic_counts = {
                TOPIC_CMD_VEL: len(self.times_cmd),
                TOPIC_ODOM: len(self.times_odom),
                TOPIC_IMU: len(self.times_imu),
                TOPIC_WHEEL_SPEEDS: len(self.times_enc),
                TOPIC_BATTERY: len(self.times_bat),
                TOPIC_JOINT_STATES: len(self.times_joint),
                TOPIC_FEEDBACK: self.feedback_msg_count,
                TOPIC_POSE: len(self.times_pose),
                TOPIC_ROBOT_DESCRIPTION: 1 if self.has_urdf else 0,
                TOPIC_TF: 1 if self.has_tf else 0,
                TOPIC_TF_STATIC: 1 if self.has_tf else 0,
            }
            # /odom_truth is sim-only (no equivalent on the real robot),
            # so it's tracked separately from core_topic_counts above --
            # its absence must NOT trigger the "MISSING core topic"
            # warning on real-robot bags.
            self.optional_topic_counts = {
                TOPIC_ODOM_TRUTH: len(self.times_odomtruth),
            }
            
        except Exception as e:
            print(f"Data Parsing Error: {e}")
            self.core_topic_counts = {}
            self.optional_topic_counts = {}

TEST_NAMES = {
    1: "Acceleration Test",
    2: "Full Braking Testing",
    3: "Steady-State Circular Driving",
    4: "In-place Skid-Steer Test",
    5: "Steady-State Max Velocity Test",
    6: "Square Trajectory Test / UMBmark",
    7: "Step Steer Input",
    8: "Coasting Testing",
    9: "Weight Transfer / Pitch Test",
    10: "Baseline Noise Floor Test",
}

def identify_test_id(folder_name):
    name = folder_name.lower()
    if "test01" in name or "test_01" in name: return 1
    if "test02" in name or "test_02" in name: return 2
    if "test03" in name or "test_03" in name: return 3
    if "test04" in name or "test_04" in name: return 4
    if "test05" in name or "test_05" in name: return 5
    if "test06" in name or "test_06" in name: return 6
    if "test07" in name or "test_07" in name: return 7
    if "test08" in name or "test_08" in name: return 8
    if "test09" in name or "test_09" in name: return 9
    if "test10" in name or "test_10" in name: return 10
    return 0

# ==========================================
# Shared KPI computation (headless, no Qt)
# ==========================================
# Extracted out of AnalyzerMainWindow.execute_analysis so the GUI and
# the batch script use the exact same KPI logic -- no risk of the two
# drifting apart. Returns (rows, bag) where rows is the list of
# (metric, value, target_param) tuples the GUI table displays, and bag
# is the BagAnalyzer instance (needed by the GUI for plotting; the
# batch script mostly ignores it).
def compute_test_kpis(bag_path, test_id):
    bag = BagAnalyzer(bag_path)

    if not bag.times_odom:
        raise ValueError("Failed to extract Odom data.")

    max_cmd_v = get_active_cmd(bag.cmd_v)
    max_cmd_w = get_active_cmd(bag.cmd_w)
    
    v_abs_sorted = np.sort(np.abs(bag.odom_v))
    w_abs_sorted = np.sort(np.abs(bag.odom_w))
    actual_v_mag = v_abs_sorted[int(len(v_abs_sorted)*0.95)] if len(v_abs_sorted) > 0 else 0.0
    actual_w_mag = w_abs_sorted[int(len(w_abs_sorted)*0.95)] if len(w_abs_sorted) > 0 else 0.0
    actual_v = actual_v_mag if np.mean(bag.odom_v) >= 0 else -actual_v_mag
    actual_w = actual_w_mag if np.mean(bag.odom_w) >= 0 else -actual_w_mag
    
    peak_ax = bag.imu_ax[np.argmax(np.abs(bag.imu_ax))] if bag.imu_ax else 0.0
    min_ax = min(bag.imu_ax) if bag.imu_ax else 0.0
    peak_ay = bag.imu_ay[np.argmax(np.abs(bag.imu_ay))] if bag.imu_ay else 0.0
    steady_ay = get_steady_state_mean(bag.imu_ay)

    test_duration = bag.times_odom[-1] - bag.times_odom[0]

    # --- A2 steady-state command tracking (see
    # CobraFlex_Critical_Review_2026-07-12.md, item A2) ---
    # The old drive-layer numbers were ambiguous: whole-run averages were
    # ramp-contaminated (undershoot) and peak-of-differentiated-encoder was
    # noise-inflated (300-400% garbage). Here we fit the SLOPE of each
    # cumulative-distance channel over the steady portion of the active
    # LINEAR command window, giving clean cruise speeds that can be compared
    # against the command and against each other:
    #   chassis (ZED odom pose) vs wheel (encoder / joint_states) vs command.
    # Only meaningful when there's a sustained linear command, so it stays
    # None for pure-rotation tests (Test 4) and is surfaced as a KPI row
    # only when max_cmd_v is nonzero.
    ss_cmd_t0, ss_cmd_t1 = active_command_window(bag.times_cmd, bag.cmd_v)
    ss_chassis_v = steady_state_velocity_from_distance(
        bag.times_odom, bag.odom_dist, ss_cmd_t0, ss_cmd_t1)
    ss_wheel_v = None
    if bag.wheel_speed_source == "encoder":
        ss_wheel_v = steady_state_velocity_from_distance(
            bag.times_enc, bag.enc_dist, ss_cmd_t0, ss_cmd_t1)
    elif bag.wheel_speed_source == "joint_states":
        ss_wheel_v = steady_state_velocity_from_distance(
            bag.times_joint, bag.joint_dist, ss_cmd_t0, ss_cmd_t1)


    # (not a noisy real-world measurement) is fine here -- it's a
    # controller setpoint, not subject to publish-rate stalls the way
    # a measured signal is.
    target_yaw_angle = safe_trapz(bag.cmd_w, bag.times_cmd) if len(bag.cmd_w) > 1 else 0.0

    # Actual rotation: derived directly from the orientation quaternion
    # (unwrap-summed absolute yaw deltas), NOT from trapezoidal
    # integration of odom.twist.angular.z. The old trapz(bag.odom_w,...)
    # approach silently mis-estimates total rotation whenever there's a
    # publish-rate gap in /zed/zed_node/odom (Isaac Sim render/physics
    # hiccups, DDS hiccups): it has to guess what angular velocity held
    # during the gap, which can badly over- or under-estimate a large
    # chunk of the total. This method only needs the yaw value AT each
    # received sample to be correct, so gaps just mean fewer samples,
    # not a fabricated/lost rotation -- same principle used by
    # angular_velocity_sweep.py's live tracking, so bag-based and
    # live-tracked results are now directly comparable.
    actual_yaw_angle = cumulative_unwrapped_angle(bag.odom_yaw) if bag.odom_yaw else 0.0

    # Odometry-pose/gyro cross-check (Test 4 & 7 use this -- see
    # gyro_integrated_yaw docstring). In Isaac Sim, /zed/zed_node/odom is
    # fed by isaac_compute_odometry and must not be described as ZED VIO.
    gyro_yaw_angle = gyro_integrated_yaw(bag.times_imu, bag.imu_wz) if bag.imu_wz else 0.0
    yaw_gyro_disagreement_deg = math.degrees(abs(abs(actual_yaw_angle) - abs(gyro_yaw_angle)))
    YAW_GYRO_DISAGREEMENT_THRESHOLD_DEG = 30.0  # same threshold as angular_velocity_sweep_v7.py

    # Pose/gyro ratio. With the corrected physics-step graph and common ROS
    # header clock, the two channels should agree near 100%. A persistent
    # deviation is a data-quality flag; it is not by itself proof of VIO
    # failure because the odometry producer differs between sim and real.
    pose_gyro_ratio = (abs(actual_yaw_angle) / abs(gyro_yaw_angle)) if abs(gyro_yaw_angle) > 1e-3 else None
    pose_gyro_str = f" | pose/gyro: {pose_gyro_ratio*100:.1f}%" if pose_gyro_ratio is not None else ""
    
    zed_dist_final = bag.odom_dist[-1] if bag.odom_dist else 0.0
    wheel_dist_final = bag.wheel_dist[-1] if bag.wheel_dist else 0.0
    # Label the wheel-side number by its actual source so a reader
    # never mistakes a /joint_states-derived estimate (sim bags) for
    # a real /cobraflex/wheel_speeds encoder reading (real-robot bags).
    wheel_src_label = {
        "encoder": "Enc",
        "joint_states": "Enc~(from /joint_states)",
        "none": "Enc(N/A)",
    }[bag.wheel_speed_source]
    start_x, start_y = bag.odom_x[0], bag.odom_y[0]
    end_x, end_y = bag.odom_x[-1], bag.odom_y[-1]
    # Same value as bag.odom_endpoint_disp (computed once in BagAnalyzer via
    # compute_path_and_endpoint) -- kept as a local alias since test_id==6
    # below already reads it under this name.
    euclidean_dist = bag.odom_endpoint_disp
    
    dist_error = abs(zed_dist_final - wheel_dist_final)
    if bag.wheel_speed_source == "none":
        dist_str = f"{zed_dist_final:.3f} m (wheel-side unavailable)"
    elif dist_error > 0.05:
        dist_str = f"Odom: {zed_dist_final:.3f} m | {wheel_src_label}: {wheel_dist_final:.3f} m"
    else:
        dist_str = f"{zed_dist_final:.3f} m"
        
    sys_status_str = f"URDF Valid: {'Yes' if bag.has_urdf else 'No'} | TF: {'Active' if bag.has_tf else 'Inactive'}"
    if bag.feedback_msg_count == 0:
        fb_status_str = f"N/A -- {TOPIC_FEEDBACK} not present"
    elif bag.feedback_warnings > 0:
        fb_status_str = f"{bag.feedback_warnings} Warnings Detected"
    else:
        fb_status_str = "Nominal (No Errors)"

    odomtruth_count = getattr(bag, 'optional_topic_counts', {}).get(TOPIC_ODOM_TRUTH, 0)
    is_sim_bag = (odomtruth_count > 0 or
                  (bag.wheel_speed_source == "joint_states" and len(bag.times_enc) == 0))
    sim_optional_topics = {TOPIC_WHEEL_SPEEDS, TOPIC_FEEDBACK, TOPIC_POSE} if is_sim_bag else set()
    missing_topics = [
        t for t, c in getattr(bag, 'core_topic_counts', {}).items()
        if c == 0 and t not in sim_optional_topics
    ]
    if missing_topics:
        completeness_str = "⚠ MISSING: " + ", ".join(missing_topics)
    else:
        completeness_str = ("OK -- required sim topics have data" if is_sim_bag
                            else "OK -- all required real-robot topics have data")
    optional_absent = [
        t for t in sim_optional_topics
        if getattr(bag, 'core_topic_counts', {}).get(t, 0) == 0
    ]
    if optional_absent:
        completeness_str += " | sim-optional absent: " + ", ".join(sorted(optional_absent))
    # /odom_truth is optional (sim-only), so its absence is a plain note,
    # not a "⚠ MISSING" warning -- avoids false alarms on real-robot bags.
    completeness_str += f" | {TOPIC_ODOM_TRUTH}: {'available (' + str(odomtruth_count) + ' msgs)' if odomtruth_count > 0 else 'not present (expected on real-robot bags)'}"

    wheel_source_str = {
        "encoder": "Real /cobraflex/wheel_speeds (real-robot bag)",
        "joint_states": "Derived from /joint_states x wheel radius (sim bag -- wheel_speeds not published by ActionGraph)",
        "none": "UNAVAILABLE -- neither wheel_speeds nor joint_states had data",
    }[bag.wheel_speed_source]

    odom_timing = getattr(bag, 'timing_stats', {}).get(TOPIC_ODOM, {})
    joint_timing = getattr(bag, 'timing_stats', {}).get(TOPIC_JOINT_STATES, {})

    def _fmt_rate(value):
        return f"{value:.2f} Hz" if value is not None else "N/A"

    if odom_timing:
        source = odom_timing.get("source", "unknown")
        if source == "header.stamp":
            if is_sim_bag:
                timebase_str = (
                    f"ROS header.stamp (simulation time) | odom sim: "
                    f"{_fmt_rate(odom_timing.get('analysis_rate'))} over "
                    f"{odom_timing.get('analysis_span', 0.0):.3f}s | rosbag arrival: "
                    f"{_fmt_rate(odom_timing.get('record_rate'))} over "
                    f"{odom_timing.get('record_span', 0.0):.3f}s | "
                    f"RTF: {odom_timing.get('rtf', 0.0):.3f}"
                )
                timebase_note = "PASS: simulation integration uses header time; arrival time is diagnostic only"
            else:
                timebase_str = (
                    f"ROS header.stamp (real sensor time) | odom: "
                    f"{_fmt_rate(odom_timing.get('analysis_rate'))} over "
                    f"{odom_timing.get('analysis_span', 0.0):.3f}s | rosbag arrival: "
                    f"{_fmt_rate(odom_timing.get('record_rate'))} over "
                    f"{odom_timing.get('record_span', 0.0):.3f}s | RTF: N/A (real robot)"
                )
                timebase_note = "PASS: real /cmd_vel uses record time; stamped sensors keep header time"
        else:
            timebase_str = f"{source} | odom rate: {_fmt_rate(odom_timing.get('analysis_rate'))}"
            timebase_note = "Fallback for bags without a usable ROS header clock; verify RTF=1 for simulation bags"
    else:
        timebase_str = "Unavailable"
        timebase_note = "Cannot verify integration clock"

    # /joint_states publish rate, contract requirement >=30 Hz. Same
    # header.stamp vs MCAP-arrival split as the odom timebase row above;
    # reuses the timing_statistics() result already computed for
    # TOPIC_JOINT_STATES instead of assuming it tracks the odom rate.
    JOINT_STATES_CONTRACT_HZ = 30.0
    if joint_timing and joint_timing.get("analysis_rate") is not None:
        joint_source = joint_timing.get("source", "unknown")
        joint_rate_val = joint_timing.get("analysis_rate")
        joint_rate_str = (
            f"{joint_source} | joint_states: {_fmt_rate(joint_rate_val)} over "
            f"{joint_timing.get('analysis_span', 0.0):.3f}s | MCAP arrival: "
            f"{_fmt_rate(joint_timing.get('record_rate'))} over "
            f"{joint_timing.get('record_span', 0.0):.3f}s"
        )
        joint_rate_note = (
            f"{'PASS' if joint_rate_val >= JOINT_STATES_CONTRACT_HZ else 'FAIL'}"
            f" -- contract requires >={JOINT_STATES_CONTRACT_HZ:.0f} Hz"
        )
    else:
        joint_rate_str = "Unavailable"
        joint_rate_note = "Cannot verify /joint_states publish rate"

    base_rows = [
        ("Data Completeness Check", completeness_str, "Sanity Check (Recorder Config)"),
        ("Analysis Timebase", timebase_str, timebase_note),
        ("Joint-States Publish Rate", joint_rate_str, joint_rate_note),
        ("Wheel-Speed Data Source", wheel_source_str, "Affects: Slip Ratio, Distance Calibration"),
    ]
    if joint_timing and joint_timing.get("duplicate_timestamps", 0) > 0:
        joint_steps = max(joint_timing.get("count", 0) - 1, 1)
        base_rows.append((
            "⚠ Duplicate Joint-State Timestamps",
            f"{joint_timing['duplicate_timestamps']}/{joint_steps} intervals "
            f"({joint_timing['duplicate_pct']:.1f}%) have identical header stamps; dt=0 and add zero distance",
            "Publisher still duplicates joint_states, but cannot inflate header-time integration"
        ))
    if bag.joint_stall_count > 0:
        base_rows.append((
            "⚠ Simulation-Time Data Gap (joint_states)",
            f"{bag.joint_stall_count} header-time gap(s) > {MAX_INTEGRATION_DT_SEC}s on /joint_states, total {bag.joint_stall_total_sec:.2f}s excluded from distance integral",
            "Stamped state gap -- inspect the publisher; MCAP arrival jitter alone does not trigger this"
        ))
    if bag.odom_stall_count > 0:
        base_rows.append((
            "⚠ Simulation-Time Data Gap (odom)",
            f"{bag.odom_stall_count} header-time gap(s) > {MAX_INTEGRATION_DT_SEC}s on /zed/zed_node/odom, total {bag.odom_stall_total_sec:.2f}s",
            "Stamped odometry gap -- inspect the physics-step publisher; MCAP arrival jitter alone does not trigger this"
        ))

    # --- Three-path distance cross-check ---
    # Historical A1 bags mixed wall time and simulation time and therefore
    # showed a false straight-line shortfall. Keep the three paths visible as
    # a regression check: corrected bags should have odom, joint-derived, and
    # odom_truth distance in close agreement.
    joint_path_dist = bag.joint_dist[-1] if len(bag.joint_dist) > 1 else None
    odomtruth_path_dist = bag.odomtruth_dist[-1] if len(bag.odomtruth_dist) > 1 else None
    odomtruth_endpoint_disp = bag.odomtruth_endpoint_disp if bag.odomtruth_x else None

    def _fmt_or_na(v):
        return f"{v:.3f} m" if v is not None else "N/A"

    if test_id == 4:
        path_metric = "Motion Distance Summary (Pure Rotation)"
        path_value = (
            f"Chassis-center path (Odom): {_fmt_or_na(zed_dist_final)} | "
            f"Mean wheel-surface travel: {_fmt_or_na(joint_path_dist)} | "
            f"Chassis-center path (OdomTruth): {_fmt_or_na(odomtruth_path_dist)}"
        )
        path_note = (
            "Wheel travel and chassis-center path are not expected to match "
            "during in-place rotation; use endpoint drift and yaw tracking as KPIs"
        )
    else:
        path_metric = "Distance Cross-Check (Path-Integrated)"
        path_value = (
            f"Odom: {_fmt_or_na(zed_dist_final)} | "
            f"JointStates: {_fmt_or_na(joint_path_dist)} | "
            f"OdomTruth: {_fmt_or_na(odomtruth_path_dist)}"
        )
        path_note = "Cross-check odometry, wheel-derived distance, and sim ground truth"
    base_rows.append((path_metric, path_value, path_note))
    base_rows.append((
        "Distance Cross-Check (Endpoint Displacement)",
        f"Odom: {_fmt_or_na(bag.odom_endpoint_disp)} | OdomTruth: {_fmt_or_na(odomtruth_endpoint_disp)}",
        "Endpoint displacement uses only first/last samples; compare with path distance above"
    ))
    odom_dup_title = ("⚠ Duplicate Odom Samples" if bag.odom_duplicate_count > 0
                      else "Odometry Duplicate Check")
    odom_dup_status = ("re-published samples detected" if bag.odom_duplicate_count > 0
                       else "PASS -- no repeated pose samples")
    base_rows.append((
        odom_dup_title,
        f"{bag.odom_duplicate_count}/{bag.odom_duplicate_steps} moving steps "
        f"({bag.odom_duplicate_pct:.1f}%) | {odom_dup_status}",
        "#101: full x/y/yaw pose check during reported motion; stationary pre-roll excluded"
    ))
    if bag.times_odomtruth:
        truth_dup_title = ("⚠ Duplicate Odom-Truth Samples" if bag.odomtruth_duplicate_count > 0
                           else "Odom-Truth Duplicate Check")
        truth_dup_status = ("re-published samples detected" if bag.odomtruth_duplicate_count > 0
                            else "PASS -- no repeated pose samples")
        base_rows.append((
            truth_dup_title,
            f"{bag.odomtruth_duplicate_count}/{bag.odomtruth_duplicate_steps} moving steps "
            f"({bag.odomtruth_duplicate_pct:.1f}%) | {truth_dup_status}",
            "#101: full x/y/yaw pose check during reported motion; compare with odom"
        ))

    # --- A2 Steady-State Command Tracking ---
    # Clean cruise-speed comparison (slope-fit over the steady window, so
    # no ramp contamination and no differentiation noise). Answers: does the
    # WHEEL overshoot the command (true drive-layer behaviour), and does the
    # CHASSIS overshoot it? If wheel~cmd but chassis>cmd the discrepancy is
    # NOT the drive layer; if both overshoot, the drive layer itself does.
    # Only emitted for tests with a sustained linear command.
    if abs(max_cmd_v) > 0.05 and (ss_chassis_v is not None or ss_wheel_v is not None):
        def _spd_pct(v):
            return f"{v:.3f} m/s ({v/abs(max_cmd_v)*100:.1f}% of cmd)" if v is not None else "N/A"
        wheel_lbl = {
            "encoder": "Wheel(enc)",
            "joint_states": "Wheel(joint_states, sim)",
            "none": "Wheel(N/A)",
        }[bag.wheel_speed_source]
        wheel_chassis_str = ""
        if ss_wheel_v is not None and ss_chassis_v is not None and ss_chassis_v > 1e-3:
            wheel_chassis_str = f" | wheel/chassis: {ss_wheel_v/ss_chassis_v*100:.1f}%"
        base_rows.append((
            "Steady-State Command Tracking (A2)",
            f"Chassis(Odom): {_spd_pct(ss_chassis_v)} | {wheel_lbl}: {_spd_pct(ss_wheel_v)}{wheel_chassis_str}",
            "A2: slope-fit cruise speed, ramp/noise-free -- localizes straight-line gap to actuation vs contact layer"
        ))

    if test_id == 4:
        total_distance_value = (
            f"Chassis-center path (Odom): {zed_dist_final:.3f} m | "
            f"Mean wheel-surface travel: {_fmt_or_na(wheel_dist_final)}"
        )
        total_distance_note = (
            "Pure rotation: these distances have different physical meanings; "
            "do not interpret their difference as longitudinal slip"
        )
    else:
        total_distance_value = dist_str
        total_distance_note = "Slip & Distance Calibration"

    base_rows += [
        ("Target Velocity (V, W)", f"V: {max_cmd_v:.3f} m/s | W: {max_cmd_w:.3f} rad/s" if max_cmd_v != 0.0 or max_cmd_w != 0.0 else "No Data / 0.000", "Command Target"),
        ("Actual Velocity (V, W)", f"V: {actual_v:.3f} m/s | W: {actual_w:.3f} rad/s", "Kinematics Analysis"),
        ("Test Duration", f"{test_duration:.2f} s", "General Info"),
        ("Start -> End Coordinates", f"({start_x:.2f}, {start_y:.2f}) -> ({end_x:.2f}, {end_y:.2f})", "General Info"),
        ("Total Distance Traveled", total_distance_value, total_distance_note),
        ("System Architecture Status", sys_status_str, "Digital Twin Synchronization"),
        ("Hardware Feedback Status", fb_status_str, "Actuator Overload Check")
    ]

    specific_rows = []
    
    if test_id == 1: 
        v_smooth = smooth_data(bag.odom_v, 9)
        rise_time, overshoot, effective_ax = None, None, None
        peak_joint_v = max(bag.joint_v) if bag.joint_v else 0.0

        target_abs = abs(max_cmd_v)
        achieved_speed = (ss_chassis_v if ss_chassis_v is not None
                          else abs(actual_v))
        reached_target_band = (target_abs > 0 and
                               achieved_speed >= 0.90 * target_abs)
        if abs(actual_v) > 0.05 and reached_target_band:
            try:
                v_abs = np.abs(v_smooth)
                idx_10 = next(i for i, v in enumerate(v_abs) if v >= 0.1*target_abs)
                idx_90 = next(i for i, v in enumerate(v_abs) if v >= 0.9*target_abs)
                rise_time = bag.times_odom[idx_90] - bag.times_odom[idx_10]
                if rise_time > 0:
                    effective_ax = (0.8 * target_abs) / rise_time
                max_peak_v = max(v_abs)
                overshoot = ((max_peak_v - target_abs) / target_abs) * 100.0
            except StopIteration:
                rise_time, overshoot, effective_ax = None, None, None

        if reached_target_band:
            rise_time_str = f"{rise_time:.3f} s" if rise_time is not None else "N/A -- threshold crossing not observed"
            effective_ax_str = f"{effective_ax:.3f} m/s^2" if effective_ax is not None else "N/A -- rise time unavailable"
            overshoot_str = f"{max(0.0, overshoot):.1f} %" if overshoot is not None else "N/A -- target crossing unavailable"
        else:
            saturation_note = (
                f"target not reached; steady speed {achieved_speed:.3f} m/s "
                f"({achieved_speed / target_abs * 100.0:.1f}% of command)"
                if target_abs > 0 else "no nonzero target"
            )
            rise_time_str = f"N/A -- {saturation_note}"
            effective_ax_str = f"N/A -- {saturation_note}"
            overshoot_str = f"N/A -- {saturation_note}"

        # Longitudinal slip from STEADY-STATE cruise speeds (slope-fit),
        # not peak-of-differentiated wheel speed. The old peak method used
        # max(|d(enc_dist)/dt|), which on real encoder data spikes to
        # 300-400% of the true speed from quantization/timing jitter and
        # produced meaningless slip numbers. ss_wheel_v / ss_chassis_v are
        # the ramp-free, noise-free cruise speeds computed above.
        if ss_wheel_v is not None and ss_wheel_v > 0.05 and ss_chassis_v is not None:
            slip_ratio = (ss_wheel_v - ss_chassis_v) / ss_wheel_v * 100.0
            slip_wheel_disp, slip_chassis_disp = ss_wheel_v, ss_chassis_v
        else:
            slip_ratio = 0.0
            slip_wheel_disp = ss_wheel_v if ss_wheel_v is not None else 0.0
            slip_chassis_disp = ss_chassis_v if ss_chassis_v is not None else abs(actual_v)
        slip_note = {
            "encoder": "",
            "joint_states": " [wheel speed from /joint_states, sim bag]",
            "none": " [NO WHEEL-SPEED DATA -- number is not meaningful]",
        }[bag.wheel_speed_source]

        specific_rows = [
            ("Peak Transient Accel Magnitude (IMU)", f"{abs(peak_ax):.3f} m/s^2", "Reference for Suspension Stiffness"),
            ("Effective Sustained Accel", effective_ax_str, "Rigid Body: Mass / Joint Torque"),
            ("Rise Time (10-90%)", rise_time_str, "Joint: Stiffness (P-Gain)"),
            ("Overshoot Peak", overshoot_str, "Joint: Damping (D-Gain)"),
            ("Peak Actuator Velocity", f"{peak_joint_v:.3f} rad/s", "Joint: Target Velocity Limit"),
            ("Longitudinal Slip Ratio", f"{slip_ratio:.1f} % (steady wheel {slip_wheel_disp:.3f} vs chassis {slip_chassis_disp:.3f} m/s){slip_note}", "Material: Dynamic Friction (Slip Curve)"),
        ]

    elif test_id == 2: 
        pitch_rate = max(np.abs(bag.imu_wy)) if bag.imu_wy else 0.0
        peak_brake_eff = max(bag.joint_eff) if bag.joint_eff else 0.0
        braking_distance, effective_decel = 0.0, 0.0
        wheel_lock_ratio = 0.0
        
        if bag.cmd_v:
            brake_trigger_indices = [i for i, v in enumerate(bag.cmd_v) if v == 0.0 and i > len(bag.cmd_v)*0.2]
            if brake_trigger_indices:
                t_brake = bag.times_cmd[brake_trigger_indices[0]]
                odom_brake_idx = min(range(len(bag.times_odom)), key=lambda i: abs(bag.times_odom[i] - t_brake))
                v_initial = bag.odom_v[odom_brake_idx]
                dx = bag.odom_x[-1] - bag.odom_x[odom_brake_idx]
                dy = bag.odom_y[-1] - bag.odom_y[odom_brake_idx]
                braking_distance = math.hypot(dx, dy)
                if braking_distance > 0.01:
                    effective_decel = (v_initial**2) / (2.0 * braking_distance)

                # Wheel-lock / slip check during the braking segment,
                # same signal pairing as Test 1/5's slip ratio (wheel
                # surface speed vs chassis speed) but evaluated after
                # the brake trigger instead of at steady-state cruise.
                # A wheel that stops (locks) faster than the chassis
                # decelerates shows up here as wheel_v << odom_v.
                if bag.wheel_v and bag.wheel_times:
                    wj_idx = [i for i, t in enumerate(bag.wheel_times) if t >= t_brake]
                    wo_idx = [i for i, t in enumerate(bag.times_odom) if t >= t_brake]
                    if wj_idx and wo_idx:
                        peak_wheel_v_brake = max(np.abs([bag.wheel_v[i] for i in wj_idx])) if wj_idx else 0.0
                        peak_chassis_v_brake = max(np.abs([bag.odom_v[i] for i in wo_idx])) if wo_idx else 0.0
                        if peak_chassis_v_brake > 0.05:
                            wheel_lock_ratio = ((peak_chassis_v_brake - peak_wheel_v_brake) / peak_chassis_v_brake) * 100.0

        wheel_note = {
            "encoder": "",
            "joint_states": " [from /joint_states, sim bag]",
            "none": " [N/A -- no wheel-speed data]",
        }[bag.wheel_speed_source]

        specific_rows = [
            ("Peak Transient Decel (IMU)", f"{min_ax:.3f} m/s^2", "Reference for Suspension Dive"),
            ("Effective Braking Decel", f"{effective_decel:.3f} m/s^2", "Material: Dynamic Friction"),
            ("Braking Distance", f"{braking_distance:.3f} m", "Joint: Max Force limit"),
            ("Wheel-Lock Ratio (Braking)", f"{wheel_lock_ratio:.1f} %{wheel_note}", "Material: Dynamic Friction -- >0 means wheel decelerating faster than chassis (skid)"),
            ("Pitch Angle / Dive", f"Peak Rate: {pitch_rate:.3f} rad/s", "Rigid Body: COM Z-Offset"),
            ("Peak Braking Effort", f"{peak_brake_eff:.2f} Nm/A", "Joint: Max Effort Calibration")
        ]

    elif test_id == 3: # Steady-State Circular Driving
        radius = abs(actual_v / actual_w) if abs(actual_w) > 0.01 else 0.0
        r_cmd = abs(max_cmd_v / max_cmd_w) if abs(max_cmd_w) > 0.01 else 0.0
        theoretical_ay = (actual_v ** 2 / radius) if radius > 0.01 else 0.0

        if r_cmd > 0.01 and radius > 0.01:
            if radius < r_cmd * 0.9:
                steer_behavior = f"Oversteer (R_cmd {r_cmd:.2f}m -> R_actual {radius:.2f}m)"
            elif radius > r_cmd * 1.1:
                steer_behavior = f"Understeer (R_cmd {r_cmd:.2f}m -> R_actual {radius:.2f}m)"
            else:
                steer_behavior = f"Neutral (R_cmd {r_cmd:.2f}m ~ R_actual {radius:.2f}m)"
        else:
            steer_behavior = "Insufficient data (need sustained V and W)"

        # Slip Angle (beta): angle between the chassis heading and its
        # actual velocity vector during the steady-state turn, from the
        # ratio of lateral to longitudinal odom velocity.
        slip_angle_deg = 0.0
        if bag.odom_vy and bag.odom_v:
            vy_ss = get_steady_state_mean(bag.odom_vy)
            vx_ss = get_steady_state_mean(bag.odom_v)
            if abs(vx_ss) > 0.05:
                slip_angle_deg = math.degrees(math.atan2(vy_ss, vx_ss))

        specific_rows = [
            ("Commanded Radius (R_cmd = v/w)", f"{r_cmd:.3f} m", "Reference: ideal no-slip radius"),
            ("Peak Transient Lat-Accel Magnitude", f"{abs(peak_ay):.3f} m/s^2", "Reference for Chassis Roll"),
            ("Steady-State Lat-Accel (Measured)", f"{steady_ay:.3f} m/s^2", "Material: Lateral Friction"),
            ("Theoretical Centripetal Accel (v^2/R)", f"{theoretical_ay:.3f} m/s^2", "Compare vs measured Ay -> slip indicator"),
            ("Steady-state Radius (Actual)", f"{radius:.3f} m", "Friction Combine Mode"),
            ("Slip Behavior", steer_behavior, "Rigid Body: COM X/Y-Offset"),
            ("Slip Angle (Steady-State)", f"{slip_angle_deg:.2f} deg", "Material: Lateral Friction, Friction Combine Mode"),
        ]

    elif test_id == 4: # In-place Skid-Steer Test
        # 1. Angular displacement via integral (whole-run, LEGACY -- see
        # note on the row below for why this can be time-window-misaligned)
        rotational_slip = 0.0
        angular_tracking_ratio = 0.0
        if abs(target_yaw_angle) > 0.01:
            rotational_slip = ((abs(target_yaw_angle) - abs(actual_yaw_angle)) / abs(target_yaw_angle)) * 100.0
            # Same comparison as rotational_slip, expressed as
            # actual/target*100. This is a tracking ratio: values below
            # 100% are under-tracking and values above 100% are overshoot.
            angular_tracking_ratio = (abs(actual_yaw_angle) / abs(target_yaw_angle)) * 100.0

        # 1b. Steady-state tracking ratio, FIXED 2-9s window relative to
        # command start (CobraFlex_Work_Log_2026-07-20.md Section 11,
        # Section 13: this is now the authoritative Test 4 tracking
        # metric; the whole-run integral above is kept only as a legacy
        # diagnostic). Fitting a slope over an explicit common window
        # (instead of integrating bag.cmd_w and bag.odom_yaw separately
        # over whatever range each happened to have data) removes the
        # time-window misalignment that produced spurious 33%/39%/48%
        # readings when the first few odometry samples were missing.
        rot_cmd_t0, rot_cmd_t1 = active_command_window(bag.times_cmd, bag.cmd_w)
        ss_w_t0 = ss_w_t1 = None
        steady_state_w = None
        steady_state_tracking_ratio = None
        steady_state_status = None
        if rot_cmd_t0 is not None and rot_cmd_t1 is not None:
            ss_w_t0 = rot_cmd_t0 + 2.0
            ss_w_t1 = rot_cmd_t0 + 9.0
            odom_t0, odom_t1 = bag.times_odom[0], bag.times_odom[-1]
            if rot_cmd_t1 < ss_w_t1 - ANALYSIS_WINDOW_TOL_SEC:
                steady_state_status = (
                    "N/A -- sustained command ended before the complete 2-9s "
                    "steady-state window"
                )
            elif (odom_t0 > ss_w_t0 + ANALYSIS_WINDOW_TOL_SEC
                  or odom_t1 < ss_w_t1 - ANALYSIS_WINDOW_TOL_SEC):
                steady_state_status = (
                    "N/A -- odometry does not cover the complete 2-9s "
                    "post-command window"
                )
            else:
                steady_state_w = rotational_steady_state_from_yaw(
                    bag.times_odom, bag.odom_yaw, ss_w_t0, ss_w_t1)
                if steady_state_w is not None and abs(max_cmd_w) > 0.01:
                    steady_state_tracking_ratio = abs(steady_state_w) / abs(max_cmd_w) * 100.0
                else:
                    steady_state_status = (
                        "N/A -- too few odometry samples for the complete 2-9s fit"
                    )
        else:
            steady_state_status = "N/A -- no sustained active command window found"

        # 1c. Startup transient (cmd_t0 to cmd_t0+2s), reported separately
        # instead of being silently averaged into the tracking ratio --
        # this is where the reset-first pivot anomaly (see Work Log
        # Section 3-9) actually lived, and where any future step-response
        # characterization should look.
        transient_w_mean = None
        transient_t0 = transient_t1 = None
        transient_status = None
        if rot_cmd_t0 is not None:
            requested_t0 = rot_cmd_t0
            requested_t1 = rot_cmd_t0 + 2.0
            odom_t0, odom_t1 = bag.times_odom[0], bag.times_odom[-1]
            if requested_t0 < odom_t0 - ANALYSIS_WINDOW_TOL_SEC:
                transient_status = (
                    "N/A -- command began before the first odometry sample; "
                    "recorder pre-roll was not captured"
                )
            elif (rot_cmd_t1 is not None
                  and rot_cmd_t1 < requested_t1 - ANALYSIS_WINDOW_TOL_SEC):
                transient_status = (
                    "N/A -- sustained command ended before the complete 0-2s window"
                )
            elif odom_t1 < requested_t1 - ANALYSIS_WINDOW_TOL_SEC:
                transient_status = (
                    "N/A -- odometry does not cover the complete 0-2s "
                    "post-command window"
                )
            else:
                # Clamp only sub-tolerance negative roundoff (for example
                # -0.00s after record/header mapping) to the first odom sample.
                transient_t0 = max(requested_t0, odom_t0)
                transient_t1 = requested_t1
                transient_vals = [abs(w) for t, w in zip(bag.times_odom, bag.odom_w)
                                  if transient_t0 <= t <= transient_t1]
                if transient_vals:
                    transient_w_mean = float(np.mean(transient_vals))
                else:
                    transient_status = "N/A -- no odometry samples in the complete 0-2s window"
        else:
            transient_status = "N/A -- no sustained active command window found"

        # 2. Breakaway detection at the CHASSIS level: first command
        # magnitude at which the chassis (odom yaw rate) actually starts
        # moving. This is the static-friction breakaway point as seen
        # by the ground/tire interface.
        breakaway_cmd_w = None
        w_smooth = zero_phase_filter(bag.odom_w, 9)
        for i, w_val in enumerate(w_smooth):
            if abs(w_val) > 0.05:
                t_break = bag.times_odom[i]
                cmd_idx = (np.abs(np.array(bag.times_cmd) - t_break)).argmin()
                cmd_at_break = abs(bag.cmd_w[cmd_idx])
                # Ignore chassis/estimator noise that occurs before the
                # actual yaw command. A zero command is not a breakaway KPI.
                if cmd_at_break > 0.01:
                    breakaway_cmd_w = cmd_at_break
                    break

        # 3. Breakaway detection at the WHEEL/ACTUATOR level: first
        # command magnitude at which the wheel itself starts spinning
        # (bag.wheel_v -- real encoder on the real robot, /joint_states
        # derived on sim bags). On a perfectly rigid driveline this
        # should match the chassis-level breakaway above; a large gap
        # points at driveline compliance/backlash rather than pure
        # ground-contact static friction.
        breakaway_cmd_w_wheel = None
        if bag.wheel_v and bag.wheel_times and bag.times_cmd:
            wv_smooth = zero_phase_filter(bag.wheel_v, 9)
            for i, wv in enumerate(wv_smooth):
                if abs(wv) > WHEEL_BREAKAWAY_SPEED_THRESHOLD_MPS:
                    t_break_w = bag.wheel_times[i]
                    cmd_idx = (np.abs(np.array(bag.times_cmd) - t_break_w)).argmin()
                    cmd_at_break = abs(bag.cmd_w[cmd_idx])
                    if cmd_at_break > 0.01:
                        breakaway_cmd_w_wheel = cmd_at_break
                        break
        wheel_note = {
            "encoder": "",
            "joint_states": " [from /joint_states, sim bag]",
            "none": " [N/A -- no wheel-speed data]",
        }[bag.wheel_speed_source]

        valid_volts = [v for v in bag.bat_vol if v > 9.0]
        if bag.feedback_msg_count == 0:
            voltage_drop_str = f"N/A -- {TOPIC_FEEDBACK} not present"
        elif len(valid_volts) >= 2:
            voltage_drop = float(np.mean(valid_volts[:10]) - min(valid_volts))
            voltage_drop_str = f"{voltage_drop:.2f} V"
        else:
            voltage_drop_str = f"N/A -- insufficient {TOPIC_BATTERY} samples"
        peak_turn_eff_str = (f"{max(bag.joint_eff):.2f} Nm/A"
                             if bag.joint_eff else "N/A -- joint effort unavailable")

        breakaway_chassis_str = (
            f"Target W @ breakaway: {breakaway_cmd_w:.3f} rad/s"
            if breakaway_cmd_w is not None
            else "N/A -- no valid post-command chassis breakaway detected"
        )
        breakaway_wheel_str = (
            f"Target W @ breakaway: {breakaway_cmd_w_wheel:.3f} rad/s{wheel_note}"
            if breakaway_cmd_w_wheel is not None
            else f"N/A -- no valid post-command wheel breakaway detected{wheel_note}"
        )

        pose_gyro_warning = (
            " -- ⚠ DISAGREEMENT >30 deg; inspect odometry and gyro time alignment"
            if yaw_gyro_disagreement_deg > YAW_GYRO_DISAGREEMENT_THRESHOLD_DEG
            else ""
        )

        steady_state_str = (
            f"{steady_state_tracking_ratio:.1f} % (W_ss={steady_state_w:.3f} rad/s over "
            f"[{ss_w_t0:.2f}s,{ss_w_t1:.2f}s], cmd={max_cmd_w:.3f} rad/s)"
            if steady_state_tracking_ratio is not None
            else steady_state_status
        )
        transient_str = (
            f"mean|W|={transient_w_mean:.3f} rad/s over [{transient_t0:.2f}s,{transient_t1:.2f}s] "
            f"-- excluded from steady-state ratio above, not itself a tracking KPI"
            if transient_w_mean is not None
            else transient_status
        )

        specific_rows = [
            ("Total Angular Displacement", f"Cmd: {target_yaw_angle:.2f} rad ({math.degrees(target_yaw_angle):.1f} deg) | Act: {actual_yaw_angle:.2f} rad ({math.degrees(actual_yaw_angle):.1f} deg)", "Material: Static/Dynamic Friction"),
            ("Odom Pose Yaw vs Integrated Gyro", f"OdomPose: {math.degrees(actual_yaw_angle):.1f} deg | Gyro: {math.degrees(gyro_yaw_angle):.1f} deg | Diff: {yaw_gyro_disagreement_deg:.1f} deg" + pose_gyro_str + pose_gyro_warning, "Data-quality cross-check; identify the odometry producer before assigning sensor fault"),
            ("Angular Tracking Ratio (Steady-State 2-9s)", steady_state_str, "PRIMARY tracking metric (2026-07-20) -- fixed common window, replaces the whole-run integral below for reporting"),
            ("Startup Transient (0-2s post-command)", transient_str, "Reset-first pivot anomaly and step-response behavior live here, not in the steady-state ratio"),
            ("Angular Tracking Ratio (legacy, whole-run integral)", f"{angular_tracking_ratio:.1f} %", "DEPRECATED for reporting -- target_yaw_angle and actual_yaw_angle can come from mismatched time windows if odometry's first samples are missing; kept for continuity with older logs only"),
            ("Rotational Slip Ratio (legacy)", f"{rotational_slip:.1f} %", "Deficit-style metric: (Cmd-Act)/Cmd*100 -- negative when overshooting"),
            ("Breakaway Yaw Command (Chassis)", breakaway_chassis_str, "Material: Static Friction Calibration"),
            ("Breakaway Yaw Command (Wheel)", breakaway_wheel_str, "Joint: Joint Friction (driveline vs ground)"),
            ("Actuator Voltage Drop", voltage_drop_str, "Requires feedback plus valid battery samples"),
            ("Peak Actuator Effort", peak_turn_eff_str, "Joint: Joint Friction, Max Force"),
        ]

    elif test_id == 5: 
        max_eff = max(bag.joint_eff) if bag.joint_eff else 0.0

        # Slip tends to peak near maximum commanded speed (highest
        # torque demand), so cross-check wheel-surface speed vs actual
        # chassis speed here too, same signal/logic as Test 1's slip
        # ratio but evaluated at the steady-state max-speed segment.
        # Max-speed slip from STEADY-STATE cruise speeds (slope-fit), same
        # fix as Test 1: the old peak-of-differentiated wheel speed inflated
        # to 300-400% on real encoder data. ss_wheel_v / ss_chassis_v are
        # the ramp-free, noise-free cruise speeds computed at the top of
        # compute_test_kpis.
        if ss_wheel_v is not None and ss_wheel_v > 0.05 and ss_chassis_v is not None:
            max_speed_slip = (ss_wheel_v - ss_chassis_v) / ss_wheel_v * 100.0
            ms_wheel_disp, ms_chassis_disp = ss_wheel_v, ss_chassis_v
        else:
            max_speed_slip = 0.0
            ms_wheel_disp = ss_wheel_v if ss_wheel_v is not None else 0.0
            ms_chassis_disp = ss_chassis_v if ss_chassis_v is not None else abs(actual_v)
        wheel_note = {
            "encoder": "",
            "joint_states": " [from /joint_states, sim bag]",
            "none": " [N/A -- no wheel-speed data]",
        }[bag.wheel_speed_source]

        specific_rows = [
            ("Steady-State Error", f"{abs(actual_v - max_cmd_v):.3f} m/s", "Joint: Velocity Target Limit"),
            ("Sustained Actuator Effort", f"{max_eff:.2f} Nm/A", "Joint: Damping (High-speed drag)"),
            ("Max-Speed Slip Ratio", f"{max_speed_slip:.1f} % (steady wheel {ms_wheel_disp:.3f} vs chassis {ms_chassis_disp:.3f} m/s){wheel_note}", "Joint: Damping (High-speed drag) / Material: Dynamic Friction"),
        ]

    elif test_id == 6: 
        pose_closure = 0.0
        pose_row_label = "Pose Closure Error (Corrected)"
        pose_row_note = "Cross-check for physical drift"
        if bag.pose_x and bag.pose_y:
            p_start_x, p_start_y = bag.pose_x[0], bag.pose_y[0]
            p_end_x, p_end_y = bag.pose_x[-1], bag.pose_y[-1]
            pose_closure = math.hypot(p_end_x - p_start_x, p_end_y - p_start_y)
        else:
            # /zed/zed_node/pose is not published by this Isaac Sim graph.
            # Reuse odom closure as an explicit fallback instead of silently
            # reporting zero; this fallback is not an independent channel.
            pose_closure = euclidean_dist
            pose_row_label = "Pose Closure Error (odom fallback)"
            pose_row_note = "No /zed/zed_node/pose in this bag -- reused /zed/zed_node/odom (sim bag)"

        # Final Heading Error: was a dead placeholder string
        # ("Check Trajectory Gap") before -- now the actual
        # unwrap-summed yaw delta between the first and last odom
        # sample. For a closed square path this should return to ~0.
        # NOTE: this single-bag number is NOT yet the full UMBmark
        # systematic/non-systematic decomposition (Borenstein & Feng
        # 1996) -- that requires averaging this value across matched
        # CW and CCW run pairs (systematic error has the same sign
        # regardless of direction; non-systematic/random error
        # doesn't). Do that comparison at the batch level once
        # multiple Test 6 runs are aggregated.
        final_heading_error_deg = 0.0
        if bag.odom_yaw and len(bag.odom_yaw) > 1:
            final_heading_error_deg = math.degrees(unwrap_delta(bag.odom_yaw[0], bag.odom_yaw[-1]))

        specific_rows = [
            ("Odom Closure Error (Kinematic)", f"{euclidean_dist:.3f} m", "Articulation: Wheel Radius"),
            (pose_row_label, f"{pose_closure:.3f} m", pose_row_note),
            ("Final Heading Error (Closure)", f"{final_heading_error_deg:.2f} deg", "Articulation: Track Width -- pair with CW/CCW run for UMBmark systematic-error split"),
        ]

    elif test_id == 7: 
        roll_rate = max(np.abs(bag.imu_wx)) if bag.imu_wx else 0.0
        step_window = yaw_step_window(bag.times_cmd, bag.cmd_w)
        yaw_response = estimate_step_response(
            bag.times_cmd, bag.cmd_w, bag.times_odom, bag.odom_w)

        test7_target_yaw = None
        test7_pose_yaw = None
        test7_gyro_yaw = None
        test7_twist_yaw = None
        test7_pose_gyro_diff = None
        test7_pose_gyro_ratio = None
        test7_pose_tracking = None
        test7_ss_pose_rate = None
        test7_ss_gyro_rate = None
        test7_ss_tracking = None
        test7_window_str = "N/A -- yaw step not found"
        if step_window is not None:
            step_start, step_end, step_level = step_window
            step_duration = step_end - step_start
            test7_window_str = (
                f"{step_duration:.3f} s yaw hold | start={step_start:.3f} s | "
                f"end={step_end:.3f} s | W={step_level:.3f} rad/s"
            )
            test7_target_yaw = step_level * step_duration
            test7_pose_yaw = pose_yaw_change_window(
                bag.times_odom, bag.odom_yaw, step_start, step_end)
            test7_gyro_yaw = integrate_series_window(
                bag.times_imu, bag.imu_wz, step_start, step_end)
            test7_twist_yaw = integrate_series_window(
                bag.times_odom, bag.odom_w, step_start, step_end)
            if test7_pose_yaw is not None and test7_gyro_yaw is not None:
                test7_pose_gyro_diff = abs(abs(test7_pose_yaw) - abs(test7_gyro_yaw))
                if abs(test7_gyro_yaw) > 1e-3:
                    test7_pose_gyro_ratio = abs(test7_pose_yaw / test7_gyro_yaw)
            if test7_pose_yaw is not None and abs(test7_target_yaw) > 1e-6:
                test7_pose_tracking = abs(test7_pose_yaw / test7_target_yaw)

            tail_start = max(step_start + 0.5 * step_duration, step_end - 1.0)
            test7_ss_pose_rate = rotational_steady_state_from_yaw(
                bag.times_odom, bag.odom_yaw, tail_start, step_end)
            gyro_tail = [w for t, w in zip(bag.times_imu, bag.imu_wz)
                         if tail_start <= t < step_end]
            if gyro_tail:
                test7_ss_gyro_rate = float(np.mean(gyro_tail))
            if test7_ss_pose_rate is not None and abs(step_level) > 1e-6:
                test7_ss_tracking = abs(test7_ss_pose_rate / step_level)

        def _format_step_time(value):
            resolution = yaw_response.get("resolution")
            if value is None:
                return "N/A"
            if resolution is not None and value <= resolution * 1.05:
                return f"<{resolution * 1000.0:.1f} ms (one odom sample)"
            return f"{value:.3f} s"

        yaw_response_str = (
            f"Delay(10%): {_format_step_time(yaw_response.get('delay'))} | "
            f"Tau(10-63%): {_format_step_time(yaw_response.get('tau'))}"
        )
        if yaw_response.get("status") != "ok":
            yaw_response_str += f" | {yaw_response.get('status')}"
        yaw_response_str += " | QUALIFIED: limited by odometry rate/gaps and sensor timestamp alignment"

        def _angle_or_na(value):
            return f"{value:.3f} rad ({math.degrees(value):.2f} deg)" if value is not None else "N/A"

        pose_gyro_value = (
            f"OdomPose: {_angle_or_na(test7_pose_yaw)} | "
            f"Gyro: {_angle_or_na(test7_gyro_yaw)}"
        )
        if test7_pose_gyro_diff is not None:
            pose_gyro_value += f" | Diff: {math.degrees(test7_pose_gyro_diff):.2f} deg"
        if test7_pose_gyro_ratio is not None:
            pose_gyro_value += f" | pose/gyro: {test7_pose_gyro_ratio*100:.1f}%"

        tracking_value = "N/A"
        if test7_ss_pose_rate is not None:
            tracking_value = f"Pose slope: {test7_ss_pose_rate:.5f} rad/s"
            if test7_ss_gyro_rate is not None:
                tracking_value += f" | Gyro mean: {test7_ss_gyro_rate:.5f} rad/s"
            if test7_ss_tracking is not None:
                tracking_value += f" | Pose/cmd: {test7_ss_tracking*100:.2f}%"

        specific_rows = [
            ("Test 07 Analysis Window", test7_window_str, "Yaw-step interval only; excludes the preceding straight segment and post-stop tail"),
            ("Angular Displacement (Yaw-Step Window)", f"Cmd: {_angle_or_na(test7_target_yaw)} | Pose: {_angle_or_na(test7_pose_yaw)} | Twist integral: {_angle_or_na(test7_twist_yaw)}", "Primary Test 07 tracking result"),
            ("Odom Pose Yaw vs Integrated Gyro (Yaw-Step Window)", pose_gyro_value, "Data-quality cross-check on a common 5 s window"),
            ("Steady-State Yaw Tracking (Final 1 s)", tracking_value, "Report mean ± SD across repetitions at batch level"),
            ("Yaw Step Response (Delay / Tau)", yaw_response_str, "Rigid Body: Diagonal Inertia Matrix"),
            ("Peak Lateral Accel Magnitude (Ay)", f"{abs(peak_ay):.3f} m/s^2", "Rigid Body: COM Height"),
            ("Peak Roll Rate (Wx)", f"{roll_rate:.3f} rad/s", "Rigid Body: Diagonal Inertia Matrix (Ixx)")
        ]

    elif test_id == 8: 
        # Coast phase starts where cmd_v is explicitly cut to 0 after a
        # sustained nonzero hold (cobraflex_test_control_v5.py's Test 8
        # publishes an explicit publish_cmd(0,0) to begin coasting).
        # Linear-fit the velocity decay during that segment to get a
        # deceleration figure -- that slope IS the rolling-resistance
        # deceleration, not just "visualized in chart".
        coast_decel = 0.0        # chassis-side (ZED odom) decay
        wheel_coast_decel = 0.0  # wheel-side (encoder / joint_states) decay

        if bag.cmd_v:
            zero_indices = [i for i, v in enumerate(bag.cmd_v) if abs(v) < 0.01 and i > len(bag.cmd_v) * 0.2]
            if zero_indices:
                t_coast_start = bag.times_cmd[zero_indices[0]]

                odom_idx = [i for i, t in enumerate(bag.times_odom) if t >= t_coast_start]
                if len(odom_idx) > 5:
                    t_c = np.array([bag.times_odom[i] for i in odom_idx])
                    v_c = np.array([bag.odom_v[i] for i in odom_idx])                    if (t_c[-1] - t_c[0]) > 0.2:
                        slope, _ = np.polyfit(t_c - t_c[0], v_c, 1)
                        coast_decel = -slope  # positive = decelerating

                if bag.wheel_v and bag.wheel_times:
                    joint_idx = [i for i, t in enumerate(bag.wheel_times) if t >= t_coast_start]
                    if len(joint_idx) > 5:
                        t_wc = np.array([bag.wheel_times[i] for i in joint_idx])
                        v_wc = np.array([bag.wheel_v[i] for i in joint_idx])
                        if (t_wc[-1] - t_wc[0]) > 0.2:
                            wslope, _ = np.polyfit(t_wc - t_wc[0], v_wc, 1)
                            wheel_coast_decel = -wslope

        wheel_note = {
            "encoder": "",
            "joint_states": " [from /joint_states, sim bag]",
            "none": " [N/A -- no wheel-speed data]",
        }[bag.wheel_speed_source]

        specific_rows = [
            ("Peak Initial Decel (IMU)", f"{min_ax:.3f} m/s^2", "Check for engine braking jerk"),
            ("Rolling Resistance (Chassis Decel)", f"{coast_decel:.4f} m/s^2", "Rigid Body: Linear Damping"),
            ("Rolling Resistance (Wheel Decel)", f"{wheel_coast_decel:.4f} m/s^2{wheel_note}", "Joint: Joint Friction"),
            ("Velocity Decay Curve", "Visualized in Chart -- slope above is the fitted line", "Cross-check: Chassis vs Wheel decel"),
        ]

    elif test_id == 9:
        # Weight Transfer / Pitch Test -- from the project spreadsheet's
        # planned-but-unimplemented 9th test, targeting centerOfMass.
        pitch_rate_peak = max(np.abs(bag.imu_wy)) if bag.imu_wy else 0.0
        ax_swing = (max(bag.imu_ax) - min(bag.imu_ax)) if bag.imu_ax else 0.0
        sensitivity = (pitch_rate_peak / ax_swing) if ax_swing > 0.01 else 0.0
        specific_rows = [
            ("Peak Pitch Rate (Wy)", f"{pitch_rate_peak:.3f} rad/s", "Reference for Dive/Squat dynamics"),
            ("Longitudinal Accel Swing (Fwd/Rev)", f"{ax_swing:.3f} m/s^2", "Achieved pulse magnitude"),
            ("Pitch Sensitivity (Wy / Ax)", f"{sensitivity:.4f} (rad/s)/(m/s^2)", "Rigid Body: COM Z-Offset"),
        ]

    elif test_id == 10:
        # Baseline Noise Floor Test (new). Vehicle should be stationary
        # the whole bag -- any nonzero reading here is sensor bias/noise,
        # not real motion. Use this to sanity-check the other 9 tests.
        ax_bias = float(np.mean(bag.imu_ax)) if bag.imu_ax else 0.0
        ay_bias = float(np.mean(bag.imu_ay)) if bag.imu_ay else 0.0
        ax_noise = float(np.std(bag.imu_ax)) if bag.imu_ax else 0.0
        odom_drift = (max(math.hypot(x - bag.odom_x[0], y - bag.odom_y[0])
                          for x, y in zip(bag.odom_x, bag.odom_y))
                      if bag.odom_x else 0.0)
        specific_rows = [
            ("IMU Static Bias (Ax, Ay)", f"Ax: {ax_bias:.4f} | Ay: {ay_bias:.4f} m/s^2", "Subtract as offset before fitting other tests"),
            ("IMU Noise Std-Dev (Ax)", f"{ax_noise:.4f} m/s^2", "Noise floor -- ignore signal smaller than this"),
            ("Odom Drift While Stationary", f"{odom_drift:.3f} m", "Odometry drift/noise floor at rest"),
        ]

    return base_rows + specific_rows, bag


# ==========================================
# PyQt5 UI Dashboard Design
# ==========================================
class AnalyzerMainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.current_bag_path = None
        self.initUI()
        self.TEST_NAMES = TEST_NAMES

    def initUI(self):
        self.setWindowTitle('CobraFlex ROS Bag Analyzer')
        self.resize(1650, 950)
        
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)

        left_panel = QVBoxLayout()
        file_group = QGroupBox("1. Dataset Selection")
        file_layout = QVBoxLayout()
        self.lbl_current_bag = QLabel("No data loaded yet.")
        self.lbl_current_bag.setWordWrap(True)
        self.lbl_detected_test = QLabel("Detected Test Type: N/A")
        self.lbl_detected_test.setFont(QFont("Arial", 12, QFont.Bold))
        self.lbl_detected_test.setStyleSheet("color: #1976D2;")
        
        btn_load = QPushButton("Browse / Load Rosbag")
        btn_load.clicked.connect(self.load_bag)
        
        file_layout.addWidget(btn_load)
        file_layout.addWidget(self.lbl_current_bag)
        file_layout.addWidget(self.lbl_detected_test)
        file_group.setLayout(file_layout)
        
        result_group = QGroupBox("2. Extracted Kinematics")
        result_layout = QVBoxLayout()
        
        # Add Odom vs Pose explanation label
        lbl_info = QLabel(
            "NOTE: /zed/zed_node/odom is the odometry topic. In Isaac Sim it is produced by "
            "isaac_compute_odometry and is not ZED VIO. /zed/zed_node/pose is a separate "
            "ZED pose/SLAM channel only when that topic is actually recorded."
        )
        lbl_info.setStyleSheet("color: #555555; font-size: 11px;")
        result_layout.addWidget(lbl_info)
        
        self.table_results = QTableWidget(0, 3)
        self.table_results.setHorizontalHeaderLabels(["Analysis Parameters", "Extracted Value", "Target Isaac Parameters"])
        self.table_results.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.table_results.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeToContents)
        self.table_results.horizontalHeader().setSectionResizeMode(2, QHeaderView.Stretch)
        self.table_results.setAlternatingRowColors(True)
        
        result_layout.addWidget(self.table_results)
        result_group.setLayout(result_layout)
        
        left_panel.addWidget(file_group, 1)
        left_panel.addWidget(result_group, 5)
        
        right_panel = QVBoxLayout()
        graph_group = QGroupBox("3. Dynamic Response Visualization")
        graph_layout = QVBoxLayout()
        
        self.figure = Figure(figsize=(10, 12)) 
        self.canvas = FigureCanvas(self.figure)
        
        graph_layout.addWidget(self.canvas)
        graph_group.setLayout(graph_layout)
        right_panel.addWidget(graph_group)
        
        main_layout.addLayout(left_panel, 2)
        main_layout.addLayout(right_panel, 3)

    def load_bag(self):
        options = QFileDialog.Options()
        options |= QFileDialog.ShowDirsOnly
        start_dir = BAG_DIR_DEFAULT if os.path.exists(BAG_DIR_DEFAULT) else os.path.expanduser("~")
        
        bag_dir = QFileDialog.getExistingDirectory(self, "Select Rosbag Directory", start_dir, options=options)
        
        if bag_dir:
            if not os.path.exists(os.path.join(bag_dir, 'metadata.yaml')):
                QMessageBox.critical(self, "Selection Error", "Invalid Rosbag directory.\nPlease select the directory containing 'metadata.yaml'.")
                return

            self.current_bag_path = bag_dir
            folder_name = os.path.basename(bag_dir)
            self.lbl_current_bag.setText(f"Loaded: {folder_name}")
            
            test_id = self.identify_test_id(folder_name)
            test_name = self.TEST_NAMES.get(test_id, "Unknown Test Profile")
            self.lbl_detected_test.setText(f"No.{test_id} - {test_name}")
            
            self.execute_analysis(test_id)

    def identify_test_id(self, folder_name):
        return identify_test_id(folder_name)

    def populate_table(self, data_rows):
        self.table_results.setRowCount(0)
        for i, (metric, value, param) in enumerate(data_rows):
            self.table_results.insertRow(i)
            self.table_results.setItem(i, 0, QTableWidgetItem(str(metric)))
            val_item = QTableWidgetItem(str(value))
            val_item.setFont(QFont("Arial", 10, QFont.Bold))
            if str(metric) == "Data Completeness Check" and str(value).startswith("⚠"):
                val_item.setForeground(QColor("red"))
            else:
                val_item.setForeground(QColor("black"))
            self.table_results.setItem(i, 1, val_item)
            param_item = QTableWidgetItem(str(param))
            param_item.setForeground(QColor("#1976D2")) 
            self.table_results.setItem(i, 2, param_item)

    # ==========================================
    # Plotting Engine
    # ==========================================
    def plot_universal_graphs(self, bag, title_v="Velocity Response", title_a="Acceleration Response"):
        self.figure.clear()
        
        # 1. 2D Trajectory
        ax1 = self.figure.add_subplot(311)
        if bag.odom_x and bag.odom_y:
            ax1.plot(bag.odom_x, bag.odom_y, 'b-', label='Odom Trajectory ((0,0) Aligned)')
            ax1.plot(bag.odom_x[0], bag.odom_y[0], 'go', label='Start')
            ax1.plot(bag.odom_x[-1], bag.odom_y[-1], 'ro', label='End')
        if bag.pose_x and bag.pose_y:
            ax1.plot(bag.pose_x, bag.pose_y, 'c--', label='ZED Pose/SLAM (when recorded)')
        ax1.set_title("2D Map Trajectory (m)", fontweight='bold')
        ax1.set_ylabel("Y (m)"); ax1.axis('equal'); ax1.grid(True, linestyle='--'); ax1.legend()
        
        # 2. Velocity-Time
        ax2 = self.figure.add_subplot(312)
        v_smooth = smooth_data(bag.odom_v, 9)
        w_smooth = smooth_data(bag.odom_w, 9)
        
        if bag.times_cmd and bag.cmd_v:
            ax2.step(bag.times_cmd, bag.cmd_v, 'purple', linestyle='-.', label='Target Linear V', where='post')
            ax2.step(bag.times_cmd, bag.cmd_w, 'cyan', linestyle=':', label='Target Angular W', where='post')
            
        if bag.times_odom:
            ax2.plot(bag.times_odom, v_smooth, 'b-', label='Actual Linear V (m/s)')
            w_max_abs = max(np.abs(bag.odom_w)) if bag.odom_w else 0
            if w_max_abs > 0.1:
                ax2_w = ax2.twinx()
                ax2_w.plot(bag.times_odom, w_smooth, 'g-', label='Actual Angular W (rad/s)')
                ax2_w.set_ylabel("Yaw Rate (rad/s)", color='g')
                ax2_w.tick_params(axis='y', labelcolor='g')
            else:
                ax2.plot(bag.times_odom, w_smooth, 'g-', label='Actual Angular W (rad/s)')
            
        ax2.set_title(title_v, fontweight='bold'); ax2.set_ylabel("Velocity (m/s)", color='b')
        ax2.grid(True, linestyle='--'); ax2.legend(loc='upper left')
        
        # 3. Accel-Time
        ax3 = self.figure.add_subplot(313, sharex=ax2)
        if bag.times_imu and bag.imu_ax:
            ax3.plot(bag.times_imu, smooth_data(bag.imu_ax, 11), 'r-', label='Longitudinal Accel (Ax)')
            ax3.plot(bag.times_imu, smooth_data(bag.imu_ay, 11), 'm-', label='Lateral Accel (Ay)')
        ax3.set_title(title_a, fontweight='bold'); ax3.set_xlabel("Time (s)"); ax3.set_ylabel("Accel (m/s^2)")
        ax3.grid(True, linestyle='--'); ax3.legend(loc='upper right')
        
        self.figure.tight_layout(pad=2.0)
        self.canvas.draw()

    # ==========================================
    # Smart Data Extractor & Table Builder
    # ==========================================
    def execute_analysis(self, test_id):
        try:
            rows, bag = compute_test_kpis(self.current_bag_path, test_id)
        except ValueError as e:
            QMessageBox.warning(self, "Warning", str(e))
            return
        self.populate_table(rows)
        self.plot_universal_graphs(bag)

def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    window = AnalyzerMainWindow()
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()