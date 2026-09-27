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
                 self.odomtruth_duplicate_pct) = count_duplicate_samples(