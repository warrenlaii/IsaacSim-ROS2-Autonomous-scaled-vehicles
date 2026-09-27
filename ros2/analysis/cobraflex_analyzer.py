#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cobraflex_analyzer -- CobraFlex rosbag analysis, one command.

    python3 cobraflex_analyzer.py BAG_ROOT --output results.xlsx

Replaces running cobraflex_rosbag_analyzer.py (GUI),
cobraflex_batch_analyze_v3.py and cobraflex_test04_analyzer_v12.py
separately and reconciling three workbooks by hand. One scan, one test-id
detection result, one workbook.

WHAT IS AND IS NOT IN THIS FILE
-------------------------------
The Test04 deep analysis engine (formerly cobraflex_test04_analyzer_v12.py)
is part of this file, in Section 0. Its KPI code was moved verbatim during
the merge; nothing was recomputed, re-derived or retuned.

The generic Test01-Test10 KPIs are NOT here. They stay in
cobraflex_rosbag_analyzer.compute_test_kpis(), which this file imports
and drives. That separation is load-bearing rather than stylistic: the generic analyzer and
the Test04 engine define same-named things with different meanings.
quaternion_to_yaw() takes four floats in one and a quaternion object in the
other, and each module carries its own WHEEL_RADIUS_M. A single namespace
would leave one binding for each name, silently breaking one caller and
letting this file's geometry configuration reach into the generic analyzer's independent KPI
definitions. Keeping the generic analyzer a separate module is a correctness requirement.

cobraflex_wheel_cmd_debug_analyzer_v8.py is not called from any code path
here. It is retained only as a frozen regression reference.

LAYOUT
------
    0. Test04 deep analysis engine -- constants, bag reading, windowing,
       KPIs, split detection, branch assignment, summaries, run_pipeline()
    1. Configuration and geometry
    2. Bag discovery and test-id routing
    3. Generic Test01-Test10 KPIs (adapter over the generic analyzer)
    4. Test04 deep analysis adapter
    5. Workbook schema, and the command line

Exit codes:
    0  every discovered bag was analysed
    1  finished, but at least one bag was skipped or failed
    2  could not run at all (bad configuration, no bags, unwritable output)

Requires: rosbags, numpy, pandas, openpyxl. PyYAML only for --config in
YAML form. PyQt5 is pulled in by the generic analyzer; the offscreen
platform is selected below so this runs headless over SSH.
"""
from __future__ import annotations

# Handover baseline note:
# The final thesis uses a measured wheel-centre separation of 0.153 m for
# both the Differential Controller and analysis. Historical pre-final analysis
# scripts used 0.154 m; rerunning historical bags with this handover version
# can therefore produce a small change in track-dependent derived KPIs.

import argparse
import json
import math
import os
import re
import sys
import tempfile
import traceback
import warnings
from dataclasses import dataclass, field, asdict

# The generic analyzer imports PyQt5 at module scope. Forcing the offscreen platform before
# that import lets the whole pipeline run with no display.
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402
from openpyxl import load_workbook  # noqa: E402
from openpyxl.styles import Alignment, Font, PatternFill  # noqa: E402
from openpyxl.utils import get_column_letter  # noqa: E402
from openpyxl.chart import LineChart, Reference  # noqa: E402

from rosbags.rosbag2 import Reader  # noqa: E402
from rosbags.typesys import Stores, get_typestore  # noqa: E402


# =====================================================================
# SECTION 0 -- Test04 DEEP ANALYSIS ENGINE
#
# Moved verbatim from cobraflex_test04_analyzer_v12.py. Four top-level
# names collided with the integration layer; resolved as follows:
#
#   find_bag_dirs    removed -- the layer's recursive version called with
#                    recursive=False performs the identical scan
#   parse_condition  removed -- the layer's version additionally accepts
#                    the testNN_ spelling
#   WHEELS           identical in both; defined once, here
#   main             removed -- the file has a single entry point
#
# No other name was renamed, so every KPI, threshold and comment below
# reads exactly as it did in v12.
# =====================================================================

_trapezoid = getattr(np, "trapezoid", None) or np.trapz

# ---------------------------------------------------------------------
# Topics
# ---------------------------------------------------------------------
TOPIC_WHEEL_CMD_DEBUG = "/cobraflex/wheel_cmd_debug"
TOPIC_JOINT_STATES = "/joint_states"
TOPIC_ODOM_TRUTH = "/odom_truth"
TOPIC_CMD_VEL = "/cmd_vel"

# Scalar force-magnitude topics, one per wheel (see #136 -- magnitude
# only, no normal/friction decomposition available on Isaac Sim 6.0.0).
CONTACT_FORCE_TOPICS = {
    "FL": "/cobraflex/contact_force/front_left",
    "FR": "/cobraflex/contact_force/front_right",
    "RL": "/cobraflex/contact_force/rear_left",
    "RR": "/cobraflex/contact_force/rear_right",
}

WHEEL_NAME_MAP = {
    "front_left_wheel_joint": "FL",
    "rear_left_wheel_joint": "RL",
    "front_right_wheel_joint": "FR",
    "rear_right_wheel_joint": "RR",
}
WHEELS = ("FL", "RL", "FR", "RR")
LEFT_WHEELS = ("FL", "RL")
RIGHT_WHEELS = ("FR", "RR")

# ---------------------------------------------------------------------
# Platform geometry -- from the handover spec (docs/14_isaacsim_handover_
# spec.md). Used only for the slip-ratio conversion between wheel angular
# speed and the body yaw rate it would produce under a no-slip rolling
# assumption. If these ever change in the USD model, they must be changed
# here too or every slip number silently shifts.
# ---------------------------------------------------------------------
WHEEL_RADIUS_M = 0.03725
WHEEL_SEPARATION_M = 0.153
HALF_TRACK_M = WHEEL_SEPARATION_M / 2.0

# Measured four-corner static load (grams), from the CoG calibration
# campaign. Reference only -- printed alongside the simulated contact
# forces so the two distributions can be compared. NOT used in any
# computation.
# Four-corner static wheel loads, in grams. Disabled by default: this is a
# static weighing of the physical robot, whereas the simulated contact force
# is a magnitude sampled during rotation, so the comparison is an optional
# side diagnostic and not part of any yaw, tracking, branch or slip
# computation. Enable it explicitly, and state which of the two value sets
# is in use, since they are not interchangeable.
CORNER_LOAD_COMPARISON_ENABLED = False
STATIC_CORNER_LOAD_G = None
STATIC_CORNER_LOAD_SOURCE = "disabled"


def set_corner_load_comparison(values_g, source="user-supplied"):
    """Enable the optional static corner-load comparison.

    `values_g` must give all four wheels. `source` is written into the
    workbook alongside the derived columns so a reader can tell which value
    set produced them.
    """
    global CORNER_LOAD_COMPARISON_ENABLED, STATIC_CORNER_LOAD_G, STATIC_CORNER_LOAD_SOURCE
    if not values_g:
        CORNER_LOAD_COMPARISON_ENABLED = False
        STATIC_CORNER_LOAD_G = None
        STATIC_CORNER_LOAD_SOURCE = "disabled"
        return
    missing = [w for w in ("FL", "RL", "FR", "RR") if w not in values_g]
    if missing:
        raise ValueError(
            f"corner_load_comparison.values_g is missing {', '.join(missing)}; "
            "all four corners are required or the shares are meaningless")
    CORNER_LOAD_COMPARISON_ENABLED = True
    STATIC_CORNER_LOAD_G = {w: float(values_g[w]) for w in ("FL", "RL", "FR", "RR")}
    STATIC_CORNER_LOAD_SOURCE = source

# ---------------------------------------------------------------------
# Analysis parameters (unchanged from v8 unless noted)
# ---------------------------------------------------------------------
ACTIVE_WHEEL_THRESHOLD = 0.01
STEADY_STATE_START_RATIO = 0.5
SYMMETRY_FLAG_TOL = 1e-4

# Legacy fixed cluster centres (Change Log #131/#133/#135), calibrated at
# W=0.8 ONLY. Retained solely for --legacy-centers regression runs; the
# default path in v9 derives centres per condition instead. See item 3 in
# the module docstring for why the fixed centres cannot be used across a
# W sweep.
LEGACY_LOW_MODE_CENTER_RAD_S = 0.340
LEGACY_HIGH_MODE_CENTER_RAD_S = 0.501

# v9 adaptive split detection. All three are heuristics chosen to be
# transparent rather than optimal -- SplitDetection prints the underlying
# gap statistics so a verdict can be overruled by inspection.
SPLIT_MIN_RUNS = 4          # below this, report "insufficient N", not a verdict
# Cluster-cut threshold, as a fraction of the group's own mean |W_ss|:
# any gap between consecutive sorted values that exceeds this fraction
# starts a new cluster. Scale-relative rather than absolute so the same
# number works at W=0.2 and W=0.8. 5% was checked against every baseline
# condition currently on hand (W=0.2/0.4/0.5/0.55 -> 1 cluster,
# W=0.6 -> 3, W=0.8 -> 2); it is still a heuristic, and SplitDetection
# prints every gap so a verdict can be overruled by inspection.
SPLIT_GAP_FRACTION = 0.05

TIME_SERIES_DT = 0.05
TIME_SERIES_MAX_T = 10.0
DIVERGENCE_ONSET_FRACTION = 0.5

BREAKAWAY_THRESHOLD_RAD_S = 0.1
STEADY_STATE_SETTLE_S = 2.0

BREAKAWAY_ALIGN_PRE_S = 0.3
BREAKAWAY_ALIGN_POST_S = 2.0
BREAKAWAY_ALIGN_DT = 0.02

# Commanded-W grouping tolerance. Runs whose derived commanded W rounds
# to the same value at this precision are treated as one condition.
W_GROUP_DECIMALS = 2

# v10: Max Drive Force comparison and zero-force functional check.
# Folder-name force tokens are treated as experiment metadata because the
# current ROS bag manifest does not publish the authored/runtime joint-drive
# maxForce value. Future runs should prefer the unambiguous vForce1p8 form.
MAX_FORCE_GROUP_DECIMALS = 4
ZERO_FORCE_TOL_NM = 1e-9
NO_MOTION_WHEEL_PEAK_RAD_S = 0.01
NO_MOTION_BODY_PEAK_RAD_S = 0.01

# v12: Joint Drive Damping and Solver are additional experimental factors,
# parsed from the folder name for the same reason Max Force is: the bag
# manifest does not publish the authored joint-drive/solver configuration.
DAMPING_GROUP_DECIMALS = 4

# v12: cross-workbook consistent Low/High reference centres. These are the
# established baseline centres (Change Log #131/#133/#135, PGS, W=0.8) so a
# single workbook does not have to re-derive its own Low/High split from
# scratch. Applies ONLY to Solver == "PGS" (the frozen baseline solver); TGS
# and any unrecognised solver always fall back to pure in-workbook adaptive
# detection (see build_reference_centers()). Override or extend with
# --baseline-json path/to/file.json, format:
#   {"0.8": {"Low": 0.340, "High": 0.501}, "0.6": {...}}
BASELINE_CENTERS_SOLVER = "PGS"
BASELINE_CENTERS_DEFAULT = {
    0.8: {"Low": 0.340, "High": 0.501},
}
# A new adaptive cluster only counts as "already covered by baseline" if it
# sits within this many rad/s of a baseline centre; otherwise it is reported
# as an additional, out-of-baseline cluster (e.g. a damping-shifted or
# solver-specific state) rather than being folded into Low or High.
BASELINE_MATCH_TOL_RAD_S = 0.08

# v12: breakaway must be anchored to command start (not to the first
# /joint_states sample) and must be a SUSTAINED crossing, not a single
# noisy sample above threshold.
BREAKAWAY_SUSTAIN_S = 0.10
COMMAND_ACTIVE_THRESHOLD_RAD_S = 1e-3

# v12: steady-state window quality. A sample-count floor alone (min_samples)
# does not rule out a late-breakaway run whose "steady" window covers well
# under a second of real settled data. Both a sample floor and a duration
# floor are enforced from here on.
STEADY_STATE_MIN_WINDOW_S = 1.0

# v12: window quality is topic-relative. A single sample-count floor is not
# transferable between topics publishing at different rates -- on the 0804
# bags /joint_states and /odom_truth run at ~60 Hz while the contact-force
# topics run at ~13 Hz, so a floor of 5 samples corresponds to 0.08 s of
# wheel data but 0.39 s of contact data. The completeness check compares the
# samples actually inside the window against what the topic's OWN median
# publish rate predicts for a window of that length, so it scales with the
# topic instead of requiring a hand-tuned floor per topic, and it also
# catches a dropout inside a window that is otherwise long enough.
STEADY_STATE_MIN_SAMPLES = 5
# Calibrated against the 0804 bags, where a gapless contact-force stream
# scores 0.897-0.908 (the shortfall is ordinary publish jitter, not loss).
# 0.70 therefore leaves roughly 20 points of margin below the observed
# healthy value while still flagging a 40% dropout. Falling below it does
# NOT reject the window -- `complete_enough` is reported separately from
# `valid` -- so a false positive costs only a warning line, whereas a
# false negative lets a gap-ridden window be read as a clean measurement.
WINDOW_MIN_RATE_COMPLETENESS = 0.70

# v12: W_ss cross-check tolerance -- flag a run when the breakaway-anchored
# W_ss, the fraction-based body tail mean, and the whole-recording
# angle/time estimate disagree by more than this relative fraction.
W_SS_CONSISTENCY_REL_TOL = 0.10


# =====================================================================
# Geometry / math helpers
# =====================================================================
def quaternion_to_yaw(qx, qy, qz, qw):
    """Yaw (rotation about Z) from a quaternion -- same formula as
    cobraflex_rosbag_analyzer.py, kept in sync so angular-displacement
    numbers stay directly comparable between the two scripts."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def unwrap_delta(prev_yaw, curr_yaw):
    delta = curr_yaw - prev_yaw
    while delta > math.pi:
        delta -= 2.0 * math.pi
    while delta < -math.pi:
        delta += 2.0 * math.pi
    return delta


def cumulative_unwrapped_angle(yaw_list):
    """Total signed rotation (rad) via shortest-path unwrapping between
    consecutive yaw samples."""
    if len(yaw_list) < 2:
        return 0.0
    total = 0.0
    for i in range(1, len(yaw_list)):
        total += unwrap_delta(yaw_list[i - 1], yaw_list[i])
    return total


def wheel_omega_to_body_rate(wheel_omega):
    """Body yaw rate (rad/s) that a given wheel angular speed (rad/s)
    WOULD produce for in-place rotation if the contact patch did not slip
    at all. This is the denominator of the true slip ratio.

    Skid-steer in-place rotation: both sides counter-rotate, so the
    surface speed at each wheel is omega*r and the body turns at
    (omega*r)/(track/2).
    """
    return abs(wheel_omega) * WHEEL_RADIUS_M / HALF_TRACK_M


def body_rate_to_wheel_omega(body_rate):
    """Inverse of wheel_omega_to_body_rate() -- used to derive the
    commanded W from the commanded wheel speed when /cmd_vel is absent."""
    return abs(body_rate) * HALF_TRACK_M / WHEEL_RADIUS_M


def extract_scalar_force(msg):
    """Best-effort extraction of a scalar force magnitude from an unknown
    message type. See v8 docstring -- unchanged. Returns (value, field)
    with field == "UNRECOGNIZED" if nothing matched."""
    if hasattr(msg, "data"):
        try:
            return float(msg.data), "data"
        except (TypeError, ValueError):
            pass
    if hasattr(msg, "value"):
        try:
            return float(msg.value), "value"
        except (TypeError, ValueError):
            pass
    if hasattr(msg, "force") and hasattr(msg.force, "x"):
        f = msg.force
        return float((f.x ** 2 + f.y ** 2 + f.z ** 2) ** 0.5), "force (Vector3 magnitude)"
    if hasattr(msg, "wrench") and hasattr(msg.wrench, "force"):
        f = msg.wrench.force
        return float((f.x ** 2 + f.y ** 2 + f.z ** 2) ** 0.5), "wrench.force (Vector3 magnitude)"
    return None, "UNRECOGNIZED"






def commanded_w_from_folder(folder_name):
    """Fallback commanded-W recovery from the folder name (e.g.
    test_04_w055_t10s_v3 -> 0.55). Only used when neither /cmd_vel nor
    /cobraflex/wheel_cmd_debug is present in the bag. Returns NaN if the
    name does not carry a w-tag."""
    m = re.search(r"_w(\d+)_", folder_name.lower())
    if not m:
        return float("nan")
    digits = m.group(1)
    # "w02" -> 0.2, "w055" -> 0.55, "w08" -> 0.8, "w10" -> 1.0
    return float(digits) / (10.0 ** (len(digits) - 1))


def max_force_from_folder(folder_name):
    """Parse Max Drive Force in N*m/wheel from a force token.

    Preferred, unambiguous notation uses ``p`` as the decimal point:
    ``vMaxDriveForce1p8`` -> 1.8 and
    ``vMaxDriveForce0p125`` -> 0.125. The legacy shorter ``vForce``
    prefix remains accepted.

    The compact notation already used by the experiment is retained:
    ``180`` -> 1.80, ``025`` -> 0.25, ``0125`` -> 0.125, ``008`` ->
    0.08, ``005`` -> 0.05 and any all-zero token -> 0. Returns NaN when
    no token is present. This value is provenance parsed from the folder
    name; it is not runtime feedback.
    """
    # v12: accept the current file-naming shorthand "vMDForce..." in
    # addition to "vMaxDriveForce..." / "vForce...". Without this alias
    # every current-batch run parsed Max Force as N/A (sheet "Funknown").
    m = re.search(r"(?:^|_)v(?:maxdrive|md)?force([0-9]+(?:p[0-9]+)?)(?:_|$)",
                  folder_name.lower())
    if not m:
        return float("nan")

    token = m.group(1)
    if "p" in token:
        try:
            return float(token.replace("p", "."))
        except ValueError:
            return float("nan")

    if set(token) == {"0"}:
        return 0.0
    if token.startswith("0"):
        return float(f"0.{token[1:]}")
    if len(token) >= 3:
        return float(token) / 100.0
    if len(token) == 2:
        return float(token) / 10.0
    return float(token)


def damping_from_folder(folder_name):
    """Parse Joint Drive Damping from a folder-name ``Damping...`` token.

    Same decimal convention as Max Force (a single leading zero is stripped
    and the remainder read as a fraction), matching the values actually in
    use:

        Damping005  -> 0.05
        Damping0075 -> 0.075
        Damping01   -> 0.10

    An unambiguous ``Damping0p05`` / ``Damping0p1`` form (``p`` as decimal
    point) is also accepted and is recommended for future recordings, same
    as ``vForce1p8``. Returns NaN when no token is present -- callers must
    treat that as *unknown*, not as "no damping test in this batch".
    """
    m = re.search(r"(?:^|_)damping([0-9]+(?:p[0-9]+)?)(?:_|$)",
                  folder_name.lower())
    if not m:
        return float("nan")

    token = m.group(1)
    if "p" in token:
        try:
            return float(token.replace("p", "."))
        except ValueError:
            return float("nan")

    if set(token) == {"0"}:
        return 0.0
    if token.startswith("0"):
        return float(f"0.{token[1:]}")
    if len(token) >= 3:
        return float(token) / 100.0
    if len(token) == 2:
        return float(token) / 10.0
    return float(token)


def solver_from_folder(folder_name):
    """Parse the PhysX solver ("PGS" or "TGS") from the folder name.

    Recognised tokens: ``_pgs_``/``_tgs_`` or ``solverPGS``/``solverTGS``
    (case-insensitive), as a standalone underscore-delimited token so this
    does not accidentally match inside an unrelated word. Returns
    ``"unknown"`` -- never a silent default -- when no token is found;
    per item 15, unknown-solver runs must not be folded into either
    solver's summary statistics.
    """
    m = re.search(r"(?:^|_)(?:solver)?(pgs|tgs)(?:_|$)", folder_name.lower())
    return m.group(1).upper() if m else "unknown"


def numeric_group_value(value, decimals):
    """Round a finite numeric grouping value or return ``'unknown'``."""
    try:
        number = float(value)
    except (TypeError, ValueError):
        return "unknown"
    return round(number, decimals) if math.isfinite(number) else "unknown"


def experiment_group_key(row):
    """Return the v12 analysis condition:
    (commanded W, Max Drive Force, Joint Drive Damping, Solver).

    All four are independent experimental factors (item 4). Grouping only
    by (W, Force) as earlier versions did would silently pool runs that
    differ in Damping or Solver into the same condition."""
    return (
        numeric_group_value(row.get("_w_cmd_num"), W_GROUP_DECIMALS),
        numeric_group_value(row.get("_max_force_nm"), MAX_FORCE_GROUP_DECIMALS),
        numeric_group_value(row.get("_damping_num"), DAMPING_GROUP_DECIMALS),
        row.get("_solver", "unknown"),
    )


def reference_group_key(row):
    """Return the v12 shared Low/High reference key: (commanded W, Solver).

    Deliberately narrower than experiment_group_key(): Max Force and
    Damping variations at the same (W, Solver) share one Low/High
    reference (item 6, "Damping ~0.495 rad/s should read as the existing
    High"), but Solver is NOT pooled across PGS/TGS -- a different solver
    is a different dynamical system, not a variant of the same experiment
    (item 6, "TGS ~0.158 rad/s ... cannot be forced into Low")."""
    return (
        numeric_group_value(row.get("_w_cmd_num"), W_GROUP_DECIMALS),
        row.get("_solver", "unknown"),
    )


def group_sort_key(key):
    """Stable mixed numeric/string sort key for (W, Force, Damping, Solver)
    groups (or any leading-numeric-then-string subset of that tuple)."""
    out = []
    for value in key:
        if isinstance(value, str):
            out.append((True, value, 0.0))
        else:
            out.append((False, "", value))
    return tuple(out)


def numeric_column(frame, column_name):
    """Return one numeric Series, or an empty Series if the column is absent."""
    if column_name not in frame.columns:
        return pd.Series(dtype=float)
    return pd.to_numeric(frame[column_name], errors="coerce").dropna()


# =====================================================================
# Bag loading
# =====================================================================
TOPIC_CLOCK = "/clock"


def msg_time_s(msg, fallback_ns):
    """v12 (item 7): prefer the message's own header.stamp (sim time, as
    published by the node that generated it) over the rosbag2 recorder's
    storage timestamp. The storage timestamp is when the recorder process
    received the message -- on a machine under load, or with any
    wall/sim-clock mismatch, that is NOT the same instant across different
    topics, which is exactly the failure mode item 7 flags for cross-topic
    alignment (breakaway anchor, steady-state windows, W_ss vs body-angle
    cross-checks all depend on times being comparable across topics).

    Returns (time_s, source) where source is "header.stamp" when a finite,
    non-zero stamp was found, else "bag storage timestamp" as a documented
    fallback (not a silent one -- the source is recorded).
    """
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None) if header is not None else None
    if stamp is not None:
        sec = getattr(stamp, "sec", None)
        nanosec = getattr(stamp, "nanosec", None)
        if sec is not None and nanosec is not None and (sec != 0 or nanosec != 0):
            return float(sec) + float(nanosec) * 1e-9, "header.stamp"
    return fallback_ns / 1e9, "bag storage timestamp"


def load_bag(bag_path):
    """Single pass over the bag, dispatching all topics of interest.
    Reading once (not once per topic) keeps I/O cost flat as topic count
    grows."""
    typestore = get_typestore(Stores.ROS2_HUMBLE)

    cmd_times = []
    cmd_series = {w: [] for w in WHEELS}

    joint_times = []
    joint_series = {w: [] for w in WHEELS}

    odomtruth_times = []
    odomtruth_yaw = []
    odomtruth_w = []

    cmdvel_times = []
    cmdvel_wz = []
    # Recorded so that a rotation-only command profile can be distinguished
    # from an arc: without the linear channel, "W is non-zero" alone would
    # match Test03 as readily as Test04.
    cmdvel_linear_x = []

    contact_topic_to_wheel = {v: k for k, v in CONTACT_FORCE_TOPICS.items()}
    contact_times = {w: [] for w in WHEELS}
    contact_series = {w: [] for w in WHEELS}
    contact_field_used = {w: None for w in WHEELS}

    clock_times = []

    # v12 (item 16): raw per-topic quality bookkeeping. "raw_stamp" is the
    # storage timestamp for EVERY message seen on the topic, independent of
    # whether the message payload parsed successfully -- duplicate/gap/
    # publish-rate checks must see every message, not only the ones that
    # passed the by-name wheel-mapping filter.
    topic_raw_stamps = {}
    time_source_counts = {}

    # v12 (2026-08-04): the joint-name ORDER of each JointState topic, taken
    # from the first message that carries one. /joint_states and
    # /cobraflex/wheel_cmd_debug were found to publish the same four wheels
    # in DIFFERENT orders ([FL, FR, RL, RR] vs [FL, RL, FR, RR]), so any
    # index-based reader transposes FR and RL between the two. Both topics
    # are mapped by name here and are therefore unaffected, but the
    # condition was previously invisible; it is now recorded so a mismatch
    # can be surfaced in ParsingWarnings.
    joint_name_order = {}

    # v12 (2026-08-04): storage-timestamp -> header.stamp offset pairs,
    # collected from every stamped topic. See the mapping step after the
    # read loop for why this is required.
    clock_offset_pairs = []
    cmdvel_time_sources = set()
    contact_time_sources = {w: set() for w in ("FL", "RL", "FR", "RR")}

    def note_time_source(topic, source):
        time_source_counts.setdefault(topic, {"header.stamp": 0, "bag storage timestamp": 0})
        time_source_counts[topic][source] += 1

    with Reader(bag_path) as reader:
        for conn, timestamp, rawdata in reader.messages():
            topic_raw_stamps.setdefault(conn.topic, []).append(timestamp / 1e9)

            if conn.topic == TOPIC_CLOCK:
                try:
                    msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
                    clock_times.append(
                        float(msg.clock.sec) + float(msg.clock.nanosec) * 1e-9)
                except Exception:  # noqa: BLE001
                    pass
                continue

            if conn.topic in contact_topic_to_wheel:
                wheel = contact_topic_to_wheel[conn.topic]
                msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
                val, field_used = extract_scalar_force(msg)
                if val is None:
                    continue
                t_s, src = msg_time_s(msg, timestamp)
                note_time_source(conn.topic, src)
                contact_time_sources[wheel].add(src)
                contact_times[wheel].append(t_s)
                contact_series[wheel].append(val)
                if contact_field_used[wheel] is None:
                    contact_field_used[wheel] = field_used
                continue

            if conn.topic == TOPIC_WHEEL_CMD_DEBUG:
                msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
                if not hasattr(msg, "name") or not hasattr(msg, "velocity"):
                    continue
                joint_name_order.setdefault(conn.topic, tuple(msg.name))
                if len(msg.name) != len(msg.velocity):
                    continue
                by_name = {WHEEL_NAME_MAP[n]: float(v)
                           for n, v in zip(msg.name, msg.velocity)
                           if n in WHEEL_NAME_MAP}
                if len(by_name) != 4:
                    continue
                t_s, src = msg_time_s(msg, timestamp)
                note_time_source(conn.topic, src)
                cmd_times.append(t_s)
                for wheel in cmd_series:
                    cmd_series[wheel].append(by_name[wheel])

            elif conn.topic == TOPIC_JOINT_STATES:
                msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
                if not hasattr(msg, "name") or not hasattr(msg, "velocity"):
                    continue
                joint_name_order.setdefault(conn.topic, tuple(msg.name))
                if len(msg.name) != len(msg.velocity) or len(msg.velocity) == 0:
                    continue
                by_name = {WHEEL_NAME_MAP[n]: float(v)
                           for n, v in zip(msg.name, msg.velocity)
                           if n in WHEEL_NAME_MAP}
                if len(by_name) != 4:
                    # /joint_states may carry non-wheel joints on some
                    # configurations -- only accept messages reporting all
                    # four wheels, same defensive stance as v1-v8.
                    continue
                t_s, src = msg_time_s(msg, timestamp)
                note_time_source(conn.topic, src)
                if src == "header.stamp":
                    clock_offset_pairs.append((timestamp / 1e9, t_s))
                joint_times.append(t_s)
                for wheel in joint_series:
                    joint_series[wheel].append(by_name[wheel])

            elif conn.topic == TOPIC_ODOM_TRUTH:
                msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
                q = msg.pose.pose.orientation
                t_s, src = msg_time_s(msg, timestamp)
                note_time_source(conn.topic, src)
                if src == "header.stamp":
                    clock_offset_pairs.append((timestamp / 1e9, t_s))
                odomtruth_times.append(t_s)
                odomtruth_yaw.append(quaternion_to_yaw(q.x, q.y, q.z, q.w))
                odomtruth_w.append(float(msg.twist.twist.angular.z))

            elif conn.topic == TOPIC_CMD_VEL:
                msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
                try:
                    wz = float(msg.angular.z)
                except AttributeError:
                    continue
                # geometry_msgs/Twist has no header; TwistStamped does.
                # msg_time_s() falls back to the bag storage timestamp for
                # plain Twist automatically and records that as the source.
                t_s, src = msg_time_s(msg, timestamp)
                note_time_source(conn.topic, src)
                cmdvel_time_sources.add(src)
                cmdvel_times.append(t_s)
                cmdvel_wz.append(wz)
                cmdvel_linear_x.append(float(getattr(msg.linear, "x", 0.0)))

    # -----------------------------------------------------------------
    # v12 (2026-08-04): map headerless /cmd_vel onto the header.stamp clock.
    #
    # geometry_msgs/Twist carries no header, so msg_time_s() necessarily
    # falls back to the rosbag2 storage timestamp -- a Unix epoch value of
    # ~1.79e9 s. Every stamped topic (/joint_states, /odom_truth) reports
    # header.stamp, which under use_sim_time is simulation time counted from
    # session start, i.e. ~1e2 s. Mixing the two puts the command-start
    # anchor roughly 1.8 billion seconds after every wheel sample, so the
    # breakaway-anchored steady-state window selects nothing and EVERY
    # steady-state KPI silently degrades to N/A with the run labelled
    # Invalid. This is not hypothetical: it is what the 0804 bags do.
    #
    # The offset is taken as the median over all stamped messages rather
    # than from a single anchor pair, so one late-delivered message cannot
    # shift the whole timebase. It is only applied when /cmd_vel actually
    # used the storage fallback AND a stamped reference exists.
    # std_msgs/Float32 has no header either, so the four contact-force
    # topics land on the same storage clock as /cmd_vel and need the same
    # mapping. Without it the contact window test `t >= anchor` is
    # trivially true for every message -- an epoch timestamp always exceeds
    # a simulation-time anchor -- so the "steady-state" contact average
    # silently covered the WHOLE recording, including the stationary
    # pre-command phase, while claiming to use the breakaway-anchored
    # window.
    clock_offset_s = float("nan")
    if clock_offset_pairs:
        offsets = [hdr - storage for storage, hdr in clock_offset_pairs]
        clock_offset_s = float(np.median(offsets))
        if cmdvel_times and cmdvel_time_sources == {"bag storage timestamp"}:
            cmdvel_times = [t + clock_offset_s for t in cmdvel_times]
        for wheel, srcs in contact_time_sources.items():
            if contact_times[wheel] and srcs == {"bag storage timestamp"}:
                contact_times[wheel] = [t + clock_offset_s
                                        for t in contact_times[wheel]]

    return {
        "clock_offset_s": clock_offset_s,
        "cmd_times": cmd_times, "cmd_series": cmd_series,
        "joint_times": joint_times, "joint_series": joint_series,
        "odomtruth_times": odomtruth_times, "odomtruth_yaw": odomtruth_yaw,
        "odomtruth_w": odomtruth_w,
        "cmdvel_times": cmdvel_times, "cmdvel_wz": cmdvel_wz,
        "cmdvel_linear_x": cmdvel_linear_x,
        "contact_times": contact_times, "contact_series": contact_series,
        "contact_field_used": contact_field_used,
        "clock_times": clock_times,
        "topic_raw_stamps": topic_raw_stamps,
        "time_source_counts": time_source_counts,
        "joint_name_order": joint_name_order,
    }





# =====================================================================
# Windowing helpers
# =====================================================================
def steady_state_mean(arr, start_ratio=STEADY_STATE_START_RATIO):
    arr = np.asarray(arr, dtype=float)
    if len(arr) == 0:
        return float("nan")
    window = arr[int(len(arr) * start_ratio):]
    return float(np.mean(window)) if len(window) else float("nan")


def steady_state_std(arr, start_ratio=STEADY_STATE_START_RATIO):
    arr = np.asarray(arr, dtype=float)
    if len(arr) == 0:
        return float("nan")
    window = arr[int(len(arr) * start_ratio):]
    return float(np.std(window)) if len(window) else float("nan")


def median_publish_rate(times):
    """Median publish rate of a timestamp series, in Hz.

    The median (not mean) of the inter-sample intervals is used so that a
    single long gap -- a simulation stall, a dropped block of messages --
    does not drag the reference rate down and thereby make the
    completeness check pass a window it should have flagged.
    """
    t = np.asarray(times, dtype=float)
    if t.size < 3:
        return float("nan")
    dt = np.diff(t)
    dt = dt[dt > 0]
    if dt.size == 0:
        return float("nan")
    med = float(np.median(dt))
    return 1.0 / med if med > 0 else float("nan")


def window_rate_completeness(all_times, n_in_window, window_length_s):
    """Observed window samples divided by the count the topic's own median
    rate predicts for a window of this length.

    Returns NaN when the rate cannot be estimated (too few samples), which
    callers must treat as "unknown", never as a pass.
    """
    rate = median_publish_rate(all_times)
    if rate != rate or window_length_s != window_length_s or window_length_s <= 0:
        return float("nan")
    expected = rate * window_length_s
    if expected <= 0:
        return float("nan")
    return float(n_in_window) / expected


def breakaway_anchored_window(times, values, anchor_time, breakaway_time,
                              settle_s=STEADY_STATE_SETTLE_S,
                              min_samples=STEADY_STATE_MIN_SAMPLES,
                              min_window_s=STEADY_STATE_MIN_WINDOW_S):
    """Select samples at or after (anchor_time + breakaway_time + settle_s).

    v12 (item 9): a sample-count floor alone does not rule out a
    late-breakaway run whose "steady" window covers well under a second of
    real settled recording (e.g. 5 samples spread over 0.05s at a high
    publish rate). A duration floor (min_window_s) is now enforced in
    addition to the existing sample-count floor. Returns an empty array
    when either requirement is not met -- callers must treat that as
    "insufficient", never as zero.

    `anchor_time` is the breakaway-timing anchor (command start, see
    find_command_start_time()) -- kept as a positional parameter named
    generically rather than "joint_t0" since it is no longer always the
    first /joint_states sample (item 8)."""
    if breakaway_time != breakaway_time or not len(times):  # NaN or empty
        return np.array([])
    times_arr = np.asarray(times, dtype=float)
    values_arr = np.asarray(values, dtype=float)
    anchor = anchor_time + breakaway_time + settle_s
    mask = times_arr >= anchor
    windowed = values_arr[mask]
    if len(windowed) < min_samples:
        return np.array([])
    win_times = times_arr[mask]
    if len(win_times) >= 2 and (win_times[-1] - win_times[0]) < min_window_s:
        return np.array([])
    return windowed


def breakaway_anchored_window_info(times, values, anchor_time, breakaway_time,
                                   settle_s=STEADY_STATE_SETTLE_S,
                                   min_samples=STEADY_STATE_MIN_SAMPLES,
                                   min_window_s=STEADY_STATE_MIN_WINDOW_S,
                                   min_completeness=WINDOW_MIN_RATE_COMPLETENESS):
    """v12 (item 9): same selection as breakaway_anchored_window(), but
    returns the window's own start/end/length/sample-count for the Index
    sheet, so a "steady-state" KPI can be audited rather than trusted
    blindly. `valid` mirrors whether breakaway_anchored_window() would have
    accepted the window (sample-count and duration floors met).

    v12 (2026-08-04) additionally reports `completeness` and
    `complete_enough` -- see window_rate_completeness(). These are reported
    separately from `valid` rather than folded into it, so that tightening
    the completeness rule later cannot silently change which runs the
    existing sample/duration floors already accepted. Callers decide which
    of the two verdicts a given KPI requires.
    """
    info = {"start_s": float("nan"), "end_s": float("nan"),
            "length_s": float("nan"), "n_samples": 0, "valid": False,
            "rate_hz": float("nan"), "completeness": float("nan"),
            "complete_enough": False}
    if breakaway_time != breakaway_time or not len(times):
        return info
    times_arr = np.asarray(times, dtype=float)
    anchor = anchor_time + breakaway_time + settle_s
    mask = times_arr >= anchor
    win_times = times_arr[mask]
    info["n_samples"] = int(len(win_times))
    if len(win_times):
        info["start_s"] = float(win_times[0])
        info["end_s"] = float(win_times[-1])
        info["length_s"] = float(win_times[-1] - win_times[0])
    info["valid"] = (
        info["n_samples"] >= min_samples
        and (info["n_samples"] < 2 or info["length_s"] >= min_window_s))
    info["rate_hz"] = median_publish_rate(times_arr)
    info["completeness"] = window_rate_completeness(
        times_arr, info["n_samples"], info["length_s"])
    info["complete_enough"] = bool(
        info["completeness"] == info["completeness"]
        and info["completeness"] >= min_completeness)
    return info


def breakaway_anchored_mean(times, values, anchor_time, breakaway_time,
                            settle_s=STEADY_STATE_SETTLE_S):
    w = breakaway_anchored_window(times, values, anchor_time, breakaway_time, settle_s)
    return float(np.mean(w)) if len(w) else float("nan")


def breakaway_anchored_std(times, values, anchor_time, breakaway_time,
                           settle_s=STEADY_STATE_SETTLE_S):
    w = breakaway_anchored_window(times, values, anchor_time, breakaway_time, settle_s)
    return float(np.std(w)) if len(w) else float("nan")


def resample_series(raw_times, raw_values, t_grid):
    """Interpolate onto t_grid with the run's own recording start as t=0.
    Points past the run's last sample become NaN (not flat-extrapolated),
    so a short run cannot silently bias a group mean."""
    if len(raw_times) < 2:
        return np.full_like(t_grid, np.nan, dtype=float)
    t_rel = np.array(raw_times) - raw_times[0]
    vals = np.interp(t_grid, t_rel, raw_values)
    return np.where(t_grid <= t_rel[-1], vals, np.nan)


def wheel_avg_series(times, series):
    """Mean of |FL|, |RL|, |FR|, |RR| at each sample -- one scalar 'how
    fast are the wheels actually turning' signal. Per-wheel and per-side
    values are reported separately in the Index sheet."""
    if not times:
        return [], []
    n = min([len(times)] + [len(series[w]) for w in WHEELS])
    vals = [sum(abs(series[w][i]) for w in WHEELS) / 4.0 for i in range(n)]
    return list(times[:n]), vals


def find_command_start_time(cmd_times, cmd_series, cmdvel_times, cmdvel_wz, fallback_t0):
    """v12 (item 8): breakaway must be timed from when the ROTATION COMMAND
    actually starts, not from the first /joint_states sample -- the two can
    lag/lead each other by an uncontrolled amount depending on node startup
    order. Prefers /cmd_vel (the direct commanded body rate, the actual
    control input); falls back to the four-wheel wheel_cmd_debug topic;
    falls back to the caller-supplied joint_t0 only when neither command
    topic carries an active sample, and that fallback is reported explicitly
    via the returned source string (surfaced in the Index sheet) rather
    than silently reused as if it were equally reliable."""
    if cmdvel_times and cmdvel_wz:
        for t, wz in zip(cmdvel_times, cmdvel_wz):
            if abs(wz) > COMMAND_ACTIVE_THRESHOLD_RAD_S:
                return t, "/cmd_vel first active sample"
    fl_series = cmd_series.get("FL") if cmd_series else None
    if cmd_times and fl_series:
        for t, v in zip(cmd_times, fl_series):
            if abs(v) > COMMAND_ACTIVE_THRESHOLD_RAD_S:
                return t, "wheel_cmd_debug first active sample"
    return fallback_t0, "FALLBACK: first /joint_states sample (no active command found)"


def find_breakaway_time(times, wheel_avg_vals, anchor_time,
                        threshold=BREAKAWAY_THRESHOLD_RAD_S,
                        sustain_s=BREAKAWAY_SUSTAIN_S):
    """Time (relative to `anchor_time`, the command start -- see
    find_command_start_time()) at which the wheel-average |omega| first
    SUSTAINS >= threshold for at least sustain_s of continuous recorded
    time. v12 (item 8): a single noisy sample above threshold no longer
    counts as breakaway on its own. The result is clamped to the run's own
    recorded time span, so a crossing cannot be reported past the end of
    the recording."""
    if not times or not wheel_avg_vals or anchor_time != anchor_time:
        return float("nan")
    times_arr = np.asarray(times, dtype=float)
    vals_arr = np.abs(np.asarray(wheel_avg_vals, dtype=float))
    n = len(times_arr)
    if n == 0:
        return float("nan")
    recording_end = float(times_arr[-1])

    # v12 (2026-08-04) FIX: the previous acceptance test was
    #     times_arr[j - 1] - t_start >= sustain_s
    # where j indexes the first sample whose time is NOT less than
    # t_start + sustain_s. times_arr[j - 1] is therefore by construction
    # strictly earlier than t_start + sustain_s, so that difference can
    # never reach sustain_s and the test could not pass for ANY regularly
    # sampled signal. At 60 Hz with sustain_s = 0.10 s the closest it got
    # was 0.083 s. Every genuine crossing was consequently rejected and the
    # function fell through to the "ran off the recording end" branch,
    # reporting breakaway as (recording_end - anchor). That value pushes
    # the settle-anchored steady-state window past the last sample, so the
    # window came back empty and every steady-state KPI degraded to N/A
    # with the run labelled Invalid.
    #
    # The correct test is whether the interval [t_start, t_start+sustain_s)
    # was crossed without ever dropping below the threshold, which is
    # exactly the loop's own exit condition: reaching a sample at or after
    # t_start + sustain_s (j < n) means the full duration was spanned.
    for i in range(n):
        if vals_arr[i] < threshold:
            continue