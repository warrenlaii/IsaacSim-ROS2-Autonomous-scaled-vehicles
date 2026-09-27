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
            continue        t_start = times_arr[i]
        j = i
        sustained = True
        while j < n and times_arr[j] < t_start + sustain_s:
            if vals_arr[j] < threshold:
                sustained = False
                break
            j += 1
        if not sustained:
            continue
        spans_full_duration = j < n
        # A crossing found within the final sustain_s of the recording is
        # still accepted, matching the original intent -- but it is now
        # only reachable when no earlier, fully-spanned crossing exists.
        ran_off_recording_end = j >= n
        if spans_full_duration or ran_off_recording_end:
            candidate = t_start - anchor_time
            if candidate != candidate:
                return float("nan")
            span = max(recording_end - anchor_time, 0.0)
            return float(min(max(candidate, 0.0), span))
    return float("nan")


def align_to_breakaway(raw_times, raw_values, joint_t0, breakaway_time, grid):
    """Resample onto `grid` where grid=0 is THIS run's breakaway instant.
    Out-of-range points become NaN."""
    if len(raw_times) < 2 or breakaway_time != breakaway_time:
        return np.full_like(grid, np.nan, dtype=float)
    t_rel = np.array(raw_times) - joint_t0 - breakaway_time
    vals = np.interp(grid, t_rel, raw_values)
    return np.where((grid >= t_rel[0]) & (grid <= t_rel[-1]), vals, np.nan)


def find_divergence_onset(t_grid, divergence, fraction=DIVERGENCE_ONSET_FRACTION):
    """First t_grid point where |divergence| reaches `fraction` of its own
    final value (mean of the last 20% of valid samples)."""
    div = np.asarray(divergence, dtype=float)
    valid_idx = np.where(~np.isnan(div))[0]
    if len(valid_idx) < 5:
        return float("nan"), float("nan")
    tail_start = valid_idx[int(len(valid_idx) * 0.8)]
    tail = div[tail_start:][~np.isnan(div[tail_start:])]
    if len(tail) == 0:
        return float("nan"), float("nan")
    final_div = float(np.mean(np.abs(tail)))
    threshold = fraction * final_div
    for i in valid_idx:
        if abs(div[i]) >= threshold:
            return float(t_grid[i]), final_div
    return float("nan"), final_div


# =====================================================================
# v9: adaptive per-condition split detection
# =====================================================================
def detect_split(w_ss_values):
    """Cluster one condition group's W_ss values and report how many
    discrete states it contains.

    METHOD -- single-linkage cut on the sorted values: any gap between
    consecutive values larger than SPLIT_GAP_FRACTION x mean(|W_ss|)
    starts a new cluster. The result is the cluster count, each cluster's
    centre, size and internal spread, plus every gap that triggered a cut.

    WHY NOT A TWO-CLUSTER TEST -- an earlier draft of this function forced
    a binary split by taking the largest gap and requiring it to dominate
    the second-largest. That is structurally wrong whenever a condition
    holds three or more states: at W=0.6 the intermediate group
    (~0.28 rad/s, n=2) sits between the main group (~0.25) and the quiet
    branch (~0.36), so the second-largest gap is itself large, the
    dominance ratio falls to 2.25, and the test returns "no split" on data
    that visibly contains three clusters. Counting clusters instead of
    asserting two avoids that failure mode entirely, and surfaces
    intermediate states rather than dissolving them into whichever
    neighbour they happen to sit closer to.

    Groups smaller than SPLIT_MIN_RUNS get no verdict at all -- with N=3
    a genuine multi-state condition and a unimodal one are not
    distinguishable, and reporting either would be overclaiming.
    """
    vals = np.asarray([abs(v) for v in w_ss_values if v == v], dtype=float)
    n = len(vals)
    result = {
        "n": n, "n_clusters": 0, "split": False, "reason": "",
        "clusters": [], "cut_gaps": [], "gap_threshold": float("nan"),
        "largest_gap": float("nan"),
    }
    if n < SPLIT_MIN_RUNS:
        # v12 (item 5): insufficient N for a split VERDICT is not the same
        # claim as "this data is invalid". A small-N group (e.g. 3
        # independent TGS resets) still has real, valid W_ss values -- it
        # is just too small to confirm or rule out a second branch. Report
        # it as one unsplit cluster (so downstream labelling reads
        # "Single", never "Invalid") while keeping the reason text explicit
        # about the sample-size caveat.
        result["reason"] = (f"insufficient N (have {n}, need >= {SPLIT_MIN_RUNS}) for a "
                            f"split verdict -- treated as one unsplit cluster, not Invalid")
        result["insufficient_n"] = True
        if n >= 1:
            result["clusters"] = [{
                "n": n, "center": float(np.mean(vals)),
                "min": float(np.min(vals)), "max": float(np.max(vals)),
                "spread": float(np.max(vals) - np.min(vals)),
            }]
            result["n_clusters"] = 1
        return result

    ordered = np.sort(vals)
    mean_abs = float(np.mean(ordered))
    if mean_abs <= 0:
        result["reason"] = "group mean is zero -- cannot scale a relative threshold"
        return result

    threshold = SPLIT_GAP_FRACTION * mean_abs
    gaps = np.diff(ordered)
    result["gap_threshold"] = threshold
    result["largest_gap"] = float(np.max(gaps)) if len(gaps) else float("nan")

    cut_idx = [i for i, gp in enumerate(gaps) if gp > threshold]
    result["cut_gaps"] = [round(float(gaps[i]), 5) for i in cut_idx]
    chunks = np.split(ordered, [i + 1 for i in cut_idx])

    for chunk in chunks:
        result["clusters"].append({
            "n": int(len(chunk)),
            "center": float(np.mean(chunk)),
            "min": float(np.min(chunk)),
            "max": float(np.max(chunk)),
            "spread": float(np.max(chunk) - np.min(chunk)),
        })
    result["n_clusters"] = len(result["clusters"])
    result["split"] = result["n_clusters"] > 1

    if result["split"]:
        centres = ", ".join(f"{c['center']:.4f} (n={c['n']})" for c in result["clusters"])
        result["reason"] = (f"{result['n_clusters']} clusters at centres {centres}; "
                            f"cut on gap(s) > {threshold:.4f}")
    else:
        result["reason"] = (f"single cluster -- largest gap {result['largest_gap']:.4f} "
                            f"does not exceed the {threshold:.4f} cut threshold "
                            f"({SPLIT_GAP_FRACTION * 100:.0f}% of group mean)")
    return result


def label_clusters(n_clusters):
    """Human-readable mode labels for a detected cluster count, ordered by
    ascending |W_ss|. Two clusters keep the established Low/High naming so
    v9 output stays comparable with every prior workbook; three or more
    get explicit Mid labels rather than being squeezed into a binary that
    would misrepresent them."""
    if n_clusters <= 1:
        return ["Single"]
    if n_clusters == 2:
        return ["Low", "High"]
    return ["Low"] + [f"Mid{i}" for i in range(2, n_clusters)] + ["High"]


def classify_legacy(w_ss_actual):
    """Pre-v9 nearest-centre classification against the fixed W=0.8
    centres. Retained for --legacy-centers regression runs only."""
    if w_ss_actual != w_ss_actual:
        return "N/A", float("nan")
    d_low = abs(abs(w_ss_actual) - LEGACY_LOW_MODE_CENTER_RAD_S)
    d_high = abs(abs(w_ss_actual) - LEGACY_HIGH_MODE_CENTER_RAD_S)
    return ("Low", d_low) if d_low <= d_high else ("High", d_high)


def build_reference_centers(w_key, solver_key, reference_rows, baseline_centers):
    """v12 (item 6): build the shared Low/High(+extra) reference for one
    (commanded W, Solver), preferring an established cross-workbook
    baseline over re-deriving centres from this workbook alone.

    If `solver_key` matches BASELINE_CENTERS_SOLVER and `baseline_centers`
    has an entry for this W, every eligible run is assigned to its nearest
    baseline centre when within BASELINE_MATCH_TOL_RAD_S; runs that fall
    outside that tolerance for every baseline centre are pooled separately
    and clustered among themselves (an "out-of-baseline" state -- e.g. a
    damping- or force-shifted branch the baseline did not anticipate),
    rather than being forced into Low or High.

    Otherwise (no baseline for this W, or a different solver such as TGS)
    falls back to pure in-workbook adaptive detect_split(), scoped to this
    solver only -- a different solver is a different dynamical system, not
    a variant of the PGS baseline experiment.
    """
    w_ss_values = [r.get("_w_ss_num", float("nan")) for r in reference_rows]
    adaptive = detect_split(w_ss_values)

    baseline = None
    if solver_key == BASELINE_CENTERS_SOLVER and isinstance(w_key, (int, float)):
        for bw, centers in baseline_centers.items():
            if abs(float(bw) - w_key) < 1e-6:
                baseline = centers
                break
    if baseline is None:
        adaptive["source"] = (
            "in-workbook adaptive detection (no established baseline for "
            f"W={w_key}, Solver={solver_key})")
        return adaptive

    vals = np.asarray([abs(v) for v in w_ss_values if v == v], dtype=float)
    clusters = []
    for label, center in sorted(baseline.items(), key=lambda kv: kv[1]):
        clusters.append({"n": 0, "center": float(center), "min": float(center),
                          "max": float(center), "spread": 0.0,
                          "baseline_label": label})

    extra_vals = []
    for v in vals:
        dists = [abs(float(v) - c["center"]) for c in clusters]
        idx = int(np.argmin(dists))
        if dists[idx] <= BASELINE_MATCH_TOL_RAD_S:
            c = clusters[idx]
            c["min"] = float(v) if c["n"] == 0 else min(c["min"], float(v))
            c["max"] = float(v) if c["n"] == 0 else max(c["max"], float(v))
            c["n"] += 1
        else:
            extra_vals.append(float(v))
    for c in clusters:
        c["spread"] = c["max"] - c["min"]

    if extra_vals:
        extra_split = detect_split(extra_vals)
        for c in extra_split["clusters"]:
            c = dict(c)
            c["baseline_label"] = None
            clusters.append(c)

    return {
        "n": int(len(vals)),
        "n_clusters": len(clusters),
        "split": len(clusters) > 1,
        "clusters": clusters,
        "reason": (f"baseline centres for W={w_key} ({BASELINE_CENTERS_SOLVER}): "
                   + ", ".join(f"{lbl}={c:.4f}" for lbl, c in
                               sorted(baseline.items(), key=lambda kv: kv[1]))
                   + f"; {len(extra_vals)} run(s) fell outside the "
                   f"{BASELINE_MATCH_TOL_RAD_S} rad/s match tolerance for "
                   f"every baseline centre"),
        "source": "established baseline (Change Log #131/#133/#135, or --baseline-json)",
        "gap_threshold": adaptive.get("gap_threshold", float("nan")),
        "largest_gap": adaptive.get("largest_gap", float("nan")),
        "cut_gaps": adaptive.get("cut_gaps", []),
    }


def label_reference_clusters(assignment):
    """Human-readable labels aligned index-for-index with
    assignment['clusters']. Clusters carrying a baseline_label (from
    build_reference_centers()) keep that label (Low/High/...). Clusters
    with no baseline_label -- either because there is no baseline for this
    (W, Solver) at all, or because they are out-of-baseline extras next to
    a baseline -- get Single/Low/High/Mid (pure-adaptive case) or an
    explicit Out-of-Baseline tag (baseline-plus-extras case)."""
    clusters = assignment.get("clusters", [])
    if not clusters:
        return []
    has_baseline = any(c.get("baseline_label") for c in clusters)
    # v12 (2026-08-04): a group too small for a split verdict must not be
    # given a branch label either. "Single" reads as a finding -- that the
    # condition is unimodal -- which is exactly the claim detect_split()
    # already declined to make at this N. Labelling it Unclassified keeps
    # the W_ss values usable while stating that no branch identity was
    # established. Conditions matched to an established baseline are
    # unaffected: there the label comes from the baseline, not from this
    # workbook's N.
    if not has_baseline and assignment.get("insufficient_n"):
        return ["Unclassified"] * len(clusters)
    labels = [None] * len(clusters)
    if has_baseline:
        n_extra = sum(1 for c in clusters if not c.get("baseline_label"))
        extra_i = 0
        for i, c in enumerate(clusters):
            lbl = c.get("baseline_label")
            if lbl:
                labels[i] = lbl
            else:
                extra_i += 1
                labels[i] = "Out-of-Baseline" if n_extra == 1 else f"Out-of-Baseline-{extra_i}"
    else:
        order = sorted(range(len(clusters)), key=lambda i: clusters[i]["center"])
        base_labels = label_clusters(len(clusters))
        for rank, i in enumerate(order):
            labels[i] = base_labels[rank] if rank < len(base_labels) else f"M{rank + 1}"
    return labels


def assign_modes(index_rows, use_legacy=False, baseline_centers=None):
    """Pass 2: classify branches consistently across Max Force conditions.

    Zero-force runs are functional checks and never enter Low/High
    clustering. Any other run whose wheel/body telemetry shows no motion is
    labelled ``No Motion`` and excluded from the cluster fit.

    The Low/High reference centres are derived by pooling eligible runs at
    the same commanded W across non-zero Max Force conditions. Each
    individual Max Force condition is still clustered separately for the
    SplitDetection diagnostic, but its run labels are assigned against the
    shared commanded-W reference. This preserves branch identity when a
    condition selects only one of the established branches.
    """
    if use_legacy:
        for row in index_rows:
            force = row.get("_max_force_nm", float("nan"))
            zero_force = force == force and abs(force) <= ZERO_FORCE_TOL_NM
            motion_state = row.get("_motion_state", "Inconclusive")
            if zero_force:
                check = row.get("_zero_force_check", "")
                row["Mode"] = ("No Motion" if check.startswith("PASS") else
                               ("Zero-Force FAIL" if check.startswith("FAIL") else "N/A"))
                row["Mode Distance to Nearest Center (rad/s)"] = "N/A"
                row["Mode Basis"] = "zero-force functional check; excluded from clustering"
            elif motion_state == "No Motion":
                row["Mode"] = "No Motion"
                row["Mode Distance to Nearest Center (rad/s)"] = "N/A"
                row["Mode Basis"] = "no motion; excluded from legacy fixed-centre classification"
            else:
                mode, dist = classify_legacy(row.get("_w_ss_num", float("nan")))
                row["Mode"] = "Invalid" if mode == "N/A" else mode
                row["Mode Distance to Nearest Center (rad/s)"] = (
                    round(dist, 4) if dist == dist else "N/A")
                row["Mode Basis"] = (
                    "legacy fixed centres (0.340 / 0.501, W=0.8 calibration)"
                    if mode != "N/A" else
                    "invalid: no finite steady-state body angular velocity")
        return [{
            "Commanded W (rad/s)": "ALL",
            "Max Drive Force (N·m/wheel)": "ALL",
            "N Runs": len(index_rows),
            "Split Detected": "N/A -- legacy fixed-centre mode",
            "Verdict Reason": ("run with --legacy-centers; per-condition detection "
                               "skipped; zero-force/no-motion exclusions retained"),
        }]

    if baseline_centers is None:
        baseline_centers = BASELINE_CENTERS_DEFAULT

    groups = {}
    for row in index_rows:
        groups.setdefault(experiment_group_key(row), []).append(row)

    # v12: shared branch reference per (commanded W, Solver) -- NOT per W
    # alone. Max Force and Damping are experimental factors whose effect on
    # branch choice is the outcome being tested, so they share one
    # reference; Solver is a different dynamical system (different PhysX
    # integrator) and must never share a PGS reference with TGS runs or
    # vice versa (item 6).
    reference_rows_by_key = {}
    for row in index_rows:
        force_key = numeric_group_value(row.get("_max_force_nm"), MAX_FORCE_GROUP_DECIMALS)
        is_zero_force = (
            force_key != "unknown" and abs(force_key) <= ZERO_FORCE_TOL_NM)
        w_ss = row.get("_w_ss_num", float("nan"))
        if (not is_zero_force
                and row.get("_motion_state") == "Motion"
                and w_ss == w_ss):
            reference_rows_by_key.setdefault(reference_group_key(row), []).append(row)
    reference_by_key = {
        ref_key: build_reference_centers(ref_key[0], ref_key[1], rows, baseline_centers)
        for ref_key, rows in reference_rows_by_key.items()
    }

    split_records = []
    for key in sorted(groups, key=group_sort_key):
        w_key, force_key, damping_key, solver_key = key
        rows = groups[key]
        ref_key = (w_key, solver_key)
        zero_force_group = (
            force_key != "unknown" and abs(force_key) <= ZERO_FORCE_TOL_NM)
        eligible_rows = [
            row for row in rows
            if row.get("_motion_state") == "Motion"
            and row.get("_w_ss_num", float("nan")) ==
            row.get("_w_ss_num", float("nan"))
        ]
        no_motion_n = sum(row.get("_motion_state") == "No Motion" for row in rows)

        if zero_force_group:
            pass_n = fail_n = inconclusive_n = 0
            for row in rows:
                check = row.get("_zero_force_check", "")
                if check.startswith("PASS"):
                    row["Cluster"] = "N/A"
                    row["Mode"] = "No Motion"
                    pass_n += 1
                elif check.startswith("FAIL"):
                    row["Cluster"] = "N/A"
                    row["Mode"] = "Zero-Force FAIL"
                    fail_n += 1
                else:
                    row["Cluster"] = "N/A"
                    row["Mode"] = "Invalid"
                    inconclusive_n += 1
                row["Mode Distance to Nearest Center (rad/s)"] = "N/A"
                row["Mode Basis"] = (
                    "Max Force=0 functional check; excluded from bimodal clustering")

            if fail_n:
                verdict = (f"ZERO-FORCE FAIL: {fail_n}/{len(rows)} run(s) moved; "
                           "verify the edited joint/drive layer or runtime override")
            elif pass_n and not inconclusive_n:
                verdict = (f"ZERO-FORCE PASS: all {pass_n} run(s) had an active "
                           "command and no wheel/body motion")
            else:
                verdict = (f"ZERO-FORCE INCONCLUSIVE: pass={pass_n}, fail={fail_n}, "
                           f"inconclusive={inconclusive_n}")
            split_records.append({
                "Commanded W (rad/s)": w_key,
                "Max Drive Force (N·m/wheel)": force_key,
                "Joint Drive Damping": damping_key,
                "Solver": solver_key,
                "N Runs": len(rows),
                "N Clustered Runs": 0,
                "No-Motion Runs": no_motion_n,
                "Invalid Runs": inconclusive_n,
                "Clusters Detected": "N/A",
                "Reference Clusters at Commanded W": "N/A",
                "Reference Centres (rad/s)": "-",
                "Reference Source": "N/A",
                "Split Detected": "N/A — zero-force gate",
                "Gap Cut Threshold (rad/s)": "N/A",
                "Largest Gap (rad/s)": "N/A",
                "Cut Gaps (rad/s)": "-",
                "Verdict Reason": verdict,
            })
            continue

        res = detect_split([r.get("_w_ss_num", float("nan"))
                            for r in eligible_rows])
        reference = reference_by_key.get(ref_key, res)
        assignment = reference if reference.get("clusters") else res
        labels = label_reference_clusters(assignment) if assignment["clusters"] else ["N/A"]

        # Assign each valid run to the nearest shared (W, Solver) reference
        # centre. A condition containing only High therefore remains High
        # instead of being renamed Single; an unclassifiable run (no
        # breakaway / no finite W_ss) remains Invalid regardless of N.
        invalid_n = 0
        for row in rows:
            w_ss = row.get("_w_ss_num", float("nan"))
            if row.get("_motion_state") == "No Motion":
                row["Cluster"] = "N/A"
                row["Mode"] = "No Motion"
                row["Mode Distance to Nearest Center (rad/s)"] = "N/A"
            elif w_ss != w_ss or not assignment["clusters"]:
                row["Cluster"] = "N/A"
                row["Mode"] = "Invalid"
                row["Mode Distance to Nearest Center (rad/s)"] = "N/A"
                invalid_n += 1
            else:
                a = abs(w_ss)
                idx = int(np.argmin(
                    [abs(a - c["center"]) for c in assignment["clusters"]]))
                row["Cluster"] = f"M{idx + 1}"
                row["Mode"] = labels[idx] if idx < len(labels) else f"M{idx + 1}"
                row["Mode Distance to Nearest Center (rad/s)"] = round(
                    abs(a - assignment["clusters"][idx]["center"]), 4)
            row["Mode Basis"] = (
                f"shared (W={w_key}, Solver={solver_key}) reference "
                f"[{assignment.get('source', 'in-workbook adaptive detection')}], "
                f"reference N={assignment['n']}, {assignment['n_clusters']} cluster(s); "
                f"local condition Max Force={force_key}, Damping={damping_key}, "
                f"N clustered={res['n']}, {res['n_clusters']} local cluster(s)"
                + (" [insufficient N for a local split verdict]"
                   if res.get("insufficient_n") else ""))

        rec = {
            "Commanded W (rad/s)": w_key,
            "Max Drive Force (N·m/wheel)": force_key,
            "Joint Drive Damping": damping_key,
            "Solver": solver_key,
            "N Runs": len(rows),
            "N Clustered Runs": res["n"],
            "No-Motion Runs": no_motion_n,
            "Invalid Runs": invalid_n,
            "Clusters Detected": res["n_clusters"] if res["n_clusters"] else "N/A",
            "Reference Clusters at Commanded W": (
                assignment["n_clusters"] if assignment["n_clusters"] else "N/A"),
            "Reference Centres (rad/s)": (
                ", ".join(f"{c['center']:.4f}" for c in assignment["clusters"])
                if assignment["clusters"] else "-"),
            "Reference Source": assignment.get("source", "in-workbook adaptive detection"),
            "Split Detected": ("YES" if res["split"] else
                               ("no" if res["n_clusters"] == 1 else "N/A")),
            "Gap Cut Threshold (rad/s)": (round(res["gap_threshold"], 5)
                                          if res["gap_threshold"] == res["gap_threshold"] else "N/A"),
            "Largest Gap (rad/s)": (round(res["largest_gap"], 5)
                                    if res["largest_gap"] == res["largest_gap"] else "N/A"),
            "Cut Gaps (rad/s)": ", ".join(str(g) for g in res["cut_gaps"]) or "-",
            "Verdict Reason": (
                f"Local condition: {res['reason']}. "
                f"Branch labels use the shared (W={w_key}, Solver={solver_key}) reference."),
        }
        # One column quartet per cluster SLOT (M1, M2, M3...), not per label.
        # An earlier version embedded the mode label in the column name itself
        # (e.g. "M1 (Single) N" on one row, "M1 (Low) N" on another) -- across
        # a mixed batch that produces a different column set per row, so the
        # sheet fans out to dozens of mostly-empty columns and the same
        # cluster slot is unreadable across rows. Fixed M1/M2/M3... column
        # names plus a separate "Mode" sub-column keep one addressable column
        # per slot regardless of what any given condition's clusters were
        # labelled.
        for i, c in enumerate(res["clusters"]):
            if assignment["clusters"]:
                ref_idx = int(np.argmin([
                    abs(c["center"] - ref["center"])
                    for ref in assignment["clusters"]
                ]))
                lbl = (
                    labels[ref_idx]
                    if ref_idx < len(labels) else f"M{ref_idx + 1}")
            else:
                lbl = f"M{i + 1}"
            rec[f"M{i + 1} Mode"] = lbl
            rec[f"M{i + 1} N"] = c["n"]
            rec[f"M{i + 1} Center"] = round(c["center"], 4)
            rec[f"M{i + 1} Spread"] = round(c["spread"], 5)
        split_records.append(rec)

    # Normalise column sets across all rows (pandas would otherwise union
    # them in first-seen order, which is fine functionally but can leave
    # ragged trailing columns depending on which condition was processed
    # first -- fix the order explicitly so M1 < M2 < M3 always reads
    # left-to-right regardless of row order).
    all_slot_cols = []
    seen = set()
    for rec in split_records:
        for k in rec:
            if k.startswith("M") and k not in seen and k[1:2].isdigit():
                seen.add(k)
    for rec in split_records:
        for col in sorted(seen, key=lambda k: (int(k.split()[0][1:]), k)):
            rec.setdefault(col, "")
    return split_records


# =====================================================================
# Per-run KPI computation (pass 1 -- no Mode assigned here)
# =====================================================================
def compute_run_kpis(data, folder_name):
    """Returns (rows, numerics) where `rows` is the ordered list of
    (metric, value) pairs written to the Index sheet, and `numerics`
    carries the raw floats that pass 2 needs for classification."""
    rows = []
    numerics = {}

    # ---- Command layer ----
    cmd_t = data["cmd_times"]
    cmd_s = data["cmd_series"]
    rows.append(("Cmd Message Count", len(cmd_t)))

    cmd_ss = {w: float("nan") for w in WHEELS}
    if cmd_t:
        arrs = {w: np.array(cmd_s[w]) for w in WHEELS}
        rows.append(("Cmd Left FL-RL Max Abs Diff",
                     float(np.max(np.abs(arrs["FL"] - arrs["RL"])))))
        rows.append(("Cmd Right FR-RR Max Abs Diff",
                     float(np.max(np.abs(arrs["FR"] - arrs["RR"])))))
        for w in WHEELS:
            cmd_ss[w] = steady_state_mean(arrs[w])
            rows.append((f"Cmd Steady-State {w} (rad/s)", round(cmd_ss[w], 4)))
        # Pooled per-side values kept for continuity with v1-v8 columns.
        rows.append(("Cmd Steady-State Left (rad/s)", round(cmd_ss["FL"], 4)))
        rows.append(("Cmd Steady-State Right (rad/s)", round(cmd_ss["FR"], 4)))
    else:
        rows.append(("Note", "No wheel_cmd_debug messages found"))

    # ---- Commanded W recovery, in order of reliability ----
    # 1) /cmd_vel angular.z is the actual commanded body rate.
    # 2) wheel command -> body rate via no-slip inverse kinematics.
    # 3) folder-name w-tag (last resort; naming conventions have drifted).
    w_cmd = float("nan")
    w_cmd_source = "none"
    cmdvel_wz = [abs(v) for v in data.get("cmdvel_wz", []) if abs(v) > 1e-6]
    if cmdvel_wz:
        w_cmd = float(np.median(cmdvel_wz))
        w_cmd_source = "/cmd_vel angular.z (median of active samples)"
    elif cmd_ss["FL"] == cmd_ss["FL"] and abs(cmd_ss["FL"]) > 1e-9:
        w_cmd = wheel_omega_to_body_rate(cmd_ss["FL"])
        w_cmd_source = "derived from wheel command via no-slip kinematics"
    else:
        w_cmd = commanded_w_from_folder(folder_name)
        if w_cmd == w_cmd:
            w_cmd_source = "parsed from folder name (least reliable)"
    rows.append(("Commanded W (rad/s)", round(w_cmd, 4) if w_cmd == w_cmd else "N/A"))
    rows.append(("Commanded W Source", w_cmd_source))
    # v12 (2026-08-04): report the storage->header clock offset applied to
    # the headerless /cmd_vel stream, so a reader can tell whether the
    # command anchor and the wheel samples were on the same timebase.
    _clk = data.get("clock_offset_s", float("nan"))
    rows.append(("Clock Offset Applied to /cmd_vel (s)",
                 round(_clk, 3) if _clk == _clk else
                 "none (/cmd_vel already on the header clock, or no stamped reference)"))
    numerics["_w_cmd_num"] = w_cmd

    # ---- v11/v12: Max Drive Force experiment metadata ----
    max_force_nm = max_force_from_folder(folder_name)
    rows.append(("Max Drive Force (N·m/wheel)",
                 round(max_force_nm, 4) if max_force_nm == max_force_nm else "N/A"))
    rows.append(("Max Drive Force Source",
                 "parsed from folder-name vMaxDriveForce/vMDForce/vForce token"
                 if max_force_nm == max_force_nm else
                 "WARNING: unknown -- add vMaxDriveForce1p8, vMaxDriveForce0p25, "
                 "vMaxDriveForce0p125 or vMaxDriveForce0 to the folder name"))
    numerics["_max_force_nm"] = max_force_nm

    # ---- v12 (item 2): Joint Drive Damping experiment metadata ----
    damping_nm = damping_from_folder(folder_name)
    rows.append(("Joint Drive Damping",
                 round(damping_nm, 4) if damping_nm == damping_nm else "N/A"))
    rows.append(("Joint Drive Damping Source",
                 "parsed from folder-name Damping token"
                 if damping_nm == damping_nm else
                 "WARNING: unknown -- add Damping005/Damping0075/Damping01 "
                 "(or Damping0p05 etc.) to the folder name"))
    numerics["_damping_num"] = damping_nm

    # ---- v12 (item 3): Solver experiment metadata ----
    solver = solver_from_folder(folder_name)
    rows.append(("Solver", solver))
    rows.append(("Solver Source",
                 "parsed from folder-name PGS/TGS token"
                 if solver != "unknown" else
                 "WARNING: unknown -- add a _PGS_ or _TGS_ token to the folder "
                 "name; this run is excluded from solver-specific baselines"))
    numerics["_solver"] = solver

    # ---- Drive layer: actual per-wheel speed vs command ----
    joint_t = data["joint_times"]
    joint_s = data["joint_series"]
    rows.append(("Joint Message Count", len(joint_t)))

    # v12 (2026-08-04): cross-topic joint ORDER check. Both JointState
    # topics are read by name, so a differing order is harmless here, but
    # it is a standing trap for any future index-based reader and must be
    # visible rather than assumed away.
    name_order = data.get("joint_name_order", {})
    order_js = name_order.get(TOPIC_JOINT_STATES)
    order_cmd = name_order.get(TOPIC_WHEEL_CMD_DEBUG)
    order_mismatch = ""
    if order_js and order_cmd:
        rows.append(("Joint Order /joint_states", ", ".join(order_js)))
        rows.append(("Joint Order /cobraflex/wheel_cmd_debug", ", ".join(order_cmd)))
        if tuple(order_js) != tuple(order_cmd):
            order_mismatch = (
                f"/joint_states publishes [{', '.join(order_js)}] but "
                f"/cobraflex/wheel_cmd_debug publishes [{', '.join(order_cmd)}] "
                "-- both are mapped by name here, so this run's figures are "
                "unaffected; any index-based reader would transpose wheels.")
            rows.append(("\u26a0 Joint Order Mismatch Across Topics", order_mismatch))
    numerics["_joint_order_mismatch"] = order_mismatch

    act_ss = {w: float("nan") for w in WHEELS}
    wheel_avg_ss = float("nan")
    wheel_tail_mean = float("nan")
    wheel_peak = float("nan")
    breakaway_time = float("nan")
    joint_t0 = None

    if joint_t:
        first_joint_t = joint_t[0]
        anchor_time, anchor_source = find_command_start_time(
            cmd_t, cmd_s, data.get("cmdvel_times", []), data.get("cmdvel_wz", []),
            first_joint_t)
        # v12 (item 8): joint_t0 now holds the BREAKAWAY-TIMING ANCHOR
        # (command start when found), not necessarily the first
        # /joint_states sample -- every breakaway_anchored_*() call below
        # keeps using this name so the rest of the function is unchanged,
        # but its meaning has shifted; the two are reported separately.
        joint_t0 = anchor_time
        rows.append(("First /joint_states Sample (s, bag-relative epoch)",
                     round(first_joint_t, 3)))
        rows.append(("Command Start / Breakaway Anchor (s, bag-relative epoch)",
                     round(anchor_time, 3)))
        rows.append(("Command Start Anchor Source", anchor_source))

        bt, bv = wheel_avg_series(joint_t, joint_s)
        wheel_tail_mean = steady_state_mean(bv)
        wheel_peak = float(np.max(np.abs(bv))) if len(bv) else float("nan")
        rows.append(("Wheel Avg |omega| Tail-Window (rad/s)",
                     round(wheel_tail_mean, 6)
                     if wheel_tail_mean == wheel_tail_mean else "N/A"))
        rows.append(("Wheel Avg |omega| Peak (rad/s)",
                     round(wheel_peak, 6) if wheel_peak == wheel_peak else "N/A"))
        breakaway_time = find_breakaway_time(bt, bv, anchor_time)
        rows.append(("Breakaway Time (s)",
                     round(breakaway_time, 3) if breakaway_time == breakaway_time else "N/A"))
        rows.append(("Breakaway Time Definition",
                     f"relative to command start (see Command Start Anchor Source); "
                     f"requires sustained crossing of {BREAKAWAY_THRESHOLD_RAD_S} rad/s "
                     f"for >= {BREAKAWAY_SUSTAIN_S}s (item 8)"))

        # v12 (item 9): audit the primary steady-state window instead of
        # trusting the KPIs derived from it blindly.
        win_info = breakaway_anchored_window_info(bt, bv, joint_t0, breakaway_time)
        rows.append(("SS Window Start (s, bag-relative epoch)",
                     round(win_info["start_s"], 3) if win_info["start_s"] == win_info["start_s"] else "N/A"))
        rows.append(("SS Window End (s, bag-relative epoch)",
                     round(win_info["end_s"], 3) if win_info["end_s"] == win_info["end_s"] else "N/A"))
        rows.append(("SS Window Length (s)",
                     round(win_info["length_s"], 3) if win_info["length_s"] == win_info["length_s"] else "N/A"))
        rows.append(("SS Window N Samples", win_info["n_samples"]))
        rows.append(("SS Window Valid (>= 5 samples AND >= "
                     f"{STEADY_STATE_MIN_WINDOW_S}s)", win_info["valid"]))

        # --- v9: genuine per-wheel breakdown ---
        # v12 (items 11-12): per-wheel std values are also collected here
        # for the aggregate Wheel Std Mean/RMS/Max below, and each wheel
        # gets dedicated oscillation diagnostics -- peak-to-peak, the std
        # of the (actual - command) tracking error, and a threshold-
        # crossing count -- because the pooled 4-wheel-average std used in
        # earlier versions can partially cancel out-of-phase oscillation
        # across wheels (item 11).
        per_wheel_std = {}
        for w in WHEELS:
            act_ss[w] = breakaway_anchored_mean(
                joint_t, np.array(joint_s[w]), joint_t0, breakaway_time)
            std_w = breakaway_anchored_std(
                joint_t, np.abs(np.array(joint_s[w])), joint_t0, breakaway_time)
            per_wheel_std[w] = std_w
            rows.append((f"Actual Steady-State {w} (rad/s)",
                         round(act_ss[w], 4) if act_ss[w] == act_ss[w] else "N/A"))
            rows.append((f"Steady-State Std {w} (rad/s)",
                         round(std_w, 4) if std_w == std_w else "N/A"))
            if (cmd_ss[w] == cmd_ss[w] and abs(cmd_ss[w]) > 1e-9
                    and act_ss[w] == act_ss[w]):
                ratio = 100.0 * abs(act_ss[w]) / abs(cmd_ss[w])
                rows.append((f"Wheel Tracking Ratio {w} (%)", round(ratio, 1)))
            else:
                rows.append((f"Wheel Tracking Ratio {w} (%)", "N/A"))

            # Oscillation diagnostics within the same breakaway-anchored
            # steady-state window used for the mean/std above.
            win_w = breakaway_anchored_window(
                joint_t, np.abs(np.array(joint_s[w])), joint_t0, breakaway_time)
            if len(win_w) >= 2:
                p2p = float(np.max(win_w) - np.min(win_w))
                rows.append((f"Peak-to-Peak {w} (rad/s)", round(p2p, 4)))
                if cmd_ss[w] == cmd_ss[w]:
                    err = win_w - abs(cmd_ss[w])
                    rows.append((f"Tracking Error Std {w} (rad/s)", round(float(np.std(err)), 4)))
                    mean_w = float(np.mean(win_w))
                    crossings = int(np.sum(np.diff(np.sign(win_w - mean_w)) != 0))
                    rows.append((f"Mean-Crossing Count {w} (per window)", crossings))
                else:
                    rows.append((f"Tracking Error Std {w} (rad/s)", "N/A"))
                    rows.append((f"Mean-Crossing Count {w} (per window)", "N/A"))
            else:
                rows.append((f"Peak-to-Peak {w} (rad/s)", "N/A"))
                rows.append((f"Tracking Error Std {w} (rad/s)", "N/A"))
                rows.append((f"Mean-Crossing Count {w} (per window)", "N/A"))

        # v12 (item 11): aggregate the four per-wheel stds instead of only
        # reporting the std of the already-averaged 4-wheel signal (which
        # can hide out-of-phase oscillation via cancellation).
        valid_stds = [v for v in per_wheel_std.values() if v == v]
        if valid_stds:
            rows.append(("Wheel Std Mean of 4 (rad/s)", round(float(np.mean(valid_stds)), 4)))
            rows.append(("Wheel Std RMS of 4 (rad/s)",
                         round(float(np.sqrt(np.mean(np.square(valid_stds)))), 4)))
            rows.append(("Wheel Std Max of 4 (rad/s)", round(float(np.max(valid_stds)), 4)))
        else:
            rows.append(("Wheel Std Mean of 4 (rad/s)", "N/A"))
            rows.append(("Wheel Std RMS of 4 (rad/s)", "N/A"))
            rows.append(("Wheel Std Max of 4 (rad/s)", "N/A"))
        numerics["_wheel_std_mean4"] = float(np.mean(valid_stds)) if valid_stds else float("nan")
        numerics["_wheel_std_rms4"] = (
            float(np.sqrt(np.mean(np.square(valid_stds)))) if valid_stds else float("nan"))
        numerics["_wheel_std_max4"] = float(np.max(valid_stds)) if valid_stds else float("nan")

        # v12 (item 12): dominant oscillation frequency from the FL wheel's
        # steady-state window (representative; all four wheels share the
        # same commanded frequency content under skid-steer symmetry). A
        # simple FFT peak, excluding the DC bin, over the (roughly) evenly
        # spaced breakaway-anchored window.
        win_fl_t = None
        win_fl_v = breakaway_anchored_window(
            joint_t, np.abs(np.array(joint_s["FL"])), joint_t0, breakaway_time)
        if len(win_fl_v) >= 8:
            dt_est = win_info["length_s"] / max(len(win_fl_v) - 1, 1)
            if dt_est > 0:
                demeaned = win_fl_v - np.mean(win_fl_v)
                spectrum = np.abs(np.fft.rfft(demeaned))
                freqs = np.fft.rfftfreq(len(demeaned), d=dt_est)
                if len(spectrum) > 1:
                    peak_idx = 1 + int(np.argmax(spectrum[1:]))
                    rows.append(("Dominant Oscillation Frequency, FL (Hz)",
                                round(float(freqs[peak_idx]), 3)))
                else:
                    rows.append(("Dominant Oscillation Frequency, FL (Hz)", "N/A"))
            else:
                rows.append(("Dominant Oscillation Frequency, FL (Hz)", "N/A"))
        else:
            rows.append(("Dominant Oscillation Frequency, FL (Hz)", "N/A"))

        # Per-wheel spread within each side -- non-zero means the two
        # wheels on one side are NOT behaving as a rigid pair, which the
        # pooled per-side columns would hide entirely.
        for label, pair in (("Left", LEFT_WHEELS), ("Right", RIGHT_WHEELS)):
            a, b = (abs(act_ss[pair[0]]), abs(act_ss[pair[1]]))
            if a == a and b == b:
                rows.append((f"{label} Intra-Side Spread |{pair[0]}|-|{pair[1]}| (rad/s)",
                             round(abs(a - b), 4)))
            else:
                rows.append((f"{label} Intra-Side Spread |{pair[0]}|-|{pair[1]}| (rad/s)", "N/A"))

        # --- pooled per-side values, unchanged from v8 ---
        left_t = list(joint_t) + list(joint_t)
        act_left_ss = breakaway_anchored_mean(
            left_t, np.concatenate([np.array(joint_s["FL"]), np.array(joint_s["RL"])]),
            joint_t0, breakaway_time)
        right_t = list(joint_t) + list(joint_t)
        act_right_ss = breakaway_anchored_mean(
            right_t, np.concatenate([np.array(joint_s["FR"]), np.array(joint_s["RR"])]),
            joint_t0, breakaway_time)
        rows.append(("Actual Steady-State Left, Wheels (rad/s)",
                     round(act_left_ss, 4) if act_left_ss == act_left_ss else "N/A"))
        rows.append(("Actual Steady-State Right, Wheels (rad/s)",
                     round(act_right_ss, 4) if act_right_ss == act_right_ss else "N/A"))
        for label, act, cmd in (("Left", act_left_ss, cmd_ss["FL"]),
                                ("Right", act_right_ss, cmd_ss["FR"])):
            if cmd == cmd and abs(cmd) > 1e-9 and act == act:
                rows.append((f"Wheel Tracking Ratio, {label} (%)",
                             round(100.0 * abs(act) / abs(cmd), 1)))
            else:
                rows.append((f"Wheel Tracking Ratio, {label} (%)", "N/A"))

        wheel_ss_std = breakaway_anchored_std(bt, bv, joint_t0, breakaway_time)
        rows.append(("Steady-State Std, Wheel (rad/s)",
                     round(wheel_ss_std, 4) if wheel_ss_std == wheel_ss_std else "N/A"))
        numerics["_wheel_std_num"] = wheel_ss_std

        wheel_avg_ss = breakaway_anchored_mean(bt, bv, joint_t0, breakaway_time)
        rows.append(("Wheel Avg |omega| Steady-State (rad/s)",
                     round(wheel_avg_ss, 4) if wheel_avg_ss == wheel_avg_ss else "N/A"))

        if any(act_ss[w] != act_ss[w] for w in WHEELS) or wheel_ss_std != wheel_ss_std:
            rows.append(("Note",
                         (f"Insufficient post-breakaway data for one or more wheel "
                          f"steady-state KPIs -- breakaway at {breakaway_time:.2f}s leaves "
                          f"less than {STEADY_STATE_SETTLE_S}s of settled recording.")
                         if breakaway_time == breakaway_time else
                         (f"No breakaway detected -- wheels never exceeded "
                          f"{BREAKAWAY_THRESHOLD_RAD_S} rad/s in this recording.")))
    else:
        rows.append(("Note", "No /joint_states messages found -- "
                             "drive-layer tracking not verifiable for this run"))

    # ---- Body-level rotation ----
    ot_t = data["odomtruth_times"]
    ot_yaw = data["odomtruth_yaw"]
    ot_w = data["odomtruth_w"]
    rows.append(("Odom-Truth Message Count", len(ot_t)))

    w_ss_actual = float("nan")
    total_angle_rad = float("nan")
    body_tail_mean = float("nan")
    body_peak = float("nan")

    if ot_t and len(ot_yaw) > 1:
        total_angle_rad = cumulative_unwrapped_angle(ot_yaw)
        rows.append(("Total Angular Displacement, Actual (deg)",
                     round(math.degrees(total_angle_rad), 2)))

        ot_w_abs = [abs(w) for w in ot_w]
        body_tail_mean = steady_state_mean(ot_w_abs)
        body_peak = float(np.max(ot_w_abs)) if len(ot_w_abs) else float("nan")
        rows.append(("Body |W| Tail-Window (rad/s)",
                     round(body_tail_mean, 6)
                     if body_tail_mean == body_tail_mean else "N/A"))
        rows.append(("Body |W| Peak (rad/s)",
                     round(body_peak, 6) if body_peak == body_peak else "N/A"))
        if joint_t0 is not None:
            w_ss_actual = breakaway_anchored_mean(ot_t, ot_w, joint_t0, breakaway_time)
            body_ss_std = breakaway_anchored_std(ot_t, ot_w_abs, joint_t0, breakaway_time)
        else:
            w_ss_actual = steady_state_mean(ot_w)
            body_ss_std = steady_state_std(ot_w_abs)
            rows.append(("Note", "No /joint_states data -- body steady-state KPIs fell "
                                 "back to a fraction-based window, not breakaway-anchored."))
        rows.append(("W_ss Actual (rad/s)",
                     round(w_ss_actual, 4) if w_ss_actual == w_ss_actual else "N/A"))
        rows.append(("Steady-State Std, Body (rad/s)",
                     round(body_ss_std, 4) if body_ss_std == body_ss_std else "N/A"))

        # v12 (item 10): a third, independent W_ss estimate from total
        # angle / elapsed time over the post-breakaway portion of the
        # recording, cross-checked against the breakaway-anchored mean and
        # the whole-recording fraction-based tail mean. Large disagreement
        # among the three flags a run whose "steady-state" number should
        # not be trusted at face value (e.g. a short or noisy recording).
        w_ss_angle_time = float("nan")
        if (joint_t0 is not None and breakaway_time == breakaway_time
                and len(ot_t) > 1):
            settle_anchor = joint_t0 + breakaway_time + STEADY_STATE_SETTLE_S
            idx = [i for i, t in enumerate(ot_t) if t >= settle_anchor]
            if len(idx) >= 2:
                sub_yaw = [ot_yaw[i] for i in idx]
                sub_t = [ot_t[i] for i in idx]
                dt_span = sub_t[-1] - sub_t[0]
                if dt_span > 1e-6:
                    w_ss_angle_time = cumulative_unwrapped_angle(sub_yaw) / dt_span
        rows.append(("W_ss via Angle/Time, Post-Breakaway (rad/s)",
                     round(w_ss_angle_time, 4) if w_ss_angle_time == w_ss_angle_time else "N/A"))

        estimates = [v for v in (w_ss_actual, body_tail_mean, w_ss_angle_time)
                    if v == v]
        estimates_abs = [abs(v) for v in estimates]
        if len(estimates_abs) >= 2 and max(estimates_abs) > 1e-9:
            spread_rel = (max(estimates_abs) - min(estimates_abs)) / max(estimates_abs)
            consistent = spread_rel <= W_SS_CONSISTENCY_REL_TOL
            rows.append(("W_ss Consistency Check",
                         f"{'OK' if consistent else 'FLAG'} -- "
                         f"{len(estimates_abs)} estimate(s), max relative spread "
                         f"{spread_rel * 100:.1f}% "
                         f"(tolerance {W_SS_CONSISTENCY_REL_TOL * 100:.0f}%)"))
            numerics["_w_ss_consistency_flag"] = not consistent
        else:
            rows.append(("W_ss Consistency Check", "N/A -- fewer than 2 finite estimates"))
            numerics["_w_ss_consistency_flag"] = False
    else:
        rows.append(("Note", "No /odom_truth messages found -- "
                             "cannot compute actual rotation"))

    numerics["_w_ss_num"] = w_ss_actual
    numerics["_wheel_tail_num"] = wheel_tail_mean
    numerics["_wheel_peak_num"] = wheel_peak
    numerics["_body_tail_num"] = body_tail_mean
    numerics["_body_peak_num"] = body_peak

    # ---- v10: zero-force functionality and no-motion state ----
    command_active = w_cmd == w_cmd and abs(w_cmd) > 1e-6
    motion_telemetry_ok = wheel_peak == wheel_peak and body_peak == body_peak
    no_motion = (
        command_active
        and motion_telemetry_ok
        and wheel_peak <= NO_MOTION_WHEEL_PEAK_RAD_S
        and body_peak <= NO_MOTION_BODY_PEAK_RAD_S
    )
    if not command_active or not motion_telemetry_ok:
        motion_state = "Inconclusive"
    else:
        motion_state = "No Motion" if no_motion else "Motion"

    is_zero_force = max_force_nm == max_force_nm and abs(max_force_nm) <= ZERO_FORCE_TOL_NM
    if not is_zero_force:
        zero_force_check = (
            "Not a zero-force gate test" if max_force_nm == max_force_nm
            else "Inconclusive — Max Force not parsed"
        )
    elif not command_active:
        zero_force_check = "INCONCLUSIVE — no active rotation command"
    elif not motion_telemetry_ok:
        zero_force_check = "INCONCLUSIVE — missing wheel/body motion telemetry"
    elif no_motion:
        zero_force_check = "PASS — Max Force=0 blocked wheel/body motion"
    else:
        zero_force_check = "FAIL — motion detected with Max Force=0"

    rows.append(("Motion State", motion_state))
    rows.append(("Zero-Force Functional Check", zero_force_check))
    rows.append(("No-Motion Wheel Peak Threshold (rad/s)",
                 NO_MOTION_WHEEL_PEAK_RAD_S))
    rows.append(("No-Motion Body Peak Threshold (rad/s)",
                 NO_MOTION_BODY_PEAK_RAD_S))
    numerics["_motion_state"] = motion_state
    numerics["_zero_force_check"] = zero_force_check

    # ---- v9: true kinematic slip ratio ----
    # Steady-state form: compares the body rate actually achieved against
    # the body rate the MEASURED wheel speed would produce with no slip.
    if wheel_avg_ss == wheel_avg_ss and w_ss_actual == w_ss_actual and wheel_avg_ss > 1e-9:
        no_slip_rate = wheel_omega_to_body_rate(wheel_avg_ss)
        slip_ss = 100.0 * (1.0 - abs(w_ss_actual) / no_slip_rate)
        rows.append(("No-Slip Body Rate from Wheels (rad/s)", round(no_slip_rate, 4)))
        rows.append(("Slip Ratio, Steady-State (%)", round(slip_ss, 1)))
    else:
        rows.append(("No-Slip Body Rate from Wheels (rad/s)", "N/A"))
        rows.append(("Slip Ratio, Steady-State (%)", "N/A"))

    # Integrated form: cumulative wheel-surface travel vs cumulative body
    # rotation over the whole recording. Includes the pre-breakaway stick
    # phase and the transient, so it is systematically different from the
    # steady-state figure -- both are reported rather than picking one.
    #
    # v12 (2026-08-04): the integrated form now uses the SIGNED geometric
    # prediction
    #     W_no_slip(t) = r * (w_R_bar(t) - w_L_bar(t)) / D
    # instead of accumulating mean(|omega_i|). Under the expected in-place
    # rotation sign pattern (left and right counter-rotating) the two are
    # algebraically identical, which is why the steady-state figure is left
    # unchanged. They diverge whenever a wheel momentarily reverses: the
    # absolute form folds that excursion back onto the positive side and
    # inflates the implied no-slip rotation, whereas the signed form lets it
    # cancel as the geometry says it should. Both are emitted; the absolute
    # column is retained so earlier workbooks remain reconcilable.
    if joint_t and len(joint_t) > 1 and total_angle_rad == total_angle_rad:
        bt, bv = wheel_avg_series(joint_t, joint_s)
        t_arr = np.asarray(bt, dtype=float)        surface_travel = float(_trapezoid(np.asarray(bv, dtype=float), t_arr)) * WHEEL_RADIUS_M
        implied_rot_abs = surface_travel / HALF_TRACK_M
        rows.append(("Wheel Surface Travel, Integrated (m)", round(surface_travel, 4)))

        jt = np.asarray(joint_t, dtype=float)
        w_left = (np.asarray(joint_s["FL"], dtype=float)
                  + np.asarray(joint_s["RL"], dtype=float)) / 2.0
        w_right = (np.asarray(joint_s["FR"], dtype=float)
                   + np.asarray(joint_s["RR"], dtype=float)) / 2.0
        w_no_slip_signed = WHEEL_RADIUS_M * (w_right - w_left) / WHEEL_SEPARATION_M
        implied_rot_signed = float(_trapezoid(w_no_slip_signed, jt))

        rows.append(("Implied No-Slip Rotation, Signed (deg)",
                     round(math.degrees(implied_rot_signed), 2)))
        rows.append(("Implied No-Slip Rotation, Absolute LEGACY (deg)",
                     round(math.degrees(implied_rot_abs), 2)
                     if implied_rot_abs > 1e-9 else "N/A"))
        rows.append(("Implied No-Slip Rotation Sign-Form Delta (deg)",
                     round(math.degrees(abs(implied_rot_signed) - implied_rot_abs), 2)
                     if implied_rot_abs > 1e-9 else "N/A"))

        if abs(implied_rot_signed) > 1e-9:
            slip_int = 100.0 * (1.0 - abs(total_angle_rad) / abs(implied_rot_signed))
            rows.append(("Slip Ratio, Integrated (%)", round(slip_int, 1)))
        else:
            rows.append(("Slip Ratio, Integrated (%)", "N/A"))
        if implied_rot_abs > 1e-9:
            rows.append(("Slip Ratio, Integrated Absolute LEGACY (%)",
                         round(100.0 * (1.0 - abs(total_angle_rad) / implied_rot_abs), 1)))
        else:
            rows.append(("Slip Ratio, Integrated Absolute LEGACY (%)", "N/A"))
    else:
        rows.append(("Wheel Surface Travel, Integrated (m)", "N/A"))
        rows.append(("Implied No-Slip Rotation, Signed (deg)", "N/A"))
        rows.append(("Implied No-Slip Rotation, Absolute LEGACY (deg)", "N/A"))
        rows.append(("Implied No-Slip Rotation Sign-Form Delta (deg)", "N/A"))
        rows.append(("Slip Ratio, Integrated (%)", "N/A"))
        rows.append(("Slip Ratio, Integrated Absolute LEGACY (%)", "N/A"))

    # Command-referenced figure, reproducing the legacy generic-analyzer "Rotational Slip
    # Ratio (legacy)" definition. Emitted ONLY so v9 output can be
    # reconciled against older workbooks -- it is not a contact-patch
    # slip measurement and must not be cited as one (see docstring item 2).
    if w_cmd == w_cmd and abs(w_cmd) > 1e-9 and w_ss_actual == w_ss_actual:
        rows.append(("Slip Ratio, Command-Referenced LEGACY (%) -- do not cite as slip",
                     round(100.0 * (1.0 - abs(w_ss_actual) / abs(w_cmd)), 1)))
    else:
        rows.append(("Slip Ratio, Command-Referenced LEGACY (%) -- do not cite as slip", "N/A"))

    # ---- Contact force (scalar magnitude only -- see #136) ----
    contact_t = data.get("contact_times", {})
    contact_s = data.get("contact_series", {})
    contact_field = data.get("contact_field_used", {})

    total_contact_msgs = sum(len(v) for v in contact_t.values())
    rows.append(("Contact Force Message Count (all 4 wheels)", total_contact_msgs))
    fields_seen = sorted({f for f in contact_field.values() if f})
    rows.append(("Contact Force Field Used",
                 ", ".join(fields_seen) if fields_seen else "N/A (no messages)"))

    if total_contact_msgs > 0:
        # v12 (2026-08-04): the contact-force average now uses the SAME
        # absolute breakaway-anchored window as every wheel and body KPI.
        # It previously used a 0.5-0.9 fraction-of-recording window, so a
        # contact-force statistic and a wheel statistic from the same run
        # described different time spans and were not comparable. The old
        # fraction-window value is still emitted, clearly labelled, so
        # numbers already carried into the workbook can be reconciled.
        contact_ss = {}
        contact_legacy = {}
        for w in WHEELS:
            times_w = contact_t.get(w, [])
            series_w = contact_s.get(w, [])

            legacy = steady_state_mean(series_w)
            contact_legacy[w] = legacy

            ss = float("nan")
            info = None
            if times_w and joint_t0 is not None:
                info = breakaway_anchored_window_info(
                    times_w, series_w, joint_t0, breakaway_time)
                if info["valid"]:
                    ss = breakaway_anchored_mean(
                        times_w, series_w, joint_t0, breakaway_time)
            contact_ss[w] = ss

            rows.append((f"Contact Force Steady-State {w} (N)",
                         round(ss, 3) if ss == ss else "N/A"))
            rows.append((f"Contact Force Steady-State {w} LEGACY fraction-window (N)",
                         round(legacy, 3) if legacy == legacy else "N/A"))
            if info is not None:
                rows.append((f"Contact Force Window Samples {w}", info["n_samples"]))
                rows.append((f"Contact Force Publish Rate {w} (Hz)",
                             round(info["rate_hz"], 2)
                             if info["rate_hz"] == info["rate_hz"] else "N/A"))
                rows.append((f"Contact Force Window Completeness {w}",
                             round(info["completeness"], 3)
                             if info["completeness"] == info["completeness"] else "N/A"))
                if not info["valid"]:
                    rows.append((f"⚠ Contact Force Window {w}",
                                 "Rejected -- fewer than "
                                 f"{STEADY_STATE_MIN_SAMPLES} samples or shorter "
                                 f"than {STEADY_STATE_MIN_WINDOW_S}s inside the "
                                 "breakaway-anchored window"))
                elif not info["complete_enough"]:
                    rows.append((f"⚠ Contact Force Window {w}",
                                 "Sparse -- observed samples below "
                                 f"{WINDOW_MIN_RATE_COMPLETENESS:.0%} of the rate-"
                                 "implied count; value reported but treat as "
                                 "low-confidence"))
            else:
                rows.append((f"⚠ Contact Force Window {w}",
                             "No window -- missing timestamps or no breakaway"))

        # Aggregates (sum, left/right, front/rear, share) are only defined
        # when all four wheels produced a value. Summing whichever wheels
        # happened to survive would silently rescale every share and ratio,
        # so a partial set reports N/A instead.
        all_four = all(contact_ss[w] == contact_ss[w] for w in WHEELS)
        if all_four:
            total_force = float(sum(contact_ss[w] for w in WHEELS))
            l_sum = contact_ss["FL"] + contact_ss["RL"]
            r_sum = contact_ss["FR"] + contact_ss["RR"]
            f_sum = contact_ss["FL"] + contact_ss["FR"]
            rear_sum = contact_ss["RL"] + contact_ss["RR"]
            rows.append(("Contact Force SUM, Steady-State (N)", round(total_force, 3)))
            rows.append(("Contact Force Left Sum (FL+RL, N)", round(l_sum, 3)))
            rows.append(("Contact Force Right Sum (FR+RR, N)", round(r_sum, 3)))
            rows.append(("Contact Force Front Sum (FL+FR, N)", round(f_sum, 3)))
            rows.append(("Contact Force Rear Sum (RL+RR, N)", round(rear_sum, 3)))
            rows.append(("Contact Force R/L Ratio",
                         round(r_sum / l_sum, 4) if l_sum > 1e-9 else "N/A"))
            rows.append(("Contact Force F/R Ratio",
                         round(f_sum / rear_sum, 4) if rear_sum > 1e-9 else "N/A"))
            if total_force > 1e-9:
                for w in WHEELS:
                    rows.append((f"Contact Force Share {w} (%)",
                                 round(100.0 * contact_ss[w] / total_force, 1)))

            # Same ratio on the old fraction window, so the effect of the
            # window change on an already-reported figure is visible in the
            # workbook rather than having to be reconstructed.
            if all(contact_legacy[w] == contact_legacy[w] for w in WHEELS):
                l_leg = contact_legacy["FL"] + contact_legacy["RL"]
                r_leg = contact_legacy["FR"] + contact_legacy["RR"]
                if l_leg > 1e-9:
                    rows.append(("Contact Force R/L Ratio LEGACY fraction-window",
                                 round(r_leg / l_leg, 4)))
                    if l_sum > 1e-9:
                        rows.append(("Contact Force R/L Ratio Window Delta",
                                     round(abs(r_sum / l_sum - r_leg / l_leg), 4)))
        else:
            missing = [w for w in WHEELS if contact_ss[w] != contact_ss[w]]
            rows.append(("Contact Force SUM, Steady-State (N)", "N/A"))
            rows.append(("Contact Force R/L Ratio", "N/A"))
            rows.append(("⚠ Contact Force Aggregates",
                         "N/A -- aggregates require all four wheels; missing/"
                         f"rejected: {', '.join(missing)}"))
    else:
        rows.append(("Note", "No contact-force messages found for this run"))

    return rows, numerics


def make_flag(row_dict):
    """Flag runs whose four-wheel command is not left/right symmetric --
    the one condition under which the command layer itself would be
    suspect rather than the physics."""
    for key in ("Cmd Left FL-RL Max Abs Diff", "Cmd Right FR-RR Max Abs Diff"):
        val = row_dict.get(key)
        if isinstance(val, (int, float)) and val > SYMMETRY_FLAG_TOL:
            return True
    return False


# =====================================================================
# v12 (item 16): per-run data quality report
# =====================================================================
QUALITY_TOPICS = {
    "wheel_cmd_debug": TOPIC_WHEEL_CMD_DEBUG,
    "joint_states": TOPIC_JOINT_STATES,
    "odom_truth": TOPIC_ODOM_TRUTH,
    "cmd_vel": TOPIC_CMD_VEL,
    "clock": TOPIC_CLOCK,
}


def data_quality_report(data, folder_name):
    """One row per (run, topic): existence, monotonicity, duplicate count,
    max gap, and an estimated publish rate, using EVERY raw message seen on
    the topic (topic_raw_stamps), independent of whether the payload parsed
    into a usable KPI. Also reports command-vs-recorded time completeness:
    how much of the active-command span is actually covered by
    /joint_states and /odom_truth recording."""
    raw = data.get("topic_raw_stamps", {})
    time_sources = data.get("time_source_counts", {})
    rows = []
    for label, topic in QUALITY_TOPICS.items():
        stamps = raw.get(topic, [])
        row = {"Run": folder_name, "Topic": topic, "Present": bool(stamps),
              "N Messages": len(stamps)}
        if len(stamps) >= 2:
            arr = np.asarray(stamps, dtype=float)
            diffs = np.diff(arr)
            row["Monotonic"] = bool(np.all(diffs >= 0))
            row["Duplicate Timestamps"] = int(np.sum(diffs == 0))
            row["Max Gap (s)"] = round(float(np.max(diffs)), 4)
            span = arr[-1] - arr[0]
            row["Publish Rate, Mean (Hz)"] = (
                round((len(arr) - 1) / span, 2) if span > 1e-9 else "N/A")
        else:
            row["Monotonic"] = "N/A"
            row["Duplicate Timestamps"] = "N/A"
            row["Max Gap (s)"] = "N/A"
            row["Publish Rate, Mean (Hz)"] = "N/A"
        src_counts = time_sources.get(topic, {})
        total_src = sum(src_counts.values())
        row["Time Source"] = (
            ", ".join(f"{k}={v}" for k, v in src_counts.items() if v)
            if total_src else "N/A (topic absent or unparsed)")
        rows.append(row)

    # Command vs recorded-time completeness: how much of the interval from
    # the first active command sample to the last one is actually covered
    # by /joint_states and /odom_truth recordings.
    cmdvel_t = data.get("cmdvel_times", [])
    cmdvel_wz = data.get("cmdvel_wz", [])
    active_t = [t for t, wz in zip(cmdvel_t, cmdvel_wz)
                if abs(wz) > COMMAND_ACTIVE_THRESHOLD_RAD_S]
    if active_t:
        cmd_span = (active_t[-1] - active_t[0], active_t[0], active_t[-1])
    else:
        cmd_span = (float("nan"), float("nan"), float("nan"))
    completeness_row = {"Run": folder_name, "Topic": "(command completeness)",
                        "Present": "N/A", "N Messages": "N/A",
                        "Monotonic": "N/A", "Duplicate Timestamps": "N/A",
                        "Max Gap (s)": "N/A", "Publish Rate, Mean (Hz)": "N/A",
                        "Time Source": "N/A"}
    if cmd_span[0] == cmd_span[0] and cmd_span[0] > 1e-6:
        joint_t = data.get("joint_times", [])
        odom_t = data.get("odomtruth_times", [])
        for label, series in (("Joint States", joint_t), ("Odom Truth", odom_t)):
            covered = [t for t in series if cmd_span[1] <= t <= cmd_span[2]]
            ratio = (
                (covered[-1] - covered[0]) / cmd_span[0]
                if len(covered) >= 2 else 0.0)
            completeness_row[f"{label} Coverage of Active-Command Span"] = (
                f"{ratio * 100:.1f}%")
    rows.append(completeness_row)
    return rows


# =====================================================================
# Group-level time series (built per commanded-W condition)
# =====================================================================
def build_group_time_series(run_records):
    """Low/High group-mean curves and their divergence, for one condition
    group. Runs whose Mode is neither Low nor High are skipped."""
    t_grid = np.arange(0.0, TIME_SERIES_MAX_T + TIME_SERIES_DT, TIME_SERIES_DT)
    wheel_by_mode = {"Low": [], "High": []}
    body_by_mode = {"Low": [], "High": []}

    for rec in run_records:
        if rec.get("mode") not in ("Low", "High"):
            continue
        wt, wv = wheel_avg_series(rec["joint_times"], rec["joint_series"])
        wheel_by_mode[rec["mode"]].append(resample_series(wt, wv, t_grid))
        bv = [abs(w) for w in rec["odomtruth_w"]]
        body_by_mode[rec["mode"]].append(
            resample_series(rec["odomtruth_times"], bv, t_grid))

    def group_mean(arr_list):
        if not arr_list:
            return np.full_like(t_grid, np.nan, dtype=float)
        with warnings.catch_warnings():
            # All-NaN columns past a run's own recording end are expected
            # (see resample_series), not an error.
            warnings.filterwarnings("ignore", message="Mean of empty slice")
            return np.nanmean(np.vstack(arr_list), axis=0)

    low_wheel = group_mean(wheel_by_mode["Low"])
    high_wheel = group_mean(wheel_by_mode["High"])
    low_body = group_mean(body_by_mode["Low"])
    high_body = group_mean(body_by_mode["High"])

    wheel_div = np.abs(low_wheel - high_wheel)
    body_div = np.abs(low_body - high_body)
    w_on, w_fin = find_divergence_onset(t_grid, wheel_div)
    b_on, b_fin = find_divergence_onset(t_grid, body_div)

    return {
        "t_grid": t_grid,
        "low_wheel": low_wheel, "high_wheel": high_wheel,
        "low_body": low_body, "high_body": high_body,
        "wheel_divergence": wheel_div, "body_divergence": body_div,
        "wheel_onset": w_on, "wheel_final": w_fin,
        "body_onset": b_on, "body_final": b_fin,
        "n_low": len(wheel_by_mode["Low"]), "n_high": len(wheel_by_mode["High"]),
    }


def w_tag(w_value):
    """Short, filename/sheet-safe label for a commanded-W group."""
    if w_value == "unknown" or w_value != w_value:
        return "unknown"
    return f"{w_value:.2f}".replace(".", "p")


def force_tag(force_value):
    """Short, sheet-safe Max Force label."""
    if force_value == "unknown" or force_value != force_value:
        return "unknown"
    return f"{force_value:.4g}".replace(".", "p").replace("-", "m")


def damping_tag(damping_value):
    """Short, sheet-safe Joint Drive Damping label."""
    if damping_value == "unknown" or damping_value != damping_value:
        return "unk"
    return f"{damping_value:.4g}".replace(".", "p")


def solver_tag(solver_value):
    """Short, sheet-safe Solver label."""
    return solver_value if solver_value else "unknown"


def condition_tag(key):
    """Compact sheet tag for one (commanded W, Max Force, Damping, Solver)
    condition. Accepts either the full 4-tuple or the legacy (W, Force)
    pair for backward compatibility with any external callers."""
    if len(key) == 4:
        w_value, force_value, damping_value, solver_value = key
        return (f"w{w_tag(w_value)}_F{force_tag(force_value)}_"
                f"D{damping_tag(damping_value)}_{solver_tag(solver_value)}")
    return f"w{w_tag(key[0])}_F{force_tag(key[1])}"


# =====================================================================
# Main
# =====================================================================
def run_pipeline(root, out_path, use_legacy=False, baseline_centers=None,
                 bag_dirs=None, quiet=False):
    """Run the whole Test04 analysis and write the workbook.

    Extracted from main() so that cobraflex_analyzer.py can drive this
    module as a library without re-implementing any KPI. `bag_dirs` lets
    the caller supply an already-discovered and already-filtered list of
    (folder_name, folder_path) pairs -- the handover entry point does its own
    recursive discovery and test-id routing -- while passing None keeps the
    original standalone behaviour of scanning `root` directly.

    Returns a dict with the run counts, so a caller can set an exit code
    without parsing stdout.
    """
    if baseline_centers is None:
        baseline_centers = dict(BASELINE_CENTERS_DEFAULT)

    def _say(*a):
        if not quiet:
            print(*a)

    if not HAVE_GENERIC_ANALYZER:
        _say("NOTE: cobraflex_rosbag_analyzer could not be imported "
             f"({GENERIC_ANALYZER_IMPORT_ERROR}).")
        _say("      Running in Test04-only mode; standard per-test KPI sheets "
             "will not be produced.\n")

    if bag_dirs is None:
        # Standalone fallback only; the CLI normally supplies an
        # already-routed list. recursive=False reproduces the scan the
        # Test04 tool performed on its own.
        bag_dirs = find_bag_dirs(root, recursive=False)
    if not bag_dirs:
        _say(f"No bag folders (containing metadata.yaml) found under {root}")
        return {"processed": 0, "skipped": 0, "failed": 0, "output": None}

    _say(f"Found {len(bag_dirs)} bag folder(s) under {root}\n")

    index_rows = []      # Test04 deep-analysis rows
    run_records = []     # raw series kept for the time-series sheets
    per_test_rows = {tid: [] for tid in TEST_NAMES}   # generic standard KPIs
    generic_index_rows = []
    data_quality_rows = []   # v12 (item 16)
    wheel_cmd_vs_actual_rows = []  # v12 (item 19)
    skipped, failed = [], []

    # ---------------- Pass 1: per-run KPIs, no Mode yet ----------------
    for folder_name, folder_path in bag_dirs:
        test_id = identify_test_id(folder_name)
        if test_id == 0:
            skipped.append(folder_name)
            _say(f"  SKIP (unrecognized test_id): {folder_name}")
            continue

        # generic standard KPI set, when available (the batch_analyze_v3 half).
        if HAVE_GENERIC_ANALYZER:
            try:
                generic_rows, _bag = compute_test_kpis(folder_path, test_id)
                rd = {"Run": folder_name,
                      "Condition": parse_condition(folder_name, test_id)}
                stall_flag = ""
                for metric, value, _param in generic_rows:
                    rd[metric] = value
                    if metric.startswith("\u26a0 Publish-Rate Stall"):
                        stall_flag = ((stall_flag + " | " if stall_flag else "")
                                      + metric.split("(")[-1].rstrip(")"))
                per_test_rows.setdefault(test_id, []).append(rd)
                generic_index_rows.append({
                    "Run": folder_name, "Test ID": test_id,
                    "Test Name": TEST_NAMES.get(test_id, f"Test{test_id}"),
                    "Condition": parse_condition(folder_name, test_id),
                    "Stall Warning": stall_flag,
                })
            except Exception as e:  # noqa: BLE001
                failed.append((folder_name, f"generic KPIs: {e}"))
                _say(f"  WARN: generic KPIs failed for {folder_name} ({e})")

        if test_id != 4:
            continue  # deep bimodal analysis applies to in-place rotation only

        try:
            data = load_bag(folder_path)
            kpi_rows, numerics = compute_run_kpis(data, folder_name)
        except Exception as e:  # noqa: BLE001
            failed.append((folder_name, str(e)))
            _say(f"  FAIL: {folder_name}  ({e})")
            continue

        data_quality_rows.extend(data_quality_report(data, folder_name))

        # v12 (item 19, extended): one consolidated sheet of time + all
        # four wheel joints' commanded/actual velocity AND the body-level
        # commanded/actual angular velocity at the same timestamps.
        # Wheel-actual is native at /joint_states sample times; wheel-cmd
        # is held at its last /cmd sample (step-function hold, correct for
        # a piecewise-constant command sampled onto a faster-ticking
        # actual-state topic). Body-target is /cmd_vel held the same way
        # (falling back to the wheel-command-derived body rate when
        # /cmd_vel is absent); body-actual is /odom_truth linearly
        # interpolated onto the same timestamps, NaN outside its own
        # recorded span (never flat-extrapolated).
        jt = data["joint_times"]
        if jt:
            cmd_t_arr = np.asarray(data["cmd_times"], dtype=float)
            cmdvel_t_arr = np.asarray(data.get("cmdvel_times", []), dtype=float)
            cmdvel_wz_arr = np.asarray(data.get("cmdvel_wz", []), dtype=float)
            odom_t_arr = np.asarray(data.get("odomtruth_times", []), dtype=float)
            odom_w_arr = np.asarray(data.get("odomtruth_w", []), dtype=float)
            jt_arr = np.asarray(jt, dtype=float)

            if len(odom_t_arr) >= 2:
                body_actual_interp = np.interp(jt_arr, odom_t_arr, odom_w_arr)
                body_actual_interp = np.where(
                    (jt_arr >= odom_t_arr[0]) & (jt_arr <= odom_t_arr[-1]),
                    body_actual_interp, np.nan)
            else:
                body_actual_interp = np.full_like(jt_arr, np.nan)

            for i, t in enumerate(jt):
                row_wv = {"Run": folder_name, "Time (s, bag-relative epoch)": round(t, 4)}
                if len(cmd_t_arr):
                    hold_idx = int(np.searchsorted(cmd_t_arr, t, side="right")) - 1
                    hold_idx = max(hold_idx, 0)
                    for w in WHEELS:
                        cs = data["cmd_series"][w]
                        row_wv[f"Cmd {w} (rad/s)"] = (
                            round(cs[hold_idx], 4) if hold_idx < len(cs) else "N/A")
                else:
                    hold_idx = None
                    for w in WHEELS:
                        row_wv[f"Cmd {w} (rad/s)"] = "N/A"
                for w in WHEELS:
                    row_wv[f"Actual {w} (rad/s)"] = round(data["joint_series"][w][i], 4)

                # Body-level target: prefer /cmd_vel (the direct commanded
                # body rate), held the same step-function way as the wheel
                # commands above; fall back to the wheel-command-derived
                # body rate (no-slip inverse kinematics) only when /cmd_vel
                # was not recorded at all.
                if len(cmdvel_t_arr):
                    cv_idx = int(np.searchsorted(cmdvel_t_arr, t, side="right")) - 1
                    cv_idx = max(cv_idx, 0)
                    row_wv["Cmd Body W (rad/s)"] = (
                        round(float(cmdvel_wz_arr[cv_idx]), 4)
                        if cv_idx < len(cmdvel_wz_arr) else "N/A")
                    row_wv["Cmd Body W Source"] = "/cmd_vel (held)"
                elif hold_idx is not None and hold_idx < len(data["cmd_series"]["FL"]):
                    row_wv["Cmd Body W (rad/s)"] = round(
                        wheel_omega_to_body_rate(data["cmd_series"]["FL"][hold_idx]), 4)
                    row_wv["Cmd Body W Source"] = "derived from wheel command (no /cmd_vel)"
                else:
                    row_wv["Cmd Body W (rad/s)"] = "N/A"
                    row_wv["Cmd Body W Source"] = "N/A"

                bav = body_actual_interp[i]
                row_wv["Actual Body W (rad/s)"] = round(float(bav), 4) if bav == bav else "N/A"

                wheel_cmd_vs_actual_rows.append(row_wv)

        row_dict = {"Run": folder_name,
                    "Condition": parse_condition(folder_name, test_id)}
        for metric, value in kpi_rows:
            row_dict[metric] = value
        row_dict.update(numerics)
        row_dict["Flag"] = make_flag(row_dict)
        index_rows.append(row_dict)

        bval = row_dict.get("Breakaway Time (s)")
        run_records.append({
            "folder_name": folder_name,
            "w_cmd": numerics.get("_w_cmd_num", float("nan")),
            "max_force_nm": numerics.get("_max_force_nm", float("nan")),
            "damping": numerics.get("_damping_num", float("nan")),
            "solver": numerics.get("_solver", "unknown"),
            "breakaway_time": float(bval) if isinstance(bval, (int, float)) else float("nan"),
            "joint_times": data["joint_times"],
            "joint_series": data["joint_series"],
            "odomtruth_times": data["odomtruth_times"],
            "odomtruth_w": data["odomtruth_w"],
            "row": row_dict,
        })
        wc = numerics.get("_w_cmd_num", float("nan"))
        force = numerics.get("_max_force_nm", float("nan"))
        damping_v = numerics.get("_damping_num", float("nan"))
        solver_v = numerics.get("_solver", "unknown")
        wc_text = f"{wc:.3f}" if wc == wc else "unknown"
        force_text = f"{force:.4g}" if force == force else "unknown"
        damping_text = f"{damping_v:.4g}" if damping_v == damping_v else "unknown"
        warn_bits = []
        if force != force:
            warn_bits.append("Max Force")
        if damping_v != damping_v:
            warn_bits.append("Damping")
        if solver_v == "unknown":
            warn_bits.append("Solver")
        warn_text = f"  [WARNING: unparsed {', '.join(warn_bits)}]" if warn_bits else ""
        _say(f"  OK: {folder_name}  (cmd W={wc_text} rad/s, "
              f"Max Force={force_text} N·m/wheel, Damping={damping_text}, "
              f"Solver={solver_v}){warn_text}")

    _say(f"\nProcessed {len(index_rows)} Test04 bag(s), "
          f"skipped {len(skipped)}, failed {len(failed)}")

    # ---------------- Pass 2: per-condition mode assignment ----------------
    split_records = assign_modes(index_rows, use_legacy=use_legacy,
                                  baseline_centers=baseline_centers)
    for rec in run_records:
        rec["mode"] = rec["row"].get("Mode", "N/A")

    _say("\nSplit detection by (commanded W, Max Force, Damping, Solver):")
    for srec in split_records:
        _say(f"  W={srec['Commanded W (rad/s)']}, "
              f"Max Force={srec.get('Max Drive Force (N·m/wheel)', 'unknown')}, "
              f"Damping={srec.get('Joint Drive Damping', 'unknown')}, "
              f"Solver={srec.get('Solver', 'unknown')}: "
              f"N={srec['N Runs']} "
              f"-> {srec['Split Detected']}  ({srec['Verdict Reason']})")

    # ---------------- Summaries ----------------
    idx_df = pd.DataFrame(index_rows)
    summary_cols = [
        "W_ss Actual (rad/s)", "Total Angular Displacement, Actual (deg)",
        "Breakaway Time (s)", "Steady-State Std, Wheel (rad/s)",
        "Steady-State Std, Body (rad/s)",
        "Wheel Avg |omega| Steady-State (rad/s)",
        "Slip Ratio, Steady-State (%)", "Slip Ratio, Integrated (%)",
        "SS Window Length (s)", "W_ss via Angle/Time, Post-Breakaway (rad/s)",
        "Wheel Std Mean of 4 (rad/s)", "Wheel Std RMS of 4 (rad/s)",
        "Wheel Std Max of 4 (rad/s)", "Dominant Oscillation Frequency, FL (Hz)",
    ] + [f"Wheel Tracking Ratio {w} (%)" for w in WHEELS] \
      + [f"Steady-State Std {w} (rad/s)" for w in WHEELS] \
      + [f"Peak-to-Peak {w} (rad/s)" for w in WHEELS] \
      + [f"Tracking Error Std {w} (rad/s)" for w in WHEELS]

    mode_summary_rows, slip_summary_rows, contact_summary_rows = [], [], []
    max_force_summary_rows = []
    if not idx_df.empty:
        condition_keys = sorted(
            {experiment_group_key(row) for row in index_rows},
            key=group_sort_key,
        )
        for condition_key in condition_keys:
            wkey, force_key, damping_key, solver_key = condition_key
            group_rows = [
                row for row in index_rows
                if experiment_group_key(row) == condition_key
            ]
            sub_all = pd.DataFrame(group_rows)

            force_summary = {
                "Commanded W (rad/s)": wkey,
                "Max Drive Force (N·m/wheel)": force_key,
                "Joint Drive Damping": damping_key,
                "Solver": solver_key,
                "N Runs": len(sub_all),
                "Motion Runs": int((sub_all.get("Motion State") == "Motion").sum()),
                "No-Motion Runs": int((sub_all.get("Motion State") == "No Motion").sum()),
                "Inconclusive Motion Runs": int(
                    (sub_all.get("Motion State") == "Inconclusive").sum()),
                "Low Runs": int((sub_all.get("Mode") == "Low").sum()),
                "High Runs": int((sub_all.get("Mode") == "High").sum()),
                "Single-Branch Runs": int((sub_all.get("Mode") == "Single").sum()),
                "Invalid Runs": int((sub_all.get("Mode") == "Invalid").sum()),
            }
            low_high_n = force_summary["Low Runs"] + force_summary["High Runs"]
            force_summary["Valid Low/High Runs"] = low_high_n
            force_summary["High Proportion among Low/High"] = (
                force_summary["High Runs"] / low_high_n if low_high_n else "N/A")
            zero_checks = sub_all.get("Zero-Force Functional Check")
            if zero_checks is not None:
                force_summary["Zero-Force PASS Runs"] = int(
                    zero_checks.astype(str).str.startswith("PASS").sum())
                force_summary["Zero-Force FAIL Runs"] = int(
                    zero_checks.astype(str).str.startswith("FAIL").sum())
                force_summary["Zero-Force Inconclusive Runs"] = int(
                    zero_checks.astype(str).str.startswith("INCONCLUSIVE").sum())

            for col, label in (
                ("Wheel Avg |omega| Tail-Window (rad/s)", "Wheel Tail Mean (rad/s)"),
                ("Wheel Avg |omega| Peak (rad/s)", "Wheel Peak Mean (rad/s)"),
                ("Body |W| Tail-Window (rad/s)", "Body Tail Mean (rad/s)"),
                ("Body |W| Peak (rad/s)", "Body Peak Mean (rad/s)"),
                ("W_ss Actual (rad/s)", "Body W_ss Mean (rad/s)"),
            ):
                vals = numeric_column(sub_all, col)
                if len(vals):
                    force_summary[label] = round(float(vals.mean()), 6)

            if force_key != "unknown" and abs(force_key) <= ZERO_FORCE_TOL_NM:
                if force_summary.get("Zero-Force FAIL Runs", 0):
                    interpretation = (
                        "FAIL: motion occurred with Max Force=0; edited drive may not "
                        "control the runtime wheel drives.")
                elif (force_summary.get("Zero-Force PASS Runs", 0) == len(sub_all)):
                    interpretation = (
                        "PASS: Max Force=0 blocked motion, so the drive limit is active.")
                else:
                    interpretation = (
                        "INCONCLUSIVE: zero-force runs lack an active command or "
                        "complete wheel/body telemetry.")
            elif force_summary["No-Motion Runs"] == len(sub_all):
                interpretation = (
                    "All runs were stationary; this force is insufficient or the "
                    "drive/command path needs checking.")
            elif not low_high_n and force_summary["Invalid Runs"]:
                interpretation = (
                    "No valid branch-classified run; motion, if present, did not "
                    "reach breakaway or a valid steady-state W.")
            elif force_summary["Low Runs"] and force_summary["High Runs"]:
                interpretation = (
                    "Both Low and High branches remain within this Max Force condition.")
            elif force_summary["High Runs"] and not force_summary["Low Runs"]:
                interpretation = (
                    f"All {force_summary['High Runs']} valid branch-classified runs "
                    "selected High; branch switching was not observed in this batch.")
            elif force_summary["Low Runs"] and not force_summary["High Runs"]:
                interpretation = (
                    f"All {force_summary['Low Runs']} valid branch-classified runs "
                    "selected Low; branch switching was not observed in this batch.")
            elif force_summary["Single-Branch Runs"]:
                interpretation = (
                    "One response branch observed in this Max Force condition.")
            elif low_high_n:
                interpretation = (
                    "Only one classified Low/High branch observed in this condition.")
            else:
                interpretation = (
                    "No Low/High conclusion; inspect N, telemetry and split threshold.")
            force_summary["Interpretation"] = interpretation
            max_force_summary_rows.append(force_summary)

            # v12 (2026-08-04): "Unclassified" is included so that a
            # condition too small for a branch verdict still appears in
            # ModeSummary/SlipSummary/ContactForceSummary. Omitting it made
            # those sheets disappear entirely for small-N batches, which
            # loses the measurements as well as the (correctly withheld)
            # verdict. Out-of-Baseline variants are matched by prefix.
            observed_modes = [m for m in sub_all.get("Mode", pd.Series(dtype=object)).unique()
                              if isinstance(m, str)]
            ordered = ["Low", "Mid2", "Mid3", "High", "Single", "Unclassified",
                       "No Motion", "Invalid", "Zero-Force FAIL"]
            for mode_value in (ordered
                               + sorted(m for m in observed_modes if m not in ordered)):
                subset = sub_all[sub_all.get("Mode") == mode_value]
                if len(subset) == 0:
                    continue
                summary = {"Commanded W (rad/s)": wkey,
                           "Max Drive Force (N·m/wheel)": force_key,
                           "Joint Drive Damping": damping_key,
                           "Solver": solver_key,
                           "Mode": mode_value, "N Runs": len(subset)}
                for col in summary_cols:
                    vals = numeric_column(subset, col)
                    if len(vals):
                        summary[f"{col} - Mean"] = float(vals.mean())
                        summary[f"{col} - Std"] = float(vals.std())
                mode_summary_rows.append(summary)

                slip_row = {"Commanded W (rad/s)": wkey,
                            "Max Drive Force (N·m/wheel)": force_key,
                            "Joint Drive Damping": damping_key,
                            "Solver": solver_key,
                            "Mode": mode_value, "N Runs": len(subset)}
                for col, label in (
                    ("Slip Ratio, Steady-State (%)", "Slip SS (%)"),
                    ("Slip Ratio, Integrated (%)", "Slip Integrated (%)"),
                    ("Slip Ratio, Command-Referenced LEGACY (%) -- do not cite as slip",
                     "Slip LEGACY (%)"),
                    ("Wheel Avg |omega| Steady-State (rad/s)", "Wheel Avg (rad/s)"),
                    ("W_ss Actual (rad/s)", "Body W_ss (rad/s)"),
                ):
                    vals = numeric_column(subset, col)
                    if len(vals):
                        slip_row[label] = round(float(vals.mean()), 3)
                if ("Slip SS (%)" in slip_row) and ("Slip LEGACY (%)" in slip_row):
                    slip_row["LEGACY minus TRUE (pp)"] = round(
                        slip_row["Slip LEGACY (%)"] - slip_row["Slip SS (%)"], 1)
                slip_summary_rows.append(slip_row)

                crow = {"Commanded W (rad/s)": wkey,
                        "Max Drive Force (N·m/wheel)": force_key,
                        "Joint Drive Damping": damping_key,
                        "Solver": solver_key,
                        "Mode": mode_value, "N Runs": len(subset)}
                for w in WHEELS:
                    vals = numeric_column(
                        subset, f"Contact Force Steady-State {w} (N)")
                    if len(vals):
                        crow[f"{w} (N)"] = round(float(vals.mean()), 3)
                rl = numeric_column(subset, "Contact Force R/L Ratio")
                if len(rl):
                    crow["Sim R/L Ratio"] = round(float(rl.mean()), 4)
                if not CORNER_LOAD_COMPARISON_ENABLED:
                    # Disabled by default: emitting a "Measured Static Share"
                    # column when no corner weights were supplied would put a
                    # fabricated reference next to a real measurement.
                    contact_summary_rows.append(crow)
                    continue
                static_total = sum(STATIC_CORNER_LOAD_G.values())
                crow["Static Corner-Load Source"] = STATIC_CORNER_LOAD_SOURCE
                crow["Measured Static R/L Ratio"] = round(
                    (STATIC_CORNER_LOAD_G["FR"] + STATIC_CORNER_LOAD_G["RR"]) /
                    (STATIC_CORNER_LOAD_G["FL"] + STATIC_CORNER_LOAD_G["RL"]), 4)
                for w in WHEELS:
                    crow[f"Measured Static Share {w} (%)"] = round(
                        100.0 * STATIC_CORNER_LOAD_G[w] / static_total, 1)
                contact_summary_rows.append(crow)

    # ---------------- Per-condition time series ----------------
    groups = {}
    for rec in run_records:
        key = (
            numeric_group_value(rec["w_cmd"], W_GROUP_DECIMALS),
            numeric_group_value(rec["max_force_nm"], MAX_FORCE_GROUP_DECIMALS),
            numeric_group_value(rec.get("damping", float("nan")), DAMPING_GROUP_DECIMALS),
            rec.get("solver", "unknown"),
        )
        groups.setdefault(key, []).append(rec)

    ts_sheets, onset_rows, aligned_sheets = {}, [], {}
    align_grid = np.arange(-BREAKAWAY_ALIGN_PRE_S,
                           BREAKAWAY_ALIGN_POST_S + BREAKAWAY_ALIGN_DT,
                           BREAKAWAY_ALIGN_DT)

    for key, recs in sorted(groups.items(), key=lambda kv: group_sort_key(kv[0])):
        w_key, force_key, damping_key, solver_key = key
        tag = condition_tag(key)

        # Breakaway-aligned per-run overlays: written for EVERY condition,
        # split or not -- the per-run trajectory shape is informative even
        # when all runs land in one mode.
        wheel_cols = {"Time Relative to Breakaway (s)": align_grid}
        body_cols = {"Time Relative to Breakaway (s)": align_grid}
        for rec in recs:
            if rec["breakaway_time"] != rec["breakaway_time"] or not rec["joint_times"]:
                continue
            jt0 = rec["joint_times"][0]
            wt, wv = wheel_avg_series(rec["joint_times"], rec["joint_series"])
            label = f"{rec['folder_name']} ({rec['mode']})"
            wheel_cols[label] = align_to_breakaway(wt, wv, jt0, rec["breakaway_time"], align_grid)
            body_cols[label] = align_to_breakaway(
                rec["odomtruth_times"], [abs(w) for w in rec["odomtruth_w"]],
                jt0, rec["breakaway_time"], align_grid)
        aligned_sheets[f"BAWh_{tag}"] = pd.DataFrame(wheel_cols)
        aligned_sheets[f"BABd_{tag}"] = pd.DataFrame(body_cols)

        # Group-mean divergence only makes sense where two modes exist.
        n_low = sum(1 for r in recs if r["mode"] == "Low")
        n_high = sum(1 for r in recs if r["mode"] == "High")
        if n_low == 0 or n_high == 0:
            onset_rows.append({
                "Commanded W (rad/s)": w_key,
                "Max Drive Force (N·m/wheel)": force_key,
                "Joint Drive Damping": damping_key,
                "Solver": solver_key,
                "Signal": "-",
                "N Low Runs": n_low, "N High Runs": n_high,
                "Onset Time (s)": "N/A", "Final Divergence (rad/s)": "N/A",
                "Interpretation": (
                    "No Low/High pair in this W + Max Force condition; "
                    "no divergence curve to compute."),
            })
            continue

        ts = build_group_time_series(recs)
        ts_sheets[f"GM_{tag}"] = pd.DataFrame({
            "Time (s)": ts["t_grid"],
            "Low Wheel Avg |omega| (rad/s)": ts["low_wheel"],
            "High Wheel Avg |omega| (rad/s)": ts["high_wheel"],
            "Wheel Divergence (rad/s)": ts["wheel_divergence"],
            "Low Body |W| (rad/s)": ts["low_body"],
            "High Body |W| (rad/s)": ts["high_body"],
            "Body Divergence (rad/s)": ts["body_divergence"],
        })

        if ts["wheel_onset"] == ts["wheel_onset"] and ts["body_onset"] == ts["body_onset"]:
            if ts["wheel_onset"] < ts["body_onset"] - TIME_SERIES_DT:
                note = ("Wheel signal diverges earlier -- split visible at the drive "
                        "layer before body-level rotation.")
            elif ts["body_onset"] < ts["wheel_onset"] - TIME_SERIES_DT:
                note = "Body signal diverges earlier -- unexpected; re-check wheel-side data quality."
            else:
                note = ("Wheel and body diverge at approximately the same time -- consistent "
                        "with a shared upstream cause (e.g. contact/friction state) rather "
                        "than one causing the other.")
        else:
            note = "Not enough valid data in one or both signals to compare onset times."

        for sig, onset, final in (("Wheel (joint_states)", ts["wheel_onset"], ts["wheel_final"]),
                                  ("Body (odom_truth)", ts["body_onset"], ts["body_final"])):
            onset_rows.append({
                "Commanded W (rad/s)": w_key,
                "Max Drive Force (N·m/wheel)": force_key,
                "Joint Drive Damping": damping_key,
                "Solver": solver_key,
                "Signal": sig,
                "N Low Runs": ts["n_low"], "N High Runs": ts["n_high"],
                "Onset Time (s)": round(onset, 3) if onset == onset else "N/A",
                "Final Divergence (rad/s)": round(final, 4) if final == final else "N/A",
                "Interpretation": note if sig.startswith("Body") else "",
            })

    # ---------------- v12: DampingSummary (item 2) ----------------
    damping_summary_rows = []
    if not idx_df.empty and "Joint Drive Damping" in idx_df.columns:
        dgroup_keys = sorted(
            {(numeric_group_value(row.get("_w_cmd_num"), W_GROUP_DECIMALS),
              numeric_group_value(row.get("_damping_num"), DAMPING_GROUP_DECIMALS))
             for row in index_rows},
            key=group_sort_key)
        for wkey, dkey in dgroup_keys:
            sub = pd.DataFrame([
                row for row in index_rows
                if numeric_group_value(row.get("_w_cmd_num"), W_GROUP_DECIMALS) == wkey
                and numeric_group_value(row.get("_damping_num"), DAMPING_GROUP_DECIMALS) == dkey
            ])
            drow = {
                "Commanded W (rad/s)": wkey,
                "Joint Drive Damping": dkey,
                "N Runs": len(sub),
                "Low Runs": int((sub.get("Mode") == "Low").sum()),
                "High Runs": int((sub.get("Mode") == "High").sum()),
            }
            low_high = drow["Low Runs"] + drow["High Runs"]
            drow["High Proportion among Low/High"] = (
                drow["High Runs"] / low_high if low_high else "N/A")
            for col, label in (
                ("Wheel Std Mean of 4 (rad/s)", "Wheel Std Mean of 4 (rad/s)"),
                ("W_ss Actual (rad/s)", "Body W_ss Mean (rad/s)"),
                ("Total Angular Displacement, Actual (deg)", "Rotation Mean (deg)"),
            ):
                vals = numeric_column(sub, col)
                if len(vals):
                    drow[label] = round(float(vals.mean()), 4)
            damping_summary_rows.append(drow)

    # ---------------- v12: SolverSummary, PGS vs TGS (item 13) ----------------
    solver_summary_rows = []
    if not idx_df.empty and "Solver" in idx_df.columns:
        sgroup_keys = sorted(
            {(numeric_group_value(row.get("_w_cmd_num"), W_GROUP_DECIMALS),
              row.get("_solver", "unknown"))
             for row in index_rows},
            key=group_sort_key)
        for wkey, skey in sgroup_keys:
            sub = pd.DataFrame([
                row for row in index_rows
                if numeric_group_value(row.get("_w_cmd_num"), W_GROUP_DECIMALS) == wkey
                and row.get("_solver", "unknown") == skey
            ])
            srow = {"Commanded W (rad/s)": wkey, "Solver": skey, "N Runs": len(sub)}
            for col, label in (
                ("Wheel Std Mean of 4 (rad/s)", "Wheel Std Mean of 4 (rad/s)"),
                ("Wheel Std RMS of 4 (rad/s)", "Wheel Std RMS of 4 (rad/s)"),
                ("Wheel Std Max of 4 (rad/s)", "Wheel Std Max of 4 (rad/s)"),
                ("W_ss Actual (rad/s)", "Body W_ss Mean (rad/s)"),
                ("Total Angular Displacement, Actual (deg)", "Rotation Mean (deg)"),
                ("Slip Ratio, Steady-State (%)", "Slip SS Mean (%)"),
            ):
                vals = numeric_column(sub, col)
                if len(vals):
                    srow[label] = round(float(vals.mean()), 4)
            solver_summary_rows.append(srow)
        # Direct PGS-vs-TGS reduction comparison at any W where both exist.
        by_w = {}
        for row in solver_summary_rows:
            by_w.setdefault(row["Commanded W (rad/s)"], {})[row["Solver"]] = row
        for wkey, by_solver in by_w.items():
            if "PGS" in by_solver and "TGS" in by_solver:
                pgs, tgs = by_solver["PGS"], by_solver["TGS"]
                crow = {"Commanded W (rad/s)": wkey, "Comparison": "TGS vs PGS"}
                for label in ("Wheel Std Mean of 4 (rad/s)", "Body W_ss Mean (rad/s)",
                             "Rotation Mean (deg)"):
                    if label in pgs and label in tgs and pgs[label]:
                        crow[f"{label} -- Reduction Ratio (TGS/PGS)"] = round(
                            tgs[label] / pgs[label], 4) if pgs[label] else "N/A"
                solver_summary_rows.append(crow)

    # ---------------- v12: ParsingWarnings (item 15) ----------------
    parsing_warning_rows = []
    for row in index_rows:
        unknown_bits = []
        if row.get("_max_force_nm") != row.get("_max_force_nm"):
            unknown_bits.append("Max Drive Force")
        if row.get("_damping_num") != row.get("_damping_num"):
            unknown_bits.append("Joint Drive Damping")
        if row.get("_solver", "unknown") == "unknown":
            unknown_bits.append("Solver")
        if unknown_bits:
            parsing_warning_rows.append({
                "Run": row.get("Run"),
                "Unparsed Fields": ", ".join(unknown_bits),
                "Note": ("Excluded from the corresponding factor's summary/"
                        "reference-centre pooling -- fix the folder name to "
                        "include this in aggregate statistics."),
            })
        if row.get("_joint_order_mismatch"):
            parsing_warning_rows.append({
                "Run": row.get("Run"),
                "Unparsed Fields": "Joint order differs across JointState topics",
                "Note": row.get("_joint_order_mismatch"),
            })

    # ---------------- v12: AnalysisConfig (item 18) ----------------
    import datetime as _dt
    analysis_config_rows = [
        {"Parameter": "Analyzer Version", "Value": "cobraflex_analyzer (Test04 engine lineage v12)"},
        {"Parameter": "Generated At", "Value": _dt.datetime.now().isoformat(timespec="seconds")},
        {"Parameter": "Legacy Fixed Centres Mode", "Value": use_legacy},
        {"Parameter": "Baseline Centres Source",
         "Value": ("--legacy-centers (fixed W=0.8 centres only)" if use_legacy else
                   f"BASELINE_CENTERS_DEFAULT + --baseline-json override; "
                   f"solver scope={BASELINE_CENTERS_SOLVER}; "
                   f"entries={baseline_centers}")},
        {"Parameter": "Baseline Match Tolerance (rad/s)", "Value": BASELINE_MATCH_TOL_RAD_S},
        {"Parameter": "Breakaway Threshold (rad/s)", "Value": BREAKAWAY_THRESHOLD_RAD_S},
        {"Parameter": "Breakaway Sustain Duration (s)", "Value": BREAKAWAY_SUSTAIN_S},
        {"Parameter": "Command Active Threshold (rad/s)", "Value": COMMAND_ACTIVE_THRESHOLD_RAD_S},
        {"Parameter": "Steady-State Settle Delay After Breakaway (s)", "Value": STEADY_STATE_SETTLE_S},
        {"Parameter": "Steady-State Window Min Samples", "Value": 5},
        {"Parameter": "Steady-State Window Min Duration (s)", "Value": STEADY_STATE_MIN_WINDOW_S},
        {"Parameter": "Window Min Rate Completeness", "Value": WINDOW_MIN_RATE_COMPLETENESS},
        {"Parameter": "Contact Force Window",
         "Value": "breakaway-anchored absolute window, identical to wheel/body "
                  "(v12 2026-08-04); the 0.5-0.9 fraction-window value is kept "
                  "in the LEGACY columns for reconciliation"},
        {"Parameter": "Integrated Slip No-Slip Form",
         "Value": "signed r*(w_R_bar - w_L_bar)/D; absolute mean(|omega|) form "
                  "retained in the LEGACY columns"},
        {"Parameter": "W_ss Consistency Relative Tolerance", "Value": W_SS_CONSISTENCY_REL_TOL},
        {"Parameter": "Split Detection Min N", "Value": SPLIT_MIN_RUNS},
        {"Parameter": "Split Detection Gap Fraction", "Value": SPLIT_GAP_FRACTION},
        {"Parameter": "Time Source Policy",
         "Value": "header.stamp preferred per-message; falls back to rosbag2 storage "
                  "timestamp when no header.stamp is present (item 7); see "
                  "DataQuality sheet's Time Source column per run/topic"},
        {"Parameter": "Wheel Radius (m)", "Value": WHEEL_RADIUS_M},
        {"Parameter": "Wheel Separation (m)", "Value": WHEEL_SEPARATION_M},
        {"Parameter": "Bag Root", "Value": root},
        {"Parameter": "Output Path", "Value": out_path},
        {"Parameter": "Generic KPI Module Available", "Value": HAVE_GENERIC_ANALYZER},
    ]

    # ---------------- Write workbook ----------------
    drop_cols = [c for c in idx_df.columns if c.startswith("_")] if not idx_df.empty else []
    with pd.ExcelWriter(out_path, engine="openpyxl") as writer:
        if not idx_df.empty:
            out_df = idx_df.drop(columns=drop_cols)
            first = [
                "Run", "Condition", "Commanded W (rad/s)",
                "Max Drive Force (N·m/wheel)", "Joint Drive Damping", "Solver",
                "Motion State", "Zero-Force Functional Check", "Mode",
            ]
            cols = ([c for c in first if c in out_df.columns]
                    + [c for c in out_df.columns if c not in first + ["Flag"]]
                    + (["Flag"] if "Flag" in out_df.columns else []))
            out_df = out_df[cols]
        else:            out_df = pd.DataFrame([{"Note": "No Test04 bags processed"}])
        out_df.to_excel(writer, sheet_name="Index", index=False)

        pd.DataFrame(split_records).to_excel(writer, sheet_name="SplitDetection", index=False)
        if max_force_summary_rows:
            pd.DataFrame(max_force_summary_rows).to_excel(
                writer, sheet_name="MaxForceSummary", index=False)
        if damping_summary_rows:
            pd.DataFrame(damping_summary_rows).to_excel(
                writer, sheet_name="DampingSummary", index=False)
        if solver_summary_rows:
            pd.DataFrame(solver_summary_rows).to_excel(
                writer, sheet_name="SolverSummary", index=False)
        if mode_summary_rows:
            pd.DataFrame(mode_summary_rows).to_excel(writer, sheet_name="ModeSummary", index=False)
        if slip_summary_rows:
            pd.DataFrame(slip_summary_rows).to_excel(writer, sheet_name="SlipSummary", index=False)
        if contact_summary_rows:
            pd.DataFrame(contact_summary_rows).to_excel(
                writer, sheet_name="ContactForceSummary", index=False)
        if onset_rows:
            pd.DataFrame(onset_rows).to_excel(writer, sheet_name="DivergenceOnset", index=False)
        if parsing_warning_rows:
            pd.DataFrame(parsing_warning_rows).to_excel(
                writer, sheet_name="ParsingWarnings", index=False)
        if data_quality_rows:
            pd.DataFrame(data_quality_rows).to_excel(
                writer, sheet_name="DataQuality", index=False)
        if wheel_cmd_vs_actual_rows:
            pd.DataFrame(wheel_cmd_vs_actual_rows).to_excel(
                writer, sheet_name="WheelCmdVsActual", index=False)
        pd.DataFrame(analysis_config_rows).to_excel(
            writer, sheet_name="AnalysisConfig", index=False)

        for name, df in ts_sheets.items():
            df.to_excel(writer, sheet_name=name[:31], index=False)
        for name, df in aligned_sheets.items():
            df.to_excel(writer, sheet_name=name[:31], index=False)

        # generic standard KPI sheets (batch_analyze_v3 behaviour), if available
        if HAVE_GENERIC_ANALYZER and generic_index_rows:
            pd.DataFrame(generic_index_rows).to_excel(
                writer, sheet_name="V25_Index", index=False)
            for tid in sorted(per_test_rows):
                rows = per_test_rows[tid]
                if not rows:
                    continue
                df = pd.DataFrame(rows)
                cols = ["Run", "Condition"] + [c for c in df.columns
                                               if c not in ("Run", "Condition")]
                df[cols].to_excel(writer, sheet_name=f"Test{tid:02d}"[:31], index=False)

        if skipped or failed:
            problems = ([{"Folder": f, "Issue": "Unrecognized test_id"} for f in skipped]
                        + [{"Folder": f, "Issue": e} for f, e in failed])
            pd.DataFrame(problems).to_excel(writer, sheet_name="Skipped_Failed", index=False)

    # ---------------- Cosmetic pass ----------------
    from openpyxl import load_workbook
    wb = load_workbook(out_path)
    header_font = Font(name="Arial", bold=True, color="FFFFFF")
    header_fill = PatternFill("solid", start_color="1F4E78", end_color="1F4E78")
    warn_fill = PatternFill("solid", start_color="FCE4D6", end_color="FCE4D6")
    low_fill = PatternFill("solid", start_color="DDEBF7", end_color="DDEBF7")
    high_fill = PatternFill("solid", start_color="FFF2CC", end_color="FFF2CC")
    pass_fill = PatternFill("solid", start_color="E2F0D9", end_color="E2F0D9")
    fail_fill = PatternFill("solid", start_color="F4CCCC", end_color="F4CCCC")
    no_motion_fill = PatternFill("solid", start_color="EDEDED", end_color="EDEDED")
    invalid_fill = PatternFill("solid", start_color="FCE4D6", end_color="FCE4D6")

    for sheet_name in wb.sheetnames:
        ws = wb[sheet_name]
        for cell in ws[1]:
            cell.font = header_font
            cell.fill = header_fill
            cell.alignment = Alignment(wrap_text=True, vertical="center")
        ws.freeze_panes = "A2"
        for col_cells in ws.columns:
            length = max((len(str(c.value)) if c.value is not None else 0)
                         for c in col_cells)
            ws.column_dimensions[get_column_letter(col_cells[0].column)].width = \
                min(max(length + 2, 10), 60)

        if sheet_name == "Index":
            header = [c.value for c in ws[1]]
            flag_col = header.index("Flag") + 1 if "Flag" in header else None
            mode_col = header.index("Mode") + 1 if "Mode" in header else None
            zero_check_col = (
                header.index("Zero-Force Functional Check") + 1
                if "Zero-Force Functional Check" in header else None)
            for row in ws.iter_rows(min_row=2, min_col=1, max_col=ws.max_column):
                if flag_col and row[flag_col - 1].value is True:
                    for cell in row:
                        cell.fill = warn_fill
                elif zero_check_col and str(row[zero_check_col - 1].value).startswith("FAIL"):
                    for cell in row:
                        cell.fill = fail_fill
                elif zero_check_col and str(row[zero_check_col - 1].value).startswith("PASS"):
                    for cell in row:
                        cell.fill = pass_fill
                elif mode_col:
                    mv = row[mode_col - 1].value
                    if mv == "Low":
                        for cell in row:
                            cell.fill = low_fill
                    elif mv == "High":
                        for cell in row:
                            cell.fill = high_fill
                    elif mv == "No Motion":
                        for cell in row:
                            cell.fill = no_motion_fill
                    elif mv == "Invalid":
                        for cell in row:
                            cell.fill = invalid_fill

        if sheet_name == "MaxForceSummary":
            header = [c.value for c in ws[1]]
            if "High Proportion among Low/High" in header:
                col_idx = header.index("High Proportion among Low/High") + 1
                for cell in ws.iter_cols(
                        min_col=col_idx, max_col=col_idx, min_row=2,
                        max_row=ws.max_row):
                    for item in cell:
                        if isinstance(item.value, (int, float)):
                            item.number_format = "0.0%"

    # Charts: one per group-mean sheet, one per breakaway-aligned sheet.
    for sheet_name in list(wb.sheetnames):
        if sheet_name.startswith("GM_"):
            ws_ts = wb[sheet_name]
            n_rows = ws_ts.max_row
            cats = Reference(ws_ts, min_col=1, min_row=2, max_row=n_rows)
            for title, cols, anchor in (
                ("Wheel-Level: Low vs High Group Mean", (2, 3, 4), "J2"),
                ("Body-Level: Low vs High Group Mean", (5, 6, 7), "J22"),
            ):
                ch = LineChart()
                ch.title = f"{title} -- {sheet_name}"
                ch.y_axis.title = "rad/s"
                ch.x_axis.title = "Time (s)"
                ch.width, ch.height = 24, 10
                for col in cols:
                    ch.add_data(Reference(ws_ts, min_col=col, min_row=1, max_row=n_rows),
                                titles_from_data=True)
                ch.set_categories(cats)
                ws_ts.add_chart(ch, anchor)

        elif sheet_name.startswith(("BAWh_", "BABd_")):
            ws_al = wb[sheet_name]
            n_rows, n_cols = ws_al.max_row, ws_al.max_column
            if n_cols < 2:
                continue
            # An overlay of more than ~15 runs is unreadable; the numbers
            # stay in the sheet, only the chart is skipped.
            if n_cols - 1 > 15:
                ws_al.cell(row=1, column=n_cols + 2,
                           value=f"Chart omitted: {n_cols - 1} runs in this condition "
                                 f"(overlay readable up to 15). Plot manually if needed.")
                continue
            ch = LineChart()
            ch.title = sheet_name
            ch.y_axis.title = "rad/s"
            ch.x_axis.title = "Time Relative to Breakaway (s)"
            ch.width, ch.height = 28, 14
            for col in range(2, n_cols + 1):
                ch.add_data(Reference(ws_al, min_col=col, min_row=1, max_row=n_rows),
                            titles_from_data=True)
            ch.set_categories(Reference(ws_al, min_col=1, min_row=2, max_row=n_rows))
            ws_al.add_chart(ch, "M2")

    wb.save(out_path)
    _say(f"\nSaved: {out_path}")
    if use_legacy:
        _say("WARNING: --legacy-centers was used. Mode labels come from the fixed "
             "W=0.8 centres and are NOT valid for other commanded speeds.")

    return {"processed": len(index_rows), "skipped": len(skipped),
            "failed": len(failed), "output": out_path}


# =====================================================================
# SECTION 1 -- CONFIGURATION AND GEOMETRY
# =====================================================================

# The example configuration, emitted by --write-example-config. Kept in
# the analyzer rather than as a separate file so the documented
# defaults cannot drift away from the code that implements them.
EXAMPLE_CONFIG_YAML = """# CobraFlex analyzer -- thesis handover example configuration.
#
# Every section is optional. Running without --config uses the values shown
# here as defaults, so this file is only needed to change something.
#
#   python3 cobraflex_analyzer.py BAG_ROOT -o out.xlsx -c this_file.yaml
#
# Unknown keys are reported in the AnalysisConfig sheet and ignored. Values
# that would corrupt the arithmetic (a non-positive length, an incomplete
# joint map, a partial corner-load set) are fatal rather than warnings.

analysis:
  analyzer_version: thesis-handover
  # When true, any bag routed to Test04 also gets the deep analysis.
  # --no-deep on the command line overrides this for a single run.
  auto_deep_test04: true

geometry:
  wheel_radius_m: 0.03725

  # Three separate fields on purpose because they answer different questions:
  #
  #   controller_wheel_distance_m -- measured wheel-centre separation used by
  #       the Differential Controller to convert /cmd_vel into wheel targets.
  #   geometric_track_m -- measured wheel-centre separation used by the
  #       no-slip yaw and slip calculations in this analyzer.
  #   wheelbase_m -- measured front-to-rear axle separation. It is not used
  #       by the yaw kinematics.
  #
  # The final thesis baseline uses 0.153 m for wheel separation in both the
  # controller and analysis, and 0.154 m for wheelbase.
  controller_wheel_distance_m: 0.153
  geometric_track_m: 0.153
  wheelbase_m: 0.154
  physical_track_independently_measured: true

joint_names:
  FL: front_left_wheel_joint
  FR: front_right_wheel_joint
  RL: rear_left_wheel_joint
  RR: rear_right_wheel_joint

corner_load_comparison:
  # Disabled by default. This compares a STATIC weighing of the physical
  # robot against a contact-force magnitude sampled DURING rotation. It is
  # an optional side diagnostic and feeds nothing else -- no yaw, tracking,
  # branch or slip figure depends on it.
  enabled: false

  # If enabled, all four corners are required, and `source` must say which
  # value set is in use. The raw four-corner readings and the mass-
  # reconciled set are different numbers and must never be mixed in one
  # comparison; record which one these are.
  # values_g:
  #   FL: 1054
  #   FR: 1290
  #   RL: 1007
  #   RR: 998
  # source: "raw four-corner scale readings, sum 4349 g (not mass-reconciled)"

output:
  # Per-condition breakaway-aligned time-series sheets. Turning this off
  # produces a smaller workbook with all KPI sheets intact.
  include_time_series: true
"""


ANALYZER_VERSION = "thesis-handover"

# ---------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------
# The geometry fields remain separate because they serve different purposes.
# The final thesis baseline uses the measured wheel-centre separation of
# 0.153 m in both the Differential Controller and the analysis, while the
# measured wheelbase is 0.154 m.
DEFAULT_GEOMETRY = {
    "wheel_radius_m": 0.03725,
    "controller_wheel_distance_m": 0.153,
    "geometric_track_m": 0.153,
    "wheelbase_m": 0.154,
    "physical_track_independently_measured": True,
    "note": ("Final thesis baseline: measured wheel-centre separation "
             "0.153 m for controller and analysis; measured wheelbase 0.154 m."),
}

DEFAULT_JOINT_NAMES = {
    "FL": "front_left_wheel_joint",
    "FR": "front_right_wheel_joint",
    "RL": "rear_left_wheel_joint",
    "RR": "rear_right_wheel_joint",
}

TEST_IDS = tuple(range(1, 11))


class ConfigError(ValueError):
    """Raised for a configuration file that cannot be used safely.

    Deliberately fatal rather than warn-and-continue: a negative wheel
    radius or a missing joint name silently produces plausible-looking but
    wrong numbers, which is worse than not running at all.
    """


@dataclass
class AnalysisConfig:
    analyzer_version: str = ANALYZER_VERSION
    auto_deep_test04: bool = True
    geometry: dict = field(default_factory=lambda: dict(DEFAULT_GEOMETRY))
    joint_names: dict = field(default_factory=lambda: dict(DEFAULT_JOINT_NAMES))
    corner_load_enabled: bool = False
    corner_load_values_g: dict | None = None
    corner_load_source: str = "disabled"
    include_time_series: bool = True
    config_path: str | None = None
    unknown_keys: list = field(default_factory=list)

    # -- validation ---------------------------------------------------
    def validate(self):
        g = self.geometry
        for key in ("wheel_radius_m", "controller_wheel_distance_m",
                    "geometric_track_m", "wheelbase_m"):
            if key not in g:
                raise ConfigError(f"geometry.{key} is required")
            try:
                val = float(g[key])
            except (TypeError, ValueError):
                raise ConfigError(f"geometry.{key} must be a number, got {g[key]!r}")
            if val <= 0:
                raise ConfigError(
                    f"geometry.{key} must be positive, got {val}. A non-positive "
                    "radius or track makes every derived speed and yaw wrong "
                    "rather than merely inaccurate.")
            g[key] = val

        missing = [w for w in ("FL", "FR", "RL", "RR") if not self.joint_names.get(w)]
        if missing:
            raise ConfigError(
                "joint_names must name all four wheels; missing "
                f"{', '.join(missing)}. Without a complete map, samples would "
                "be dropped or a three-wheel mean would masquerade as four.")

        if self.corner_load_enabled:
            if not self.corner_load_values_g:
                raise ConfigError(
                    "corner_load_comparison.enabled is true but values_g is empty")
            missing = [w for w in ("FL", "FR", "RL", "RR")
                       if w not in self.corner_load_values_g]
            if missing:
                raise ConfigError(
                    "corner_load_comparison.values_g needs all four corners; "
                    f"missing {', '.join(missing)}")
            self.corner_load_values_g = {
                w: float(self.corner_load_values_g[w]) for w in ("FL", "FR", "RL", "RR")}
        return self

    # -- reporting ----------------------------------------------------
    def to_rows(self):
        rows = [
            {"Parameter": "Analyzer Version", "Value": self.analyzer_version},
            {"Parameter": "Config File", "Value": self.config_path or "none (built-in defaults)"},
            {"Parameter": "Auto Deep Analysis for Test04", "Value": self.auto_deep_test04},
        ]
        purposes = {
            "wheel_radius_m": "wheel rolling radius; wheel omega -> surface speed",
            "controller_wheel_distance_m":
                "differential-controller parameter; /cmd_vel -> per-wheel target",
            "geometric_track_m":
                "analysis parameter; measured wheel speeds -> no-slip yaw prediction",
            "wheelbase_m": "front-to-rear axle separation; not used by yaw kinematics",
        }
        for key, purpose in purposes.items():
            rows.append({"Parameter": f"geometry.{key}", "Value": self.geometry[key],
                         "Purpose": purpose,
                         "Source": "handover spec / USD nominal value"})
        rows.append({
            "Parameter": "geometry.physical_track_independently_measured",
            "Value": self.geometry.get("physical_track_independently_measured", False),
            "Purpose": "provenance flag",
            "Source": self.geometry.get("note", "")})
        for wheel, name in sorted(self.joint_names.items()):
            rows.append({"Parameter": f"joint_names.{wheel}", "Value": name})
        rows.append({"Parameter": "corner_load_comparison.enabled",
                     "Value": self.corner_load_enabled,
                     "Source": self.corner_load_source})
        if self.corner_load_enabled:
            for wheel, grams in sorted(self.corner_load_values_g.items()):
                rows.append({"Parameter": f"corner_load_comparison.values_g.{wheel}",
                             "Value": grams, "Source": self.corner_load_source})
        for key in self.unknown_keys:
            rows.append({"Parameter": f"UNKNOWN CONFIG KEY: {key}",
                         "Value": "ignored",
                         "Purpose": "not recognised by this analyzer version"})
        return rows

    def as_dict(self):
        return asdict(self)


_KNOWN_SECTIONS = {"analysis", "geometry", "joint_names", "test04",
                   "contact_force", "corner_load_comparison", "output"}


def load_config(path=None):
    """Load an optional YAML (or JSON) configuration.

    Returns built-in defaults when `path` is None, so the analyzer runs with
    no configuration file at all. Unknown keys are recorded and reported in
    the AnalysisConfig sheet rather than silently dropped, but they are not
    fatal -- only values that would corrupt the arithmetic are.
    """
    cfg = AnalysisConfig()
    if not path:
        return cfg.validate()
    if not os.path.exists(path):
        raise ConfigError(f"config file not found: {path}")

    text = open(path, encoding="utf-8").read()
    raw = None
    try:
        import yaml  # optional dependency
        raw = yaml.safe_load(text)
    except ImportError:
        try:
            raw = json.loads(text)
        except json.JSONDecodeError:
            raise ConfigError(
                "PyYAML is not installed and the config file is not valid JSON. "
                "Install PyYAML (pip install pyyaml) or supply JSON.")
    if not isinstance(raw, dict):
        raise ConfigError("config file must contain a mapping at the top level")

    cfg.config_path = path
    cfg.unknown_keys = sorted(k for k in raw if k not in _KNOWN_SECTIONS)

    analysis = raw.get("analysis") or {}
    cfg.analyzer_version = str(analysis.get("analyzer_version", ANALYZER_VERSION))
    cfg.auto_deep_test04 = bool(analysis.get("auto_deep_test04", True))

    cfg.geometry.update(raw.get("geometry") or {})
    cfg.joint_names.update(raw.get("joint_names") or {})

    corner = raw.get("corner_load_comparison") or {}
    cfg.corner_load_enabled = bool(corner.get("enabled", False))
    cfg.corner_load_values_g = corner.get("values_g")
    if cfg.corner_load_enabled:
        cfg.corner_load_source = str(
            corner.get("source", f"config file {os.path.basename(path)}"))

    output = raw.get("output") or {}
    cfg.include_time_series = bool(output.get("include_time_series", True))
    return cfg.validate()



# =====================================================================
# SECTION 2 -- BAG DISCOVERY AND TEST-ID ROUTING
# =====================================================================

def find_bag_dirs(root, recursive=True):
    """Every directory at or under `root` that contains a metadata.yaml.

    Recursive by default: the 0714/0804 batches nest bags one or two levels
    below the batch folder, and the non-recursive scan used by the older
    tools silently reported "no bags found" for those layouts.
    """
    root = os.path.abspath(root)
    if os.path.isfile(os.path.join(root, "metadata.yaml")):
        return [(os.path.basename(root), root)]
    found = []
    if not recursive:
        for name in sorted(os.listdir(root)):
            full = os.path.join(root, name)
            if os.path.isdir(full) and os.path.exists(os.path.join(full, "metadata.yaml")):
                found.append((name, full))
        return found
    for dirpath, dirnames, filenames in os.walk(root):
        if "metadata.yaml" in filenames:
            found.append((os.path.basename(dirpath), dirpath))
            dirnames[:] = []          # a bag folder has no bags inside it
    return sorted(found, key=lambda p: p[1])


# ---------------------------------------------------------------------
# Test-id detection
# ---------------------------------------------------------------------
_SIDECAR_NAMES = ("run_metadata.yaml", "run_metadata.json", "run_metadata.yml")


def _sidecar_test_id(bag_path):
    for name in _SIDECAR_NAMES:
        full = os.path.join(bag_path, name)
        if not os.path.exists(full):
            continue
        text = open(full, encoding="utf-8").read()
        data = None
        try:
            import yaml
            data = yaml.safe_load(text)
        except ImportError:
            try:
                data = json.loads(text)
            except json.JSONDecodeError:
                data = None
        if isinstance(data, dict) and "test_id" in data:
            try:
                tid = int(data["test_id"])
            except (TypeError, ValueError):
                return None
            if tid in TEST_IDS:
                return tid
    return None


def detect_test_id(folder_name, bag_path, folder_name_detector, bag_inference=None):
    """Resolve a bag's test_id, reporting how it was resolved.

    Priority: sidecar metadata, then the folder name, then -- only when the
    name is uninformative -- an inference from the command profile inside
    the bag. Returns (test_id, source, confidence). test_id is 0 when
    nothing resolved it; the caller must route that to Skipped_Failed
    rather than guessing.

    The presence of a topic is deliberately NOT evidence: /joint_states and
    the contact-force topics are recorded for every test, so treating them
    as a Test04 signature would relabel Test01 and Test03 runs.
    """
    tid = _sidecar_test_id(bag_path)
    if tid:
        return tid, "sidecar", "high"

    tid = folder_name_detector(folder_name)
    if tid:
        return tid, "folder_name", "high"

    if bag_inference is not None:
        tid = bag_inference(bag_path)
        if tid:
            return tid, "bag_inference", "low"

    return 0, "unknown", "none"


def infer_test04_from_command_profile(bag_path, reader_fn):
    """Infer Test04 from a sustained in-place rotation command.

    Requires |V_cmd| to stay near zero while |W_cmd| stays non-zero for a
    meaningful stretch. Returns 4 or None -- never a different test id,
    since no other test has a signature this pipeline can claim to
    recognise from content alone.
    """
    try:
        data = reader_fn(bag_path)
    except Exception:  # noqa: BLE001 -- inference must never abort a batch
        return None
    wz = [abs(v) for v in (data.get("cmdvel_wz") or [])]
    if not wz:
        return None
    active = [v for v in wz if v > 1e-3]
    if len(active) < 20 or len(active) < 0.25 * len(wz):
        return None
    lin = data.get("cmdvel_linear_x")
    if lin:
        if max(abs(v) for v in lin) > 0.01:
            return None
    else:
        # No linear channel recorded: a rotation-only command profile is
        # then only suggestive, not decisive. Still returned, but the
        # caller labels it low confidence and writes a ParsingWarnings row.
        pass
    return 4


# =====================================================================
# SECTION 3 -- GENERIC Test01-Test10 KPIs (adapter over the generic analyzer)
#
# The formulas are not reimplemented. generic analyzer's compute_test_kpis() stays the
# single definition of every generic KPI so that handover output remains
# directly comparable with the existing workbooks; this section only
# imports the generic analyzer headlessly, turns its human-readable composite strings into
# typed numeric columns, and keeps the original text alongside.
#
# The typed extraction replaces the regex reverse-parsing that
# cobraflex_batch_analyze_v3.py performed on already-formatted output. It
# is still string parsing -- the numbers are only available in that form --
# but it now happens in one place, and a format drift leaves blank numeric
# cells and a warning instead of a silently wrong value.
# =====================================================================

HAVE_GENERIC_ANALYZER = True
GENERIC_ANALYZER_IMPORT_ERROR = None
try:
    from cobraflex_rosbag_analyzer import (  # noqa: F401
        compute_test_kpis, identify_test_id, TEST_NAMES)
except Exception as exc:  # noqa: BLE001
    HAVE_GENERIC_ANALYZER = False
    GENERIC_ANALYZER_IMPORT_ERROR = str(exc)
    TEST_NAMES = {i: f"Test{i:02d}" for i in range(1, 11)}

    def identify_test_id(folder_name):  # type: ignore[misc]
        m = re.match(r"test_?(\d{1,2})[_\-]", folder_name.lower())
        return int(m.group(1)) if m else 0

    def compute_test_kpis(bag_path, test_id):  # type: ignore[misc]
        raise RuntimeError(
            f"cobraflex_rosbag_analyzer is not importable: {GENERIC_ANALYZER_IMPORT_ERROR}")


# ---------------------------------------------------------------------
# Typed extraction
# ---------------------------------------------------------------------
_SS_RE = re.compile(
    r"([\d.]+)\s*%\s*\(W_ss=(-?[\d.]+)\s*rad/s over \[([\d.]+)s,([\d.]+)s\],\s*"
    r"cmd=([\d.]+)\s*rad/s\)")
_TRANSIENT_RE = re.compile(
    r"mean\|W\|=(-?[\d.]+)\s*rad/s over \[([\d.]+)s,([\d.]+)s\]")
_LEADING_NUMBER_RE = re.compile(r"^\s*(-?\d+(?:\.\d+)?)(?:\s|$|%)")

# Units the generic analyzer appends to otherwise-numeric KPI cells. A value is only
# converted when the entire remainder of the cell is one of these, so
# "1.973 m" yields a number while "Understeer (R_cmd 0.20m -> ...)" and
# "0/889 moving steps (0.0%)" are left as text -- they are sentences that
# happen to start with a digit, not measurements.
_KNOWN_UNITS = {
    "m": "m", "s": "s", "%": "%", "hz": "Hz", "n": "N", "kg": "kg",
    "deg": "deg", "rad": "rad", "m/s": "m/s", "rad/s": "rad/s",
    "m/s^2": "m/s^2", "rad/s^2": "rad/s^2", "n·m": "N·m", "n*m": "N·m",
    "g": "g", "v": "V", "a": "A", "w": "W",
}
_VALUE_WITH_UNIT_RE = re.compile(r"^\s*(-?\d+(?:\.\d+)?)\s*([^\s\d]{1,7})\s*$")


def _as_number(value):
    """Return a float when the cell is genuinely numeric, else None.

    Strings such as "N/A", "N/A -- no active command window" and any text
    that merely starts with a digit but continues into prose are left
    alone; only a bare number (optionally followed by a percent sign) is
    converted, so a unit-bearing label is never mistaken for a measurement.
    """
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        return float(value)
    if not isinstance(value, str):
        return None
    text = value.strip()
    if not text or text.upper().startswith("N/A"):
        return None
    m = _LEADING_NUMBER_RE.match(text)
    if not m:
        return None
    remainder = text[m.end():].strip()
    if remainder and remainder != "%":
        return None
    return float(m.group(1))


def _value_with_unit(value):
    """Split "1.973 m" into (1.973, "m"), or return None.

    The unit must be recognised. An unknown trailing token is treated as
    prose rather than a unit, because inventing a numeric column from a
    string this function does not understand is how a label becomes a
    measurement.
    """
    if not isinstance(value, str):
        return None
    text = value.strip()
    if not text or text.upper().startswith("N/A"):
        return None
    m = _VALUE_WITH_UNIT_RE.match(text)
    if not m:
        return None
    unit = _KNOWN_UNITS.get(m.group(2).lower())
    if unit is None:
        return None
    return float(m.group(1)), unit


def add_typed_columns(row_dict, test_id):
    """Add numeric columns derived from generic analyzer's composite strings.

    The text columns are kept, not replaced: they carry the window bounds
    and caveats that make a number auditable. Returns a list of warning
    strings for formats that did not match.
    """
    warnings = []

    if test_id == 4:
        ss_text = row_dict.get("Angular Tracking Ratio (Steady-State 2-9s)")
        if isinstance(ss_text, str) and not ss_text.upper().startswith("N/A"):
            m = _SS_RE.search(ss_text)
            if m:
                row_dict["SS Tracking Ratio (%)"] = float(m.group(1))
                row_dict["W_ss (rad/s)"] = float(m.group(2))
                row_dict["SS Window Start (s)"] = float(m.group(3))
                row_dict["SS Window End (s)"] = float(m.group(4))
                row_dict["Cmd W (rad/s)"] = float(m.group(5))
            else:
                warnings.append(
                    "Angular Tracking Ratio string did not match the expected "
                    "format; numeric columns left blank rather than guessed")

        tr_text = row_dict.get("Startup Transient (0-2s post-command)")
        if isinstance(tr_text, str) and not tr_text.upper().startswith("N/A"):
            m = _TRANSIENT_RE.search(tr_text)
            if m:
                row_dict["Transient Mean|W| (rad/s)"] = float(m.group(1))
                row_dict["Transient Window Start (s)"] = float(m.group(2))
                row_dict["Transient Window End (s)"] = float(m.group(3))
            else:
                warnings.append(
                    "Startup Transient string did not match the expected "
                    "format; numeric columns left blank rather than guessed")

    # Generic pass, in two stages.
    #
    # A bare number ("14.82", "61.7 %") becomes a real numeric in place, so
    # Excel can filter, sort and chart it instead of treating it as text.
    #
    # A unit-bearing value ("1.973 m") gets a COMPANION column carrying the
    # number with the unit named in the column header, and the original
    # cell is left untouched. Stripping the unit in place would make
    # "1.973 m" and "1.973 rad" indistinguishable downstream, and the
    # original text is also the provenance record.
    for key in list(row_dict.keys()):
        if key in ("Run", "Condition"):
            continue
        num = _as_number(row_dict[key])
        if num is not None:
            row_dict[key] = num
            continue
        parsed = _value_with_unit(row_dict[key])
        if parsed is not None:
            value, unit = parsed
            row_dict[f"{key} [{unit}]"] = value

    return warnings


def parse_condition(folder_name, test_id):
    """Folder name with the leading test_NN_ prefix removed.

    Left as free text on purpose: the remainder encodes speed, distance and
    configuration tags whose convention differs per test, and the commanded
    V/W are recovered from the bag contents rather than from the name.
    """
    prefix = f"test_{test_id:02d}_"
    lowered = folder_name.lower()
    if lowered.startswith(prefix):
        return folder_name[len(prefix):]
    alt = f"test{test_id:02d}_"
    if lowered.startswith(alt):
        return folder_name[len(alt):]
    return folder_name


def compute_generic_row(folder_name, folder_path, test_id):
    """One generic KPI row plus warnings and any stall flag.

    Raises on an unreadable bag so the caller can record it in
    Skipped_Failed; it must never return a partially filled row that would
    read as a successful analysis.
    """
    rows, _bag = compute_test_kpis(folder_path, test_id)
    row_dict = {"Run": folder_name, "Condition": parse_condition(folder_name, test_id)}
    stall_flags = []
    for metric, value, _param in rows:
        row_dict[metric] = value
        if isinstance(metric, str) and metric.startswith("⚠ Publish-Rate Stall"):
            stall_flags.append(metric.split("(")[-1].rstrip(")"))
    warnings = add_typed_columns(row_dict, test_id)
    return row_dict, warnings, " | ".join(stall_flags)


# =====================================================================
# SECTION 4 -- Test04 DEEP ANALYSIS (adapter over v12)
#
# The deep KPIs, breakaway/steady-state windowing, slip, contact force,
# split detection and branch assignment all remain in
# cobraflex_test04_analyzer_v12, driven here via its run_pipeline() entry
# point, so there is exactly one implementation of each Test04 quantity and
# the older workbooks stay reproducible.
# =====================================================================

def deep_module_version():
    return "Test04 engine, in-file (lineage: cobraflex_test04_analyzer v12)"


def apply_config(config):
    """Push the handover configuration into the deep module's constants.

    Only settings that the deep module actually consumes are pushed. The
    geometry values are applied to the two constants the Test04 kinematics
    read; a mismatch between these and the USD model shifts every slip
    figure silently, which is why they are also written into the
    AnalysisConfig sheet.
    """
    geom = config.geometry
    WHEEL_RADIUS_M = float(geom["wheel_radius_m"])
    WHEEL_SEPARATION_M = float(geom["geometric_track_m"])
    HALF_TRACK_M = WHEEL_SEPARATION_M / 2.0

    WHEEL_NAME_MAP = {config.joint_names[w]: w
                           for w in ("FL", "FR", "RL", "RR")}

    set_corner_load_comparison(
        config.corner_load_values_g if config.corner_load_enabled else None,
        config.corner_load_source)


def run_deep(root, out_path, bag_dirs, config, use_legacy=False,
             baseline_centers=None, quiet=False):
    """Run the Test04 deep analysis over an already-filtered bag list.

    `bag_dirs` comes from the handover router, which has already resolved and
    filtered test ids. Passing it in rather than letting the deep module
    rescan keeps a single detection result for the whole run -- otherwise
    the two layers could disagree about which bags are Test04.
    """
    apply_config(config)
    return run_pipeline(root, out_path, use_legacy=use_legacy,
                             baseline_centers=baseline_centers,
                             bag_dirs=bag_dirs, quiet=quiet)


def read_cmd_profile(bag_path):
    """Minimal command-channel read used for test-id inference.

    Returns the same key names the core inference helper expects. Reads the
    bag through the deep module's own loader so the inference cannot
    disagree with the analysis about what the bag contains.
    """
    data = load_bag(bag_path)
    return {"cmdvel_wz": data.get("cmdvel_wz", []),
            "cmdvel_linear_x": data.get("cmdvel_linear_x", [])}


def load_baseline_centers_from_json(path):
    """Baseline branch centres from a JSON file, format
    {"0.8": {"Low": 0.340, "High": 0.501}}.

    Centres are scoped to a specific solver and commanded W by the deep
    module; this only loads them.
    """
    import json
    with open(path, encoding="utf-8") as fh:
        raw = json.load(fh)
    merged = dict(BASELINE_CENTERS_DEFAULT)
    for key, centers in raw.items():
        merged[float(key)] = {str(k): float(v) for k, v in centers.items()}
    return merged


# =====================================================================
# SECTION 5a -- WORKBOOK SCHEMA
#
# No KPI is computed here. This takes the sheets the analysis sections
# produced and enforces one stable layout, so a reader always finds the
# same sheet names in the same roles regardless of which bags were present.
#
# Two naming problems from the older tools are resolved:
#
#   * The deep analysis wrote its per-run table to a sheet called `Index`
#     while the generic per-run table went to `V25_Index`, so `Index` meant
#     different things depending on which tool produced the file. Here
#     `Index` is always the one-row-per-bag routing table and the deep
#     per-run table is `Test04_Deep`.
#   * Per-wheel Test04 values were only available as ~30 columns spread
#     across the deep table. `Test04_Wheels` reshapes them to one row per
#     (run, wheel). It is a reshape of existing numbers, not a
#     recomputation.
# =====================================================================

EXCEL_MAX_ROWS = 1_048_576
SHEET_NAME_LIMIT = 31

FIXED_SHEETS = [
    "Index",
    *[f"Test{i:02d}" for i in range(1, 11)],
    "Test04_Deep", "Test04_Wheels", "WheelCmdVsActual", "SlipSummary",
    "ContactForceSummary", "SplitDetection", "ModeSummary", "MaxForceSummary",
    "DampingSummary", "SolverSummary", "DivergenceOnset", "DataQuality",
    "ParsingWarnings", "Skipped_Failed", "AnalysisConfig",
]

# Per-wheel deep columns, as (column template, output label).
_WHEEL_COLUMN_TEMPLATES = [
    ("Cmd Steady-State {w} (rad/s)", "Target Mean (rad/s)"),
    ("Actual Steady-State {w} (rad/s)", "Actual Mean (rad/s)"),
    ("Wheel Tracking Ratio {w} (%)", "Tracking Ratio (%)"),
    ("Steady-State Std {w} (rad/s)", "Steady-State Std (rad/s)"),
    ("Peak-to-Peak {w} (rad/s)", "Peak-to-Peak (rad/s)"),
    ("Tracking Error Std {w} (rad/s)", "Tracking Error Std (rad/s)"),
    ("Mean-Crossing Count {w} (per window)", "Mean-Crossing Count"),
    ("Contact Force Steady-State {w} (N)", "Contact Force Mean (N)"),
    ("Contact Force Window Samples {w}", "Contact Force Window Samples"),
    ("Contact Force Publish Rate {w} (Hz)", "Contact Force Rate (Hz)"),
    ("Contact Force Window Completeness {w}", "Contact Force Completeness"),
    ("Contact Force Share {w} (%)", "Contact Force Share (%)"),
]


def build_test04_wheels(deep_df):
    """One row per (run, wheel) from the deep per-run table.

    Signed and absolute error are derived from the target/actual pair that
    is already in the table. RMSE over the window is not recoverable from
    per-run means, so it is left out entirely rather than approximated by
    something that would be read as an RMSE.
    """
    if deep_df is None or deep_df.empty:
        return pd.DataFrame([{"Note": "No Test04 runs analysed"}])

    rows = []
    for _, run in deep_df.iterrows():
        for wheel in WHEELS:
            row = {"Run": run.get("Run"),
                   "Condition": run.get("Condition"),
                   "Commanded W (rad/s)": run.get("Commanded W (rad/s)"),
                   "Solver": run.get("Solver"),
                   "Mode": run.get("Mode"),
                   "Wheel": wheel,
                   "Side": "Left" if wheel in ("FL", "RL") else "Right",
                   "Axle": "Front" if wheel.startswith("F") else "Rear"}
            for template, label in _WHEEL_COLUMN_TEMPLATES:
                col = template.format(w=wheel)
                if col in deep_df.columns:
                    row[label] = run.get(col)
            target = row.get("Target Mean (rad/s)")
            actual = row.get("Actual Mean (rad/s)")
            if isinstance(target, (int, float)) and isinstance(actual, (int, float)):
                if target == target and actual == actual:
                    row["Signed Tracking Error (rad/s)"] = actual - target
                    row["Absolute Tracking Error (rad/s)"] = abs(actual - target)
            rows.append(row)
    return pd.DataFrame(rows)


def _safe_sheet_name(name, taken):
    name = re.sub(r"[\[\]:*?/\\]", "_", str(name))[:SHEET_NAME_LIMIT]
    if name not in taken:
        return name
    stem = name[:SHEET_NAME_LIMIT - 3]
    for i in range(2, 100):
        candidate = f"{stem}_{i:02d}"
        if candidate not in taken:
            return candidate
    raise ValueError(f"cannot find a free sheet name for {name!r}")


def _split_oversized(df):
    """Split a frame that would exceed Excel's row limit into chunks."""
    limit = EXCEL_MAX_ROWS - 1
    if len(df) <= limit:
        return [df]
    return [df.iloc[i:i + limit] for i in range(0, len(df), limit)]


def write_workbook(out_path, sheets, sheet_order=None):
    """Write `sheets` (an ordered mapping of name -> DataFrame) and format it.

    Every fixed sheet is created even when it has no rows -- an absent
    sheet is indistinguishable from an analysis that was never attempted,
    whereas an explicit Note says the step ran and found nothing.
    """
    order = list(sheet_order or [])
    for name in sheets:
        if name not in order:
            order.append(name)

    taken = set()
    with pd.ExcelWriter(out_path, engine="openpyxl") as writer:
        for name in order:
            df = sheets.get(name)
            if df is None or (hasattr(df, "empty") and df.empty):
                df = pd.DataFrame([{"Note": f"No rows produced for {name}"}])
            chunks = _split_oversized(df)
            for i, chunk in enumerate(chunks):
                base = name if len(chunks) == 1 else f"{name}_{i + 1:02d}"
                final = _safe_sheet_name(base, taken)
                taken.add(final)
                chunk.to_excel(writer, sheet_name=final, index=False)

    _format_workbook(out_path)
    return verify_workbook(out_path)


def _format_workbook(path):
    wb = load_workbook(path)
    header_font = Font(name="Arial", bold=True, color="FFFFFF")
    header_fill = PatternFill("solid", start_color="1F4E78", end_color="1F4E78")
    warn_fill = PatternFill("solid", start_color="FCE4D6", end_color="FCE4D6")

    for sheet_name in wb.sheetnames:
        ws = wb[sheet_name]
        for cell in ws[1]:
            cell.font = header_font
            cell.fill = header_fill
            cell.alignment = Alignment(wrap_text=True, vertical="center")
        ws.freeze_panes = "A2"
        if ws.max_row > 1 and ws.max_column >= 1:
            ws.auto_filter.ref = ws.dimensions
        for col_cells in ws.columns:
            length = max((len(str(c.value)) if c.value is not None else 0)
                         for c in col_cells)
            letter = get_column_letter(col_cells[0].column)
            ws.column_dimensions[letter].width = min(max(length + 2, 10), 60)

        header = [c.value for c in ws[1]]
        # Warning columns are tinted so a problem is visible without        # reading every cell. Colour is decoration only: the same
        # information is always present as text in the cell itself.
        warn_cols = [i + 1 for i, h in enumerate(header)
                     if isinstance(h, str) and (h.startswith("⚠")
                                                or "Warning" in h
                                                or "Unparsed" in h)]
        for col_idx in warn_cols:
            for row in ws.iter_rows(min_row=2, min_col=col_idx, max_col=col_idx):
                for cell in row:
                    if cell.value not in (None, "", False):
                        cell.fill = warn_fill
    wb.save(path)


def verify_workbook(path):
    """Reopen the saved file and report what it contains.

    A workbook that pandas wrote but openpyxl cannot reopen is a silent
    delivery failure, so this runs on every write rather than on request.
    """
    wb = load_workbook(path)
    report = {"sheets": list(wb.sheetnames), "rows": {}}
    for name in wb.sheetnames:
        report["rows"][name] = wb[name].max_row - 1
        if len(name) > SHEET_NAME_LIMIT:
            raise ValueError(f"sheet name exceeds Excel's limit: {name!r}")
    missing = [s for s in FIXED_SHEETS if s not in wb.sheetnames]
    report["missing_fixed_sheets"] = missing
    return report


# =====================================================================
# SECTION 5b -- COMMAND LINE
# =====================================================================

EXIT_OK = 0
EXIT_PARTIAL = 1
EXIT_FATAL = 2

# Deep-analysis sheets carried through to the unified workbook, with the
# name they are published under. The deep module's own "Index" becomes
# "Test04_Deep": in the handover workbook, "Index" always means the routing table.
_DEEP_SHEET_MAP = {
    "Index": "Test04_Deep",
    "SplitDetection": "SplitDetection",
    "MaxForceSummary": "MaxForceSummary",
    "DampingSummary": "DampingSummary",
    "SolverSummary": "SolverSummary",
    "ModeSummary": "ModeSummary",
    "SlipSummary": "SlipSummary",
    "ContactForceSummary": "ContactForceSummary",
    "DivergenceOnset": "DivergenceOnset",
    "DataQuality": "DataQuality",
    "WheelCmdVsActual": "WheelCmdVsActual",
    "AnalysisConfig": "AnalysisConfig",
}


def build_parser():
    p = argparse.ArgumentParser(
        prog="cobraflex_analyzer.py",
        description="Unified CobraFlex rosbag analyzer (Test01-Test10 plus "
                    "Test04 deep analysis).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""examples:
  # everything under a batch folder, deep analysis automatic for Test04
  python3 cobraflex_analyzer.py bags/0804 --output results_0804.xlsx

  # only Test03 and Test04, with a project configuration
  python3 cobraflex_analyzer.py bags/0804 -o out.xlsx --test 3,4 \\
      --config cobraflex_analysis.yaml

  # generic KPIs only, no Test04 deep pass
  python3 cobraflex_analyzer.py bags/0804 -o out.xlsx --no-deep
""")
    p.add_argument("bag_root", metavar="BAG_ROOT", nargs="?",
                   help="directory containing rosbag2 folders (searched recursively)")
    p.add_argument("-o", "--output", default="cobraflex_results.xlsx",
                   help="output .xlsx path (default: %(default)s)")
    p.add_argument("-c", "--config", default=None,
                   help="optional YAML/JSON configuration file")
    p.add_argument("--test", default=None,
                   help="restrict to these test ids, e.g. --test 4 or --test 1,3,4")
    p.add_argument("--no-deep", action="store_true",
                   help="skip the Test04 deep analysis even when Test04 bags are present")
    p.add_argument("--no-recursive", action="store_true",
                   help="only scan the immediate children of BAG_ROOT")
    p.add_argument("--legacy-centers", action="store_true",
                   help="force the fixed W=0.8 branch centres (valid at W=0.8 only)")
    p.add_argument("--baseline-json", default=None,
                   help="JSON file of baseline branch centres, e.g. "
                        '{"0.8": {"Low": 0.340, "High": 0.501}}')
    p.add_argument("--overwrite", action="store_true",
                   help="overwrite the output file if it already exists")
    p.add_argument("-q", "--quiet", action="store_true",
                   help="only report warnings, errors and the final summary")
    p.add_argument("--write-example-config", metavar="PATH", default=None,
                   help="write a documented example configuration file and exit")
    p.add_argument("--version", action="version",
                   version=f"cobraflex_analyzer {ANALYZER_VERSION}")
    return p


def parse_test_filter(text):
    if not text:
        return None
    ids = set()
    for chunk in str(text).replace(" ", "").split(","):
        if not chunk:
            continue
        if not chunk.isdigit():
            raise ValueError(f"--test expects integers, got {chunk!r}")
        value = int(chunk)
        if value not in TEST_IDS:
            raise ValueError(f"--test id out of range (1-10): {value}")
        ids.add(value)
    return ids or None


def main(argv=None):
    args = build_parser().parse_args(argv)

    def say(*a):
        if not args.quiet:
            print(*a)

    if args.write_example_config:
        path = os.path.abspath(args.write_example_config)
        if os.path.exists(path):
            print(f"ERROR: {path} already exists; refusing to overwrite.",
                  file=sys.stderr)
            return EXIT_FATAL
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(EXAMPLE_CONFIG_YAML)
        print(f"Wrote example configuration: {path}")
        return EXIT_OK

    # ---- configuration -------------------------------------------------
    try:
        config = load_config(args.config)
        test_filter = parse_test_filter(args.test)
    except (ConfigError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return EXIT_FATAL

    out_path = os.path.abspath(args.output)
    if os.path.exists(out_path) and not args.overwrite:
        print(f"ERROR: {out_path} already exists. Pass --overwrite to replace it.",
              file=sys.stderr)
        return EXIT_FATAL
    out_dir = os.path.dirname(out_path) or "."
    if not os.path.isdir(out_dir):
        print(f"ERROR: output directory does not exist: {out_dir}", file=sys.stderr)
        return EXIT_FATAL

    if not args.bag_root:
        print("ERROR: BAG_ROOT is required (omit it only with "
              "--write-example-config).", file=sys.stderr)
        return EXIT_FATAL
    if not os.path.isdir(args.bag_root):
        print(f"ERROR: BAG_ROOT is not a directory: {args.bag_root}", file=sys.stderr)
        return EXIT_FATAL

    # ---- discovery and routing -----------------------------------------
    bags = find_bag_dirs(args.bag_root, recursive=not args.no_recursive)
    if not bags:
        print(f"ERROR: no rosbag2 folders (with metadata.yaml) found under "
              f"{args.bag_root}", file=sys.stderr)
        return EXIT_FATAL

    say(f"Found {len(bags)} bag folder(s) under {args.bag_root}\n")

    index_rows, warning_rows, problem_rows = [], [], []
    routed = []
    for folder_name, folder_path in bags:
        def inference(path):
            return infer_test04_from_command_profile(path, read_cmd_profile)

        test_id, source, confidence = detect_test_id(
            folder_name, folder_path, identify_test_id, inference)

        row = {"Run": folder_name,
               "Test ID": test_id or "",
               "Test Name": TEST_NAMES.get(test_id, ""),
               "Condition": parse_condition(folder_name, test_id) if test_id else "",
               "Detection Source": source,
               "Detection Confidence": confidence,
               "Path": os.path.relpath(folder_path, args.bag_root)}

        if test_id == 0:
            row["Status"] = "Skipped"
            problem_rows.append({"Folder": folder_name, "Path": row["Path"],
                                 "Issue": "Unrecognized test_id",
                                 "Detail": "No sidecar run_metadata, no test_NN_ "
                                           "prefix, and no recognisable command "
                                           "profile. Rename the folder or add a "
                                           "sidecar; not guessed."})
            index_rows.append(row)
            say(f"  SKIP  {folder_name}  (unrecognized test_id)")
            continue

        if confidence == "low":
            warning_rows.append({
                "Run": folder_name,
                "Unparsed Fields": "test_id resolved by content inference",
                "Note": "The folder name did not identify the test. The id was "
                        "inferred from the command profile and is low confidence; "
                        "confirm before citing anything from this run."})

        if test_filter and test_id not in test_filter:
            row["Status"] = "Filtered out"
            index_rows.append(row)
            continue

        row["Status"] = "Analysed"
        index_rows.append(row)
        routed.append((folder_name, folder_path, test_id, row))

    if not routed:
        print("ERROR: no bags left to analyse after test-id detection and "
              "filtering.", file=sys.stderr)
        return EXIT_FATAL

    # ---- generic Test01-Test10 KPIs ------------------------------------
    per_test_rows = {tid: [] for tid in TEST_IDS}
    if not HAVE_GENERIC_ANALYZER:
        say(f"NOTE: cobraflex_rosbag_analyzer is not importable "
            f"({GENERIC_ANALYZER_IMPORT_ERROR}).")
        say("      Generic per-test sheets will be empty; the Test04 deep "
            "analysis is unaffected.\n")
    else:
        for folder_name, folder_path, test_id, row in routed:
            try:
                rd, warns, stall = compute_generic_row(
                    folder_name, folder_path, test_id)
            except Exception as exc:  # noqa: BLE001
                row["Status"] = "Generic KPI failed"
                row["Stall Warning"] = ""
                problem_rows.append({"Folder": folder_name,
                                     "Path": row["Path"],
                                     "Issue": "Generic KPI computation failed",
                                     "Detail": f"{type(exc).__name__}: {exc}"})
                say(f"  FAIL  {folder_name}  ({type(exc).__name__}: {exc})")
                continue
            per_test_rows[test_id].append(rd)
            row["Stall Warning"] = stall
            for w in warns:
                warning_rows.append({"Run": folder_name,
                                     "Unparsed Fields": "typed numeric extraction",
                                     "Note": w})
            say(f"  OK    {folder_name}  -> Test {test_id} "
                f"({TEST_NAMES.get(test_id, '')})")

    # ---- Test04 deep analysis ------------------------------------------
    deep_sheets = {}
    deep_bags = [(n, p) for n, p, tid, _ in routed if tid == 4]
    should_run_deep = (config.auto_deep_test04 and not args.no_deep
                       and bool(deep_bags))

    if should_run_deep:
        say(f"\nRunning Test04 deep analysis on {len(deep_bags)} bag(s)...")
        baseline_centers = None
        if args.baseline_json:
            try:
                baseline_centers = load_baseline_centers_from_json(
                    args.baseline_json)
            except Exception as exc:  # noqa: BLE001
                print(f"ERROR: could not read --baseline-json: {exc}", file=sys.stderr)
                return EXIT_FATAL
        tmp_dir = tempfile.mkdtemp(prefix="cobraflex_handover_")
        tmp_path = os.path.join(tmp_dir, "test04_deep.xlsx")
        try:
            run_deep(args.bag_root, tmp_path, deep_bags, config,
                                  use_legacy=args.legacy_centers,
                                  baseline_centers=baseline_centers,
                                  quiet=args.quiet)
            book = pd.read_excel(tmp_path, sheet_name=None)
            for src, dest in _DEEP_SHEET_MAP.items():
                if src in book:
                    deep_sheets[dest] = book[src]
            # Per-condition time-series sheets keep their generated names.
            if config.include_time_series:
                for name, frame in book.items():
                    if name.startswith(("BAWh_", "BABd_")):
                        deep_sheets[name] = frame
        except Exception as exc:  # noqa: BLE001
            problem_rows.append({"Folder": "-", "Path": "-",
                                 "Issue": "Test04 deep analysis failed",
                                 "Detail": f"{type(exc).__name__}: {exc}"})
            say("  Test04 deep analysis failed; generic sheets are still written.")
            if not args.quiet:
                traceback.print_exc()
        finally:
            for leftover in (tmp_path,):
                if os.path.exists(leftover):
                    os.remove(leftover)
            if os.path.isdir(tmp_dir):
                os.rmdir(tmp_dir)
    elif args.no_deep and deep_bags:
        say(f"\nSkipping Test04 deep analysis for {len(deep_bags)} bag(s) (--no-deep).")

    # ---- assemble the workbook -----------------------------------------
    sheets = {"Index": pd.DataFrame(index_rows)}
    for tid in TEST_IDS:
        rows = per_test_rows[tid]
        if rows:
            df = pd.DataFrame(rows)
            lead = [c for c in ("Run", "Condition") if c in df.columns]
            df = df[lead + [c for c in df.columns if c not in lead]]
            sheets[f"Test{tid:02d}"] = df

    deep_config = deep_sheets.pop("AnalysisConfig", None)
    sheets.update(deep_sheets)
    sheets["Test04_Wheels"] = build_test04_wheels(deep_sheets.get("Test04_Deep"))
    sheets["ParsingWarnings"] = _merge_frames(
        deep_sheets.get("ParsingWarnings"), pd.DataFrame(warning_rows))
    sheets["Skipped_Failed"] = pd.DataFrame(problem_rows)

    config_rows = list(config.to_rows())
    config_rows.append({"Parameter": "Generic KPI Module",
                        "Value": "cobraflex_rosbag_analyzer" if HAVE_GENERIC_ANALYZER
                                 else f"unavailable ({GENERIC_ANALYZER_IMPORT_ERROR})"})
    config_rows.append({"Parameter": "Test04 Deep Module",
                        "Value": deep_module_version()})
    config_rows.append({"Parameter": "Test04 Deep Analysis Run",
                        "Value": should_run_deep})
    config_rows.append({"Parameter": "Bag Root", "Value": os.path.abspath(args.bag_root)})
    config_rows.append({"Parameter": "Output Path", "Value": out_path})
    config_rows.append({"Parameter": "Test Filter",
                        "Value": ",".join(str(t) for t in sorted(test_filter))
                                 if test_filter else "none (all tests)"})
    if deep_config is not None:
        config_rows.extend(deep_config.to_dict("records"))
    sheets["AnalysisConfig"] = pd.DataFrame(config_rows)

    try:
        report = write_workbook(out_path, sheets, sheet_order=FIXED_SHEETS)
    except Exception as exc:  # noqa: BLE001
        print(f"ERROR: could not write {out_path}: {exc}", file=sys.stderr)
        return EXIT_FATAL

    analysed = sum(len(v) for v in per_test_rows.values())
    skipped = sum(1 for r in index_rows if r.get("Status") == "Skipped")
    filtered = sum(1 for r in index_rows if r.get("Status") == "Filtered out")
    failed = sum(1 for r in index_rows if r.get("Status") == "Generic KPI failed")

    say(f"\nSaved: {out_path}")
    say(f"  sheets written : {len(report['sheets'])}")
    say(f"  bags analysed  : {analysed}")
    say(f"  filtered out   : {filtered}")
    say(f"  skipped        : {skipped}")
    say(f"  failed         : {failed}")
    if report["missing_fixed_sheets"]:
        say(f"  NOTE: fixed sheets absent: {', '.join(report['missing_fixed_sheets'])}")

    return EXIT_PARTIAL if (skipped or failed) else EXIT_OK


def _merge_frames(a, b):
    frames = [f for f in (a, b) if f is not None and not f.empty]
    if not frames:
        return pd.DataFrame()
    return pd.concat(frames, ignore_index=True, sort=False)


if __name__ == "__main__":
    sys.exit(main())