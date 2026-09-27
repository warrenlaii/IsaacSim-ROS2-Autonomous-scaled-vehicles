#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Regression test suite for cobraflex_analyzer.py.

Two groups, both on synthetic data with known analytic answers so a failure
points at a formula or a rule rather than at one particular bag:

  PART A -- the Test04 deep engine (Section 0 of the analyzer, formerly
            cobraflex_test04_analyzer_v12.py). Covers the 2026-08-04 fixes:
            breakaway detection, steady-state windowing, rate completeness,
            and the signed vs absolute no-slip forms.

  PART B -- the integration layer. Covers configuration validation, bag
            discovery, test-id routing, typed numeric extraction and the
            workbook schema.

    python3 tests/test_cobraflex_analyzer.py
    python3 -m pytest tests/test_cobraflex_analyzer.py -q
"""
import json
import os
import sys
import tempfile

import numpy as np
import pandas as pd

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# A plain import, not importlib.spec_from_file_location(): the module
# defines dataclasses under `from __future__ import annotations`, whose
# field resolution needs the module registered in sys.modules.
import cobraflex_analyzer as analyzer

# Aliases kept so each assertion still names the concern it exercises.
core = xl = generic = cli = analyzer


# #####################################################################
# PART A -- Test04 deep engine
# #####################################################################

# ---------------------------------------------------------------------
# 1. Breakaway detection must fire on a genuine sustained crossing.
#    Regression test for the off-by-one sustain check, which rejected
#    every real crossing and fell through to (recording_end - anchor).
# ---------------------------------------------------------------------
def test_breakaway_fires_on_sustained_crossing():
    rate = 60.0
    t = np.arange(0.0, 10.0, 1.0 / rate)
    v = np.where(t < 5.0, 0.006, 1.65)          # stiction, then breakaway
    bt = analyzer.find_breakaway_time(list(t), list(v), anchor_time=4.5)
    assert abs(bt - 0.5) < 2.0 / rate, f"expected ~0.50 s, got {bt}"


def test_breakaway_rejects_single_noise_spike():
    rate = 60.0
    t = np.arange(0.0, 10.0, 1.0 / rate)
    v = np.full_like(t, 0.006)
    v[120] = 5.0                                 # one isolated sample
    v[t >= 7.0] = 1.65                           # the real crossing
    bt = analyzer.find_breakaway_time(list(t), list(v), anchor_time=0.0)
    assert abs(bt - 7.0) < 2.0 / rate, f"expected ~7.0 s, got {bt}"


def test_breakaway_is_nan_when_wheels_never_move():
    t = np.arange(0.0, 10.0, 1.0 / 60.0)
    v = np.full_like(t, 0.006)
    bt = analyzer.find_breakaway_time(list(t), list(v), anchor_time=0.0)
    assert bt != bt, f"expected NaN, got {bt}"


# ---------------------------------------------------------------------
# 2. Steady-state window must be non-empty for a well-formed run, and
#    must reject a window that is too short or too sparse.
# ---------------------------------------------------------------------
def test_steady_state_window_selected_after_settle():
    rate = 60.0
    t = np.arange(0.0, 12.0, 1.0 / rate)
    v = np.where(t < 5.0, 0.0, 1.65)
    info = analyzer.breakaway_anchored_window_info(
        list(t), list(v), anchor_time=4.5, breakaway_time=0.5)
    assert info["valid"], info
    assert abs(info["start_s"] - 7.0) < 2.0 / rate
    assert info["n_samples"] > 250


def test_window_rejected_when_shorter_than_duration_floor():
    rate = 60.0
    t = np.arange(0.0, 7.2, 1.0 / rate)          # only 0.2 s past settle
    v = np.full_like(t, 1.65)
    info = analyzer.breakaway_anchored_window_info(
        list(t), list(v), anchor_time=4.5, breakaway_time=0.5)
    assert not info["valid"], info


# ---------------------------------------------------------------------
# 3. Rate completeness must scale with the topic's own publish rate, and
#    must catch a dropout inside an otherwise long-enough window.
# ---------------------------------------------------------------------
def test_completeness_is_one_for_a_gapless_series():
    t = np.arange(0.0, 10.0, 1.0 / 13.0)
    c = analyzer.window_rate_completeness(t, n_in_window=len(t),
                                     window_length_s=t[-1] - t[0])
    assert abs(c - 1.0) < 0.02, c


def test_completeness_detects_a_dropout():
    t = np.concatenate([np.arange(0.0, 4.0, 1.0 / 13.0),
                        np.arange(8.0, 10.0, 1.0 / 13.0)])
    c = analyzer.window_rate_completeness(t, n_in_window=len(t),
                                     window_length_s=t[-1] - t[0])
    assert c < analyzer.WINDOW_MIN_RATE_COMPLETENESS, c


def test_completeness_is_rate_relative_not_count_based():
    """A 13 Hz topic and a 60 Hz topic covering the same window must both
    score ~1.0, which a fixed sample-count floor cannot express."""
    for rate in (13.0, 60.0):
        t = np.arange(0.0, 8.0, 1.0 / rate)
        c = analyzer.window_rate_completeness(t, len(t), t[-1] - t[0])
        assert abs(c - 1.0) < 0.02, (rate, c)


def test_median_rate_ignores_one_long_stall():
    t = np.concatenate([np.arange(0.0, 5.0, 1.0 / 60.0),
                        np.arange(9.0, 14.0, 1.0 / 60.0)])
    assert abs(analyzer.median_publish_rate(t) - 60.0) < 1.0


# ---------------------------------------------------------------------
# 4. Signed vs absolute no-slip yaw.
# ---------------------------------------------------------------------
def _no_slip_signed(w_fl, w_rl, w_fr, w_rr):
    wl = (w_fl + w_rl) / 2.0
    wr = (w_fr + w_rr) / 2.0
    return analyzer.WHEEL_RADIUS_M * (wr - wl) / analyzer.WHEEL_SEPARATION_M


def test_signed_and_absolute_agree_under_expected_sign_pattern():
    """Counter-rotating wheels: the two forms are algebraically identical,
    which is why the steady-state KPI was deliberately left unchanged."""
    w = 1.6537
    signed = _no_slip_signed(-w, -w, w, w)
    absolute = analyzer.wheel_omega_to_body_rate(w)
    assert abs(signed - absolute) < 1e-12, (signed, absolute)


def test_signed_and_absolute_diverge_when_one_wheel_reverses():
    """FR momentarily reversing: the absolute form folds the excursion back
    onto the positive side and overstates the no-slip prediction."""
    signed = _no_slip_signed(-1.65, -1.65, -3.27, 1.65)
    absolute = analyzer.wheel_omega_to_body_rate(
        (1.65 + 1.65 + 3.27 + 1.65) / 4.0)
    assert abs(signed) < abs(absolute)
    assert abs(abs(signed) - abs(absolute)) > 0.5


def test_signed_form_is_near_zero_when_both_sides_co_rotate():
    """Both sides turning the same way is not in-place rotation at all; the
    signed form reports ~0 yaw, the absolute form reports a large one."""
    signed = _no_slip_signed(1.65, 1.65, 1.65, 1.65)
    absolute = analyzer.wheel_omega_to_body_rate(1.65)
    assert abs(signed) < 1e-9
    assert absolute > 0.5


# ---------------------------------------------------------------------
# 5. Geometry constants must stay in sync with the handover spec.
# ---------------------------------------------------------------------
def test_geometry_constants_match_handover_spec():
    assert analyzer.WHEEL_RADIUS_M == 0.03725
    assert analyzer.WHEEL_SEPARATION_M == 0.153
    assert abs(analyzer.HALF_TRACK_M - 0.0765) < 1e-12


# ---------------------------------------------------------------------
# 6. Max Drive Force token parsing (legacy spellings must keep working).
# ---------------------------------------------------------------------
def test_max_force_tokens():
    cases = {
        "test_04_w08_vMaxDriveForce0_rep1of1": 0.0,
        "test_04_w08_vMaxDriveForce008_rep1of1": 0.08,
        "test_04_w08_vMaxDriveForce0125_rep1of1": 0.125,
        "test_04_w08_vMaxDriveForce025_rep1of1": 0.25,
        "test_04_w08_vMaxDriveForce180_rep1of1": 1.8,
    }
    for folder, expected in cases.items():
        got = analyzer.max_force_from_folder(folder)
        assert abs(got - expected) < 1e-9, (folder, got, expected)


def test_unknown_force_token_is_nan_not_zero():
    got = analyzer.max_force_from_folder("test_04_w08_vinitalsetting_3_rep1of1")
    assert got != got, f"expected NaN for an unparsed token, got {got}"


# #####################################################################
# PART B -- integration layer
# #####################################################################

# ---------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------
def test_defaults_are_valid_without_a_config_file():
    cfg = core.load_config(None)
    assert cfg.geometry["wheel_radius_m"] == 0.03725
    assert cfg.corner_load_enabled is False


def test_three_geometry_fields_exist_separately():
    cfg = core.load_config(None)
    for key in ("controller_wheel_distance_m", "geometric_track_m", "wheelbase_m"):
        assert key in cfg.geometry, key


def test_final_handover_geometry_values():
    geometry = core.load_config(None).geometry
    assert geometry["controller_wheel_distance_m"] == 0.153
    assert geometry["geometric_track_m"] == 0.153
    assert geometry["wheelbase_m"] == 0.154


def test_geometry_fields_are_reported_with_distinct_purposes():
    rows = core.load_config(None).to_rows()
    purposes = {r["Parameter"]: r.get("Purpose", "") for r in rows}
    a = purposes["geometry.controller_wheel_distance_m"]
    b = purposes["geometry.geometric_track_m"]
    assert a and b and a != b, "the two track fields must not share one description"


def test_track_is_documented_as_measured():
    rows = core.load_config(None).to_rows()
    flag = [r for r in rows
            if r["Parameter"] == "geometry.physical_track_independently_measured"]
    assert flag and flag[0]["Value"] is True


def test_negative_wheel_radius_is_fatal():
    cfg = core.AnalysisConfig()
    cfg.geometry["wheel_radius_m"] = -0.1
    try:
        cfg.validate()
    except core.ConfigError:
        return
    raise AssertionError("a negative wheel radius must be rejected, not warned about")


def test_incomplete_joint_map_is_fatal():
    cfg = core.AnalysisConfig()
    cfg.joint_names.pop("RR")
    try:
        cfg.validate()
    except core.ConfigError:
        return
    raise AssertionError("a three-wheel joint map must be rejected")


def test_partial_corner_load_is_fatal():
    cfg = core.AnalysisConfig()
    cfg.corner_load_enabled = True
    cfg.corner_load_values_g = {"FL": 1054, "FR": 1290, "RL": 1007}
    try:
        cfg.validate()
    except core.ConfigError:
        return
    raise AssertionError("three of four corners must be rejected")


def test_unknown_config_keys_are_reported_not_fatal():
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "cfg.json")
        with open(path, "w", encoding="utf-8") as fh:
            json.dump({"analysis": {"auto_deep_test04": False},
                       "no_such_section": {"x": 1}}, fh)
        cfg = core.load_config(path)
    assert cfg.auto_deep_test04 is False
    assert "no_such_section" in cfg.unknown_keys
    assert any("UNKNOWN CONFIG KEY" in r["Parameter"] for r in cfg.to_rows())


# ---------------------------------------------------------------------
# Discovery and routing
# ---------------------------------------------------------------------
def _make_bag(root, name):
    path = os.path.join(root, name)
    os.makedirs(path, exist_ok=True)
    open(os.path.join(path, "metadata.yaml"), "w").close()
    return path


def test_discovery_is_recursive():
    with tempfile.TemporaryDirectory() as d:
        _make_bag(os.path.join(d, "day1"), "test_04_w08_rep1of1")
        _make_bag(os.path.join(d, "day2", "morning"), "test_01_vel02_rep1of1")
        found = core.find_bag_dirs(d)
    assert len(found) == 2, found


def test_discovery_does_not_descend_into_a_bag():
    with tempfile.TemporaryDirectory() as d:
        bag = _make_bag(d, "test_04_w08_rep1of1")
        os.makedirs(os.path.join(bag, "inner"))
        open(os.path.join(bag, "inner", "metadata.yaml"), "w").close()
        found = core.find_bag_dirs(d)
    assert len(found) == 1, found


def test_sidecar_beats_folder_name():
    with tempfile.TemporaryDirectory() as d:
        bag = _make_bag(d, "test_01_mislabelled")
        with open(os.path.join(bag, "run_metadata.json"), "w", encoding="utf-8") as fh:
            json.dump({"test_id": 4}, fh)
        tid, source, conf = core.detect_test_id(
            "test_01_mislabelled", bag, generic.identify_test_id)
    assert (tid, source, conf) == (4, "sidecar", "high")


def test_unrecognised_name_is_not_guessed():
    with tempfile.TemporaryDirectory() as d:
        bag = _make_bag(d, "random_folder")
        tid, source, _ = core.detect_test_id("random_folder", bag,
                                             generic.identify_test_id)
    assert tid == 0 and source == "unknown"


def test_inference_is_marked_low_confidence():
    def infer(_path):
        return 4
    with tempfile.TemporaryDirectory() as d:
        bag = _make_bag(d, "random_folder")
        tid, source, conf = core.detect_test_id("random_folder", bag,
                                                generic.identify_test_id, infer)
    assert (tid, source, conf) == (4, "bag_inference", "low")


def test_inference_rejects_an_arc_profile():
    def reader(_path):
        return {"cmdvel_wz": [0.8] * 200, "cmdvel_linear_x": [0.2] * 200}
    assert core.infer_test04_from_command_profile("x", reader) is None


def test_inference_accepts_in_place_rotation():
    def reader(_path):
        return {"cmdvel_wz": [0.8] * 200, "cmdvel_linear_x": [0.0] * 200}
    assert core.infer_test04_from_command_profile("x", reader) == 4


def test_test_filter_parsing():
    assert cli.parse_test_filter("1,3,4") == {1, 3, 4}
    assert cli.parse_test_filter(None) is None
    for bad in ("0", "11", "four"):
        try:
            cli.parse_test_filter(bad)
        except ValueError:
            continue
        raise AssertionError(f"--test {bad} should have been rejected")


# ---------------------------------------------------------------------
# Typed numeric extraction
# ---------------------------------------------------------------------
def test_bare_numbers_are_converted_in_place():
    row = {"Run": "r", "Condition": "c", "Ratio": "61.7 %", "Count": "8"}
    generic.add_typed_columns(row, test_id=4)
    assert row["Ratio"] == 61.7 and row["Count"] == 8.0


def test_unit_bearing_values_get_a_companion_column():
    row = {"Run": "r", "Condition": "c", "Total Distance Traveled": "1.973 m"}
    generic.add_typed_columns(row, test_id=3)
    assert row["Total Distance Traveled"] == "1.973 m", "original must be kept"
    assert row["Total Distance Traveled [m]"] == 1.973


def test_prose_starting_with_a_digit_is_left_alone():
    row = {"Run": "r", "Condition": "c",
           "Odometry Duplicate Check": "0/889 moving steps (0.0%) | PASS"}
    generic.add_typed_columns(row, test_id=1)
    assert isinstance(row["Odometry Duplicate Check"], str)
    assert not any(k.endswith("]") for k in row)


def test_na_is_never_converted():
    row = {"Run": "r", "Condition": "c", "X": "N/A -- no active command window"}
    generic.add_typed_columns(row, test_id=4)
    assert row["X"] == "N/A -- no active command window"


def test_test04_composite_string_is_split():
    row = {"Run": "r", "Condition": "c",
           "Angular Tracking Ratio (Steady-State 2-9s)":
               "61.7 % (W_ss=0.494 rad/s over [2.32s,9.32s], cmd=0.800 rad/s)"}
    warnings = generic.add_typed_columns(row, test_id=4)
    assert not warnings
    assert row["SS Tracking Ratio (%)"] == 61.7
    assert row["W_ss (rad/s)"] == 0.494
    assert row["Cmd W (rad/s)"] == 0.8


def test_format_drift_warns_and_leaves_cells_blank():
    row = {"Run": "r", "Condition": "c",
           "Angular Tracking Ratio (Steady-State 2-9s)": "ratio was 61.7 percent"}
    warnings = generic.add_typed_columns(row, test_id=4)
    assert warnings, "a format change must warn"
    assert "SS Tracking Ratio (%)" not in row


# ---------------------------------------------------------------------
# Workbook schema
# ---------------------------------------------------------------------
def test_test04_wheels_reshape():
    deep = pd.DataFrame([{
        "Run": "r1", "Condition": "w08", "Commanded W (rad/s)": 0.8,
        "Solver": "PGS", "Mode": "Unclassified",
        "Cmd Steady-State FL (rad/s)": -1.6537,
        "Actual Steady-State FL (rad/s)": -1.6005,
        "Wheel Tracking Ratio FL (%)": 96.8,
        "Cmd Steady-State FR (rad/s)": 1.6537,
        "Actual Steady-State FR (rad/s)": 1.7109,
        "Wheel Tracking Ratio FR (%)": 103.5,
    }])
    out = xl.build_test04_wheels(deep)
    assert len(out) == 4, "one row per wheel"
    fl = out[out["Wheel"] == "FL"].iloc[0]
    assert fl["Side"] == "Left" and fl["Axle"] == "Front"
    assert abs(fl["Signed Tracking Error (rad/s)"] - 0.0532) < 1e-9
    assert abs(fl["Absolute Tracking Error (rad/s)"] - 0.0532) < 1e-9
    fr = out[out["Wheel"] == "FR"].iloc[0]
    assert fr["Side"] == "Right"


def test_test04_wheels_handles_no_runs():
    out = xl.build_test04_wheels(pd.DataFrame())
    assert len(out) == 1 and "Note" in out.columns


def test_fixed_sheets_are_all_created_even_when_empty():
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "out.xlsx")
        report = xl.write_workbook(path, {"Index": pd.DataFrame([{"Run": "r"}])},
                                   sheet_order=xl.FIXED_SHEETS)
    assert not report["missing_fixed_sheets"], report["missing_fixed_sheets"]


def test_index_is_the_routing_table_not_the_deep_table():
    assert xl.FIXED_SHEETS[0] == "Index"
    assert "Test04_Deep" in xl.FIXED_SHEETS
    assert "V25_Index" not in xl.FIXED_SHEETS


def test_sheet_names_stay_within_the_excel_limit():
    long_name = "BAWh_" + "x" * 60
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "out.xlsx")
        report = xl.write_workbook(path, {long_name: pd.DataFrame([{"a": 1}])})
    assert all(len(s) <= 31 for s in report["sheets"])


def test_duplicate_truncated_names_do_not_collide():
    a = "BAWh_" + "y" * 40 + "_A"
    b = "BAWh_" + "y" * 40 + "_B"
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "out.xlsx")
        report = xl.write_workbook(path, {a: pd.DataFrame([{"v": 1}]),
                                          b: pd.DataFrame([{"v": 2}])})
    assert len(set(report["sheets"])) == len(report["sheets"])

# ---------------------------------------------------------------------
# Example configuration emitted by --write-example-config
# ---------------------------------------------------------------------
def test_example_config_is_loadable():
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "example.yaml")
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(analyzer.EXAMPLE_CONFIG_YAML)
        try:
            import yaml  # noqa: F401
        except ImportError:
            return
        cfg = core.load_config(path)
    assert cfg.geometry["wheel_radius_m"] == 0.03725
    assert cfg.corner_load_enabled is False


def test_example_config_documents_every_geometry_field():
    text = analyzer.EXAMPLE_CONFIG_YAML
    for key in ("wheel_radius_m", "controller_wheel_distance_m",
                "geometric_track_m", "wheelbase_m",
                "physical_track_independently_measured"):
        assert key in text, key


if __name__ == "__main__":
    failures = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print(f"  PASS  {name}")
            except AssertionError as exc:
                failures += 1
                print(f"  FAIL  {name}: {exc}")
    print(f"\n{failures} failure(s)")
    raise SystemExit(1 if failures else 0)