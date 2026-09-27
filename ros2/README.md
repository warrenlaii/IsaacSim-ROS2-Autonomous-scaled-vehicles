# ROS 2 Control and Analysis Tools

This directory contains the maintained Python tools used to run CobraFlex test profiles, record ROS 2 data, and analyse recorded rosbag2 datasets for the thesis handover.

## Structure

```text
ros2/
├── control/
│   └── cobraflex_test_control.py
├── analysis/
│   ├── cobraflex_rosbag_analyzer.py
│   ├── cobraflex_analyzer.py
│   └── tests/
│       └── test_cobraflex_analyzer.py
└── README.md
```

## Tools

### `control/cobraflex_test_control.py`

GUI-based ROS 2 test controller and recorder.

It publishes `/cmd_vel`, supports the thesis test profiles, records selected ROS 2 topics with rosbag2, uses an advancing `/clock` for simulation-timed runs, and includes recorder-readiness and post-recording coverage checks.

Run it from a sourced ROS 2 environment:

```bash
python3 ros2/control/cobraflex_test_control.py
```

### `analysis/cobraflex_rosbag_analyzer.py`

Interactive single-bag GUI analyzer and the shared generic Test01-Test10 KPI module.

The analyzer keeps simulation/header time separate from rosbag arrival time, supports real and simulated datasets, and provides the KPI functions imported by the unified analyzer.

```bash
python3 ros2/analysis/cobraflex_rosbag_analyzer.py
```

### `analysis/cobraflex_analyzer.py`

Recommended command-line entry point for batch analysis.

It recursively discovers rosbag2 folders, runs the generic Test01-Test10 KPIs, adds the deep Test04 analysis when applicable, and writes one Excel workbook.

```bash
python3 ros2/analysis/cobraflex_analyzer.py /path/to/bag_root --output results.xlsx
```

Useful options are available through:

```bash
python3 ros2/analysis/cobraflex_analyzer.py --help
```

### `analysis/tests/test_cobraflex_analyzer.py`

Synthetic regression tests for the unified analyzer. The tests cover Test04 windowing and breakaway logic, configuration validation, bag discovery and routing, typed KPI extraction, geometry constants, and workbook schema behavior.

Run from the analysis directory:

```bash
cd ros2/analysis
python3 -m pytest tests/test_cobraflex_analyzer.py -q
```

## Dependencies

The control tool requires a sourced ROS 2 environment with `rclpy` and the standard message packages used by the script. Its GUI also requires PyQt5 and Matplotlib.

The analysis tools require Python 3 with `rosbags`, NumPy, pandas, openpyxl, Matplotlib, and PyQt5. SciPy is optional for filtering, and PyYAML is required only when YAML configuration files are used.

Both the controller and generic analyzer can optionally import a colocated `cobraflex_topics.py`. If it is not present, the verified fallback topic definitions embedded in the scripts are used.

## Handover conventions

The maintained handover filenames are intentionally unversioned. Development-version provenance is retained inside the source headers and Git history.

The final thesis geometry used by the deep analyzer is:

- wheel radius: 0.03725 m
- wheel-centre separation for controller/analysis: 0.153 m
- wheelbase: 0.154 m

Historical analyzer versions used 0.154 m for the track-related calculation. Re-running historical bags with the handover analyzer can therefore produce a small change in track-dependent derived KPIs.

Raw rosbag2 recordings are not stored in this repository. Keep raw `.mcap`, `.db3`, and rosbag2 directories in the project or institutional archive and commit only scripts and processed handover results.
