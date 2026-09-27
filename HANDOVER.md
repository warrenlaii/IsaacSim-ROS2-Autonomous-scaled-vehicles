# CobraFlex Thesis Handover

This document is the operational handover for the frozen CobraFlex thesis baseline. It connects the repository assets into one practical workflow:

> **clone -> Git LFS -> open the integrated Isaac Sim scene -> verify ROS 2 -> run a test -> record rosbag2 data -> analyse the result**

The goal is to let a new researcher start from the delivered thesis baseline without reconstructing the original development workspace.

---

## 1. Handover scope

This repository delivers:

- the formal CobraFlex vehicle USD;
- the formal RoadRunner-derived environment USD;
- the integrated Isaac Sim scene;
- the final ROS 2 command/state interface;
- repeatable test-control and rosbag2 recording tools;
- offline analysis tools;
- processed validation results;
- RoadRunner/OpenDRIVE and lightweight lane-network handover assets.

It does **not** deliver:

- raw thesis rosbag2 datasets in Git;
- a trained reinforcement-learning policy;
- a complete RL training loop;
- a real-time hardware-in-the-loop connection;
- a claim of full dynamic equivalence between simulation and the physical vehicle.

The frozen simulation baseline is documented in `config/baseline.yaml`. Do not silently replace its parameters with diagnostic settings.

---

## 2. Reference workstation configuration

The thesis workstation used:

| Component | Thesis environment |
| --- | --- |
| Operating system | Ubuntu 24.04.4 LTS |
| ROS 2 | Jazzy Jalisco |
| NVIDIA Isaac Sim | 6.0.0 |
| Python | 3.12 host environment |
| Road authoring | MathWorks RoadRunner R2025b |
| Simulation physics | CPU dynamics, PGS, 240 Hz |
| State / TF / joint publication | 60 Hz through gate step 4 |

The physical CobraFlex branch used a different software stack. This handover procedure focuses on the **simulation workstation**.

---

## 3. Clone the repository and retrieve Git LFS assets

The formal USD files are stored with Git LFS.

Install and initialise Git LFS before using the repository:

```bash
git lfs install
git clone https://github.com/warrenlaii/IsaacSim-ROS2-Autonomous-scaled-vehicles.git
cd IsaacSim-ROS2-Autonomous-scaled-vehicles
git lfs pull
```

Confirm that the USD assets are registered with LFS:

```bash
git lfs ls-files
```

The formal thesis assets are:

```text
assets/vehicle/ADMIT14_cobraflex_baseline_v1.usd
assets/environment/ADMIT14_RoadRunner_Map_v1.usd
assets/scenes/ADMIT14_Integrated_Scene_v1.usd
```

### Important

If a USD file is only a small text file containing a Git LFS pointer, the LFS object has not been retrieved correctly. Run:

```bash
git lfs pull
```

again before opening Isaac Sim.

Do not overwrite the three `v1` USD files. Any future physics, geometry, material, sensor, or ROS 2 changes should be saved as a new version.

---

## 4. Confirm the frozen baseline before running

Review:

```text
config/baseline.yaml
config/ros2_topics.yaml
```

The principal frozen values are:

| Parameter | Thesis baseline |
| --- | ---: |
| Isaac Sim | 6.0.0 |
| Physics rate | 240 Hz |
| GPU Dynamics | OFF |
| Solver | PGS |
| Broadphase | GPU |
| Articulation iterations | 32 position / 1 velocity |
| Vehicle mass | 3.5 kg |
| Wheelbase | 0.154 m |
| Wheel-centre separation | 0.153 m |
| Wheel radius | 0.03725 m |
| Joint drive | Force |
| Joint stiffness | 0 |
| Joint damping | 10,000 |
| Max drive force | 1.8 N.m per wheel |
| Rigid-body angular damping | 0.05 |
| Wheel / ground friction | static 0.5, dynamic 0.4, Multiply |
| State / TF / joint rate | 60 Hz |

> **Diagnostic warning:** the 0.08 N.m Max Drive Force condition was an experimental ablation. It is not the delivered thesis baseline.

---

## 5. Start the ROS 2 workstation environment

On the simulation workstation, source ROS 2 Jazzy:

```bash
source /opt/ros/jazzy/setup.bash
```

Confirm that ROS 2 tooling is available:

```bash
ros2 --help
```

Keep this terminal available for topic inspection and command-line checks.

---

## 6. Open the integrated Isaac Sim scene

Start **NVIDIA Isaac Sim 6.0.0** and open:

```text
assets/scenes/ADMIT14_Integrated_Scene_v1.usd
```

Use the integrated scene for the normal handover workflow. The separate vehicle and environment USDs are provided for inspection and future development.

Before pressing Play:

1. confirm that the stage loads without an unresolved-file error;
2. confirm that the CobraFlex vehicle is present on the road surface;
3. confirm that the vehicle and environment are the `v1` assets;
4. ensure the Isaac Sim ROS 2 Bridge required by the scene is enabled;
5. do not edit the formal `v1` files in place.

Then press **Play**.

### If the scene opens with missing referenced assets

Do not repair the formal thesis USD by silently changing paths and saving over `v1`.

Instead:

1. identify the unresolved asset;
2. document the dependency;
3. create a new version if a path/reference repair is required;
4. rerun the acceptance checks below.

---

## 7. Verify the ROS 2 interface

In the sourced ROS 2 terminal:

```bash
ros2 topic list
```

At minimum, the simulation handover contract expects:

```text
/cmd_vel
/clock
/odom_truth
/joint_states
/tf
/scan
/cobraflex/wheel_cmd_debug
```

The wheel-command debug topic is diagnostic only and is not part of the control path.

The verified Lane Camera topics are:

```text
/camera/image_raw_lane
/camera/camera_info
```

Use `ros2 topic list` to confirm that they are present in the delivered runtime before collecting a new dataset.

### Check publication rates

With the simulation running:

```bash
ros2 topic hz /clock
ros2 topic hz /odom_truth
ros2 topic hz /joint_states
ros2 topic hz /tf
```

Expected simulation-time architecture:

- `/clock`: approximately **240 Hz**;
- `/odom_truth`: approximately **60 Hz**;
- `/joint_states`: approximately **60 Hz**;
- dynamic TF: approximately **60 Hz**.

The final Lane Camera acceptance recording contained valid 640 x 360 RGB8 Image/CameraInfo pairs at 60 Hz in simulation time. Wall-arrival rate can be lower under rendering and recording load and should not be interpreted as the simulation-time publication rate.

---

## 8. Basic command smoke test

Before running a full recorded profile, verify that the vehicle responds to `/cmd_vel`.

Straight command:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.20}, angular: {z: 0.0}}" -r 10
```

Stop the publisher with **Ctrl+C**, then send a zero command:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0}, angular: {z: 0.0}}" -1
```

Confirm that:

- the vehicle moves forward;
- `/odom_truth` changes;
- wheel joints update;
- TF updates;
- the vehicle stops after the zero command.

This is only a communication smoke test. It is not a thesis validation run.

---

## 9. Run a recorded test with the CobraFlex controller

The maintained controller is:

```text
ros2/control/cobraflex_test_control.py
```

Its detailed test definitions and GUI parameter meanings are documented in:

```text
ros2/control/README.md
```

Run the controller from the repository root in a sourced ROS 2 environment:

```bash
python3 ros2/control/cobraflex_test_control.py
```

The controller requires ROS 2 Python packages plus PyQt5 and Matplotlib.

### Recommended first recorded smoke test

Use a simple straight-line profile before running the full thesis matrix:

- **Test Profile:** Test 1 - Acceleration Test
- **Target Linear Vel:** 0.20 m/s
- **Target Angular Vel:** not used
- **Profile Duration:** choose a short smoke-test duration, or 10 s when reproducing the thesis Test 01 condition
- **Repeats:** 1
- **Profile Timing Source:** Simulation
- **Telemetry topics:** keep the required/core topics selected

The controller manages the command sequence and rosbag2 recording.

For repeat-enabled tests, the implemented workflow is:

```text
settle
  -> start one rosbag2 directory for the repeat
  -> 2.0 s recorded zero-command pre-roll
  -> execute test profile
  -> stop and flush recorder
  -> check selected topics for zero-message streams
```

Tests 1, 2, 3, 4, 5, 8, and 10 support this per-repeat workflow.

### Thesis validation command set

The final thesis validation matrix included:

- **Test 01:** V = 0.20, 0.40, 0.53 m/s; W = 0; 10 s;
- **Test 03:** V = 0.20, 0.40, 0.53 m/s crossed with W = 0.20, 0.40, 0.60, 1.00 rad/s;
- **Test 04:** V = 0; W = 0.20, 0.40, 0.60, 0.80 rad/s; 10 s;
- **Test 07:** V = 0.53 m/s; W step from 0 to 0.20 rad/s; matched 5 s analysis window;
- **Test 09:** V = 0.20, 0.40, 0.53 m/s; W = 0.

Do not treat a new run as a thesis result unless the complete test condition and analysis definition match the original protocol.

---

## 10. Historical data provenance and maintained scripts

The processed thesis results in `validation/` are historical results, not newly recomputed outputs from the cleaned handover scripts.

The frozen project record identifies the thesis-era baseline tooling as:

- controller: `cobraflex_test_control v12`;
- generic analyzer: `cobraflex_rosbag_analyzer v25`.

The scripts currently under `ros2/` are later maintained/cleaned handover versions with stable filenames. They preserve the analysis workflow but should **not** be treated as byte-identical copies of the exact programs used to create every historical thesis table.

This distinction matters when reprocessing old recordings. Preserve the original raw bag, the historical result, and the current reprocessed result separately rather than silently replacing the thesis result.

See `docs/data_provenance.md` for the source hierarchy and version boundaries.

---

## 11. Locate and preserve recorded data

Raw rosbag2 data are deliberately excluded from Git through `.gitignore`.

Typical raw outputs may contain:

```text
*.mcap
*.db3
metadata.yaml
```

Keep raw recordings in a dedicated experiment/archive location outside the Git repository, or in an ignored local data directory.

Do not commit large raw bags to this repository.

For traceability, preserve at least:

- bag directory name;
- test ID;
- command values;
- repeat number;
- scene/USD version;
- baseline configuration version;
- controller commit/version;
- analyzer commit/version;
- date and relevant notes.

---

## 12. Analyse one bag interactively

The single-bag GUI analyzer is:

```text
ros2/analysis/cobraflex_rosbag_analyzer.py
```

Run:

```bash
python3 ros2/analysis/cobraflex_rosbag_analyzer.py
```

Use this tool when inspecting one experiment interactively.

The maintained analyzer distinguishes simulation/header time from rosbag arrival time. Do not replace simulation-time dynamics calculations with wall-arrival timing.

---

## 13. Analyse a batch from the command line

The recommended batch-analysis entry point is:

```text
ros2/analysis/cobraflex_analyzer.py
```

Required analysis packages include:

- `rosbags`;
- NumPy;
- pandas;
- openpyxl;
- Matplotlib;
- PyQt5.

SciPy is optional for filtering. PyYAML is required only when a YAML analysis configuration is used.

Run a recursive batch analysis:

```bash
python3 ros2/analysis/cobraflex_analyzer.py /path/to/bag_root \
  --output results.xlsx
```

Inspect all available options:

```bash
python3 ros2/analysis/cobraflex_analyzer.py --help
```

The unified analyzer:

- discovers rosbag2 directories;
- identifies Test 01-Test 10 from the bag naming convention;
- runs the generic KPI analysis;
- adds the deeper Test 04 analysis when applicable;
- writes a consolidated Excel workbook.

### Analyzer regression test

Before modifying the analysis code:

```bash
cd ros2/analysis
python3 -m pytest tests/test_cobraflex_analyzer.py -q
```

The handover analyzer uses:

- wheel radius: 0.03725 m;
- wheel-centre separation: 0.153 m;
- wheelbase: 0.154 m.

Historical analyzer versions used 0.154 m in a track-related calculation. Reprocessing old bags can therefore produce a small change in track-dependent derived KPIs.

---

## 14. Compare against the processed thesis validation results

The compact reference results are under:

```text
validation/
```

Start with:

```text
validation/README.md
validation/sim_to_real_summary.csv
```

Detailed processed references are:

```text
validation/test01_straight.csv
validation/test03_curved.csv
validation/test04_rotation.csv
validation/scene_regression.csv
validation/ros2_acceptance.csv
```

### Important Test 04 rule

At higher angular-rate conditions, the final baseline produced multiple response branches.

Do **not** pool Low and High Test 04 responses into one nominal mean.

At W = 0.8 rad/s on the flat plane:

- Low / Stable: 192.14 +/- 1.16 deg, n = 3;
- High / Oscillatory: 272.70 +/- 8.45 deg, n = 4.

In the complete RoadRunner-derived scene:

- 20/20 runs entered the High / Oscillatory branch;
- mean accumulated yaw: 275.04 +/- 0.63 deg.

This complete-scene result demonstrates repeatability conditional on the observed branch and a changed branch distribution. It does not establish which scene component caused the change.

---

## 15. RoadRunner/OpenDRIVE provenance boundary

The repository contains the current logical road-network handover under:

```text
roadrunner/
```

The OpenDRIVE file includes the later Junction 63 repair and is useful for downstream planning and map development.

However, the latest 2026-08-16 OpenDRIVE handover export postdates the frozen thesis simulation baseline.

Therefore:

- do not claim that the historical thesis validation runs were rerun with the later OpenDRIVE file;
- do not rewrite historical validation results using the later map export;
- treat the `v1` USD scene and recorded thesis configuration as the experiment provenance;
- version any future regenerated scene separately.

---

## 16. Expected final acceptance state

A clean handover should be considered operational when a new user can complete all of the following:

- [ ] clone the repository;
- [ ] retrieve all three formal USD files through Git LFS;
- [ ] open `ADMIT14_Integrated_Scene_v1.usd` in Isaac Sim 6.0.0;
- [ ] start simulation playback with the ROS 2 bridge active;
- [ ] observe `/clock` at approximately 240 Hz;
- [ ] observe `/odom_truth`, `/joint_states`, and dynamic TF at approximately 60 Hz;
- [ ] verify the expected sensor topics in the delivered scene;
- [ ] publish `/cmd_vel` and move/stop the vehicle;
- [ ] run `cobraflex_test_control.py`;
- [ ] create and flush a rosbag2 recording;
- [ ] analyse a bag with `cobraflex_analyzer.py`;
- [ ] compare the result with the processed references under `validation/`;
- [ ] identify the frozen physics baseline and the documented validity limits.

If any item fails, do not modify the formal baseline until the failure has been isolated and documented.

---

## 17. Troubleshooting

### USD files are tiny pointer files

Run:

```bash
git lfs pull
```

and confirm with:

```bash
git lfs ls-files
```

### No ROS 2 topics appear

Check, in order:

1. ROS 2 Jazzy is sourced;
2. Isaac Sim is running the integrated scene;
3. simulation playback is active;
4. the ROS 2 bridge used by the scene is enabled;
5. the expected OmniGraph publishers are active.

### `/clock` exists but does not advance

The simulation may be paused or the graph may not be executing. Do not run a simulation-timed controller profile until `/clock` advances.

### State rate is not near 60 Hz

Confirm that the frozen state-publisher chain still uses a physics-step trigger with gate step 4 at 240 Hz.

### The vehicle receives `/cmd_vel` but does not move

Verify that the formal baseline is loaded and that the articulation/joint-drive settings have not been changed. In particular, confirm the 1.8 N.m per-wheel Max Drive Force baseline and do not substitute the 0.08 N.m diagnostic condition.

### Controller reports zero-message topics

Use:

```bash
ros2 topic list
```

and verify that the selected topic names exist and are actively publishing before repeating the test.

### Batch analyzer finds no bags

Point `cobraflex_analyzer.py` at the directory that contains the rosbag2 experiment folders. Bag naming should retain the Test 01-Test 10 identifier used by the controller.

---

## 18. Change-control rule

The formal thesis baseline is a historical research deliverable.

For future work:

1. preserve the three formal `v1` USD files;
2. preserve the processed validation references;
3. create a new version for physics, geometry, sensor, ROS 2, or controller changes;
4. record the new configuration before collecting validation data;
5. rerun the acceptance checklist;
6. do not retroactively relabel historical thesis results as results from a newer configuration.

The intended frozen thesis release tag is:

```text
v1.0.0-thesis
```
