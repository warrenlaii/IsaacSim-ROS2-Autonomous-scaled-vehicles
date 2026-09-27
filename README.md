# CobraFlex Digital Twin

**ROS 2-Compatible Digital Twin Platform for a 1:14 Scaled Autonomous Vehicle**

A research platform developed with **NVIDIA Isaac Sim, PhysX, OpenUSD, MathWorks RoadRunner, and ROS 2** for vehicle simulation, Sim-to-Real evaluation, and future autonomous-driving and reinforcement-learning research.

<!--
Add the final system architecture figure here after it is uploaded:
![CobraFlex system architecture](docs/images/system_architecture.png)
-->

## Overview

This repository contains the digital-twin platform developed for the **1:14 CobraFlex scaled autonomous vehicle**.

The platform combines a PhysX-based vehicle model in NVIDIA Isaac Sim, a scaled road environment generated with RoadRunner/OpenDRIVE, and a ROS 2 interface for vehicle commands, state feedback, transforms, and virtual sensors. The digital twin was calibrated and evaluated against measurements from the physical CobraFlex platform using repeatable vehicle-motion experiments.

The repository is intended as a **research handover package**. It preserves the final thesis baseline, the ROS 2 interface, simulation assets, configuration information, and validation workflow so that future work can build on a known and traceable starting point.

> **Scope:** This work provides the simulation environment and ROS 2 interface required for future learning-based control research. Reinforcement-learning policy development and training are outside the scope of the thesis.

## Key Features

- 1:14 CobraFlex vehicle represented with **PhysX Articulation**
- Scaled RoadRunner/OpenDRIVE driving environment
- ROS 2-compatible vehicle command and state interfaces
- TF, joint-state, odometry, camera, and IMU data paths
- Sim-to-Real calibration and validation workflow
- Reproducible thesis physics baseline
- Validation and diagnostic tooling for vehicle-motion experiments
- Handover-oriented separation of formal baseline assets and diagnostic variants

## System Architecture

The platform is organized into four main layers:

1. **Vehicle model** — CobraFlex rigid-body and wheel articulation model in Isaac Sim
2. **Environment** — scaled road network generated with RoadRunner and OpenDRIVE
3. **Simulation and sensing** — PhysX dynamics, rendering, and virtual sensor pipelines
4. **ROS 2 interface** — command input, state feedback, TF, joint states, and sensor streams

```text
ROS 2 / External Controller
          |
          |  /cmd_vel
          v
+---------------------------+
|       NVIDIA Isaac Sim    |
|                           |
|  CobraFlex Articulation   |
|          +                |
|  RoadRunner Environment   |
|          +                |
|   PhysX / Sensors / TF    |
+---------------------------+
          |
          +--> /clock
          +--> /odom_truth
          +--> /joint_states
          +--> /tf
          +--> camera / CameraInfo
          +--> IMU
          +--> diagnostic topics
```

Detailed architecture documentation should be placed in `docs/system_architecture.md`.

## Repository Layout

The handover repository is organized as follows:

```text
IsaacSim-ROS2-Autonomous-scaled-vehicles/
├── README.md
├── HANDOVER.md
├── CHANGELOG.md
├── LICENSE
│
├── assets/
│   ├── vehicle/
│   ├── environment/
│   └── scenes/
│
├── config/
│   ├── physics_baseline.yaml
│   ├── vehicle_parameters.yaml
│   └── ros2_topics.yaml
│
├── ros2/
│   ├── control/
│   ├── analysis/
│   └── utilities/
│
├── roadrunner/
│   ├── OpenDRIVE/
│   └── README.md
│
├── calibration/
├── validation/
├── docs/
└── thesis/
```

Historical development files, obsolete USD variants, and large raw ROS bag recordings should not be mixed with the formal handover baseline.

## Requirements

### Simulation

- **NVIDIA Isaac Sim 6.0.0**
- PhysX
- OpenUSD
- Isaac Sim ROS 2 Bridge

### Middleware

- ROS 2
- `rosbag2` for recording and playback
- MCAP support where required by the analysis workflow

### Environment Generation

- **MathWorks RoadRunner R2025b**
- OpenDRIVE export/import workflow

### Analysis

The analysis tools use Python-based processing. Install the dependencies required by the scripts included in `ros2/analysis/`.

> The exact ROS 2 distribution should match the laboratory/workstation environment used for the handover. Record it in `docs/installation.md` before the repository is released outside the project team.

## Formal Delivery Assets

The thesis delivery baseline consists of three formal USD assets:

```text
ADMIT14_cobraflex_baseline_v1.usd
ADMIT14_RoadRunner_Map_v1.usd
ADMIT14_Integrated_Scene_v1.usd
```

Recommended repository locations:

```text
assets/vehicle/ADMIT14_cobraflex_baseline_v1.usd
assets/environment/ADMIT14_RoadRunner_Map_v1.usd
assets/scenes/ADMIT14_Integrated_Scene_v1.usd
```

Diagnostic or experimental USD files must be clearly separated from these formal assets and must not overwrite the `v1` baseline.

## Quick Start

### 1. Clone the repository

```bash
git clone <repository-url>
cd IsaacSim-ROS2-Autonomous-scaled-vehicles
```

### 2. Start the ROS 2 environment

Source the ROS 2 installation used by the laboratory setup.

```bash
source /opt/ros/<distro>/setup.bash
```

### 3. Open the integrated scene

Open the following file in Isaac Sim:

```text
assets/scenes/ADMIT14_Integrated_Scene_v1.usd
```

Verify that the ROS 2 bridge is enabled and start the simulation.

### 4. Verify ROS 2 communication

```bash
ros2 topic list
```

At minimum, the core handover configuration should expose the command, clock, state, and transform interfaces described below.

### 5. Send a basic velocity command

Example straight-line command:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.20}, angular: {z: 0.0}}" -r 10
```

Stop the vehicle after the test:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0}, angular: {z: 0.0}}" -1
```

For validation runs, bag recording, analyzer usage, and reset procedures, see `HANDOVER.md`.

## ROS 2 Interface

The final architecture includes the following core interfaces.

| Interface | Direction | Purpose |
|---|---|---|
| `/cmd_vel` | ROS 2 → Simulation | Linear and angular velocity command |
| `/clock` | Simulation → ROS 2 | Simulation clock |
| `/odom_truth` | Simulation → ROS 2 | Simulation odometry / reference vehicle state |
| `/joint_states` | Simulation → ROS 2 | Wheel-joint position and velocity states |
| `/tf` | Simulation → ROS 2 | Dynamic frame transforms |
| IMU topic | Simulation → ROS 2 | Simulated inertial data |
| Lane Camera Image | Simulation → ROS 2 | RGB camera stream |
| Lane Camera `CameraInfo` | Simulation → ROS 2 | Camera calibration/projection information |
| `/cobraflex/wheel_cmd_debug` | Simulation → ROS 2 | Wheel-command diagnostics |

Sensor namespaces may depend on the final scene configuration. Record the exact deployed topic names in `config/ros2_topics.yaml`.

## Final Simulation Baseline

The formal thesis baseline uses the following core configuration.

| Parameter | Baseline |
|---|---:|
| Isaac Sim | 6.0.0 |
| Physics rate | 240 Hz |
| Dynamics | CPU dynamics / GPU Dynamics OFF |
| Solver | PGS |
| Vehicle mass | 3.50 kg |
| Wheelbase | 0.154 m |
| Track width | approx. 0.153 m |
| Wheel radius | 0.03725 m |
| Joint-drive stiffness | 0 |
| Joint-drive damping | 10,000 |
| Max drive force | 1.8 N·m per wheel |
| Rigid-body angular damping | 0.05 |
| State / TF / joint publication | 60 Hz |

The **formal USD assets are the source of truth for the delivered scene configuration**. A machine-readable summary should additionally be maintained in `config/physics_baseline.yaml`.

> **Important:** The 0.08 N·m Max Drive Force configuration was used only as a diagnostic ablation. It is not the formal delivered motor-drive baseline.

## Validation Summary

The platform was evaluated against the physical CobraFlex vehicle and through ROS 2 interface checks.

| Area | Handover status |
|---|---|
| Formal vehicle and environment assets | Completed |
| ROS 2 command and state interface | Validated |
| `/clock`, TF, and joint-state publication | Validated |
| Lane Camera payload / interface | Validated |
| Straight-line vehicle response | Evaluated against physical vehicle |
| Curved-path vehicle response | Evaluated; remaining Sim-to-Real mismatch documented |
| In-place rotation | Evaluated; contact/solver-dependent branch behaviour remains |
| RL environment interface | Delivered |
| RL policy training | Outside thesis scope |

For the RoadRunner-scene in-place-rotation regression, the final thesis dataset showed a repeatable oscillatory branch under the tested condition. This result is treated as a **documented current finding**, not as proof of a universal simulator limitation.

Detailed results, plots, and experimental definitions should be kept in `validation/` and summarized in `docs/validation_summary.md`.

## Known Limitations

This simulator should **not** be interpreted as an exact dynamic replica of the physical vehicle under every operating condition.

The main handover boundaries are:

- Curved-path dynamics retain a measurable Sim-to-Real gap.
- In-place rotation can exhibit solver/contact-dependent branch behaviour.
- The validated operating range is limited to the test conditions documented in the thesis.
- PGS/TGS and Max Drive Force sweeps are diagnostic studies and do not constitute a complete tire or motor model.
- Sensor publication and payload integrity were verified at interface level; this does not imply complete perception-model validation.
- Reinforcement-learning policy training and real-vehicle policy transfer were not performed in this thesis.

These boundaries should be considered when using the platform for future controller tuning, system identification, or reinforcement-learning experiments.

## Data and Reproducibility

Large raw ROS bag recordings are intentionally excluded from the main Git repository.

The repository should instead contain:

- experiment metadata
- final configuration files
- processed validation summaries
- analysis scripts
- small representative datasets where useful
- figures required to reproduce the reported comparisons
- a provenance record linking each result to the corresponding configuration and analysis workflow

Raw data should remain in the project/institutional archive and be referenced from `docs/data_provenance.md`.

The general rule is:

> **Do not overwrite historical experiment configurations with newer settings. Keep the formal thesis baseline frozen and create a new version for future changes.**

## Handover Rules

Future development should follow these rules:

1. Do not overwrite the three formal `v1` USD assets.
2. Create a new version when changing vehicle physics, environment geometry, or ROS 2 behaviour.
3. Keep diagnostic configurations separate from the formal baseline.
4. Record configuration changes before generating new validation datasets.
5. Preserve the link between data, controller version, analyzer version, USD asset, and physics configuration.
6. Re-run a clean-clone acceptance test before publishing a new release.

## Suggested Acceptance Test

A handover release is considered usable when a new user can, using only the repository documentation:

1. identify the three formal USD assets;
2. open the integrated scene;
3. start the ROS 2 bridge;
4. observe `/clock`;
5. observe vehicle state, TF, and joint states;
6. send `/cmd_vel`;
7. move and stop the vehicle;
8. access the Lane Camera stream;
9. record a ROS bag;
10. run the provided analyzer;
11. identify the formal physics baseline; and
12. identify the documented validity boundaries and known limitations.

## Thesis

This repository accompanies the Master's thesis:

> **Design and Implementation of a ROS 2-Compatible Digital Twin Platform for 1:14 Scaled Autonomous Vehicles in Reinforcement Learning**

**Program:** M.Eng. Automotive Systems  
**Institution:** Hochschule Esslingen  
**Author:** Warren Lai  
**Year:** 2026

The thesis focuses on digital-twin environment generation, vehicle-model calibration, ROS 2 integration, and Sim-to-Real evaluation for a scaled autonomous research vehicle.

## Citation

If you use this repository in academic work, cite the thesis. A BibTeX entry can be added or updated once the final institutional publication information is available.

```bibtex
@mastersthesis{lai2026cobraflex,
  author = {Warren Lai},
  title  = {Design and Implementation of a ROS 2-Compatible Digital Twin Platform for 1:14 Scaled Autonomous Vehicles in Reinforcement Learning},
  school = {Hochschule Esslingen},
  year   = {2026}
}
```

## License

A license has not yet been specified in this handover draft.

Before making the repository public, add an appropriate `LICENSE` file and verify that all included USD assets, RoadRunner exports, third-party models, and software components can be redistributed under the selected terms.

---

### Thesis Baseline

For reproducibility, the thesis handover should be tagged as:

```text
v1.0.0-thesis
```

Future work should build from this tagged baseline rather than modifying the archived thesis configuration in place.
