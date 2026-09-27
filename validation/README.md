# Validation Handover

This directory contains compact, processed validation results for the CobraFlex thesis handover. It is intended to make the principal verification and Sim-to-Real conclusions understandable without committing the large raw rosbag2 datasets.

## Contents

| File | Purpose |
| --- | --- |
| `sim_to_real_summary.csv` | Cross-manoeuvre summary of the principal physical, Initial-simulation, and final-baseline results |
| `test01_straight.csv` | Test 01 straight-line distance comparison |
| `test03_curved.csv` | Test 03 curved-path comparison across the matched 3 x 4 command grid |
| `test04_rotation.csv` | Test 04 in-place rotation results, with higher-rate response branches kept separate |
| `scene_regression.csv` | Flat-plane versus complete-scene Test 04 branch statistics at 0.8 rad/s |
| `ros2_acceptance.csv` | Final ROS 2 interface acceptance evidence |

## Reporting rules

The files follow the reporting rules used in the final thesis:

- Test 04 Low and High response branches are reported separately. They must not be pooled into one nominal mean.
- A reduction in Sim-to-Real error magnitude does not imply that the physical response shape has been reproduced.
- Test 03 retains the physical demand-dependent understeer comparison instead of reducing the result to one aggregate score.
- The complete-scene Test 04 result shows a change in branch occupancy. The experiment supports an association with scene configuration, not a demonstrated causal scene effect.
- Raw rosbag2 recordings and large intermediate analyzer workbooks remain outside this Git repository.

## Source hierarchy

The final thesis is the primary source for the published comparison tables and ROS 2 acceptance statements. The frozen `Master_v32_0812` calibration workbook is used where the handover benefits from more detailed processed values, especially the Test 03 radius-ratio grid and the formal scene-regression record. The repository-level source and tool lineage is documented in [`../docs/data_provenance.md`](../docs/data_provenance.md).

The simulation baseline represented here is the frozen thesis baseline. The RoadRunner/OpenDRIVE exchange artifact currently delivered under `roadrunner/OpenDRIVE/` includes a later 2026-08-16 Junction 63 repair. This directory does **not** claim that the historical validation campaigns were rerun against that later OpenDRIVE export; historical experiment results remain tied to their recorded thesis configuration.

## Interpretation boundary

The final baseline:

- preserved the already accurate straight-line response;
- substantially reduced curved-path and in-place-rotation magnitude errors;
- did not reproduce the physical demand-dependent curved-path understeer trend;
- retained multiple in-place-rotation response branches at higher angular commands;
- over-predicted the matched Test 07 yaw response;
- substantially under-predicted measured pitch sensitivity.

Accordingly, these files document an improved and quantified nominal simulation baseline, not a claim of complete dynamic equivalence.
