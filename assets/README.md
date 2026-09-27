# Formal Simulation Assets

This directory contains the frozen USD assets delivered with the CobraFlex thesis handover.

## Required files

| Repository path | Role | Size |
| --- | --- | ---: |
| `vehicle/ADMIT14_cobraflex_baseline_v1.usd` | CobraFlex vehicle baseline | 41.9 MB |
| `environment/ADMIT14_RoadRunner_Map_v1.usd` | RoadRunner environment | 1.4 MB |
| `scenes/ADMIT14_Integrated_Scene_v1.usd` | Integrated vehicle + environment scene | 150.1 MB |

The `.usd` files are tracked with **Git LFS**. Do not replace the `v1` files in place. Any future modification must be saved under a new versioned filename.

File-integrity checksums are stored separately in [`CHECKSUMS.sha256`](CHECKSUMS.sha256).

## Asset validation

After cloning the repository, run `git lfs pull` and open:

`scenes/ADMIT14_Integrated_Scene_v1.usd`

in the handover Isaac Sim environment. Check the console for unresolved asset, texture, material, or USD-reference warnings before treating a release as self-contained.
