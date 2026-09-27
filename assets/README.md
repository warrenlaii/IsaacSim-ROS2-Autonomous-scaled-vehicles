# Formal Simulation Assets

This directory contains the frozen USD assets delivered with the CobraFlex thesis handover.

## Required files

| Repository path | Role | Size | SHA-256 |
| --- | --- | ---: | --- |
| `vehicle/ADMIT14_cobraflex_baseline_v1.usd` | CobraFlex vehicle baseline | 43,897,964 bytes | `34c5237a73e75be6aa7408bcf9bdb39cb53a067c9db403983b8b76732f2c473a` |
| `environment/ADMIT14_RoadRunner_Map_v1.usd` | RoadRunner environment | 1,417,280 bytes | `5439a2a60e2cf10b4e7aa970a4892ab552d20b0ff2bf1c4dae090ac12962edec` |
| `scenes/ADMIT14_Integrated_Scene_v1.usd` | Integrated vehicle + environment scene | 157,389,486 bytes | `9b94a75ebb6b96d4accc0ede765c79f80cf4930ad179444cd87f6805d7007d43` |

The `.usd` files are tracked with **Git LFS**. Do not replace the `v1` files in place. Any future modification must be saved under a new versioned filename.

## Asset validation

After cloning the repository, run `git lfs pull` and open:

`scenes/ADMIT14_Integrated_Scene_v1.usd`

in the handover Isaac Sim environment. Check the console for unresolved asset, texture, material, or USD-reference warnings before treating a release as self-contained.
