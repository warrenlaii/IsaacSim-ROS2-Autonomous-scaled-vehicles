# Data and Result Provenance

This document defines the evidence hierarchy used by the public thesis handover repository and separates historical thesis results from later handover cleanup work.

## Source hierarchy

1. **Final thesis (August 2026)**  
   Primary source for published comparison tables, stated validation conclusions, software-stack description, and ROS 2 acceptance results.

2. **Frozen calibration workbook: Master_v32_0812**  
   Authoritative project record for the frozen physics baseline, experiment history, detailed Test 03 radius-ratio grid, Test 04 branch analysis, scene-regression statistics, and diagnostic provenance.

3. **Raw rosbag2 / MCAP recordings and intermediate workbooks**  
   These are not committed to this public Git repository. They remain part of the project/institutional archive and must be preserved separately when exact historical reprocessing is required.

4. **Files under `validation/`**  
   Compact processed handover tables transcribed from the final thesis and frozen master record. They are reference summaries, not replacements for the raw experiment archive.

## Historical versus maintained analysis tools

The frozen project record identifies the thesis-era tool chain as:

```text
controller: cobraflex_test_control v12
generic analyzer: cobraflex_rosbag_analyzer v25
```

The files currently under `ros2/` are later cleaned handover versions with stable filenames:

```text
ros2/control/cobraflex_test_control.py
ros2/analysis/cobraflex_rosbag_analyzer.py
ros2/analysis/cobraflex_analyzer.py
```

They should not be described as byte-identical copies of the exact programs used for every historical thesis result. In particular, the maintained analyzer uses the final 0.153 m wheel-centre separation, while an older analyzer path used 0.154 m for a track-related calculation.

When old bags are reprocessed, retain both the historical thesis result and the newly generated result with their tool/configuration provenance.

## Formal USD assets

The frozen thesis delivery assets are:

```text
assets/vehicle/ADMIT14_cobraflex_baseline_v1.usd
assets/environment/ADMIT14_RoadRunner_Map_v1.usd
assets/scenes/ADMIT14_Integrated_Scene_v1.usd
```

Their binary SHA-256 checksums are maintained in `assets/CHECKSUMS.sha256`. The USD files are tracked through Git LFS.

The formal `v1` assets must not be overwritten. Changes require a new version and new validation record.

## RoadRunner/OpenDRIVE boundary

The current OpenDRIVE handover file includes the 2026-08-16 Junction 63 repair. This exchange artifact postdates the frozen 2026-08-12 simulation baseline.

Therefore the repository does not claim that historical thesis dynamics runs were rerun using the later OpenDRIVE export. Future regenerated scenes must use a new version and must not retroactively relabel the historical validation data.

The files in `roadrunner/RL/` are newly generated convenience assets derived from the final delivered GeoJSON/OpenDRIVE pair. They are not claimed to be byte-identical copies of earlier development-time lane exports.

## Raw-data policy

Raw recordings are intentionally excluded by `.gitignore`:

```text
*.mcap
*.db3
*.bag
rosbag2_*/
```

For every future experiment, preserve at least:

- test ID and command condition;
- repeat number;
- date;
- USD/scene version;
- physics/configuration version;
- controller revision;
- analyzer revision;
- raw bag directory;
- generated result file;
- relevant data-quality notes.

The public repository contains enough processed evidence to understand the thesis conclusions, but exact historical recomputation requires the external raw-data archive.
