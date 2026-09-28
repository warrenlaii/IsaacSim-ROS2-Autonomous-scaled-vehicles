# Changelog

This file records notable changes to the **handover repository**. It is not the experiment log; detailed calibration and test history remain in the frozen project records.

The repository has not yet been tagged as `v1.0.0-thesis`.

## Unreleased — thesis handover preparation

### Added

- Three formal thesis USD assets under Git LFS, with SHA-256 checksums.
- Machine-readable frozen baseline in `config/baseline.yaml`.
- Verified ROS 2 handover topic contract in `config/ros2_topics.yaml`.
- Final RoadRunner/OpenDRIVE and GeoJSON exchange assets.
- Junction 63 repair provenance record.
- Lightweight RL lane-centreline and directed lane-network convenience assets.
- Maintained ROS 2 controller and analysis tools with stable handover filenames.
- Detailed controller test-profile and parameter documentation.
- Processed Sim-to-Real and ROS 2 validation tables under `validation/`.
- End-to-end `HANDOVER.md` covering clone, Git LFS, Isaac Sim startup, ROS 2 checks, test execution, recording, analysis, and acceptance.
- GitHub Actions clean-clone/static release-acceptance workflow.
- `ros2/requirements-analysis.txt` for the maintained offline analysis and regression-test dependencies.
- `docs/data_provenance.md` defining historical data/tool and map-version boundaries.
- `docs/vehicle_model.md` documenting the CobraFlex articulation structure, measured/inherited mass-property provenance, joint-drive configuration, physics baseline, and command-to-measurement path.
- `docs/calibration_and_diagnostics.md` documenting the measurement-first calibration workflow, parameter-sensitivity evidence, response-branch diagnostics, Max Drive Force sweep, and matched PGS-TGS solver comparison.
- Vehicle-model scene image and ROS 2-to-Isaac Sim command/state-flow figure in the vehicle-model documentation.

### Changed

- Normalised handover script filenames so Git history carries development-version changes instead of filename proliferation.
- Removed hard-coded local bag paths from the maintained analyzer.
- Aligned maintained analysis geometry with the final 0.153 m measured wheel-centre separation and 0.154 m wheelbase.
- Documented the distinction between historical thesis-era tools and later maintained handover scripts.
- Documented exact Lane Camera topic names from the verified final interface.
- Aligned the repository thesis title with the final submitted thesis title.
- Clarified that the 2026-08-16 OpenDRIVE repair postdates the frozen dynamics-validation baseline.
- Clarified Test 04 branch-conditioned reporting and the complete-scene result boundary.

### Removed

- Duplicate analyzer test copy and superseded batch wrapper from the formal handover set.
- Empty `.gitkeep` placeholders from populated asset directories.
- README references to directories/files that are not part of the delivered repository.

### Repository hygiene

- Generated analyzer workbooks (`cobraflex_results*.xlsx`, `results_*.xlsx`) are ignored by default to reduce accidental commits of local analysis output.

## Before `v1.0.0-thesis`

The following release checks remain intentionally open:

- perform a clean clone on a separate workstation;
- run `git lfs pull` and verify all three USD binary checksums;
- open the integrated scene in Isaac Sim 6.0.0 and check for unresolved external USD/material/texture references;
- rerun the operational ROS 2 acceptance checklist in `HANDOVER.md`;
- finalise redistribution/licensing rights before adding a `LICENSE` file.

Do not tag `v1.0.0-thesis` until these checks are complete.
