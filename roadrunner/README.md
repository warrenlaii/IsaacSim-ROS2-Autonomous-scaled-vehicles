# RoadRunner / OpenDRIVE Handover

This directory contains the final road-network exchange files delivered with the CobraFlex thesis handover.

## Formal map files

| File | Purpose |
| --- | --- |
| `OpenDRIVE/ADMIT14_RoadRunner_Map_v1.xodr` | Final OpenDRIVE 1.6 road-network export |
| `GeoJSON/ADMIT14_RoadRunner_Map_v1.geojson` | GeoJSON representation used for geometry inspection and supporting workflows |
| `docs/junction63_repair_report.md` | Provenance and acceptance record for the final Junction 63 connector repair |

The handover filenames intentionally use the same `ADMIT14_RoadRunner_Map_v1` naming as the formal USD environment asset.

## Junction 63 repair

The final OpenDRIVE file contains the repaired connector roads 306 and 322. The repair preserves endpoints, headings, road/lane IDs, and Junction 63 connections while removing the fold-back geometry. See the repair report for the acceptance checks.

## Validation provenance

The current OpenDRIVE handover file includes the 2026-08-16 Junction 63 repair. It postdates the frozen 2026-08-12 dynamics-validation baseline. Historical thesis dynamics results are therefore not presented as reruns with this later exchange artifact. See [`../docs/data_provenance.md`](../docs/data_provenance.md).

## Versioning

Do not overwrite the `v1` map files in place. Future geometry changes should use a new versioned filename and be documented separately.


## RL lane assets

The final map package also includes lightweight, directed lane representations for downstream planning and reinforcement-learning work:

| File | Purpose |
| --- | --- |
| `RL/lane_centrelines.csv` | Ordered centreline samples for each logical driving lane, stored in vehicle travel direction |
| `RL/lane_network.json` | Directed predecessor/successor graph, lane metadata, lengths, and OpenDRIVE road/lane mapping |
| `RL/README.md` | Generation conventions and coordinate/direction notes |

These files are derived from the final GeoJSON and OpenDRIVE handover exports. They are convenience assets for downstream use; the versioned OpenDRIVE file remains the logical road-network source.
