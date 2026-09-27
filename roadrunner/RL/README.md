# RL Lane Assets

These files are derived from the final RoadRunner GeoJSON and OpenDRIVE handover exports.

- `lane_centrelines.csv` contains the ordered centreline points of every logical **Driving** lane in vehicle travel direction.
- `lane_network.json` contains directed predecessor/successor topology, lane metadata, lengths, and the corresponding OpenDRIVE road/lane occurrences.

The RoadRunner GeoJSON stores lane geometry in near-origin geographic coordinates. Coordinates are converted back to the RoadRunner local metric frame using the WGS84 metres-per-degree scale at the origin. The conversion was cross-checked against the OpenDRIVE local coordinates.

For lanes with `TravelDir=Backward`, centreline points are reversed and GeoJSON predecessor/successor relations are swapped so that both geometry and graph edges follow vehicle travel direction.