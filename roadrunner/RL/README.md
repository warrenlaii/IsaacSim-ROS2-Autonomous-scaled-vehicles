# RL Lane Assets

These files are lightweight convenience assets derived from the final RoadRunner GeoJSON and OpenDRIVE handover exports.

- `lane_centrelines.csv` contains ordered centreline points for every logical **Driving** lane, stored in vehicle travel direction.
- `lane_network.json` contains directed predecessor/successor topology, lane metadata, lengths, and OpenDRIVE road/lane occurrences.

## Coordinate reconstruction

The RoadRunner GeoJSON stores the near-origin lane geometry as geographic coordinates. For this handover export, the local metric coordinates were reconstructed with the same spherical map relation used during asset generation:

```text
R = 6371008.8 m
x = radians(longitude) * R
y = log(tan(pi/4 + radians(latitude)/2)) * R
```

The reconstructed local coordinates were cross-checked against the OpenDRIVE geometry.

For lanes with `TravelDir=Backward`, centreline points are reversed and the GeoJSON predecessor/successor relations are swapped so that both point order and graph edges follow vehicle travel direction.

## Provenance

These are newly generated handover convenience files derived from the final delivered GeoJSON/OpenDRIVE pair. They should not be presented as byte-identical copies of any earlier historical lane assets referenced during thesis development.

The versioned OpenDRIVE file remains the logical road-network source.
