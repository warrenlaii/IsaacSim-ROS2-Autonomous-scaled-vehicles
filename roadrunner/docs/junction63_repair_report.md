# Junction 63 Connector Repair Report

## Result

Roads 306 and 322 were replaced by endpoint- and tangent-preserving cubic connectors. Both repaired paths maintain one curvature direction throughout, removing the wrong-way turn followed by a fold-back.

| Road | Original length | Repaired length | Original primitive turns | Net turn | Repaired curvature | Minimum radius |
|---:|---:|---:|---|---:|---:|---:|
| 306 | 2.169250 m | 1.291511 m | +88.8°, -179.0° | -90.145° | -1.229 to -1.199 m⁻¹ | 0.813 m |
| 322 | 2.178077 m | 1.334503 m | -150.7°, +60.8° | -89.855° | -1.285 to -1.075 m⁻¹ | 0.778 m |

## Acceptance checks

- Road and lane IDs preserved: **Pass**.
- Road links and Junction 63 connections preserved: **Pass**.
- Connector start/end positions and headings preserved: **Pass**.
- Curvature sign remains negative throughout both connectors: **Pass**.
- Road 8 remains merged into Road 19: **Pass**.
- Road 19 retains two ordered lane sections: **Pass**.
- GeoJSON modification required: **No**; these connector curves are generated only in OpenDRIVE.