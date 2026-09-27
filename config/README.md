# Configuration

This directory provides a compact, machine-readable summary of the frozen CobraFlex thesis baseline.

- `baseline.yaml` — vehicle geometry, mass properties, PhysX/Isaac Sim settings, joint-drive settings, material settings, publication rates, and links to the formal USD assets.
- `ros2_topics.yaml` — verified ROS 2 handover contract topics and publication rates, including the final Lane Camera topic names.

The formal USD files under `assets/` remain the authoritative delivered simulation assets. These YAML files are handover references and should be versioned together with any future baseline change.

Diagnostic settings must not be silently substituted for the thesis baseline. In particular, the 0.08 N·m Max Drive Force configuration is an ablation only; the delivered baseline uses 1.8 N·m per wheel.
