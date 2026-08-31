# Simulation Platform V1.0

This repository is the clean runtime workspace for a stable, understandable
P450 + BUNKER + AUBO + AG95 joint simulation. The target environment is Ubuntu
20.04 with ROS Noetic, Gazebo 11, PX4, MAVROS, and MoveIt.

## Source boundary

`P450-PAPER` is an immutable upstream: it may be inspected or copied from, but
it is never modified by work in this repository. Runtime source enters this
workspace only through the frozen `config/runtime_sources.json` allowlist and
the dry-run-first `tools/import_runtime.py` importer.

`config/import_provenance.json` records the immutable imported snapshot.
`config/runtime_overlay.json` records every intentional removal or modification
made while extracting the runtime. Together they preserve traceability without
turning the paper repository into a working tree.

Paper benchmark and validation artifacts, RBP, Task-aware methods, M5, Pilot,
T2, Formal, and method optimization are outside Simulation Platform V1.0. The
inherited launch wrappers are component diagnostics or explicitly classified
legacy starts; they are not Simulation Platform V1.0 platform entrypoints.

## Offline boundary gate

Run the deterministic source gate with:

```bash
env -i PATH=/usr/bin:/bin /usr/bin/python3 -B tools/runtime_boundary.py
```

This offline gate proves source selection and boundary integrity only. It does
not prove a ROS build, Gazebo launch, TF connectivity, sensor health, PX4/MAVROS
flight, MoveIt manipulation, or Air-Ground Pick demo success. Those capabilities
are introduced and verified in separate reviewed milestones.
