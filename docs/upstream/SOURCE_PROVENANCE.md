# Source Provenance and Freeze Boundary

## Published snapshots

| Component | Commit | Git tree | Role |
|---|---|---|---|
| P450 integrated source | `c4ad8f94349cd42c4bd3d7b4ea1151857b17b5f2` | `00cf3ba8806a15006f02e49b3b896439b340b8de` | Final source and evidence freeze |
| Runtime acceptance source | `87ea5ad6868eb9a3805ac900c62820b282e5e6c9` | recorded in evidence | Exact clean runtime commit |
| Ground tested source | `e3db0f95b9f827c1862d04555e575f5db3711b41` | `011478e073ddb40f79158188c070582bb1c28fba` | Side-up physical manipulation baseline |
| Ground authority | `72bea5be3a5d7944ee7413ca2bdcdab51baf6400` | see Ground history/provenance | Frozen Melodic authority |
| Prometheus PX4 | `713814f4eea5990e49dd776a38a36ad53e171f60` | external | P450 SITL dependency |
| PX4 SITL Gazebo submodule | `9566172e7c0760f66681304b963d675c5b120daf` | external | Gazebo SITL dependency |

The JSON lock file is the machine-readable authority. Runtime evidence records
the P450 source SHA-256 `e696b774...67d5`, Ground selected-source SHA-256
`c71f64c6...2bdd`, ROS/Gazebo/MoveIt versions, scenario inputs, logs, and visual
artifact hashes.

## Snapshot policy

The GitHub repository is intentionally a clean source snapshot, not a mirror of
all historic local worktrees. It preserves the tested final files and exact
origin identifiers while avoiding a 1.21 GiB legacy object pack and unrelated
prebuilt artifacts. The original local repositories remain unchanged.

Excluded items are listed in `manifests/excluded-local-artifacts.txt`. No
algorithm or safety parameter was modified during packaging.
