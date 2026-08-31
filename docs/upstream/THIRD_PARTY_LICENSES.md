# Third-Party Licenses and Provenance

The root `LICENSE` is Apache-2.0 and originates from AMOVLAB Prometheus. It does
not replace the licenses of included or fetched components. Upstream copyright
and license notices must be preserved.

| Component | Upstream | Locked commit/version | License found locally | Vendored? | Modified? |
|---|---|---|---|---:|---:|
| Prometheus/P450-SIM base | `gitee.com/amovlab/Prometheus` | `c4ad8f94349cd42c4bd3d7b4ea1151857b17b5f2` source snapshot | Apache-2.0 (`LICENSE`) | Yes | Yes |
| PX4-Autopilot fork | `gitee.com/amovlab/prometheus_px4` | `713814f4eea5990e49dd776a38a36ad53e171f60` | BSD-3-Clause (upstream `LICENSE`) | No | No local vendoring |
| PX4 `sitl_gazebo` | PX4 submodule | `9566172e7c0760f66681304b963d675c5b120daf` | TO VERIFY: no license file was present at the pinned submodule root | No | No |
| FAST-LIO | included `Modules/FAST_LIO` | paper repository commit | GPL-2.0 (`Modules/FAST_LIO/LICENSE`) | Yes | Upstream snapshot |
| AgileX `bunker_ros` | `agilexrobotics/bunker_ros` | `6ae0a1da92a0cdfb5f679e1cf2c0ad63e75a36d1` | BSD-3-Clause file; one package manifest remains `TODO` | Fetched | No |
| AgileX `ugv_gazebo_sim` | `agilexrobotics/ugv_gazebo_sim` | `27633a956c845903ee630538afeb17fe70afdd84` | Mixed BSD declarations and `TODO`; TO VERIFY for affected packages | Fetched | No |
| AUBO Melodic description/driver | `ian-chuang/aubo_robot` | `14f5b9b088be6f4f2a2afca20a68f6f8069f6045` | BSD license file; some manifests remain `TODO` | Fetched | Compatibility overlay only |
| AUBO ROS driver | `AuboRobot/aubo_ros_driver` | `295007ea64ed2313cc34963a5505edc7aa9bf0e4` | Mixed Apache-2.0/BSD/Zlib declarations; some `TODO` | Fetched | No |
| DH Robotics gripper ROS | `DH-Robotics/dh_gripper_ros` | `f59f9c2f4bc8eb116448b1d798791424bf64e337` | Mostly BSD declarations; driver manifest is `TODO` | Fetched | No |
| AG95 description used by Ground | `LARA_AUBOi5_AG95` / `scout_cobot_sim` | `0403d14e7f9cd7f4b2a47e29e9511b2277d2fbb0` / `e795e67a2a1a76b06b9647900df04d3aee8e96ab` | MIT file in selected AG95 description; surrounding repository has mixed BSD/MIT/`TODO` | Fetched | No hardware-limit changes |
| `roboticsgroup_gazebo_plugins` | `roboticsgroup/roboticsgroup_gazebo_plugins` | `509a32ea6accc58d03cebd9d670ae44635adc924` | BSD-3-Clause license file | Fetched | No; wrapper integration only |

The exact fetch URLs and commits for Ground dependencies are in
`Ground/upstream.repos`. Rows marked **TO VERIFY** are intentionally unresolved;
no license is inferred from a project name or neighbouring package. Before a
formal binary/data release, obtain legal review for mixed or undeclared
components and include their complete upstream notices in the distribution.

Apache-2.0 at the root is generally compatible with linking to permissive
BSD/MIT components, but FAST-LIO's GPL-2.0 terms and every component-specific
notice still apply to redistribution. This inventory is technical provenance,
not legal advice.
