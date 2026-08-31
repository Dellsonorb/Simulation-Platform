# M1 lightweight validation

Environment: Ubuntu 20.04, ROS Noetic, Gazebo 11, Prometheus P450_D435i,
PX4 `amovlab_sitl_default`. The mission used real Prometheus command/control;
no Gazebo UAV teleport was used.

The final four paired viewpoint/brick scenarios completed 4/4 and landed:

| View | Brick pose `(x,y,yaw)` | Position error | Yaw error | Valid points |
|---|---:|---:|---:|---:|
| center_low | `(0.00, 0.00, 0°)` | 1.00 cm | 0.19° | 285 |
| left_low | `(0.08, 0.12, 20°)` | 0.55 cm | 0.04° | 414 |
| right_low | `(-0.10, -0.12, -20°)` | 2.77 cm | 1.09° | 254 |
| center_high | `(0.12, -0.04, 65°)` | 0.55 cm | 1.14° | 341 |

Summary: position mean/median/max `1.22/0.77/2.77 cm`; XY mean/max
`1.12/2.75 cm`; Z mean/max `0.36/0.43 cm`; yaw mean/median/max
`0.61/0.64/1.14°`.

An exploratory right-side `-35°` case produced only 107 registered points and
an ambiguous PCA axis. The production gate was therefore raised to 150 points;
that insufficient observation is now rejected rather than published. This is
the current first-stage coverage limitation, not a claimed robustness result.

Observed raw image contracts were color `640x480 rgb8`, depth `640x480 16UC1`.
The raw intrinsics differed (`fx_color=462.266`, `fx_depth=319.935`), and the
aligned depth interface was exercised before point-cloud generation.

The final report and screenshots are `results/m1_final_694ae6b6.json`,
`results/m1_gazebo_694ae6b6.png`, and `results/m1_rviz_694ae6b6.png`. The report
binds the run to commit `694ae6b6504676d61b326cc5744d3da0f642dffd` and source
tree SHA-256 `c6df63d01a1557dc32a0866a279d63e65c42eb8e711b277d6a6ec28695c4361b`.
Artifact SHA-256 values are respectively `7f594e18fb02878a0b4321a10295dbf9d42a6e724495771167f2638e0539d818`,
`9cb96d549cd2a7da67eba0196e28a937ea2ac1edb93e0d5e78133d473de81358`,
and `38409e3432cd8268448f1366cfb4020baab83b0238bbef0e4ef5c2332c9fad8b`.
