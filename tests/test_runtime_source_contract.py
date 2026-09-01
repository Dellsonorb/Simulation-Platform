import unittest
from pathlib import Path

from tools.runtime_manifest import RuntimeManifest

ROOT = Path(__file__).resolve().parents[1]
EXPECTED_PACKAGE_RECORDS = {
    "prometheus_msgs": (
        "p450",
        "Modules/common/prometheus_msgs",
        "src/p450/prometheus_msgs",
        True,
    ),
    "prometheus_gazebo": (
        "p450",
        "Simulator/gazebo_simulator",
        "src/p450/prometheus_gazebo",
        True,
    ),
    "prometheus_uav_control": (
        "p450",
        "Modules/uav_control",
        "src/p450/prometheus_uav_control",
        True,
    ),
    "realsense_ros_gazebo": (
        "p450",
        "Simulator/realsense_gazebo_plugin",
        "src/p450/realsense_ros_gazebo",
        True,
    ),
    "livox_laser_gazebo_plugins": (
        "p450",
        "Simulator/livox_laser_gazebo_plugins",
        "src/p450/livox_laser_gazebo_plugins",
        True,
    ),
    "brick_aerial_perception": (
        "p450",
        "Modules/brick_aerial_perception",
        "src/p450/brick_aerial_perception",
        True,
    ),
    "bunker_aubo_description": (
        "ground",
        "Ground/src/bunker_aubo_project/bunker_aubo_description",
        "src/ground/bunker_aubo_description",
        True,
    ),
    "bunker_aubo_gazebo": (
        "ground",
        "Ground/src/bunker_aubo_project/bunker_aubo_gazebo",
        "src/ground/bunker_aubo_gazebo",
        True,
    ),
    "bunker_aubo_moveit_config": (
        "ground",
        "Ground/src/bunker_aubo_project/bunker_aubo_moveit_config",
        "src/ground/bunker_aubo_moveit_config",
        True,
    ),
    "bunker_navigation": (
        "ground",
        "Ground/src/bunker_aubo_project/bunker_navigation",
        "src/ground/bunker_navigation",
        True,
    ),
    "brick_rgbd_perception": (
        "ground",
        "Ground/src/bunker_aubo_project/brick_rgbd_perception",
        "src/ground/brick_rgbd_perception",
        True,
    ),
    "brick_visual_pick": (
        "ground",
        "Ground/src/bunker_aubo_project/brick_visual_pick",
        "src/ground/brick_visual_pick",
        True,
    ),
    "brick_pick_demo": (
        "ground",
        "Ground/src/bunker_aubo_project/brick_pick_demo",
        "src/ground/brick_pick_demo",
        True,
    ),
    "ground_pick_orchestrator": (
        "ground",
        "Ground/src/bunker_aubo_project/ground_pick_orchestrator",
        "src/ground/ground_pick_orchestrator",
        True,
    ),
    "aubo_description": (
        "vendor",
        "Ground/src/third_party/aubo_robot_melodic/aubo_description",
        "src/vendor/aubo_description",
        True,
    ),
    "bunker_description": (
        "vendor",
        "Ground/src/third_party/ugv_gazebo_sim/bunker/bunker_description",
        "src/vendor/bunker_description",
        True,
    ),
    "dh_ag95_description": (
        "vendor",
        "Ground/src/third_party/scout_cobot_sim/dh_ag95_description",
        "src/vendor/dh_ag95_description",
        True,
    ),
    "roboticsgroup_gazebo_plugins": (
        "vendor",
        "Ground/src/third_party/roboticsgroup_gazebo_plugins",
        "src/vendor/roboticsgroup_gazebo_plugins",
        True,
    ),
    "sim_platform_bringup": (
        "platform",
        None,
        "src/platform/sim_platform_bringup",
        False,
    ),
    "sim_platform_assets": (
        "platform",
        None,
        "src/platform/sim_platform_assets",
        False,
    ),
    "bunker_sim_runtime": (
        "platform",
        None,
        "src/platform/bunker_sim_runtime",
        False,
    ),
    "ground_runtime_compat": (
        "platform",
        None,
        "src/platform/ground_runtime_compat",
        False,
    ),
    "air_ground_pose_bridge": (
        "platform",
        None,
        "src/platform/air_ground_pose_bridge",
        False,
    ),
    "air_ground_pick_demo": (
        "demo",
        None,
        "src/demos/air_ground_pick_demo",
        False,
    ),
}
EXPECTED_PACKAGES = set(EXPECTED_PACKAGE_RECORDS)
EXPECTED_AUXILIARY_IMPORTS = (
    (
        "geometry_utils",
        "Modules/common/include/geometry_utils.h",
        "src/p450/prometheus_uav_control/vendor_upstream/common/include/geometry_utils.h",
    ),
    (
        "math_utils",
        "Modules/common/include/math_utils.h",
        "src/p450/prometheus_uav_control/vendor_upstream/common/include/math_utils.h",
    ),
    (
        "printf_utils",
        "Modules/common/include/printf_utils.h",
        "src/p450/prometheus_uav_control/vendor_upstream/common/include/printf_utils.h",
    ),
    (
        "param_manager_header",
        "Modules/communication/include/param_manager.hpp",
        "src/p450/prometheus_uav_control/vendor_upstream/communication/include/param_manager.hpp",
    ),
    (
        "message_convert_header",
        "Modules/communication/include/message_convert.hpp",
        "src/p450/prometheus_uav_control/vendor_upstream/communication/include/message_convert.hpp",
    ),
    (
        "param_manager_source",
        "Modules/communication/src/param_manager.cpp",
        "src/p450/prometheus_uav_control/vendor_upstream/communication/src/param_manager.cpp",
    ),
    (
        "upstream_license",
        "LICENSE",
        "docs/upstream/P450-SIM-LICENSE",
    ),
    (
        "third_party_licenses",
        "THIRD_PARTY_LICENSES.md",
        "docs/upstream/THIRD_PARTY_LICENSES.md",
    ),
    (
        "ground_upstreams",
        "Ground/upstream.repos",
        "docs/upstream/Ground-upstream.repos",
    ),
    (
        "source_provenance",
        "docs/SOURCE_PROVENANCE.md",
        "docs/upstream/SOURCE_PROVENANCE.md",
    ),
    (
        "historical_source_lock",
        "manifests/source-lock.json",
        "docs/upstream/historical-source-lock.json",
    ),
)
EXPECTED_REQUIRED_PATHS = (
    "dependencies",
    "src/p450",
    "src/ground",
    "src/vendor",
    "src/platform",
    "src/demos",
    "config",
    "tools",
    "tests",
    "docs",
)
FORBIDDEN_PACKAGES = {
    "paper_benchmark", "ground_aerial_benchmark", "system_baseline_freeze",
    "task_aware_approach", "aerial_traversability", "irm_base_placement",
    "aerial_ground_bridge",
}
EXPECTED_FORBIDDEN_TOKENS = (
    "/m5/",
    "m5_",
    "rbp_",
    "task_aware",
    "task-aware",
)

class FrozenSourceContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.manifest = RuntimeManifest.load(ROOT / "config/runtime_sources.json")

    def test_exact_package_allowlist(self):
        self.assertEqual(EXPECTED_PACKAGES, set(self.manifest.packages))

    def test_exact_package_records(self):
        actual_records = {
            name: (
                package.role,
                package.source,
                package.destination,
                package.imported,
            )
            for name, package in self.manifest.packages.items()
        }
        self.assertEqual(EXPECTED_PACKAGE_RECORDS, actual_records)

    def test_exact_auxiliary_imports(self):
        actual_imports = tuple(
            (item.name, item.source, item.destination)
            for item in self.manifest.auxiliary_imports
        )
        self.assertEqual(EXPECTED_AUXILIARY_IMPORTS, actual_imports)

    def test_exact_required_paths(self):
        self.assertEqual(EXPECTED_REQUIRED_PATHS, self.manifest.required_paths)

    def test_exact_forbidden_packages(self):
        self.assertEqual(FORBIDDEN_PACKAGES, set(self.manifest.forbidden_packages))
        self.assertFalse(EXPECTED_PACKAGES & FORBIDDEN_PACKAGES)

    def test_exact_forbidden_tokens(self):
        self.assertEqual(EXPECTED_FORBIDDEN_TOKENS, self.manifest.forbidden_tokens)

    def test_upstream_commit_is_actual_audited_checkout(self):
        self.assertEqual(
            "6809c15e3919d1aa3acb6518ad61c49e4150435f",
            self.manifest.expected_commit,
        )

    def test_imported_packages_have_sources(self):
        for package in self.manifest.packages.values():
            if package.imported:
                self.assertIsNotNone(package.source, package.name)
            else:
                self.assertIsNone(package.source, package.name)

if __name__ == "__main__":
    unittest.main()
