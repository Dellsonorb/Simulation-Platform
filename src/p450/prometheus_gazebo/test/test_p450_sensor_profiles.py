#!/usr/bin/env python3

import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
MODEL_ROOT = PACKAGE_ROOT / "gazebo_models" / "uav_models"
LAUNCH_ROOT = PACKAGE_ROOT / "launch_uav_with_sensor"
JINJA_GENERATOR = PACKAGE_ROOT / "scripts" / "jinja_gen.py"


def generate_sdf(model_name: str) -> str:
    model = MODEL_ROOT / model_name / f"{model_name}.sdf.jinja"
    with tempfile.NamedTemporaryFile(suffix=".sdf") as output:
        subprocess.run(
            [
                "python3",
                str(JINJA_GENERATOR),
                "--stdout",
                "--mavlink_id=1",
                "--mavlink_udp_port=14560",
                "--mavlink_tcp_port=4560",
                str(model),
                str(PACKAGE_ROOT),
            ],
            check=True,
            stdout=output,
        )
        output.seek(0)
        return output.read().decode("utf-8")


class P450SensorProfileTest(unittest.TestCase):
    def test_mid360_profiles_generate_valid_xml(self):
        for profile in ("p450_mid360", "p450_D435i_mid360"):
            with self.subTest(profile=profile):
                sdf = generate_sdf(profile)
                ET.fromstring(sdf)
                self.assertIn("<ros_topic>/uav1/livox/lidar</ros_topic>", sdf)

    def test_legacy_3d_lidar_profile_generates_valid_xml(self):
        sdf = generate_sdf("p450_3Dlidar")
        ET.fromstring(sdf)
        self.assertIn("model://3Dlidar", sdf)

    def test_d435i_profiles_publish_namespaced_camera_imu(self):
        for profile in ("p450_D435i", "p450_D435i_mid360"):
            with self.subTest(profile=profile):
                sdf = generate_sdf(profile)
                ET.fromstring(sdf)
                self.assertIn("<topicName>/uav1/camera/imu</topicName>", sdf)
                self.assertIn("<frameName>/uav1/camera_imu_link</frameName>", sdf)

    def test_legacy_3d_lidar_launch_uses_current_vehicle_interface(self):
        launch = ET.parse(LAUNCH_ROOT / "sitl_p450_3dlidar.launch")
        include = next(
            node
            for node in launch.findall(".//include")
            if "sitl_px4_outdoor.launch" in node.attrib.get("file", "")
        )
        names = {arg.attrib["name"] for arg in include.findall("arg")}
        self.assertIn("vehicle", names)
        self.assertNotIn("sdf", names)
        self.assertNotIn("model", names)
        args = {arg.attrib["name"]: arg.attrib.get("value") for arg in include.findall("arg")}
        self.assertEqual("p450", args.get("px4_vehicle"))

    def test_px4_airframe_is_decoupled_from_sensor_model(self):
        basic = ET.parse(PACKAGE_ROOT / "launch_basic/sitl_px4_outdoor.launch").getroot()
        args = {arg.attrib["name"]: arg.attrib.get("default") for arg in basic.findall("arg")}
        self.assertIn("px4_vehicle", args)
        px4_env = next(node for node in basic.findall("env") if node.attrib.get("name") == "PX4_SIM_MODEL")
        self.assertEqual("$(arg px4_vehicle)", px4_env.attrib.get("value"))

        for filename in (
            "sitl_p450_2dlidar.launch",
            "sitl_p450_3dlidar.launch",
            "sitl_p450_d435i.launch",
            "sitl_p450_mid360.launch",
            "sitl_p450_d435i_mid360.launch",
        ):
            with self.subTest(filename=filename):
                root = ET.parse(LAUNCH_ROOT / filename).getroot()
                include = next(
                    node for node in root.findall("include")
                    if "sitl_px4_outdoor.launch" in node.attrib.get("file", "")
                )
                values = {arg.attrib["name"]: arg.attrib.get("value") for arg in include.findall("arg")}
                self.assertEqual("p450", values.get("px4_vehicle"))

    def test_p450_mid360_launch_profiles_exist(self):
        launch_dir = LAUNCH_ROOT
        expectations = {
            "sitl_p450_mid360.launch": "p450_mid360",
            "sitl_p450_d435i_mid360.launch": "p450_D435i_mid360",
        }
        for filename, vehicle in expectations.items():
            with self.subTest(filename=filename):
                root = ET.parse(launch_dir / filename).getroot()
                args = {arg.attrib["name"]: arg.attrib.get("default") for arg in root.findall("arg")}
                self.assertEqual(vehicle, args.get("vehicle"))
                self.assertIn("gui", args)
                self.assertIn("rviz_enable", args)

    def test_fast_lio_topics_are_parameterized_by_uav_id(self):
        launch = (PACKAGE_ROOT.parents[1] / "Modules/FAST_LIO/launch/mapping_mid360_gazebo.launch").read_text()
        self.assertIn('name="lid_topic"', launch)
        self.assertIn('name="imu_topic"', launch)
        self.assertIn('name="common/lid_topic"', launch)
        self.assertIn('value="$(arg lid_topic)"', launch)
        self.assertIn('name="common/imu_topic"', launch)
        self.assertIn('value="$(arg imu_topic)"', launch)
        self.assertIn('name="odometry_log_dir"', launch)

        source = (PACKAGE_ROOT.parents[1] / "Modules/FAST_LIO/src/laserMapping.cpp").read_text()
        self.assertNotIn("/home/amov", source)
        self.assertIn('nh.param<string>("odometry_log_dir"', source)

    def test_octomap_outputs_are_sensor_specific(self):
        launch_dir = LAUNCH_ROOT
        depth = (launch_dir / "depth_to_octomap.launch").read_text()
        mid360 = (launch_dir / "mid360_to_octomap.launch").read_text()
        self.assertIn("/uav$(arg uav_id)/d435i/octomap_full", depth)
        self.assertIn("/uav$(arg uav_id)/d435i/octomap_binary", depth)
        self.assertIn("/uav$(arg uav_id)/mid360/octomap_full", mid360)
        self.assertIn("/uav$(arg uav_id)/mid360/octomap_binary", mid360)

    def test_mapping_composition_launch_connects_both_chains(self):
        path = LAUNCH_ROOT / "sitl_p450_sensor_mapping.launch"
        root = ET.parse(path).getroot()
        includes = [include.attrib["file"] for include in root.findall("include")]
        self.assertTrue(any("sitl_p450_d435i_mid360.launch" in item for item in includes))
        self.assertTrue(any("mapping_mid360_gazebo.launch" in item for item in includes))
        self.assertTrue(any("depth_to_octomap.launch" in item for item in includes))
        self.assertTrue(any("mid360_to_octomap.launch" in item for item in includes))

    def test_livox_plugin_resolves_scan_pattern_from_gazebo_model_path(self):
        plugin = (
            PACKAGE_ROOT.parents[1]
            / "Simulator/livox_laser_gazebo_plugins/src/livox_points_plugin.cpp"
        ).read_text()
        self.assertNotIn("/home/amov", plugin)
        self.assertIn('sdf->Get<std::string>("csv_file_name")', plugin)
        self.assertIn("FindFileURI", plugin)

    def test_octomap_inputs_have_connected_reference_frames(self):
        depth = (LAUNCH_ROOT / "depth_to_octomap.launch").read_text()
        mid360 = (LAUNCH_ROOT / "mid360_to_octomap.launch").read_text()
        combined = (LAUNCH_ROOT / "sitl_p450_d435i_mid360.launch").read_text()
        self.assertIn('name="frame_id" type="string" value="map"', depth)
        self.assertIn("/uav$(arg uav_id)/mid360_point_cloud_centers", mid360)
        self.assertIn("base_link /uav$(arg uav1_id)/camera_link", combined)
        self.assertIn("base_link /uav$(arg uav1_id)/camera_imu_link", combined)
        self.assertIn("base_link /uav$(arg uav1_id)/lidar_link", combined)
        self.assertIn("map_base_link base_link", combined)


if __name__ == "__main__":
    unittest.main()
