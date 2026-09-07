"""The public localization edge uses stamped physical pose, never spawn Z."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
BRINGUP = ROOT / 'src/platform/sim_platform_bringup'
MODELS = ROOT / 'src/p450/prometheus_gazebo/gazebo_models/uav_models'


class SimLocalizationWiringTests(unittest.TestCase):
    def test_both_uav_profiles_supply_stamped_base_pose_not_sensor_pose(self):
        for profile in ('p450_D435i', 'p450_D435i_mid360'):
            with self.subTest(profile=profile):
                model = ET.parse(str(MODELS / profile / (profile + '.sdf.jinja'))).getroot()
                plugin = model.find("./model/plugin[@name='sim_base_pose']")
                self.assertIsNotNone(plugin)
                self.assertEqual(plugin.get('filename'), 'libgazebo_ros_p3d.so')
                expected = dict(bodyName='base_link', frameName='world',
                                topicName='/sim/uav1/base_pose', updateRate='100',
                                gaussianNoise='0', xyzOffset='0 0 0', rpyOffset='0 0 0')
                for name, value in expected.items():
                    self.assertEqual(plugin.findtext(name), value)

    def test_both_standalones_have_one_dynamic_localization_authority(self):
        for filename in ('p450_standalone.launch', 'air_ground_standalone.launch'):
            with self.subTest(filename=filename):
                root = ET.parse(str(BRINGUP / 'launch' / filename)).getroot()
                nodes = root.findall("node[@name='sim_localization_uav1']")
                self.assertEqual(len(nodes), 1)
                self.assertEqual(nodes[0].get('pkg'), 'sim_platform_bringup')
                self.assertEqual(nodes[0].get('type'), 'sim_uav_localization.py')
                self.assertIsNone(nodes[0].get('args'))
                static = root.findall("node[@type='static_transform_publisher']")
                self.assertFalse(any('uav1/odom' in n.get('args', '') for n in static))
                self.assertEqual(root.find("node[@name='sim_world_to_map']").get('args'),
                                 '0 0 0 0 0 0 1 world map')

    def test_python_adapter_is_installed_with_declared_message_dependencies(self):
        cmake = (BRINGUP / 'CMakeLists.txt').read_text()
        self.assertIn('scripts/sim_uav_localization.py', cmake)
        deps = {n.text for n in ET.parse(str(BRINGUP / 'package.xml')).getroot().findall('exec_depend')}
        self.assertTrue({'rospy', 'nav_msgs', 'geometry_msgs', 'tf', 'tf2_ros'} <= deps)


if __name__ == '__main__':
    unittest.main()
