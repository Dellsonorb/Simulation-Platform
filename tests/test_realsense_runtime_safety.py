import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE_PATH = (
    ROOT
    / "src/p450/realsense_ros_gazebo/src/RealSensePlugin.cpp"
)


def _section(source, start, end=None):
    try:
        section = source.split(start, 1)[1]
    except IndexError as error:
        raise AssertionError("missing source section: {}".format(start)) from error
    if end is not None:
        try:
            section = section.split(end, 1)[0]
        except IndexError as error:
            raise AssertionError("missing source section: {}".format(end)) from error
    return section


class RealSenseRuntimeSafetyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = SOURCE_PATH.read_text(encoding="utf-8")
        cls.load = _section(
            cls.source,
            "void RealSensePlugin::Load(",
            "void RealSensePlugin::OnNewFrame(",
        )
        cls.on_update = _section(
            cls.source,
            "void RealSensePlugin::OnUpdate()",
        )

    def test_model_load_defers_renderer_binding(self):
        self.assertNotIn("SensorManager::Instance", self.load)
        self.assertNotIn("GetSensors()", self.load)
        self.assertIn("ConnectWorldUpdateBegin", self.load)

    def test_deferred_binding_is_model_scoped_and_has_no_order_hack(self):
        self.assertIn("SensorManager::Instance", self.on_update)
        self.assertIn("ScopedName()", self.on_update)
        self.assertIn(
            '"::" + this->rsModel->GetName() + "::"',
            self.on_update,
        )
        self.assertNotIn("bool color", self.source)
        self.assertNotIn("->Camera()->Name()", self.source)
        self.assertIsNone(
            re.search(
                r"dynamic_pointer_cast<[^;\n]+>\(sensor\)->",
                self.source,
            )
        )

    def test_all_four_renderers_are_ready_before_callback_wiring(self):
        candidates = (
            "depthCandidate",
            "colorCandidate",
            "ired1Candidate",
            "ired2Candidate",
        )
        for candidate in candidates:
            self.assertIn(candidate, self.on_update)
        readiness = (
            "if (!depthCandidate || !colorCandidate || "
            "!ired1Candidate || !ired2Candidate)"
        )
        self.assertIn(readiness, self.on_update)
        readiness_position = self.on_update.index(readiness)
        assignment_position = self.on_update.index(
            "this->depthCam = depthCandidate;"
        )
        callback_position = self.on_update.index("ConnectNewDepthFrame")
        self.assertLess(readiness_position, assignment_position)
        self.assertLess(assignment_position, callback_position)
        self.assertIn("this->updateConnection.reset();", self.on_update)


if __name__ == "__main__":
    unittest.main()
