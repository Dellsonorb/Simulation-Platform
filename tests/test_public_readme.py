#!/usr/bin/env python3

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
README = ROOT / "README.md"


class PublicReadmeTest(unittest.TestCase):
    def test_readme_has_bilingual_entrypoints_and_runtime_sections(self):
        text = README.read_text(encoding="utf-8")
        for marker in (
                "中文概览", "English Documentation", "Architecture",
                "Robot-facing interfaces", "Prerequisites", "Build",
                "Run the platform", "Air-Ground Pick Demo",
                "Sim-to-Real boundary", "Repository layout",
                "Verification", "Known limitations"):
            self.assertIn(marker, text)

    def test_readme_commands_reference_real_files_and_packages(self):
        text = README.read_text(encoding="utf-8")
        self.assertIn("catkin build --profile p450-clean", text)
        self.assertIn("source install/p450-clean/setup.bash", text)
        for script in (
                "scripts/smoke_air_ground_standalone.bash",
                "scripts/smoke_air_ground_pick_demo.bash"):
            self.assertIn(script, text)
            self.assertTrue((ROOT / script).is_file())

    def test_readme_has_no_local_machine_path_or_real_backend_overclaim(self):
        text = README.read_text(encoding="utf-8")
        self.assertNotIn("/media/lu/", text)
        self.assertNotIn("/home/lu/", text)
        self.assertIn("not included in V1.0", text)


if __name__ == "__main__":
    unittest.main()
