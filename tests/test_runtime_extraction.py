import hashlib
import json
import re
import stat
import tempfile
import unittest
from pathlib import Path

from tools.runtime_manifest import RuntimeManifest


ROOT = Path(__file__).resolve().parents[1]
BASE_PROVENANCE = ROOT / "config/import_provenance.json"
OVERLAY = ROOT / "config/runtime_overlay.json"
EXPECTED_BASE_PROVENANCE_SHA256 = (
    "6cc35f32f30cc1970350eb4daa6f678b35cedc900ba1c645c0c159bec97abd20"
)
EXPECTED_UPSTREAM_COMMIT = "6809c15e3919d1aa3acb6518ad61c49e4150435f"

EXACT_REMOVED = frozenset({
    "src/p450/brick_aerial_perception/scripts/m1_validation_monitor.py",
    "src/p450/brick_aerial_perception/config/validation_scenarios.yaml",
    "src/p450/brick_aerial_perception/test/test_validation.py",
    "src/p450/brick_aerial_perception/src/brick_aerial_perception/validation.py",
    "src/p450/brick_aerial_perception/docs/TEST_REPORT.md",
    "src/p450/brick_aerial_perception/scripts/run_m1_demo.bash",
    "src/p450/brick_aerial_perception/scripts/aerial_viewpoint_mission_multiview.py",
    "src/ground/bunker_navigation/scripts/approach_trial_monitor.py",
    "src/ground/bunker_navigation/scripts/run_approach_matrix.py",
    "src/ground/bunker_navigation/scripts/run_navigation_matrix.py",
    "src/ground/bunker_navigation/scripts/navigation_trial_monitor.py",
    "src/ground/bunker_navigation/config/navigation_scenarios.yaml",
    "src/ground/bunker_navigation/config/approach_scenarios.yaml",
    "src/ground/bunker_navigation/launch/navigation_trial.launch",
    "src/ground/bunker_navigation/launch/approach_pose_trial.launch",
    "src/ground/bunker_navigation/test/test_metrics.py",
    "src/ground/bunker_navigation/test/test_approach_metrics.py",
    "src/ground/bunker_navigation/src/bunker_navigation/metrics.py",
    "src/ground/bunker_navigation/src/bunker_navigation/approach_metrics.py",
    "src/ground/bunker_navigation/scripts/probe_rear_obstacle_safety.py",
    "src/ground/bunker_navigation/launch/rear_obstacle_safety_probe.launch",
    "src/ground/bunker_navigation/models/rear_gate_obstacle.sdf",
    "src/ground/brick_rgbd_perception/launch/pose_robustness_validation.launch",
    "src/ground/brick_rgbd_perception/test/test_validation.py",
    "src/ground/brick_rgbd_perception/src/brick_rgbd_perception/validation.py",
    "src/ground/brick_rgbd_perception/scripts/validate_pose_robustness.py",
    "src/ground/brick_rgbd_perception/config/brick_pose_robustness.yaml",
    "src/ground/brick_visual_pick/scripts/visual_pick_trial_monitor.py",
    "src/ground/brick_visual_pick/config/visual_pick_scenarios.yaml",
    "src/ground/brick_visual_pick/launch/visual_pick_trial.launch",
    "src/ground/brick_visual_pick/test/test_matrix_report.py",
    "src/ground/brick_visual_pick/test/test_trial_results.py",
    "src/ground/brick_visual_pick/test/test_pose_metrics.py",
    "src/ground/brick_visual_pick/src/brick_visual_pick/matrix_report.py",
    "src/ground/brick_visual_pick/src/brick_visual_pick/pose_metrics.py",
    "src/ground/brick_visual_pick/src/brick_visual_pick/trial_results.py",
    "src/ground/ground_pick_orchestrator/scripts/run_ground_pick_matrix.py",
    "src/ground/ground_pick_orchestrator/scripts/ground_pick_trial_monitor.py",
    "src/ground/ground_pick_orchestrator/config/mission_scenarios.yaml",
    "src/ground/ground_pick_orchestrator/launch/ground_pick_trial.launch",
    "src/ground/ground_pick_orchestrator/test/test_mission_metrics.py",
    "src/ground/ground_pick_orchestrator/src/ground_pick_orchestrator/mission_metrics.py",
})

EXACT_MODIFIED = frozenset({
    "src/p450/brick_aerial_perception/CMakeLists.txt",
    "src/p450/brick_aerial_perception/launch/m1_aerial_perception.launch",
    "src/p450/brick_aerial_perception/README.md",
    "src/p450/brick_aerial_perception/test/test_ros_contract.py",
    "src/ground/bunker_navigation/CMakeLists.txt",
    "src/ground/bunker_navigation/test/test_configuration.py",
    "src/ground/brick_rgbd_perception/CMakeLists.txt",
    "src/ground/brick_visual_pick/CMakeLists.txt",
    "src/ground/brick_visual_pick/test/test_configuration.py",
    "src/ground/ground_pick_orchestrator/CMakeLists.txt",
    "src/ground/ground_pick_orchestrator/test/test_configuration.py",
    "src/ground/ground_pick_orchestrator/config/mission.yaml",
    "src/ground/ground_pick_orchestrator/scripts/ground_pick_node.py",
})

AFFECTED_PACKAGE_ROOTS = (
    ROOT / "src/p450/brick_aerial_perception",
    ROOT / "src/ground/bunker_navigation",
    ROOT / "src/ground/brick_rgbd_perception",
    ROOT / "src/ground/brick_visual_pick",
    ROOT / "src/ground/ground_pick_orchestrator",
)


def _sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _is_regular_file(path):
    return stat.S_ISREG(path.lstat().st_mode)


def _load_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def _find_removed_entrypoint_references(root, package_roots, removed_paths):
    candidates = set()
    excluded_python_directories = {
        "docs", "generated", "results", "test", "tests", "__pycache__",
    }
    for package_root in package_roots:
        cmake = package_root / "CMakeLists.txt"
        if cmake.is_file():
            candidates.add(cmake)
        launch_root = package_root / "launch"
        if launch_root.is_dir():
            candidates.update(path for path in launch_root.rglob("*.launch")
                              if path.is_file())
        candidates.update(
            path for path in package_root.rglob("*.py")
            if (path.is_file() and not any(
                part in excluded_python_directories
                for part in path.relative_to(package_root).parts[:-1]
            ))
        )

    findings = set()
    for candidate in candidates:
        candidate_relative = candidate.relative_to(root).as_posix()
        if candidate_relative in removed_paths:
            continue
        text = candidate.read_text(encoding="utf-8")
        for removed in removed_paths:
            removed_path = Path(removed)
            package_relative = Path(*removed_path.parts[3:]).as_posix()
            package_qualified = Path(*removed_path.parts[2:]).as_posix()
            needles = {
                removed,
                package_relative,
                package_qualified,
                removed_path.name,
            }
            if (removed_path.suffix == ".py" and
                    len(removed_path.parts) > 5 and
                    removed_path.parts[3] == "src"):
                module_parts = list(removed_path.parts[4:])
                module_parts[-1] = Path(module_parts[-1]).stem
                needles.add(".".join(module_parts))
            if any(needle in text for needle in needles):
                findings.add((candidate_relative, removed))
    return tuple(sorted(findings))


class RuntimeExtractionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.manifest = RuntimeManifest.load(ROOT / "config/runtime_sources.json")
        cls.provenance = _load_json(BASE_PROVENANCE)

    def _overlay(self):
        self.assertTrue(OVERLAY.is_file(), OVERLAY.relative_to(ROOT).as_posix())
        return _load_json(OVERLAY)

    def _current_inherited_files(self):
        current = set()
        for spec in self.manifest.packages.values():
            if not spec.imported:
                continue
            package_root = ROOT / spec.destination
            for path in package_root.rglob("*"):
                if _is_regular_file(path):
                    current.add(path.relative_to(ROOT).as_posix())
        for item in self.manifest.auxiliary_imports:
            path = ROOT / item.destination
            if _is_regular_file(path):
                current.add(item.destination)
        return current

    def test_base_provenance_is_the_frozen_import_snapshot(self):
        self.assertEqual(EXPECTED_BASE_PROVENANCE_SHA256,
                         _sha256(BASE_PROVENANCE))
        self.assertEqual(1, self.provenance["schema_version"])
        self.assertEqual(EXPECTED_UPSTREAM_COMMIT,
                         self.provenance["upstream_commit"])
        self.assertEqual(867, len(self.provenance["files"]))

    def test_overlay_schema_and_exact_change_ledger(self):
        overlay = self._overlay()
        self.assertEqual(
            {"schema_version", "base_provenance",
             "base_provenance_sha256", "upstream_commit", "removed",
             "modified"},
            set(overlay),
        )
        self.assertEqual(1, overlay["schema_version"])
        self.assertEqual("config/import_provenance.json",
                         overlay["base_provenance"])
        self.assertEqual(EXPECTED_BASE_PROVENANCE_SHA256,
                         overlay["base_provenance_sha256"])
        self.assertEqual(EXPECTED_UPSTREAM_COMMIT,
                         overlay["upstream_commit"])
        self.assertEqual(sorted(EXACT_REMOVED), overlay["removed"])
        self.assertEqual(sorted(EXACT_MODIFIED),
                         list(overlay["modified"]))
        self.assertEqual(42, len(overlay["removed"]))
        self.assertEqual(13, len(overlay["modified"]))

    def test_removed_files_are_tracked_upstream_and_absent_now(self):
        recorded = self.provenance["files"]
        for relative in sorted(EXACT_REMOVED):
            self.assertIn(relative, recorded)
            self.assertFalse((ROOT / relative).exists(), relative)

    def test_modified_files_match_overlay_and_differ_from_upstream(self):
        overlay = self._overlay()
        recorded = self.provenance["files"]
        for relative in sorted(EXACT_MODIFIED):
            path = ROOT / relative
            self.assertIn(relative, recorded)
            self.assertTrue(path.is_file(), relative)
            self.assertTrue(_is_regular_file(path), relative)
            current_digest = _sha256(path)
            self.assertEqual(overlay["modified"][relative], current_digest,
                             relative)
            self.assertNotEqual(recorded[relative], current_digest, relative)

    def test_unchanged_files_still_match_base_provenance(self):
        changed = EXACT_REMOVED | EXACT_MODIFIED
        for relative, expected_digest in self.provenance["files"].items():
            if relative in changed:
                continue
            path = ROOT / relative
            self.assertTrue(path.is_file(), relative)
            self.assertTrue(_is_regular_file(path), relative)
            self.assertEqual(expected_digest, _sha256(path), relative)

    def test_current_inherited_set_is_exactly_base_minus_removed(self):
        expected = set(self.provenance["files"]) - EXACT_REMOVED
        current = self._current_inherited_files()
        self.assertEqual(825, len(expected))
        self.assertEqual(expected, current)

    def test_removed_entrypoints_have_no_active_references(self):
        package_roots = tuple(
            ROOT / spec.destination
            for spec in self.manifest.packages.values()
            if spec.imported
        )
        self.assertEqual(
            (),
            _find_removed_entrypoint_references(
                ROOT, package_roots, EXACT_REMOVED),
        )

    def test_removed_entrypoint_scan_catches_cross_package_references(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture_root = Path(temporary)
            source_package = fixture_root / "src/p450/source_package"
            consumer_package = fixture_root / "src/ground/consumer_package"
            (source_package / "scripts").mkdir(parents=True)
            (consumer_package / "scripts").mkdir(parents=True)
            reference = consumer_package / "scripts/cross_package_reference.py"
            reference.write_text(
                "ENTRYPOINT = 'retired_entrypoint.py'\n"
                "import source_package.retired_module\n",
                encoding="utf-8")

            findings = _find_removed_entrypoint_references(
                fixture_root,
                (source_package, consumer_package),
                frozenset({
                    "src/p450/source_package/scripts/retired_entrypoint.py",
                    "src/p450/source_package/src/source_package/"
                    "retired_module.py",
                }),
            )

            self.assertEqual(
                (
                    ("src/ground/consumer_package/scripts/"
                     "cross_package_reference.py",
                     "src/p450/source_package/scripts/retired_entrypoint.py"),
                    ("src/ground/consumer_package/scripts/"
                     "cross_package_reference.py",
                     "src/p450/source_package/src/source_package/"
                     "retired_module.py"),
                ),
                findings,
            )

    def test_affected_cmake_references_only_existing_scripts_and_tests(self):
        for package_root in AFFECTED_PACKAGE_ROOTS:
            cmake_path = package_root / "CMakeLists.txt"
            cmake = cmake_path.read_text(encoding="utf-8")
            for relative in re.findall(
                    r"(?:scripts|test)/[A-Za-z0-9_.+/-]+", cmake):
                self.assertTrue((package_root / relative).is_file(),
                                "%s: %s" %
                                (cmake_path.relative_to(ROOT), relative))

    def test_aerial_readme_describes_a_component_not_old_runner(self):
        readme = (ROOT / "src/p450/brick_aerial_perception/README.md") \
            .read_text(encoding="utf-8")
        self.assertNotIn("run_m1_demo", readme)
        self.assertNotIn("/home/lu/", readme)
        self.assertNotIn("validation_output", readme)
        self.assertIn("sim_platform_bringup", readme)
        self.assertIn("component", readme.lower())

    def test_runtime_sources_have_no_m5_or_formal_compatibility_symbols(self):
        active_text = []
        for spec in self.manifest.packages.values():
            if not spec.imported:
                continue
            package_root = ROOT / spec.destination
            for path in package_root.rglob("*"):
                if (path.is_file() and not path.is_symlink() and
                        (path.name == "CMakeLists.txt" or
                         path.suffix in {".py", ".yaml", ".launch"})):
                    active_text.append((path.relative_to(ROOT).as_posix(),
                                        path.read_text(encoding="utf-8")))
        for relative, text in active_text:
            self.assertIsNone(re.search(r"(?i)(?:/m5/|\bm5[_-])", text),
                              relative)

        ground_root = ROOT / "src/ground/ground_pick_orchestrator"
        ground_text = []
        for path in ground_root.rglob("*"):
            if (path.is_file() and not path.is_symlink() and
                    (path.name == "CMakeLists.txt" or
                     path.suffix in {".py", ".yaml", ".launch"})):
                ground_text.append(path.read_text(encoding="utf-8"))
        combined = "\n".join(ground_text)
        self.assertNotIn("formal_pose", combined)
        self.assertIn("legacy_pick_pose", combined)

        config = (ground_root / "config/mission.yaml").read_text(
            encoding="utf-8")
        node = (ground_root / "scripts/ground_pick_node.py").read_text(
            encoding="utf-8")
        self.assertIn("legacy_pick_pose_topic: /brick_pose", config)
        self.assertIn("approved_pose_topic: /ground_pick/approved_brick_pose",
                      config)
        self.assertIn("'~legacy_pick_pose_topic', '/brick_pose'", node)
        self.assertIn("'/ground_pick/approved_brick_pose'", node)


if __name__ == "__main__":
    unittest.main()
