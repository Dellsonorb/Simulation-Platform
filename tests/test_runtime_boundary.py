import json
import tempfile
import unittest
from dataclasses import FrozenInstanceError
from pathlib import Path

from tools.runtime_boundary import (
    ACTIVE_TEXT_NAMES,
    ACTIVE_TEXT_SUFFIXES,
    DYNAMIC_LIFECYCLE_TOKENS,
    INACTIVE_LEGACY_SPAWNS,
    JOINT_CANDIDATE_SPAWNS,
    STANDALONE_SMOKE_SPAWNS,
    Finding,
    active_text_paths,
    discover_packages,
    discover_startup_spawns,
    scan_active_content,
    scan_removed_references,
    validate_packages,
    validate_repository,
    validate_startup_spawns,
)


ROOT = Path(__file__).resolve().parents[1]


def _write(root, relative, content):
    path = root / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    if isinstance(content, bytes):
        path.write_bytes(content)
    else:
        path.write_text(content, encoding="utf-8")
    return path


def _package_xml(name, extra=""):
    return (
        "<package format=\"2\">\n"
        "  <name>%s</name>\n"
        "  <version>0.0.0</version>\n"
        "  <description>fixture</description>\n"
        "  <maintainer email=\"test@example.com\">Test</maintainer>\n"
        "  <license>MIT</license>\n"
        "%s\n"
        "</package>\n" % (name, extra)
    )


def _tokens(findings):
    return tuple(item.token for item in findings)


class FindingAndScopeTest(unittest.TestCase):
    def test_finding_is_frozen_ordered_and_results_are_deterministic(self):
        first = Finding("src/b.py", 2, "z")
        second = Finding("src/a.py", 1, "a")
        self.assertEqual((second, first), tuple(sorted((first, second))))
        with self.assertRaises(FrozenInstanceError):
            first.line = 9

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            _write(root, "src/pkg/a.py", "DeleteModel\n")
            one = scan_active_content(root, ())
            two = scan_active_content(root, ())
            self.assertEqual(one, two)
            self.assertEqual(tuple(sorted(one)), one)

    def test_exact_active_allowlist_filters_before_decode(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            expected = set()
            for name in ACTIVE_TEXT_NAMES:
                relative = "src/pkg/%s" % name
                _write(root, relative, "DeleteModel\n")
                expected.add(relative)
            for index, suffix in enumerate(sorted(ACTIVE_TEXT_SUFFIXES)):
                relative = "src/pkg/file_%02d%s" % (index, suffix)
                _write(root, relative, "DeleteModel\n")
                expected.add(relative)

            _write(root, "src/pkg/image.png", b"\xff\xfeSpawnModel")
            _write(root, "src/pkg/model.material", "DeleteModel\n")
            _write(root, "src/pkg/data.csv", "DeleteModel\n")
            _write(root, "src/pkg/notes.txt", "DeleteModel\n")
            _write(root, "src/pkg/MyCMakeLists.txt", "DeleteModel\n")

            discovered = {
                path.relative_to(root).as_posix()
                for path in active_text_paths(root)
            }
            self.assertEqual(expected, discovered)
            findings = scan_active_content(root, ())
            self.assertEqual(len(expected), len(findings))
            self.assertEqual({"DeleteModel"}, set(_tokens(findings)))

    def test_scope_and_component_exclusions_are_exact(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for prefix in ("README.py", "tools/tool.py", "tests/test.py",
                           "config/config.py"):
                _write(root, prefix, "DeleteModel\n")
            for directory in ("docs", "test", "tests", "results",
                              "generated", "__pycache__", ".pytest_cache",
                              "cache", "caches", ".cache", "license",
                              "licenses"):
                _write(root, "src/pkg/%s/ignored.py" % directory,
                       "DeleteModel\n")
            _write(root, "src/pkg/contest/found.py", "DeleteModel\n")
            _write(root, "src/pkg/documentation/found.py", "DeleteModel\n")

            self.assertEqual(
                ("src/pkg/contest/found.py",
                 "src/pkg/documentation/found.py"),
                tuple(path.relative_to(root).as_posix()
                      for path in active_text_paths(root)),
            )

    def test_invalid_utf8_in_active_file_is_a_structured_failure(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            _write(root, "src/pkg/broken.py", b"\xff\xfeDeleteModel")
            findings = scan_active_content(root, ())
            self.assertEqual(
                (Finding("src/pkg/broken.py", 0, "decode:utf-8"),),
                findings,
            )


class CommentAndTokenTest(unittest.TestCase):
    def _scan(self, relative, content, runtime_tokens=()):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            _write(root, relative, content)
            return scan_active_content(root, runtime_tokens)

    def test_comment_families_preserve_active_line_numbers(self):
        cases = (
            ("src/pkg/file.launch",
             "<!-- DeleteModel\nSpawnModel -->\n\nDeleteModel\n", 4),
            ("src/pkg/file.sdf.jinja",
             "{# DeleteModel\nSpawnModel #}\n<!-- DeleteModel -->\nDeleteModel\n", 4),
            ("src/pkg/file.cpp",
             "/* DeleteModel\nSpawnModel */\n// DeleteModel\nDeleteModel\n", 4),
            ("src/pkg/file.py",
             "# DeleteModel\nvalue = '''SpawnModel'''\n# DeleteModel\nDeleteModel\n", 2),
            ("src/pkg/file.yaml",
             "# DeleteModel\nvalue: ok # SpawnModel\n\nDeleteModel\n", 4),
            ("src/pkg/CMakeLists.txt",
             "# DeleteModel\nset(X ok) # SpawnModel\n\nDeleteModel\n", 4),
        )
        for relative, content, expected_line in cases:
            with self.subTest(relative=relative):
                findings = self._scan(relative, content)
                if relative.endswith(".py"):
                    self.assertEqual(
                        (Finding(relative, 2, "SpawnModel"),
                         Finding(relative, 4, "DeleteModel")), findings)
                else:
                    self.assertEqual(
                        (Finding(relative, expected_line, "DeleteModel"),),
                        findings,
                    )

    def test_quotes_keep_comment_markers_active_and_json_has_no_comments(self):
        cases = (
            ("src/pkg/url.cpp", 'const char *x = "http://x/SpawnModel";\n',
             "SpawnModel"),
            ("src/pkg/value.yaml", 'value: "# DeleteModel"\n',
             "DeleteModel"),
            ("src/pkg/value.json", '{"value": "DeleteModel"}\n',
             "DeleteModel"),
        )
        for relative, content, token in cases:
            with self.subTest(relative=relative):
                findings = self._scan(relative, content)
                self.assertEqual((Finding(relative, 1, token),), findings)

    def test_crlf_block_comments_preserve_line_number(self):
        findings = self._scan(
            "src/pkg/file.xml",
            b"<!-- DeleteModel\r\nSpawnModel -->\r\n\r\nDeleteModel\r\n",
        )
        self.assertEqual(
            (Finding("src/pkg/file.xml", 4, "DeleteModel"),), findings)

    def test_all_dynamic_lifecycle_tokens_are_detected(self):
        content = "\n".join(DYNAMIC_LIFECYCLE_TOKENS) + "\n"
        findings = self._scan("src/pkg/lifecycle.py", content)
        self.assertEqual(set(DYNAMIC_LIFECYCLE_TOKENS), set(_tokens(findings)))
        self.assertEqual(len(DYNAMIC_LIFECYCLE_TOKENS), len(findings))

    def test_startup_spawn_model_is_not_dynamic_lifecycle(self):
        findings = self._scan(
            "src/pkg/start.launch",
            '<launch><node pkg="gazebo_ros" type="spawn_model" '
            'name="startup"/></launch>\n',
        )
        self.assertEqual((), findings)

    def test_method_tokens_are_case_insensitive_and_m5_is_deduplicated(self):
        findings = self._scan(
            "src/pkg/method.py",
            "M5-only\nm5_value\n/m5/\nM5\nRBP_MODE\nTASK-AWARE\n",
            ("/m5/", "m5_", "rbp_", "task_aware", "task-aware"),
        )
        self.assertEqual(
            (
                Finding("src/pkg/method.py", 1, "method:m5"),
                Finding("src/pkg/method.py", 2, "method:m5"),
                Finding("src/pkg/method.py", 3, "method:m5"),
                Finding("src/pkg/method.py", 4, "method:m5"),
                Finding("src/pkg/method.py", 5, "method:rbp_"),
                Finding("src/pkg/method.py", 6, "method:task-aware"),
            ),
            findings,
        )

    def test_m5_identifier_boundaries_avoid_near_names(self):
        findings = self._scan(
            "src/pkg/near.py", "param5\nam5-foo\nm50\nm5camera\n",
            ("/m5/", "m5_"),
        )
        self.assertEqual((), findings)


class PackageBoundaryTest(unittest.TestCase):
    def _validate_fixture(self, files, expected=None, forbidden=None):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for relative, content in files.items():
                _write(root, relative, content)
            return validate_packages(
                root,
                expected or {"alpha": "src/p450/alpha"},
                forbidden or {"paper_benchmark"},
            )

    def test_real_repository_has_exact_imported_package_set(self):
        manifest = json.loads(
            (ROOT / "config/runtime_sources.json").read_text(encoding="utf-8"))
        expected = {
            name: item["destination"]
            for name, item in manifest["packages"].items()
            if item["imported"]
        }
        records = discover_packages(ROOT)
        self.assertEqual(18, len(records))
        self.assertEqual(set(expected), {item.name for item in records})
        self.assertEqual((), validate_packages(
            ROOT, expected, set(manifest["forbidden"]["package_names"])))

    def test_package_mutations_are_rejected(self):
        baseline = {
            "src/p450/alpha/package.xml": _package_xml("alpha"),
        }
        self.assertEqual((), self._validate_fixture(baseline))

        cases = {
            "unlisted": {
                **baseline,
                "src/ground/unknown/package.xml": _package_xml("unknown"),
            },
            "duplicate": {
                **baseline,
                "src/ground/duplicate/package.xml": _package_xml("alpha"),
            },
            "nested": {
                **baseline,
                "src/p450/alpha/nested/package.xml": _package_xml("nested"),
            },
            "wrong-path": {
                "src/ground/alpha/package.xml": _package_xml("alpha"),
            },
            "wrong-name": {
                "src/p450/alpha/package.xml": _package_xml("wrong"),
            },
            "malformed": {
                "src/p450/alpha/package.xml": "<package><name>alpha",
            },
        }
        for token, files in cases.items():
            with self.subTest(token=token):
                findings = self._validate_fixture(files)
                self.assertTrue(findings)
                self.assertTrue(any(token in item.token for item in findings),
                                findings)

    def test_all_dependency_tags_reject_forbidden_packages(self):
        tags = (
            "depend", "build_depend", "build_export_depend", "exec_depend",
            "run_depend", "test_depend", "buildtool_depend",
        )
        for tag in tags:
            with self.subTest(tag=tag):
                extra = "  <%s condition=\"$ROS_VERSION == 1\">\n" \
                        "    paper_benchmark\n  </%s>" % (tag, tag)
                findings = self._validate_fixture({
                    "src/p450/alpha/package.xml": _package_xml("alpha", extra),
                })
                self.assertTrue(any(
                    item.token == "dependency:paper_benchmark"
                    for item in findings), findings)

    def test_non_dependency_text_does_not_trigger_dependency_rule(self):
        package = _package_xml(
            "alpha",
            "  <!-- paper_benchmark -->\n"
            "  <url>paper_benchmark</url>",
        )
        self.assertEqual((), self._validate_fixture({
            "src/p450/alpha/package.xml": package,
        }))


class StartupSpawnBoundaryTest(unittest.TestCase):
    def test_real_startup_spawns_are_exactly_classified(self):
        expected = (JOINT_CANDIDATE_SPAWNS | STANDALONE_SMOKE_SPAWNS |
                    INACTIVE_LEGACY_SPAWNS)
        self.assertEqual((3, 3, 6), (
            len(JOINT_CANDIDATE_SPAWNS), len(STANDALONE_SMOKE_SPAWNS),
            len(INACTIVE_LEGACY_SPAWNS)))
        self.assertFalse(JOINT_CANDIDATE_SPAWNS & STANDALONE_SMOKE_SPAWNS)
        self.assertFalse(JOINT_CANDIDATE_SPAWNS & INACTIVE_LEGACY_SPAWNS)
        self.assertFalse(STANDALONE_SMOKE_SPAWNS & INACTIVE_LEGACY_SPAWNS)
        actual = discover_startup_spawns(ROOT)
        self.assertEqual(12, len(actual))
        self.assertEqual(expected, set(actual))
        self.assertEqual((), validate_startup_spawns(ROOT))

    def test_unclassified_duplicate_missing_and_structural_changes_fail(self):
        identity = ("src/pkg/start.launch", "known")

        def validate(content):
            with tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                if content is not None:
                    _write(root, identity[0], content)
                return validate_startup_spawns(
                    root, frozenset({identity}), frozenset(), frozenset())

        node = '<node pkg="gazebo_ros" type="spawn_model" '
        baseline = '<launch>' + node + 'name="known"/></launch>\n'
        self.assertEqual((), validate(baseline))

        cases = (
            (baseline.replace("</launch>",
                              node + 'name="extra"/></launch>'),
             "startup-spawn:unclassified"),
            (baseline.replace("</launch>",
                              node + 'name="known"/></launch>'),
             "startup-spawn:duplicate"),
            (None, "startup-spawn:missing"),
            (baseline.replace('pkg="gazebo_ros"', 'pkg="other"'),
             "startup-spawn:missing"),
        )
        for content, token in cases:
            with self.subTest(token=token):
                self.assertTrue(any(
                    item.token.startswith(token) for item in validate(content)
                ))


class RemovedReferenceBoundaryTest(unittest.TestCase):
    def test_real_overlay_drives_all_removed_references(self):
        overlay = json.loads(
            (ROOT / "config/runtime_overlay.json").read_text(encoding="utf-8"))
        self.assertEqual(42, len(overlay["removed"]))
        self.assertEqual((), scan_removed_references(ROOT, overlay["removed"]))

    def test_path_and_python_reference_forms_are_detected_without_near_names(self):
        removed_script = "src/p450/source_pkg/scripts/old_entry.py"
        removed_module = (
            "src/p450/source_pkg/src/source_pkg/old_module.py")
        positive_cases = (
            ("VALUE = '%s'\n" % removed_script, removed_script),
            ("VALUE = 'scripts/old_entry.py'\n", removed_script),
            ("VALUE = 'old_entry.py'\n", removed_script),
            ("import source_pkg.old_module\n", removed_module),
            ("from source_pkg import old_module\n", removed_module),
        )
        for index, (content, expected) in enumerate(positive_cases):
            with self.subTest(index=index):
                with tempfile.TemporaryDirectory() as temporary:
                    root = Path(temporary)
                    _write(root, "src/ground/consumer/file.py", content)
                    findings = scan_removed_references(
                        root, (removed_script, removed_module))
                    self.assertEqual(1, len(findings), findings)
                    self.assertEqual("removed:%s" % expected,
                                     findings[0].token)

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            _write(root, "src/ground/consumer/file.py",
                   "# old_entry.py\n"
                   "import source_pkg.old_module_helpers\n")
            self.assertEqual((), scan_removed_references(
                root, (removed_script, removed_module)))


class RepositoryIntegrationTest(unittest.TestCase):
    def _production_delta(self, mutation, removed=()):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            _write(root, "config/runtime_sources.json", json.dumps({
                "packages": {},
                "forbidden": {
                    "package_names": ["paper_benchmark"],
                    "runtime_tokens": ["/m5/", "m5_"],
                },
            }))
            _write(root, "config/runtime_overlay.json", json.dumps({
                "removed": list(removed),
            }))
            before = set(validate_repository(root))
            mutation(root)
            after = set(validate_repository(root))
            self.assertFalse(before - after)
            return after - before

    def test_validate_repository_wires_all_mutation_gates(self):
        token_delta = self._production_delta(
            lambda root: _write(root, "src/pkg/runtime.py", "DeleteModel\n"))
        self.assertEqual(
            {Finding("src/pkg/runtime.py", 1, "DeleteModel")}, token_delta)

        package_delta = self._production_delta(
            lambda root: _write(
                root, "src/ground/unknown/package.xml",
                _package_xml("unknown")))
        self.assertEqual(
            {Finding("src/ground/unknown/package.xml", 0,
                     "package:unlisted:unknown")},
            package_delta,
        )

        spawn_delta = self._production_delta(
            lambda root: _write(
                root, "src/pkg/new.launch",
                '<launch><node pkg="gazebo_ros" type="spawn_model" '
                'name="new_spawn"/></launch>\n'))
        self.assertEqual(
            {Finding("src/pkg/new.launch", 1,
                     "startup-spawn:unclassified:new_spawn")},
            spawn_delta,
        )

        removed = "src/p450/source_pkg/scripts/old_entry.py"
        reference_delta = self._production_delta(
            lambda root: _write(
                root, "src/ground/consumer/runtime.py",
                "ENTRYPOINT = 'old_entry.py'\n"),
            (removed,),
        )
        self.assertEqual(
            {Finding("src/ground/consumer/runtime.py", 1,
                     "removed:%s" % removed)},
            reference_delta,
        )

    def test_real_repository_boundary_is_clean(self):
        self.assertEqual((), validate_repository(ROOT))

    def test_gitignore_is_exactly_generated_local_outputs(self):
        self.assertEqual(
            (
                "build/", "devel/", "install/", "logs/", ".catkin_tools/",
                "__pycache__/", "*.pyc", ".import-staging-*/",
            ),
            tuple((ROOT / ".gitignore").read_text(
                encoding="utf-8").splitlines()),
        )

    def test_readme_states_platform_and_offline_gate_boundaries(self):
        text = (ROOT / "README.md").read_text(encoding="utf-8").lower()
        normalized = " ".join(text.split())
        required = (
            "p450-paper", "immutable", "runtime_sources.json", "dry-run",
            "import_provenance.json", "runtime_overlay.json", "benchmark",
            "inherited launch", "ubuntu 20.04", "ros noetic", "gazebo 11",
            "px4", "mavros", "moveit", "does not prove",
        )
        for token in required:
            with self.subTest(token=token):
                self.assertIn(token, normalized)


if __name__ == "__main__":
    unittest.main()
