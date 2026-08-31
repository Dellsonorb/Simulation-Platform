import contextlib
import hashlib
import importlib.util
import io
import json
import re
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "config/mid360_assets.json"
VALIDATOR = ROOT / "tools/mid360_assets.py"
PLUGIN_ROOT = ROOT / "src/p450/livox_laser_gazebo_plugins"
ASSET_PACKAGE_ROOT = ROOT / "src/platform/sim_platform_assets"
BRINGUP_LAUNCH = (
    ROOT / "src/platform/sim_platform_bringup/launch/p450_runtime.launch"
)
RUNTIME_CONFIG = ROOT / "config/p450_runtime.json"

EXPECTED_REMOTE = "https://gitee.com/amovlab/p_sitl_gazebo.git"
EXPECTED_PINNED_COMMIT = "9566172e7c0760f66681304b963d675c5b120daf"
EXPECTED_INTRODUCTION_COMMIT = "81bd1734c909bd1684bb03bbf4187e77a83b22a6"
EXPECTED_AUTHOR = "Eason Yi"
EXPECTED_FILES = {
    "models/MID360/model.config": (
        "fef13d37eddc6fe097b3554a570a957e885e815288858c917f57fb724ac81efa"
    ),
    "models/MID360/MID360.sdf": (
        "285813d7f1fe7c1db3ede76aad365ad43379b198f22978fbebe1d1b8599ccdc5"
    ),
    "models/MID360/meshes/MID360.dae": (
        "464a158c872c3ca7cf8124f464b3610e07c560e652894b960e1ad6061d9382e2"
    ),
    "models/MID360/scan_mode/mid360.csv": (
        "aa1fc08b6a4400608dbd6ee832b7ea3a9c3c37197e734f60f58fe5abf762269a"
    ),
}
EXPECTED_EXCLUSIONS = ("models/MID360/meshes/MID360.stl",)
REQUIRED_CATKIN_DEPENDENCIES = {
    "gazebo_dev",
    "pcl_conversions",
    "prometheus_msgs",
    "roscpp",
    "sensor_msgs",
    "tf",
}


def _sha256_bytes(content):
    return hashlib.sha256(content).hexdigest()


def _load_validator(test_case):
    test_case.assertTrue(VALIDATOR.is_file(), "missing tools/mid360_assets.py")
    spec = importlib.util.spec_from_file_location("mid360_assets", VALIDATOR)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Mid360ManifestContractTest(unittest.TestCase):
    def test_manifest_freezes_source_identity_license_status_and_exact_files(self):
        self.assertTrue(CONFIG.is_file(), "missing config/mid360_assets.json")
        payload = json.loads(CONFIG.read_text(encoding="utf-8"))

        self.assertEqual(
            {
                "schema_version",
                "source",
                "files",
                "excluded",
            },
            set(payload),
        )
        self.assertEqual(1, payload["schema_version"])
        self.assertEqual(
            {
                "remote",
                "pinned_commit",
                "introduction_commit",
                "author",
                "license_status",
                "license_note",
            },
            set(payload["source"]),
        )
        self.assertEqual(EXPECTED_REMOTE, payload["source"]["remote"])
        self.assertEqual(
            EXPECTED_PINNED_COMMIT, payload["source"]["pinned_commit"]
        )
        self.assertEqual(
            EXPECTED_INTRODUCTION_COMMIT,
            payload["source"]["introduction_commit"],
        )
        self.assertEqual(EXPECTED_AUTHOR, payload["source"]["author"])
        self.assertEqual(
            "redistribution-unresolved", payload["source"]["license_status"]
        )
        self.assertIn(
            "no tracked license",
            payload["source"]["license_note"].casefold(),
        )

        observed = {
            entry["target"]: entry["sha256"] for entry in payload["files"]
        }
        self.assertEqual(EXPECTED_FILES, observed)
        self.assertEqual(
            set(EXPECTED_FILES),
            {entry["source"] for entry in payload["files"]},
        )
        self.assertEqual(list(EXPECTED_EXCLUSIONS), payload["excluded"])


class Mid360AssetValidatorTest(unittest.TestCase):
    def setUp(self):
        self.module = _load_validator(self)
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.base = Path(self.temporary.name)
        self.asset_root = self.base / "assets"
        self.config = self.base / "mid360_assets.json"
        self.contents = {
            "models/MID360/model.config": (
                b"<model><name>MID360</name><sdf>MID360.sdf</sdf></model>\n"
            ),
            "models/MID360/MID360.sdf": (
                b"<sdf><model><link><visual><geometry><mesh>"
                b"<uri>model://MID360/meshes/MID360.dae</uri>"
                b"</mesh></geometry></visual><sensor><plugin>"
                b"<csv_file_name>mid360.csv</csv_file_name>"
                b"</plugin></sensor></link></model></sdf>\n"
            ),
            "models/MID360/meshes/MID360.dae": b"<COLLADA/>\n",
            "models/MID360/scan_mode/mid360.csv": b"x,y,z\n0,0,0\n",
        }
        for relative, content in self.contents.items():
            target = self.asset_root / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(content)
        self.payload = {
            "schema_version": 1,
            "source": {
                "remote": EXPECTED_REMOTE,
                "pinned_commit": EXPECTED_PINNED_COMMIT,
                "introduction_commit": EXPECTED_INTRODUCTION_COMMIT,
                "author": EXPECTED_AUTHOR,
                "license_status": "redistribution-unresolved",
                "license_note": (
                    "No tracked license text exists in the pinned source tree."
                ),
            },
            "files": [
                {
                    "source": relative,
                    "target": relative,
                    "sha256": _sha256_bytes(content),
                }
                for relative, content in self.contents.items()
            ],
            "excluded": list(EXPECTED_EXCLUSIONS),
        }
        self._write_config()

    def _write_config(self):
        self.config.write_text(
            json.dumps(self.payload, sort_keys=True), encoding="utf-8"
        )

    def _manifest(self):
        return self.module.AssetManifest.load(self.config)

    def _refresh_hash(self, relative):
        digest = _sha256_bytes((self.asset_root / relative).read_bytes())
        for entry in self.payload["files"]:
            if entry["target"] == relative:
                entry["sha256"] = digest
                break
        self._write_config()

    def test_valid_tree_returns_canonical_root(self):
        result = self.module.validate_asset_tree(
            self._manifest(), self.asset_root
        )
        self.assertEqual(self.asset_root.resolve(), result)

    def test_rejects_an_extra_file(self):
        extra = self.asset_root / "models/MID360/extra.txt"
        extra.write_text("not allowlisted\n", encoding="utf-8")

        with self.assertRaisesRegex(
            self.module.AssetValidationError,
            r"^unexpected asset: models/MID360/extra\.txt$",
        ):
            self.module.validate_asset_tree(self._manifest(), self.asset_root)

    def test_allows_only_explicit_source_and_install_package_metadata(self):
        metadata = {
            "CMakeLists.txt": "source metadata\n",
            "package.xml": "<package/>\n",
            "cmake/sim_platform_assetsConfig.cmake": "# generated\n",
            "cmake/sim_platform_assetsConfig-version.cmake": "# generated\n",
        }
        for relative, content in metadata.items():
            path = self.asset_root / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content, encoding="utf-8")

        try:
            result = self.module.validate_asset_tree(
                self._manifest(), self.asset_root
            )
        except self.module.AssetValidationError as error:
            self.fail("package metadata must be accepted: {}".format(error))
        self.assertEqual(self.asset_root.resolve(), result)

        unexpected = self.asset_root / "README.md"
        unexpected.write_text("not package metadata\n", encoding="utf-8")
        with self.assertRaisesRegex(
            self.module.AssetValidationError,
            r"^unexpected asset: README\.md$",
        ):
            self.module.validate_asset_tree(self._manifest(), self.asset_root)

    def test_rejects_a_missing_file(self):
        missing = "models/MID360/scan_mode/mid360.csv"
        (self.asset_root / missing).unlink()

        with self.assertRaisesRegex(
            self.module.AssetValidationError,
            "^missing asset: {}$".format(re.escape(missing)),
        ):
            self.module.validate_asset_tree(self._manifest(), self.asset_root)

    def test_rejects_a_symlink(self):
        relative = "models/MID360/meshes/MID360.dae"
        target = self.asset_root / relative
        target.unlink()
        target.symlink_to(self.asset_root / "models/MID360/MID360.sdf")

        with self.assertRaisesRegex(
            self.module.AssetValidationError,
            "^symlink is forbidden: {}$".format(re.escape(relative)),
        ):
            self.module.validate_asset_tree(self._manifest(), self.asset_root)

    def test_rejects_a_hash_mismatch(self):
        relative = "models/MID360/scan_mode/mid360.csv"
        (self.asset_root / relative).write_text("changed\n", encoding="utf-8")

        with self.assertRaisesRegex(
            self.module.AssetValidationError,
            "^sha256 mismatch: {}$".format(re.escape(relative)),
        ):
            self.module.validate_asset_tree(self._manifest(), self.asset_root)

    def test_rejects_source_and_target_path_escape(self):
        for field in ("source", "target"):
            with self.subTest(field=field):
                original = self.payload["files"][0][field]
                self.payload["files"][0][field] = "../outside"
                self._write_config()
                with self.assertRaisesRegex(
                    self.module.AssetValidationError,
                    "^{} path escapes root: \\.\\./outside$".format(field),
                ):
                    self._manifest()
                self.payload["files"][0][field] = original

    def test_rejects_an_unsupported_schema(self):
        self.payload["schema_version"] = 2
        self._write_config()

        with self.assertRaisesRegex(
            self.module.AssetValidationError,
            r"^unsupported schema_version: 2$",
        ):
            self._manifest()

    def test_rejects_a_false_resolved_license_claim(self):
        self.payload["source"]["license_status"] = "MIT"
        self._write_config()

        with self.assertRaisesRegex(
            self.module.AssetValidationError,
            r"^license_status must be redistribution-unresolved$",
        ):
            self._manifest()

    def test_rejects_unallowlisted_sdf_uri(self):
        relative = "models/MID360/MID360.sdf"
        path = self.asset_root / relative
        path.write_text(
            path.read_text(encoding="utf-8").replace(
                "model://MID360/meshes/MID360.dae",
                "model://MID360/meshes/untracked.dae",
            ),
            encoding="utf-8",
        )
        self._refresh_hash(relative)

        with self.assertRaisesRegex(
            self.module.AssetValidationError,
            r"^asset reference is not allowlisted: "
            r"models/MID360/meshes/untracked\.dae$",
        ):
            self.module.validate_asset_tree(self._manifest(), self.asset_root)

    def test_rejects_unallowlisted_relative_sdf_uri(self):
        relative = "models/MID360/MID360.sdf"
        path = self.asset_root / relative
        path.write_text(
            path.read_text(encoding="utf-8").replace(
                "</visual>",
                "</visual><visual><geometry><mesh>"
                "<uri>meshes/untracked.dae</uri>"
                "</mesh></geometry></visual>",
            ),
            encoding="utf-8",
        )
        self._refresh_hash(relative)

        with self.assertRaisesRegex(
            self.module.AssetValidationError,
            r"^asset reference is not allowlisted: "
            r"models/MID360/meshes/untracked\.dae$",
        ):
            self.module.validate_asset_tree(self._manifest(), self.asset_root)

    def test_rejects_unallowlisted_dae_uri(self):
        relative = "models/MID360/meshes/MID360.dae"
        (self.asset_root / relative).write_text(
            "<COLLADA><library_images><image>"
            "<init_from>texture.png</init_from>"
            "</image></library_images></COLLADA>\n",
            encoding="utf-8",
        )
        self._refresh_hash(relative)

        with self.assertRaisesRegex(
            self.module.AssetValidationError,
            r"^asset reference is not allowlisted: "
            r"models/MID360/meshes/texture\.png$",
        ):
            self.module.validate_asset_tree(self._manifest(), self.asset_root)

    def test_rejects_local_file_dae_uri(self):
        relative = "models/MID360/meshes/MID360.dae"
        (self.asset_root / relative).write_text(
            "<COLLADA><library_images><image>"
            "<init_from>file:///tmp/texture.png</init_from>"
            "</image></library_images></COLLADA>\n",
            encoding="utf-8",
        )
        self._refresh_hash(relative)

        with self.assertRaisesRegex(
            self.module.AssetValidationError,
            r"^asset reference escapes root: file:///tmp/texture\.png$",
        ):
            self.module.validate_asset_tree(self._manifest(), self.asset_root)

    def test_cli_prints_canonical_root_and_exact_failures(self):
        stdout = io.StringIO()
        stderr = io.StringIO()
        with contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
            result = self.module.main(
                ["--config", str(self.config), "--asset-root", str(self.asset_root)]
            )
        self.assertEqual(0, result)
        self.assertEqual(str(self.asset_root.resolve()) + "\n", stdout.getvalue())
        self.assertEqual("", stderr.getvalue())

        relative = "models/MID360/scan_mode/mid360.csv"
        (self.asset_root / relative).unlink()
        stdout = io.StringIO()
        stderr = io.StringIO()
        with contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
            result = self.module.main(
                ["--config", str(self.config), "--asset-root", str(self.asset_root)]
            )
        self.assertEqual(1, result)
        self.assertEqual("", stdout.getvalue())
        self.assertEqual(
            "mid360-assets: missing asset: {}\n".format(relative),
            stderr.getvalue(),
        )


class Mid360FutureRuntimeContractTest(unittest.TestCase):
    def test_platform_asset_package_contains_only_the_audited_closure(self):
        self.assertTrue(
            ASSET_PACKAGE_ROOT.is_dir(),
            "Task 2 must add src/platform/sim_platform_assets",
        )
        expected = {
            "CMakeLists.txt",
            "package.xml",
            *EXPECTED_FILES,
        }
        observed = {
            path.relative_to(ASSET_PACKAGE_ROOT).as_posix()
            for path in ASSET_PACKAGE_ROOT.rglob("*")
            if path.is_file()
        }
        self.assertEqual(expected, observed)

    def test_livox_cmake_discovers_and_installs_host_abi_dependencies(self):
        cmake = (PLUGIN_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("find_package(Protobuf REQUIRED)", cmake)
        self.assertIn("protobuf::libprotobuf", cmake)
        self.assertRegex(cmake, r"find_library\s*\([^)]*RayPlugin[^)]*\)")
        self.assertRegex(
            cmake,
            r"target_link_libraries\s*\([^)]*\$\{[^}]*RAY[^}]*\}[^)]*\)",
        )
        catkin = re.search(
            r"find_package\s*\(\s*catkin\s+REQUIRED\s+COMPONENTS(?P<body>.*?)\)",
            cmake,
            re.DOTALL,
        )
        self.assertIsNotNone(catkin)
        self.assertTrue(
            REQUIRED_CATKIN_DEPENDENCIES.issubset(set(catkin.group("body").split()))
        )
        self.assertRegex(
            cmake,
            r"add_dependencies\s*\(\s*livox_laser_gazebo_plugins\s+"
            r"\$\{catkin_EXPORTED_TARGETS\}\s*\)",
        )
        self.assertRegex(cmake, r"install\s*\(\s*TARGETS\s+livox_laser_gazebo_plugins")
        self.assertRegex(cmake, r"install\s*\(\s*DIRECTORY\s+include/")
        for forbidden in (
            "libprotobuf.so.9",
            "include_directories(/usr/include",
            "link_directories(",
            "message_generation",
        ):
            with self.subTest(forbidden=forbidden):
                self.assertNotIn(forbidden, cmake)

    def test_livox_manifest_matches_the_direct_dependency_surface(self):
        package = ET.parse(PLUGIN_ROOT / "package.xml").getroot()
        for tag in ("build_depend", "build_export_depend", "exec_depend"):
            with self.subTest(tag=tag):
                observed = {node.text.strip() for node in package.findall(tag)}
                self.assertEqual(REQUIRED_CATKIN_DEPENDENCIES, observed)
        licenses = {node.text.strip() for node in package.findall("license")}
        self.assertTrue(licenses)
        self.assertFalse(
            {value.casefold() for value in licenses}
            & {"todo", "unknown", "tbd", "placeholder"}
        )

    def test_livox_source_preserves_the_configured_frame(self):
        source = (PLUGIN_ROOT / "src/livox_points_plugin.cpp").read_text(
            encoding="utf-8"
        )
        self.assertNotRegex(
            source,
            r'(?:frameName|header\.frame_id)\s*=\s*"livox"',
        )

    def test_runtime_profile_is_explicitly_opt_in(self):
        launch = ET.parse(BRINGUP_LAUNCH).getroot()
        args = {node.attrib["name"]: node.attrib.get("default") for node in launch.findall("arg")}
        self.assertEqual("false", args.get("enable_mid360"))
        runtime = json.loads(RUNTIME_CONFIG.read_text(encoding="utf-8"))
        self.assertIn("sim_platform_assets", runtime["required_packages"])
        self.assertIn("livox_laser_gazebo_plugins", runtime["required_packages"])


if __name__ == "__main__":
    unittest.main()
