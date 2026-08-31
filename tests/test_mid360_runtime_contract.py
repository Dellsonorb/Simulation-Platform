import contextlib
import errno
import hashlib
import importlib.util
import io
import json
import re
import shlex
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest import mock

from tools.runtime_boundary import strip_comments


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
CMAKE_FIND_LIBRARY_OPTIONS = frozenset({
    "CMAKE_FIND_ROOT_PATH_BOTH",
    "DOC",
    "HINTS",
    "NAMES",
    "NAMES_PER_DIR",
    "NO_CACHE",
    "NO_CMAKE_ENVIRONMENT_PATH",
    "NO_CMAKE_FIND_ROOT_PATH",
    "NO_CMAKE_PATH",
    "NO_CMAKE_SYSTEM_PATH",
    "NO_DEFAULT_PATH",
    "NO_PACKAGE_ROOT_PATH",
    "NO_SYSTEM_ENVIRONMENT_PATH",
    "ONLY_CMAKE_FIND_ROOT_PATH",
    "PATHS",
    "PATH_SUFFIXES",
    "REGISTRY_VIEW",
    "REQUIRED",
})


def _sha256_bytes(content):
    return hashlib.sha256(content).hexdigest()


def _load_validator(test_case):
    test_case.assertTrue(VALIDATOR.is_file(), "missing tools/mid360_assets.py")
    spec = importlib.util.spec_from_file_location("mid360_assets", VALIDATOR)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _cmake_find_library_names(body):
    try:
        tokens = shlex.split(body, comments=False, posix=True)
    except ValueError:
        return tuple()
    upper_tokens = [token.upper() for token in tokens]
    if "NAMES" in upper_tokens:
        cursor = upper_tokens.index("NAMES") + 1
    else:
        cursor = 0
    names = []
    while cursor < len(tokens):
        if upper_tokens[cursor] in CMAKE_FIND_LIBRARY_OPTIONS:
            break
        names.extend(
            value for value in tokens[cursor].split(";") if value
        )
        cursor += 1
    return tuple(names)


class Mid360CMakeContractParserTest(unittest.TestCase):
    def test_find_library_names_accepts_positional_and_names_forms(self):
        self.assertEqual(
            ("RayPlugin",),
            _cmake_find_library_names("RayPlugin PATHS /opt/gazebo"),
        )
        self.assertEqual(
            ("RayPlugin", "AlternateRayPlugin"),
            _cmake_find_library_names(
                "NAMES RayPlugin AlternateRayPlugin HINTS /opt/gazebo"
            ),
        )

    def test_find_library_names_rejects_doc_only_mentions(self):
        self.assertEqual(
            ("WrongLibrary",),
            _cmake_find_library_names(
                'NAMES WrongLibrary DOC "RayPlugin" PATHS /opt/gazebo'
            ),
        )


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

    def _write_dae(self, document):
        relative = "models/MID360/meshes/MID360.dae"
        (self.asset_root / relative).write_text(document, encoding="utf-8")
        self._refresh_hash(relative)

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

    def test_accepts_bare_source_or_install_package_metadata_modes(self):
        metadata_contents = {
            "CMakeLists.txt": "source metadata\n",
            "package.xml": "<package/>\n",
            "cmake/sim_platform_assetsConfig.cmake": "# generated\n",
            "cmake/sim_platform_assetsConfig-version.cmake": "# generated\n",
        }
        modes = (
            frozenset(),
            frozenset({"CMakeLists.txt", "package.xml"}),
            frozenset({
                "package.xml",
                "cmake/sim_platform_assetsConfig.cmake",
                "cmake/sim_platform_assetsConfig-version.cmake",
            }),
        )
        for mode in modes:
            with self.subTest(mode=sorted(mode)):
                for relative in metadata_contents:
                    path = self.asset_root / relative
                    if path.exists():
                        path.unlink()
                for relative in mode:
                    path = self.asset_root / relative
                    path.parent.mkdir(parents=True, exist_ok=True)
                    path.write_text(
                        metadata_contents[relative], encoding="utf-8"
                    )
                self.assertEqual(
                    self.asset_root.resolve(),
                    self.module.validate_asset_tree(
                        self._manifest(), self.asset_root
                    ),
                )

    def test_rejects_mixed_or_partial_package_metadata_modes(self):
        metadata_contents = {
            "CMakeLists.txt": "source metadata\n",
            "package.xml": "<package/>\n",
            "cmake/sim_platform_assetsConfig.cmake": "# generated\n",
            "cmake/sim_platform_assetsConfig-version.cmake": "# generated\n",
        }
        invalid_modes = (
            frozenset({"package.xml"}),
            frozenset({"CMakeLists.txt"}),
            frozenset({
                "package.xml",
                "cmake/sim_platform_assetsConfig.cmake",
            }),
            frozenset(metadata_contents),
        )
        for mode in invalid_modes:
            with self.subTest(mode=sorted(mode)):
                for relative in metadata_contents:
                    path = self.asset_root / relative
                    if path.exists():
                        path.unlink()
                for relative in mode:
                    path = self.asset_root / relative
                    path.parent.mkdir(parents=True, exist_ok=True)
                    path.write_text(
                        metadata_contents[relative], encoding="utf-8"
                    )
                with self.assertRaisesRegex(
                    self.module.AssetValidationError,
                    r"^invalid package metadata mode: ",
                ):
                    self.module.validate_asset_tree(
                        self._manifest(), self.asset_root
                    )

    def test_rejects_unknown_package_root_metadata(self):
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

    def test_directory_scan_errors_fail_closed_in_api_and_cli(self):
        blocked = self.asset_root / "models/MID360/scan_mode"
        real_scandir = self.module.os.scandir

        def denied_scandir(path):
            if Path(path) == blocked:
                raise PermissionError(
                    errno.EACCES, "Permission denied", str(blocked)
                )
            return real_scandir(path)

        expected = (
            "asset tree scan failed at models/MID360/scan_mode: "
            "Permission denied"
        )
        with mock.patch.object(
            self.module.os, "scandir", side_effect=denied_scandir
        ):
            with self.assertRaisesRegex(
                self.module.AssetValidationError,
                "^{}$".format(re.escape(expected)),
            ):
                self.module.validate_asset_tree(
                    self._manifest(), self.asset_root
                )

        stdout = io.StringIO()
        stderr = io.StringIO()
        with mock.patch.object(
            self.module.os, "scandir", side_effect=denied_scandir
        ), contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
            result = self.module.main(
                [
                    "--config",
                    str(self.config),
                    "--asset-root",
                    str(self.asset_root),
                ]
            )
        self.assertEqual(1, result)
        self.assertEqual("", stdout.getvalue())
        self.assertEqual(
            "mid360-assets: {}\n".format(expected), stderr.getvalue()
        )
        self.assertEqual(1, len(stderr.getvalue().splitlines()))

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

    def test_invalid_utf8_manifest_fails_cleanly_in_api_and_cli(self):
        self.config.write_bytes(b"{\"schema_version\": \xff}")
        expected = "manifest is not valid UTF-8: {}".format(self.config)

        try:
            self._manifest()
        except self.module.AssetValidationError as error:
            self.assertEqual(expected, str(error))
        except UnicodeError as error:
            self.fail("API leaked UnicodeError: {}".format(error))
        else:
            self.fail("invalid UTF-8 manifest was accepted")

        stdout = io.StringIO()
        stderr = io.StringIO()
        try:
            with contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
                result = self.module.main(
                    [
                        "--config",
                        str(self.config),
                        "--asset-root",
                        str(self.asset_root),
                    ]
                )
        except UnicodeError as error:
            self.fail("CLI leaked UnicodeError: {}".format(error))
        self.assertEqual(1, result)
        self.assertEqual("", stdout.getvalue())
        self.assertEqual(
            "mid360-assets: {}\n".format(expected), stderr.getvalue()
        )
        self.assertEqual(1, len(stderr.getvalue().splitlines()))

    def test_schema_version_requires_an_integer_not_bool_or_float(self):
        for value in (True, False, 1.0):
            with self.subTest(value=value):
                self.payload["schema_version"] = value
                self._write_config()
                with self.assertRaisesRegex(
                    self.module.AssetValidationError,
                    "^unsupported schema_version: {}$".format(value),
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

    def test_sdf_model_scheme_is_case_insensitive(self):
        relative = "models/MID360/MID360.sdf"
        path = self.asset_root / relative
        path.write_text(
            path.read_text(encoding="utf-8").replace(
                "model://MID360/meshes/MID360.dae",
                "MoDeL://MID360/meshes/MID360.dae",
            ),
            encoding="utf-8",
        )
        self._refresh_hash(relative)

        try:
            result = self.module.validate_asset_tree(
                self._manifest(), self.asset_root
            )
        except self.module.AssetValidationError as error:
            self.fail(
                "case-insensitive model scheme must resolve: {}".format(error)
            )
        self.assertEqual(self.asset_root.resolve(), result)

    def test_sdf_rejects_every_unpermitted_uri_scheme(self):
        relative = "models/MID360/MID360.sdf"
        path = self.asset_root / relative
        for uri in (
            "model://Other/meshes/MID360.dae",
            "PACKAGE://demo/mesh.dae",
            "FiLe:/tmp/mesh.dae",
            "HTTPS://example.invalid/mesh.dae",
        ):
            with self.subTest(uri=uri):
                path.write_bytes(
                    self.contents[relative].replace(
                        b"</visual>",
                        ("<uri>{}</uri></visual>".format(uri)).encode("utf-8"),
                    )
                )
                self._refresh_hash(relative)
                with self.assertRaisesRegex(
                    self.module.AssetValidationError,
                    "^asset URI scheme is forbidden: {}$".format(
                        re.escape(uri)
                    ),
                ):
                    self.module.validate_asset_tree(
                        self._manifest(), self.asset_root
                    )

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
            r"^asset URI scheme is forbidden: file:///tmp/texture\.png$",
        ):
            self.module.validate_asset_tree(self._manifest(), self.asset_root)

    def test_dae_rejects_every_uri_scheme_case_insensitively(self):
        relative = "models/MID360/meshes/MID360.dae"
        path = self.asset_root / relative
        for uri in (
            "MoDeL://MID360/texture.png",
            "PACKAGE://demo/texture.png",
            "FiLe:/tmp/texture.png",
            "HTTPS://example.invalid/texture.png",
        ):
            with self.subTest(uri=uri):
                path.write_text(
                    "<COLLADA><library_images><image>"
                    "<init_from>{}</init_from>"
                    "</image></library_images></COLLADA>\n".format(uri),
                    encoding="utf-8",
                )
                self._refresh_hash(relative)
                with self.assertRaisesRegex(
                    self.module.AssetValidationError,
                    "^asset URI scheme is forbidden: {}$".format(
                        re.escape(uri)
                    ),
                ):
                    self.module.validate_asset_tree(
                        self._manifest(), self.asset_root
                    )

    def test_dae_allows_fragments_and_allowlisted_relative_references(self):
        relative = "models/MID360/meshes/MID360.dae"
        path = self.asset_root / relative
        for value in ("#embedded-material", "MID360.dae"):
            with self.subTest(value=value):
                path.write_text(
                    "<COLLADA><library_images><image>"
                    "<init_from>{}</init_from>"
                    "</image></library_images></COLLADA>\n".format(value),
                    encoding="utf-8",
                )
                self._refresh_hash(relative)
                self.assertEqual(
                    self.asset_root.resolve(),
                    self.module.validate_asset_tree(
                        self._manifest(), self.asset_root
                    ),
                )

    def test_dae_resource_attributes_allow_fragments_and_allowlisted_relatives(self):
        self._write_dae(
            "<COLLADA xmlns:ext='urn:test'><node "
            "source='#positions' "
            "ext:url='MID360.dae#geometry'/></COLLADA>\n"
        )

        self.assertEqual(
            self.asset_root.resolve(),
            self.module.validate_asset_tree(self._manifest(), self.asset_root),
        )

    def test_dae_resource_attributes_reject_uri_schemes(self):
        cases = (
            ("url", "FiLe:///tmp/texture.png"),
            ("source", "HTTPS://example.invalid/mesh.dae"),
            ("ext:url", "PACKAGE://demo/mesh.dae"),
        )
        for attribute, value in cases:
            with self.subTest(attribute=attribute, value=value):
                self._write_dae(
                    "<COLLADA xmlns:ext='urn:test'><node "
                    "{}='{}'/></COLLADA>\n".format(attribute, value)
                )
                with self.assertRaisesRegex(
                    self.module.AssetValidationError,
                    "^asset URI scheme is forbidden: {}$".format(
                        re.escape(value)
                    ),
                ):
                    self.module.validate_asset_tree(
                        self._manifest(), self.asset_root
                    )

    def test_dae_resource_attributes_reject_unallowlisted_relative_uri(self):
        self._write_dae(
            "<COLLADA><node url='texture.png#surface'/></COLLADA>\n"
        )

        with self.assertRaisesRegex(
            self.module.AssetValidationError,
            r"^asset reference is not allowlisted: "
            r"models/MID360/meshes/texture\.png$",
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
        validator = _load_validator(self)
        self.assertEqual(
            ASSET_PACKAGE_ROOT.resolve(),
            validator.validate_assets(CONFIG, ASSET_PACKAGE_ROOT),
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

        package = ET.parse(ASSET_PACKAGE_ROOT / "package.xml").getroot()
        description_node = package.find("description")
        self.assertIsNotNone(description_node)
        description = " ".join(
            "".join(description_node.itertext()).split()
        ).casefold()
        self.assertIn("config/mid360_assets.json", description)
        self.assertIn("redistribution", description)
        self.assertIn("unresolved", description)
        licenses = [
            (node.text or "").strip().casefold()
            for node in package.findall("license")
        ]
        self.assertTrue(licenses)
        for license_name in licenses:
            with self.subTest(license=license_name):
                self.assertTrue(license_name)
                self.assertNotIn("apache", license_name)
                self.assertNotIn("bsd", license_name)
                self.assertNotRegex(license_name, r"(?:^|[^a-z])mit(?:[^a-z]|$)")

    def test_livox_cmake_discovers_and_installs_host_abi_dependencies(self):
        cmake_path = PLUGIN_ROOT / "CMakeLists.txt"
        raw_cmake = cmake_path.read_text(encoding="utf-8")
        cmake = strip_comments(cmake_path, raw_cmake)
        for package_name in ("gazebo", "PCL", "Protobuf"):
            with self.subTest(package=package_name):
                self.assertRegex(
                    cmake,
                    r"find_package\s*\(\s*{}\s+REQUIRED\s*\)".format(
                        package_name
                    ),
                )
        find_libraries = re.finditer(
            r"find_library\s*\(\s*"
            r"(?P<variable>[A-Za-z_][A-Za-z0-9_]*)"
            r"(?P<body>[^)]*)\)",
            cmake,
            re.DOTALL,
        )
        ray_library = next(
            (
                match
                for match in find_libraries
                if "RayPlugin" in _cmake_find_library_names(
                    match.group("body")
                )
            ),
            None,
        )
        self.assertIsNotNone(ray_library)
        ray_variable = ray_library.group("variable")
        link_bodies = re.findall(
            r"target_link_libraries\s*\(\s*livox_laser_gazebo_plugins\b"
            r"(?P<body>[^)]*)\)",
            cmake,
            re.DOTALL,
        )
        self.assertTrue(link_bodies)
        required_link_items = {
            "${catkin_LIBRARIES}",
            "${GAZEBO_LIBRARIES}",
            "${PCL_LIBRARIES}",
            "protobuf::libprotobuf",
            "${%s}" % ray_variable,
        }
        self.assertTrue(
            any(
                all(item in body for item in required_link_items)
                for body in link_bodies
            ),
            "Livox target must link Catkin, Gazebo, PCL, protobuf, and the "
            "discovered RayPlugin variable in one target_link_libraries call",
        )
        absolute_guard = re.search(
            r"if\s*\([^)]*NOT\s+IS_ABSOLUTE\s+"
            r"[\"']?\$\{%s\}[\"']?[^)]*\)"
            r"(?P<body>.*?)endif\s*(?:\(\s*\))?" % re.escape(ray_variable),
            cmake,
            re.DOTALL,
        )
        self.assertIsNotNone(absolute_guard)
        self.assertRegex(
            absolute_guard.group("body"),
            r"message\s*\(\s*FATAL_ERROR\b",
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
            "/usr/include",
            "message_generation",
        ):
            with self.subTest(forbidden=forbidden):
                self.assertNotIn(forbidden, raw_cmake)
        self.assertNotRegex(
            raw_cmake,
            r"(?i)\blink_directories\s*\(",
        )

    def test_livox_manifest_matches_the_direct_dependency_surface(self):
        package = ET.parse(PLUGIN_ROOT / "package.xml").getroot()
        self.assertEqual("2", package.attrib.get("format"))
        self.assertEqual([], package.findall("depend"))
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

    def test_livox_sources_do_not_override_the_configured_frame(self):
        source_paths = tuple(sorted(
            path for path in PLUGIN_ROOT.rglob("*")
            if path.suffix in {".cpp", ".h", ".hpp"}
        ))
        self.assertTrue(source_paths)
        for source_path in source_paths:
            with self.subTest(path=source_path.relative_to(PLUGIN_ROOT)):
                active_source = strip_comments(
                    source_path,
                    source_path.read_text(encoding="utf-8"),
                )
                self.assertNotRegex(
                    active_source,
                    r'\bframeName\s*=\s*"livox"',
                )

        source_path = PLUGIN_ROOT / "src/livox_points_plugin.cpp"
        source = strip_comments(
            source_path, source_path.read_text(encoding="utf-8")
        )
        configured = 'frameName = sdf->Get<std::string>("frameName")'
        self.assertIn(configured, source)
        runtime_source = source[source.index(configured):]
        self.assertNotRegex(
            runtime_source,
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
