import hashlib
import importlib.util
import json
import os
import re
import shutil
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def _read(relative):
    return (ROOT / relative).read_text(encoding="utf-8")


def _xml(relative):
    return ET.parse(str(ROOT / relative)).getroot()


def _sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


TEMP_ROOT = ROOT / "logs/bunker_standalone/engineering-tmp"
TEMP_ROOT.mkdir(mode=0o700, parents=True, exist_ok=True)
if TEMP_ROOT.is_symlink() or (TEMP_ROOT.stat().st_mode & 0o777) != 0o700:
    raise RuntimeError("engineering temp root must be a mode-0700 directory")


EXPECTED_ASSET_HASHES = {
    "meshes/BUNKER.STL": "123359ab8c61059ecca8ca3995117dd6f07716fa508724a7aeef59f82d4e6bdc",
    "meshes/base2_Link.STL": "e982f480f1fbfca6888fa80aa16702d8b26b1b40eedddc47b7cc8aedd12c0670",
    "meshes/base_link.STL": "0cfa508a32b4e5ffc4460cb8f3a9c16a25c8c7b5e9b346c54c1d6e2b5a377bb8",
    "meshes/camera1_Link.STL": "0bb66caa54eaaaacffa53058fe5aaa31072828b54e2d9325f709b964f5347275",
    "meshes/displayer_Link.STL": "edc28eb71cf3b3f3e10911ef8b4dd8f481f061d85e819a65a8b954510245cfb4",
    "meshes/laser_Link.STL": "bbd8139b0689a69f28cd7812e6d96ee7053e07d617e1477eff286c493f2e06d7",
    "meshes/wheel1.1_Link.STL": "55049ae1b21db8a37028a5d864c1fa76f6b99068ebafb419f43ec072c228d03e",
    "meshes/wheel1.2_Link.STL": "9afb17085ea465fc18c3e1a95723467f41e983b2049fbb01296e369f738884e9",
    "meshes/wheel1.3_Link.STL": "4d59c7d382705d6934649924c6db529b3515d90a1c5b9316e2233e71cbae554c",
    "meshes/wheel1_Link.STL": "97c02fca2e406c8aa01f4ccaa0eecddbcc68d73723f7d7022aac873aa34bd676",
    "meshes/wheel2.1_Link.STL": "144e4ed5f0caa0294131516880caa2d5ebe6677c2847c895c2f6e6f99999eda2",
    "meshes/wheel2.2_Link.STL": "b17ddb11a5bb84cc79f895572602c3bdfb07ede42b8346e8f1d02b61e2f769ba",
    "meshes/wheel2.3_Link.STL": "9bb30ce26aeea80bb0e4f280e315d57686ba9a3806397df02f297693ad619ab8",
    "meshes/wheel2_Link.STL": "43842bbb532c083ddd44673fdde417912bf5ccf8e221c19a4546d7ba68c5a713",
    "meshes/wheel3.1_Link.STL": "969c0478bdbc80210952cc3a415f574589aed56e2a42e4a44d2466eca82960b5",
    "meshes/wheel3.2_Link.STL": "989a078e9367dcc077a8de97a8d37e942f700b4afae71854ad7ff206321cb17f",
    "meshes/wheel3.3_Link.STL": "47b64761348e114683aec82dbf640a8d4d51124970e80e0a0f5bd05a8abec6a8",
    "meshes/wheel3_Link.STL": "917b6a4f50ae629b09fd2e5c340250bf045ee09d647ade8cbb8cec594ad0c119",
    "meshes/wheel4.1_Link.STL": "534cf68721d036d8cc95d7ea2cd7709abc7f5f3493b468e8614d00b126914481",
    "meshes/wheel4.2_Link.STL": "2ce84dbf2dbb221a0659bb732c21e16ca83060cc1d60a2b38a736da88dedff63",
    "meshes/wheel4.3_Link.STL": "39d99651eec392eb5cfbfe0153d56197cd652f9410dfe21a30b62f3dce36671e",
    "meshes/wheel4_Link.STL": "1868ef9b98d12877ccf49e77bdfff7affe6355d1d0bc9453614e2a46c935fd1b",
    "urdf/bunker.urdf": "46956f10021a88edbc58ccc898dd686e5fb822b297eeb71ca0bc3fdf3e5dfa2f",
    "urdf/bunker.urdf.xacro": "41e6d862c476264dfb8d150c0f8e1780da2451867908ff6e38300813a2b54f74",
    "urdf/bunker.xacro": "7d0a6b500295b065e2bb193c028ac28fe4baf2ace36d864b8a22416d077576b0",
}

EXPECTED_RUNTIME_MESHES = frozenset({
    "meshes/BUNKER.STL",
    *(
        "meshes/wheel%d%s_Link.STL" % (wheel, suffix)
        for wheel in range(1, 5)
        for suffix in ("", ".1", ".2", ".3")
    ),
})

EXPECTED_UNUSED_MESHES = frozenset({
    "meshes/base2_Link.STL",
    "meshes/base_link.STL",
    "meshes/camera1_Link.STL",
    "meshes/displayer_Link.STL",
    "meshes/laser_Link.STL",
})

EXPECTED_EXEC_DEPENDS = frozenset({
    "bunker_description", "gazebo_plugins", "gazebo_ros", "geometry_msgs",
    "nav_msgs", "robot_state_publisher", "roslaunch", "rospy",
    "sensor_msgs", "tf2_ros", "xacro",
})

EXPECTED_TEST_DEPENDS = frozenset({
    "gazebo_msgs", "liburdfdom-tools", "rosgraph", "rosgraph_msgs",
    "rostest", "rosunit", "tf2_msgs",
})


def _load_bunker_assets_module():
    name = "bunker_assets_contract_fixture"
    spec = importlib.util.spec_from_file_location(
        name, ROOT / "tools/bunker_assets.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    try:
        spec.loader.exec_module(module)
    except Exception:
        sys.modules.pop(name, None)
        raise
    return module


def _asset_fixture(test_case):
    temporary = tempfile.TemporaryDirectory(dir=str(TEMP_ROOT))
    test_case.addCleanup(temporary.cleanup)
    root = Path(temporary.name) / "bunker_description"
    paths = (
        "meshes/BUNKER.STL",
        "urdf/bunker.urdf",
        "urdf/bunker.urdf.xacro",
        "urdf/bunker.xacro",
    )
    for relative in paths:
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes((relative + "\n").encode("utf-8"))
    xacro = (
        '<?xml version="1.0"?>\n'
        '<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="fixture">\n'
        '  <link name="base_link"><visual><geometry>'
        '<mesh filename="package://bunker_description/meshes/BUNKER.STL"/>'
        '</geometry></visual></link>\n'
        '</robot>\n'
    ).encode("utf-8")
    (root / "urdf/bunker.urdf.xacro").write_bytes(xacro)
    (root / "package.xml").write_text(
        '<package format="2"><name>bunker_description</name>'
        '<version>0.0.0</version><description>fixture</description>'
        '<maintainer email="fixture@example.invalid">Fixture</maintainer>'
        '<license>TODO</license></package>\n', encoding="utf-8")
    manifest = {
        "schema_version": 1,
        "source": {
            "upstream_commit": "6809c15e3919d1aa3acb6518ad61c49e4150435f",
            "package_source": "Ground/src/third_party/ugv_gazebo_sim/bunker/bunker_description",
            "frozen_renderer_input": "urdf/bunker.urdf.xacro",
            "frozen_renderer_input_sha256": hashlib.sha256(xacro).hexdigest(),
            "license_value": "TODO",
            "license_status": "redistribution-unresolved",
        },
        "files": [
            {"path": relative, "sha256": _sha256(root / relative)}
            for relative in sorted(paths)
        ],
        "runtime_referenced_meshes": ["meshes/BUNKER.STL"],
    }
    manifest_path = Path(temporary.name) / "manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, sort_keys=True), encoding="utf-8")
    return root, manifest_path


def _apply_fixture_mutation(root, manifest_path, mutation):
    payload = json.loads(manifest_path.read_text(encoding="utf-8"))
    if mutation == "escape":
        escaped = root.parent / "escape.STL"
        escaped.write_bytes(b"escape\n")
        payload["files"][0] = {
            "path": "../escape.STL", "sha256": _sha256(escaped)}
        manifest_path.write_text(json.dumps(payload), encoding="utf-8")
        return "../escape.STL"
    if mutation == "symlink":
        target = root.parent / "target.STL"
        target.write_bytes(b"target\n")
        asset = root / "meshes/BUNKER.STL"
        asset.unlink()
        asset.symlink_to(target)
        return "meshes/BUNKER.STL"
    if mutation == "extra":
        extra = root / "meshes/extra.STL"
        extra.write_bytes(b"extra\n")
        return "meshes/extra.STL"
    if mutation == "missing":
        missing = root / "urdf/bunker.urdf"
        missing.unlink()
        return "urdf/bunker.urdf"
    if mutation == "hash":
        payload["files"][0]["sha256"] = "0" * 64
        manifest_path.write_text(json.dumps(payload), encoding="utf-8")
        return payload["files"][0]["path"]
    raise AssertionError("unknown fixture mutation: %s" % mutation)


def _rewrite_manifest(manifest_path, mutate):
    payload = json.loads(manifest_path.read_text(encoding="utf-8"))
    mutate(payload)
    manifest_path.write_text(
        json.dumps(payload, sort_keys=True), encoding="utf-8")


def _rehash_fixture_xacro(root, manifest_path):
    payload = json.loads(manifest_path.read_text(encoding="utf-8"))
    relative = "urdf/bunker.urdf.xacro"
    digest = _sha256(root / relative)
    for item in payload["files"]:
        if item["path"] == relative:
            item["sha256"] = digest
    payload["source"]["frozen_renderer_input_sha256"] = digest
    manifest_path.write_text(
        json.dumps(payload, sort_keys=True), encoding="utf-8")


class BunkerAssetManifestContractTest(unittest.TestCase):
    def test_manifest_is_exact(self):
        payload = json.loads(
            (ROOT / "config/bunker_assets.json").read_text(encoding="utf-8"))
        self.assertEqual(
            {"schema_version", "source", "files", "runtime_referenced_meshes"},
            set(payload),
        )
        self.assertEqual(1, payload["schema_version"])
        self.assertEqual({
            "upstream_commit", "package_source", "frozen_renderer_input",
            "frozen_renderer_input_sha256", "license_value",
            "license_status",
        }, set(payload["source"]))
        self.assertEqual(
            "6809c15e3919d1aa3acb6518ad61c49e4150435f",
            payload["source"]["upstream_commit"],
        )
        self.assertEqual(
            "Ground/src/third_party/ugv_gazebo_sim/bunker/bunker_description",
            payload["source"]["package_source"],
        )
        self.assertEqual(
            "urdf/bunker.urdf.xacro",
            payload["source"]["frozen_renderer_input"],
        )
        self.assertEqual(
            EXPECTED_ASSET_HASHES["urdf/bunker.urdf.xacro"],
            payload["source"]["frozen_renderer_input_sha256"],
        )
        self.assertEqual("TODO", payload["source"]["license_value"])
        self.assertEqual(
            "redistribution-unresolved", payload["source"]["license_status"])
        observed = {
            item["path"]: item["sha256"] for item in payload["files"]}
        self.assertEqual(
            sorted(EXPECTED_ASSET_HASHES),
            [item["path"] for item in payload["files"]],
        )
        self.assertEqual(EXPECTED_ASSET_HASHES, observed)
        self.assertEqual(
            EXPECTED_RUNTIME_MESHES,
            frozenset(payload["runtime_referenced_meshes"]),
        )
        self.assertEqual(
            EXPECTED_UNUSED_MESHES,
            {path for path in observed if path.startswith("meshes/")}
            - EXPECTED_RUNTIME_MESHES,
        )

    def test_validator_rejects_escape_symlink_extra_missing_and_hash_mismatch(self):
        module = _load_bunker_assets_module()
        for mutation in ("escape", "symlink", "extra", "missing", "hash"):
            with self.subTest(mutation=mutation):
                fixture, manifest_path = _asset_fixture(self)
                offending = _apply_fixture_mutation(
                    fixture, manifest_path, mutation)
                with self.assertRaisesRegex(
                        module.AssetValidationError, re.escape(offending)):
                    module.validate_asset_tree(manifest_path, fixture)

    def test_real_source_tree_matches_the_exact_manifest(self):
        module = _load_bunker_assets_module()
        self.assertEqual(
            (ROOT / "src/vendor/bunker_description").resolve(),
            module.validate_asset_tree(
                ROOT / "config/bunker_assets.json",
                ROOT / "src/vendor/bunker_description"),
        )

    def test_validator_accepts_source_and_independent_install_copy(self):
        module = _load_bunker_assets_module()
        source_root, manifest_path = _asset_fixture(self)
        install_root = source_root.parent / "install/bunker_description"
        install_root.parent.mkdir(parents=True)
        shutil.copytree(source_root, install_root)
        self.assertEqual(
            source_root.resolve(),
            module.validate_asset_tree(manifest_path, source_root))
        self.assertEqual(
            (source_root.resolve(), install_root.resolve()),
            module.validate_source_install_pair(
                manifest_path, source_root, install_root))

    def test_validator_rejects_duplicate_unsorted_and_invalid_records(self):
        module = _load_bunker_assets_module()
        mutations = (
            lambda payload: payload["files"].append(
                dict(payload["files"][0])),
            lambda payload: payload["runtime_referenced_meshes"].append(
                payload["runtime_referenced_meshes"][0]),
            lambda payload: payload.__setitem__(
                "files", list(reversed(payload["files"]))),
            lambda payload: payload["files"][0].__setitem__(
                "sha256", "A" * 64),
            lambda payload: payload["source"].__setitem__(
                "frozen_renderer_input_sha256", "0" * 64),
            lambda payload: payload["source"].__setitem__(
                "license_value", "BSD"),
            lambda payload: payload["runtime_referenced_meshes"].append(
                "meshes/not-declared.STL"),
        )
        for mutation in mutations:
            with self.subTest(mutation=mutation):
                root, manifest_path = _asset_fixture(self)
                _rewrite_manifest(manifest_path, mutation)
                with self.assertRaises(module.AssetValidationError):
                    module.validate_asset_tree(manifest_path, root)

    def test_validator_rejects_wrong_package_license(self):
        module = _load_bunker_assets_module()
        root, manifest_path = _asset_fixture(self)
        package_xml = root / "package.xml"
        package_xml.write_text(
            package_xml.read_text(encoding="utf-8").replace(
                "<license>TODO</license>", "<license>BSD</license>"),
            encoding="utf-8")
        with self.assertRaises(module.AssetValidationError):
            module.validate_asset_tree(manifest_path, root)

    def test_validator_rejects_symlinked_directory_and_nonregular_file(self):
        module = _load_bunker_assets_module()
        root, manifest_path = _asset_fixture(self)
        target = root.parent / "mesh-target"
        shutil.copytree(root / "meshes", target)
        shutil.rmtree(root / "meshes")
        (root / "meshes").symlink_to(target, target_is_directory=True)
        with self.assertRaises(module.AssetValidationError):
            module.validate_asset_tree(manifest_path, root)

        root, manifest_path = _asset_fixture(self)
        mesh = root / "meshes/BUNKER.STL"
        mesh.unlink()
        os.mkfifo(str(mesh))
        with self.assertRaises(module.AssetValidationError):
            module.validate_asset_tree(manifest_path, root)

    def test_validator_rejects_malformed_package_mesh_uri(self):
        module = _load_bunker_assets_module()
        root, manifest_path = _asset_fixture(self)
        xacro = root / "urdf/bunker.urdf.xacro"
        xacro.write_text(
            xacro.read_text(encoding="utf-8").replace(
                "package://bunker_description/meshes/BUNKER.STL",
                "file:///tmp/BUNKER.STL"),
            encoding="utf-8")
        _rehash_fixture_xacro(root, manifest_path)
        with self.assertRaises(module.AssetValidationError):
            module.validate_asset_tree(manifest_path, root)


class BunkerPackageContractTest(unittest.TestCase):
    def test_manifest_dependency_surface_is_exact(self):
        root = _xml("src/platform/bunker_sim_runtime/package.xml")
        self.assertEqual(
            ["catkin"],
            [item.text for item in root.findall("buildtool_depend")])
        self.assertEqual(
            EXPECTED_EXEC_DEPENDS,
            {item.text for item in root.findall("exec_depend")})
        self.assertEqual(
            EXPECTED_TEST_DEPENDS,
            {item.text for item in root.findall("test_depend")})
        self.assertEqual([], root.findall("depend"))

    def test_vendor_install_is_minimal(self):
        source = _read("src/vendor/bunker_description/CMakeLists.txt")
        self.assertRegex(
            source,
            r"install\s*\(\s*DIRECTORY\s+meshes\s+urdf\s+"
            r"DESTINATION\s+\$\{CATKIN_PACKAGE_SHARE_DESTINATION\}\s*\)",
        )
        self.assertNotRegex(source, r"DIRECTORY[^\)]*(launch|rviz|config)")
