# BUNKER-A Standalone Runtime Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use
> `superpowers:subagent-driven-development` (recommended) or
> `superpowers:executing-plans` to implement this plan task-by-task. Steps use
> checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build and prove one install-space, independently runnable BUNKER
ground runtime with bounded command admission, planar Gazebo motion, odometry,
2D LiDAR, and one uniquely owned `world -> ground/odom -> ground/base_link`
TF tree.

**Architecture:** Add a platform-owned `bunker_sim_runtime` Catkin package
that deterministically renders the frozen vendor description, guards
`/ground/cmd_vel`, and exposes one worldless component launch plus one small
standalone world. Keep source/install provenance, pure behavior tests, the
install-only probe, and the bounded live gate separate so failures identify
the broken layer. Reuse only patterns from the accepted P450 gates; do not
introduce a dependency on P450, legacy Ground navigation/manipulation, or
paper infrastructure.

**Tech Stack:** ROS Noetic, Catkin Tools install space, Gazebo Classic 11.15.1,
`gazebo_plugins` 2.9.3, Python 3.8 with PyYAML, Bash, XML/URDF/SDF, ROS TF/TF2,
`unittest`/nosetests, `check_urdf`, `ldd`, and `/proc/<pid>/maps` inspection.

**Approved specification:**
`docs/superpowers/specs/2026-09-01-bunker-a-standalone-runtime-design.md`

**Write boundary:** Every edit, build product, log, temporary file, ROS/Gazebo
state directory, and evidence file stays below
`/media/lu/P450_PAPER/SIM/p450_sim_v1`. The repository at
`/media/lu/P450_PAPER/P450-PAPER` and all external PX4/SITL checkouts remain
read-only. `config/import_provenance.json` remains immutable.

**Non-goals:** Do not add navigation, `move_base`, AUBO, AG95, D435, MoveIt,
grasping, attachment, shared-world orchestration, RBP, Task-aware logic, M5,
Pilot/T2/Formal, benchmark matrices, lifecycle frameworks, or paper evidence.

---

## File map

### New platform package

- `src/platform/bunker_sim_runtime/CMakeLists.txt` — build/test/install surface.
- `src/platform/bunker_sim_runtime/package.xml` — exact runtime/test dependency
  boundary.
- `src/platform/bunker_sim_runtime/setup.py` — installs the pure Python module.
- `src/platform/bunker_sim_runtime/src/bunker_sim_runtime/__init__.py` — Python
  package marker only; callers import helpers from their defining modules.
- `src/platform/bunker_sim_runtime/src/bunker_sim_runtime/renderer.py` — frozen
  xacro validation and deterministic URDF transformation.
- `src/platform/bunker_sim_runtime/src/bunker_sim_runtime/velocity_guard.py` —
  pure Twist admission plus the small ROS node adapter.
- `src/platform/bunker_sim_runtime/src/bunker_sim_runtime/contracts.py` —
  frozen identity, pose, name, and spawn-command validation.
- `src/platform/bunker_sim_runtime/src/bunker_sim_runtime/sim_time.py` — strict
  XML-RPC `/use_sim_time` preflight.
- `src/platform/bunker_sim_runtime/scripts/render_bunker_runtime.py` — installed
  share-path renderer CLI; stdout is URDF only.
- `src/platform/bunker_sim_runtime/scripts/velocity_guard.py` — installed ROS
  package executable.
- `src/platform/bunker_sim_runtime/scripts/spawn_bunker_preflight.py` — spawn
  `launch-prefix` that validates pose/identity, checks a boolean true
  `/use_sim_time`, then `exec`s the bonded real spawn process.
- `src/platform/bunker_sim_runtime/launch/bunker_runtime.launch` — worldless
  Ground component.
- `src/platform/bunker_sim_runtime/launch/bunker_standalone.launch` — one-world
  diagnostic entrypoint.
- `src/platform/bunker_sim_runtime/worlds/bunker_standalone.world` — ground,
  sun, and one scan obstacle only.
- `src/platform/bunker_sim_runtime/test/test_renderer.py` — renderer behavior
  and malformed-input tests.
- `src/platform/bunker_sim_runtime/test/test_velocity_guard.py` — pure guard
  admission tests.
- `src/platform/bunker_sim_runtime/test/test_sim_time.py` — strict parameter
  type/value tests.
- `src/platform/bunker_sim_runtime/test/test_contracts.py` — identity, finite
  pose, and safe spawn-command tests.
- `src/platform/bunker_sim_runtime/test/test_spawn_bunker_preflight.py` — thin
  adapter exit-code, sim-time, and `execv` delegation tests.
- `src/platform/bunker_sim_runtime/test/test_launch_contract.py` — structured
  launch/world/TF/plugin contract tests.
- `src/platform/bunker_sim_runtime/test/tf_prefix_contract.test` and
  `src/platform/bunker_sim_runtime/test/test_tf_prefix_contract.py` — local
  Noetic `tf_prefix` behavior without Gazebo.
- `src/platform/bunker_sim_runtime/test/tf_prefix_fixture.urdf` — minimal
  two-link input for the real prefix rostest.

### New repository gates

- `config/bunker_assets.json` — exact 3-URDF/22-mesh hashes, 17-mesh runtime
  reference subset, and unresolved vendor license record.
- `tools/bunker_assets.py` — fail-closed source/install asset validator.
- `tools/bunker_live_contract.py` — ROS-independent scan, motion, TF, plugin,
  and schema-2 evidence validation.
- `tests/test_bunker_build_contract.py` — asset/package/install and boundary
  contracts.
- `tests/test_bunker_live_contract.py` — numeric/evidence boundary fixtures.
- `tests/test_bunker_shell_contract.py` — clean-wrapper and bounded-supervisor
  fixture branches.
- `scripts/with_bunker_env.bash` — clean, run-local, install-only environment.
- `scripts/validate_bunker_install.bash` — no-Gazebo install closure probe.
- `scripts/start_bunker_runtime_group.py` — tiny signal-reset/identity
  trampoline that becomes the bounded roslaunch group leader.
- `scripts/probe_bunker_standalone.py` — one-run ROS/Gazebo behavioral probe.
- `scripts/smoke_bunker_standalone.bash` — bounded process owner and atomic
  PASS/FAIL writer.
- `tests/test_bunker_runtime_group.py` — offline trampoline handshake and
  signal-disposition tests.

### Existing files changed

- `src/vendor/bunker_description/CMakeLists.txt` — install only `meshes` and
  `urdf`.
- `config/runtime_sources.json` and `tests/test_runtime_source_contract.py` —
  register the new non-imported platform package.
- `tests/test_imported_layout.py` — materialize the sixth local package.
- `config/runtime_overlay.json` and `tests/test_runtime_extraction.py` — ledger
  the one vendor CMake repair.
- `tools/runtime_boundary.py` and `tests/test_runtime_boundary.py` — classify
  the worldless BUNKER spawn as a shared-world candidate.

No other existing package is changed.

---

## Task 1: Freeze the source, asset, and package boundary in RED tests

**Files:**

- Create: `tests/test_bunker_build_contract.py`
- Modify: `tests/test_runtime_source_contract.py`
- Modify: `tests/test_imported_layout.py`
- Modify: `tests/test_runtime_extraction.py`

- [ ] **Step 1: Create the SIM-local engineering boundary and baseline**

Run this before editing any runtime/test file. It creates the only offline
temporary root and records both external repositories without taking Git
locks:

```bash
set -euo pipefail
install -d -m 700 -- "$PWD/logs/bunker_standalone"
test ! -e "$PWD/logs/bunker_standalone/bunker-a-baseline"
install -d -m 700 -- \
  "$PWD/logs/bunker_standalone/bunker-a-baseline" \
  "$PWD/logs/bunker_standalone/engineering-tmp"
GIT_OPTIONAL_LOCKS=0 git -C /media/lu/P450_PAPER/P450-PAPER/source/P450-SIM \
  rev-parse HEAD >logs/bunker_standalone/bunker-a-baseline/paper-head.txt
GIT_OPTIONAL_LOCKS=0 git -C /media/lu/P450_PAPER/P450-PAPER/source/P450-SIM \
  status --porcelain=v1 --untracked-files=all -z \
  >logs/bunker_standalone/bunker-a-baseline/paper-status.z
GIT_OPTIONAL_LOCKS=0 git \
  -C /media/lu/P450_PAPER/P450-PAPER/workspaces/dependencies/px4 \
  rev-parse HEAD >logs/bunker_standalone/bunker-a-baseline/px4-head.txt
GIT_OPTIONAL_LOCKS=0 git \
  -C /media/lu/P450_PAPER/P450-PAPER/workspaces/dependencies/px4 \
  status --porcelain=v1 --untracked-files=all -z \
  >logs/bunker_standalone/bunker-a-baseline/px4-status.z
chmod 600 logs/bunker_standalone/bunker-a-baseline/*
```

Expected: all commands exit zero; all four records are regular files below
SIM. Keep this directory untracked through final comparison.

- [ ] **Step 2: Add the exact local-package expectations**

Add this record to `EXPECTED_PACKAGE_RECORDS` in
`tests/test_runtime_source_contract.py`:

```python
"bunker_sim_runtime": (
    "platform",
    None,
    "src/platform/bunker_sim_runtime",
    False,
),
```

In `tests/test_imported_layout.py`, make the exact sets and count read:

```python
EXPECTED_MATERIALIZED_LOCAL_PACKAGES = frozenset({
    "bunker_sim_runtime",
    "sim_platform_bringup",
    "sim_platform_assets",
})
EXPECTED_ABSENT_LOCAL_PACKAGES = frozenset({
    "air_ground_pick_demo",
    "air_ground_pose_bridge",
    "ground_runtime_compat",
})

# In test_manifest_declares_exact_import_counts:
self.assertEqual(18, len(self.imported_packages))
self.assertEqual(11, len(self.manifest.auxiliary_imports))
self.assertEqual(6, len(self.local_packages))
```

- [ ] **Step 3: Freeze the asset hashes and referenced subset**

Create `tests/test_bunker_build_contract.py`. Define `ROOT` as
`Path(__file__).resolve().parents[1]`; `_read(relative)` returns UTF-8 text,
`_xml(relative)` returns `ET.parse(ROOT / relative).getroot()`, and
`_sha256(path)` streams 1 MiB chunks into `hashlib.sha256()`. Add these exact
constants:

```python
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
```

```python
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
```

Add tests that require:

```python
class BunkerAssetManifestContractTest(unittest.TestCase):
    def test_manifest_is_exact(self):
        payload = json.loads((ROOT / "config/bunker_assets.json").read_text())
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
            "redistribution-unresolved", payload["source"]["license_status"]
        )
        observed = {item["path"]: item["sha256"] for item in payload["files"]}
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
```

Define the fixture helpers as follows:

```python
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
            {"path": relative,
             "sha256": _sha256(root / relative)}
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
```

Add these complete positive/negative methods to
`BunkerAssetManifestContractTest` after the two methods above:

```python
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
```

- [ ] **Step 4: Freeze the package and vendor-install contracts**

Add exact dependency constants and structural assertions:

```python
EXPECTED_EXEC_DEPENDS = frozenset({
    "bunker_description", "gazebo_plugins", "gazebo_ros", "geometry_msgs",
    "nav_msgs", "robot_state_publisher", "roslaunch", "rospy",
    "sensor_msgs", "tf2_ros", "xacro",
})
EXPECTED_TEST_DEPENDS = frozenset({
    "gazebo_msgs", "liburdfdom-tools", "rosgraph", "rosgraph_msgs",
    "rostest", "rosunit", "tf2_msgs",
})

class BunkerPackageContractTest(unittest.TestCase):
    def test_manifest_dependency_surface_is_exact(self):
        root = ET.parse(ROOT / "src/platform/bunker_sim_runtime/package.xml").getroot()
        self.assertEqual(["catkin"], [item.text for item in root.findall("buildtool_depend")])
        self.assertEqual(EXPECTED_EXEC_DEPENDS,
                         {item.text for item in root.findall("exec_depend")})
        self.assertEqual(EXPECTED_TEST_DEPENDS,
                         {item.text for item in root.findall("test_depend")})
        self.assertEqual([], root.findall("depend"))

    def test_vendor_install_is_minimal(self):
        source = (ROOT / "src/vendor/bunker_description/CMakeLists.txt").read_text()
        self.assertRegex(
            source,
            r"install\s*\(\s*DIRECTORY\s+meshes\s+urdf\s+"
            r"DESTINATION\s+\$\{CATKIN_PACKAGE_SHARE_DESTINATION\}\s*\)",
        )
        self.assertNotRegex(source, r"DIRECTORY[^\)]*(launch|rviz|config)")
```

Add `src/vendor/bunker_description/CMakeLists.txt` to `EXACT_MODIFIED` in
`tests/test_runtime_extraction.py`, then change only the expected modified
count from `29` to `30`. Keep `42` removed, `867` frozen files, and `825`
current inherited files unchanged.

- [ ] **Step 5: Run the focused suite and verify the RED reasons**

Run:

```bash
env -i PATH=/usr/bin:/bin LANG=C.UTF-8 LC_ALL=C.UTF-8 \
  TMPDIR="$PWD/logs/bunker_standalone/engineering-tmp" \
  /usr/bin/python3 -B -m unittest -v \
  tests.test_bunker_build_contract \
  tests.test_runtime_source_contract \
  tests.test_imported_layout \
  tests.test_runtime_extraction
```

Expected: FAIL only because `config/bunker_assets.json`,
`tools/bunker_assets.py`, `src/platform/bunker_sim_runtime`, the new source
record, the vendor install rule, and its overlay hash do not yet exist.

- [ ] **Step 6: Preserve RED without committing it**

Confirm `git diff --check` is clean, keep the failing tests in the worktree,
and proceed directly to Task 2. The first commit is made only after Task 2
turns this entire focused slice GREEN; no commit in the implementation series
may intentionally leave the branch failing.

---

## Task 2: Materialize the package and install the frozen vendor assets

**Files:**

- Create: `config/bunker_assets.json`
- Create: `tools/bunker_assets.py`
- Create: `src/platform/bunker_sim_runtime/CMakeLists.txt`
- Create: `src/platform/bunker_sim_runtime/package.xml`
- Create: `src/platform/bunker_sim_runtime/setup.py`
- Create: `src/platform/bunker_sim_runtime/src/bunker_sim_runtime/__init__.py`
- Modify: `src/vendor/bunker_description/CMakeLists.txt`
- Modify: `config/runtime_sources.json`
- Modify: `config/runtime_overlay.json`

- [ ] **Step 1: Create the exact asset manifest**

Create `config/bunker_assets.json` with all 25 literal path/hash pairs sorted
by path, without generating or renaming any vendor file:

```json
{
  "schema_version": 1,
  "source": {
    "upstream_commit": "6809c15e3919d1aa3acb6518ad61c49e4150435f",
    "package_source": "Ground/src/third_party/ugv_gazebo_sim/bunker/bunker_description",
    "frozen_renderer_input": "urdf/bunker.urdf.xacro",
    "frozen_renderer_input_sha256": "41e6d862c476264dfb8d150c0f8e1780da2451867908ff6e38300813a2b54f74",
    "license_value": "TODO",
    "license_status": "redistribution-unresolved"
  },
  "files": [
    {"path": "meshes/BUNKER.STL", "sha256": "123359ab8c61059ecca8ca3995117dd6f07716fa508724a7aeef59f82d4e6bdc"},
    {"path": "meshes/base2_Link.STL", "sha256": "e982f480f1fbfca6888fa80aa16702d8b26b1b40eedddc47b7cc8aedd12c0670"},
    {"path": "meshes/base_link.STL", "sha256": "0cfa508a32b4e5ffc4460cb8f3a9c16a25c8c7b5e9b346c54c1d6e2b5a377bb8"},
    {"path": "meshes/camera1_Link.STL", "sha256": "0bb66caa54eaaaacffa53058fe5aaa31072828b54e2d9325f709b964f5347275"},
    {"path": "meshes/displayer_Link.STL", "sha256": "edc28eb71cf3b3f3e10911ef8b4dd8f481f061d85e819a65a8b954510245cfb4"},
    {"path": "meshes/laser_Link.STL", "sha256": "bbd8139b0689a69f28cd7812e6d96ee7053e07d617e1477eff286c493f2e06d7"},
    {"path": "meshes/wheel1.1_Link.STL", "sha256": "55049ae1b21db8a37028a5d864c1fa76f6b99068ebafb419f43ec072c228d03e"},
    {"path": "meshes/wheel1.2_Link.STL", "sha256": "9afb17085ea465fc18c3e1a95723467f41e983b2049fbb01296e369f738884e9"},
    {"path": "meshes/wheel1.3_Link.STL", "sha256": "4d59c7d382705d6934649924c6db529b3515d90a1c5b9316e2233e71cbae554c"},
    {"path": "meshes/wheel1_Link.STL", "sha256": "97c02fca2e406c8aa01f4ccaa0eecddbcc68d73723f7d7022aac873aa34bd676"},
    {"path": "meshes/wheel2.1_Link.STL", "sha256": "144e4ed5f0caa0294131516880caa2d5ebe6677c2847c895c2f6e6f99999eda2"},
    {"path": "meshes/wheel2.2_Link.STL", "sha256": "b17ddb11a5bb84cc79f895572602c3bdfb07ede42b8346e8f1d02b61e2f769ba"},
    {"path": "meshes/wheel2.3_Link.STL", "sha256": "9bb30ce26aeea80bb0e4f280e315d57686ba9a3806397df02f297693ad619ab8"},
    {"path": "meshes/wheel2_Link.STL", "sha256": "43842bbb532c083ddd44673fdde417912bf5ccf8e221c19a4546d7ba68c5a713"},
    {"path": "meshes/wheel3.1_Link.STL", "sha256": "969c0478bdbc80210952cc3a415f574589aed56e2a42e4a44d2466eca82960b5"},
    {"path": "meshes/wheel3.2_Link.STL", "sha256": "989a078e9367dcc077a8de97a8d37e942f700b4afae71854ad7ff206321cb17f"},
    {"path": "meshes/wheel3.3_Link.STL", "sha256": "47b64761348e114683aec82dbf640a8d4d51124970e80e0a0f5bd05a8abec6a8"},
    {"path": "meshes/wheel3_Link.STL", "sha256": "917b6a4f50ae629b09fd2e5c340250bf045ee09d647ade8cbb8cec594ad0c119"},
    {"path": "meshes/wheel4.1_Link.STL", "sha256": "534cf68721d036d8cc95d7ea2cd7709abc7f5f3493b468e8614d00b126914481"},
    {"path": "meshes/wheel4.2_Link.STL", "sha256": "2ce84dbf2dbb221a0659bb732c21e16ca83060cc1d60a2b38a736da88dedff63"},
    {"path": "meshes/wheel4.3_Link.STL", "sha256": "39d99651eec392eb5cfbfe0153d56197cd652f9410dfe21a30b62f3dce36671e"},
    {"path": "meshes/wheel4_Link.STL", "sha256": "1868ef9b98d12877ccf49e77bdfff7affe6355d1d0bc9453614e2a46c935fd1b"},
    {"path": "urdf/bunker.urdf", "sha256": "46956f10021a88edbc58ccc898dd686e5fb822b297eeb71ca0bc3fdf3e5dfa2f"},
    {"path": "urdf/bunker.urdf.xacro", "sha256": "41e6d862c476264dfb8d150c0f8e1780da2451867908ff6e38300813a2b54f74"},
    {"path": "urdf/bunker.xacro", "sha256": "7d0a6b500295b065e2bb193c028ac28fe4baf2ace36d864b8a22416d077576b0"}
  ],
  "runtime_referenced_meshes": [
    "meshes/BUNKER.STL",
    "meshes/wheel1.1_Link.STL",
    "meshes/wheel1.2_Link.STL",
    "meshes/wheel1.3_Link.STL",
    "meshes/wheel1_Link.STL",
    "meshes/wheel2.1_Link.STL",
    "meshes/wheel2.2_Link.STL",
    "meshes/wheel2.3_Link.STL",
    "meshes/wheel2_Link.STL",
    "meshes/wheel3.1_Link.STL",
    "meshes/wheel3.2_Link.STL",
    "meshes/wheel3.3_Link.STL",
    "meshes/wheel3_Link.STL",
    "meshes/wheel4.1_Link.STL",
    "meshes/wheel4.2_Link.STL",
    "meshes/wheel4.3_Link.STL",
    "meshes/wheel4_Link.STL"
  ]
}
```

- [ ] **Step 2: Implement the fail-closed asset validator**

Create `tools/bunker_assets.py` with these public types and functions:

```python
#!/usr/bin/env python3
import argparse
import hashlib
import json
import os
import re
import stat
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from typing import Mapping, Tuple

SUPPORTED_SCHEMA_VERSION = 1
EXPECTED_UPSTREAM_COMMIT = "6809c15e3919d1aa3acb6518ad61c49e4150435f"
EXPECTED_PACKAGE_SOURCE = (
    "Ground/src/third_party/ugv_gazebo_sim/bunker/bunker_description"
)
EXPECTED_RENDERER_INPUT = "urdf/bunker.urdf.xacro"
EXPECTED_LICENSE_VALUE = "TODO"
EXPECTED_LICENSE_STATUS = "redistribution-unresolved"
SHA256_PATTERN = re.compile(r"^[0-9a-f]{64}$")
PATH_PART_PATTERN = re.compile(r"^[A-Za-z0-9_.-]+$")
MESH_URI_PREFIX = "package://bunker_description/"

class AssetValidationError(ValueError):
    pass

@dataclass(frozen=True)
class AssetFile:
    path: str
    sha256: str

@dataclass(frozen=True)
class AssetManifest:
    schema_version: int
    source: Mapping[str, str]
    files: Tuple[AssetFile, ...]
    runtime_referenced_meshes: Tuple[str, ...]

    @classmethod
    def load(cls, path):
        try:
            payload = json.loads(Path(path).read_text(encoding="utf-8"))
        except (OSError, ValueError) as error:
            raise AssetValidationError(
                "cannot load asset manifest: %s" % error)
        _require_exact_keys(
            payload,
            {"schema_version", "source", "files", "runtime_referenced_meshes"},
            "manifest",
        )
        if type(payload["schema_version"]) is not int or payload["schema_version"] != 1:
            raise AssetValidationError("unsupported schema_version")
        _require_exact_keys(
            payload["source"],
            {"upstream_commit", "package_source", "frozen_renderer_input",
             "frozen_renderer_input_sha256", "license_value", "license_status"},
            "source",
        )
        if not all(
                type(value) is str and value
                for value in payload["source"].values()):
            raise AssetValidationError(
                "source metadata values must be nonempty strings")
        if payload["source"]["upstream_commit"] != EXPECTED_UPSTREAM_COMMIT:
            raise AssetValidationError("upstream commit changed")
        if payload["source"]["package_source"] != EXPECTED_PACKAGE_SOURCE:
            raise AssetValidationError("package source changed")
        if payload["source"]["license_value"] != EXPECTED_LICENSE_VALUE:
            raise AssetValidationError("vendor license value changed")
        if payload["source"]["license_status"] != EXPECTED_LICENSE_STATUS:
            raise AssetValidationError("vendor license status changed")
        if payload["source"]["frozen_renderer_input"] != EXPECTED_RENDERER_INPUT:
            raise AssetValidationError("frozen renderer input changed")
        if not SHA256_PATTERN.fullmatch(
                payload["source"]["frozen_renderer_input_sha256"]):
            raise AssetValidationError("invalid frozen renderer input digest")
        if type(payload["files"]) is not list:
            raise AssetValidationError("files must be a list")
        if type(payload["runtime_referenced_meshes"]) is not list:
            raise AssetValidationError(
                "runtime_referenced_meshes must be a list")
        files = tuple(_parse_file(item) for item in payload["files"])
        referenced = tuple(
            _safe_relative(value, "runtime reference")
            for value in payload["runtime_referenced_meshes"]
        )
        if len({item.path for item in files}) != len(files):
            raise AssetValidationError("duplicate asset path")
        if tuple(item.path for item in files) != tuple(
                sorted(item.path for item in files)):
            raise AssetValidationError("asset paths are not sorted")
        if len(set(referenced)) != len(referenced):
            raise AssetValidationError("duplicate runtime reference")
        if referenced != tuple(sorted(referenced)):
            raise AssetValidationError("runtime references are not sorted")
        declared_meshes = {
            item.path for item in files if item.path.startswith("meshes/")}
        for item in referenced:
            if not item.startswith("meshes/") or item not in declared_meshes:
                raise AssetValidationError(
                    "runtime reference is not a declared mesh: %s" % item)
        return cls(1, dict(payload["source"]), files, referenced)

def _require_exact_keys(value, expected, label):
    if type(value) is not dict:
        raise AssetValidationError("%s must be an object" % label)
    actual = set(value)
    if actual != expected:
        raise AssetValidationError(
            "%s keys differ missing=%r extra=%r" %
            (label, sorted(expected - actual), sorted(actual - expected)))

def _safe_relative(value, label):
    if (type(value) is not str or not value or "\x00" in value or
            "\\" in value):
        raise AssetValidationError("invalid %s: %r" % (label, value))
    if value.startswith("/") or re.match(r"^[A-Za-z]:", value):
        raise AssetValidationError("absolute %s: %s" % (label, value))
    parts = value.split("/")
    if (any(part in ("", ".", "..") for part in parts) or
            any(PATH_PART_PATTERN.fullmatch(part) is None for part in parts)):
        raise AssetValidationError("unsafe %s: %s" % (label, value))
    normalized = PurePosixPath(*parts).as_posix()
    if normalized != value:
        raise AssetValidationError("nonnormalized %s: %s" % (label, value))
    return normalized

def _parse_file(value):
    _require_exact_keys(value, {"path", "sha256"}, "asset file")
    relative = _safe_relative(value["path"], "asset path")
    digest = value["sha256"]
    if type(digest) is not str or SHA256_PATTERN.fullmatch(digest) is None:
        raise AssetValidationError("invalid asset digest: %s" % relative)
    if not (relative.startswith("urdf/") or
            relative.startswith("meshes/")):
        raise AssetValidationError(
            "asset outside installed closure: %s" % relative)
    return AssetFile(relative, digest)

def _canonical_directory(path):
    candidate = Path(os.path.abspath(os.fspath(path)))
    try:
        mode = candidate.lstat().st_mode
    except OSError as error:
        raise AssetValidationError("package root is unavailable: %s" % error)
    if stat.S_ISLNK(mode) or not stat.S_ISDIR(mode):
        raise AssetValidationError(
            "package root is not a real directory: %s" % candidate)
    try:
        resolved = candidate.resolve(strict=True)
    except OSError as error:
        raise AssetValidationError(
            "package root cannot be resolved: %s" % error)
    if resolved != candidate:
        raise AssetValidationError(
            "package root is noncanonical: %s" % candidate)
    return resolved

def _regular_file_within(path, root):
    root = _canonical_directory(root)
    candidate = Path(os.path.abspath(os.fspath(path)))
    try:
        relative = candidate.relative_to(root)
    except ValueError:
        raise AssetValidationError("path escapes package root: %s" % candidate)
    current = root
    for part in relative.parts:
        current = current / part
        try:
            mode = current.lstat().st_mode
        except OSError as error:
            raise AssetValidationError("missing path %s: %s" %
                                       (relative, error))
        if stat.S_ISLNK(mode):
            raise AssetValidationError("symlink is forbidden: %s" % relative)
    try:
        resolved = candidate.resolve(strict=True)
        resolved.relative_to(root)
    except (OSError, ValueError) as error:
        raise AssetValidationError("path escapes package root: %s" % error)
    if not stat.S_ISREG(resolved.stat().st_mode):
        raise AssetValidationError("not a regular file: %s" % relative)
    return resolved

def _scan_regular_files(root, directory_names):
    root = _canonical_directory(root)
    observed = set()

    def fail_walk(error):
        raise AssetValidationError("asset walk failed: %s" % error)

    for directory_name in directory_names:
        directory = root / directory_name
        try:
            mode = directory.lstat().st_mode
        except OSError as error:
            raise AssetValidationError(
                "missing asset directory %s: %s" % (directory_name, error))
        if stat.S_ISLNK(mode) or not stat.S_ISDIR(mode):
            raise AssetValidationError(
                "asset directory is not a real directory: %s" %
                directory_name)
        for current_text, directories, filenames in os.walk(
                str(directory), topdown=True, onerror=fail_walk,
                followlinks=False):
            current = Path(current_text)
            directories.sort()
            filenames.sort()
            for name in directories:
                child = current / name
                mode = child.lstat().st_mode
                if stat.S_ISLNK(mode) or not stat.S_ISDIR(mode):
                    raise AssetValidationError(
                        "invalid asset directory: %s" %
                        child.relative_to(root))
            for name in filenames:
                child = current / name
                mode = child.lstat().st_mode
                relative = child.relative_to(root).as_posix()
                if stat.S_ISLNK(mode) or not stat.S_ISREG(mode):
                    raise AssetValidationError(
                        "invalid asset file: %s" % relative)
                _regular_file_within(child, root)
                observed.add(relative)
    return observed

def _sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()

def _mesh_references(path, root):
    source = _regular_file_within(path, root)
    try:
        xml_root = ET.parse(str(source)).getroot()
    except (OSError, ET.ParseError) as error:
        raise AssetValidationError("cannot parse frozen xacro: %s" % error)
    references = set()
    for mesh in xml_root.findall(".//mesh"):
        uri = mesh.get("filename")
        if type(uri) is not str or not uri.startswith(MESH_URI_PREFIX):
            raise AssetValidationError("malformed BUNKER mesh URI: %r" % uri)
        relative = _safe_relative(
            uri[len(MESH_URI_PREFIX):], "mesh URI")
        if (not relative.startswith("meshes/") or
                len(PurePosixPath(relative).parts) != 2):
            raise AssetValidationError("malformed BUNKER mesh URI: %s" % uri)
        references.add(relative)
    return references

def validate_asset_tree(manifest, package_root):
    manifest = manifest if isinstance(manifest, AssetManifest) else AssetManifest.load(manifest)
    root = _canonical_directory(package_root)
    observed = _scan_regular_files(root, ("urdf", "meshes"))
    expected = {item.path for item in manifest.files}
    if observed != expected:
        missing, extra = sorted(expected - observed), sorted(observed - expected)
        raise AssetValidationError("asset closure mismatch missing=%r extra=%r" % (missing, extra))
    for item in manifest.files:
        if _sha256(_regular_file_within(root / item.path, root)) != item.sha256:
            raise AssetValidationError("asset hash mismatch: %s" % item.path)
    package_xml = _regular_file_within(root / "package.xml", root)
    license_values = [item.text for item in ET.parse(package_xml).getroot().findall("license")]
    if license_values != [EXPECTED_LICENSE_VALUE]:
        raise AssetValidationError("vendor package license changed: package.xml")
    frozen = manifest.source["frozen_renderer_input"]
    digest_by_path = {item.path: item.sha256 for item in manifest.files}
    if digest_by_path.get(frozen) != manifest.source["frozen_renderer_input_sha256"]:
        raise AssetValidationError("frozen renderer digest disagrees with files")
    referenced = _mesh_references(
        root / manifest.source["frozen_renderer_input"], root)
    if referenced != set(manifest.runtime_referenced_meshes):
        raise AssetValidationError("runtime mesh reference set mismatch")
    return root

def validate_source_install_pair(manifest, source_root, install_root):
    parsed = (manifest if isinstance(manifest, AssetManifest)
              else AssetManifest.load(manifest))
    source = validate_asset_tree(parsed, source_root)
    installed = validate_asset_tree(parsed, install_root)
    return source, installed

def main(argv=None):
    parser = argparse.ArgumentParser(prog="bunker_assets.py")
    parser.add_argument("--config", required=True)
    parser.add_argument("--source-root", required=True)
    parser.add_argument("--install-root")
    arguments = parser.parse_args(argv)
    try:
        if arguments.install_root:
            roots = validate_source_install_pair(
                arguments.config, arguments.source_root,
                arguments.install_root)
        else:
            roots = (validate_asset_tree(
                arguments.config, arguments.source_root),)
    except (AssetValidationError, OSError, ValueError) as error:
        sys.stderr.write("bunker-assets: %s\n" % error)
        return 1
    for root in roots:
        print(str(root))
    return 0

if __name__ == "__main__":
    sys.exit(main())
```

Invoke this validator with `/usr/bin/python3`; it is intentionally not an
installed executable and needs no executable bit.

- [ ] **Step 3: Add the package scaffold and exact dependency manifest**

Create `setup.py` and the empty public module:

```python
# setup.py
from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=["bunker_sim_runtime"],
    package_dir={"": "src"},
)
setup(**setup_args)
```

```python
# src/bunker_sim_runtime/__init__.py
"""Pure helpers for the Simulation Platform V1.0 BUNKER runtime."""
```

Create `CMakeLists.txt` initially as:

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(bunker_sim_runtime)

find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()
```

Create `package.xml` with this exact dependency surface:

```xml
<?xml version="1.0"?>
<package format="2">
  <name>bunker_sim_runtime</name>
  <version>1.0.0</version>
  <description>Standalone BUNKER runtime for Simulation Platform V1.0.</description>
  <maintainer email="sim-platform@example.invalid">Simulation Platform Maintainers</maintainer>
  <license>BSD-3-Clause</license>
  <buildtool_depend>catkin</buildtool_depend>
  <exec_depend>bunker_description</exec_depend>
  <exec_depend>gazebo_plugins</exec_depend>
  <exec_depend>gazebo_ros</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>roslaunch</exec_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>tf2_ros</exec_depend>
  <exec_depend>xacro</exec_depend>
  <test_depend>gazebo_msgs</test_depend>
  <test_depend>liburdfdom-tools</test_depend>
  <test_depend>rosgraph</test_depend>
  <test_depend>rosgraph_msgs</test_depend>
  <test_depend>rostest</test_depend>
  <test_depend>rosunit</test_depend>
  <test_depend>tf2_msgs</test_depend>
</package>
```

- [ ] **Step 4: Install only the vendor runtime assets and update ledgers**

Append exactly this to `src/vendor/bunker_description/CMakeLists.txt`:

```cmake
install(
  DIRECTORY meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
```

Add this sorted record to `config/runtime_sources.json`:

```json
"bunker_sim_runtime": {
  "role": "platform",
  "source": null,
  "destination": "src/platform/bunker_sim_runtime",
  "imported": false
}
```

Append the install rule after exactly one blank line at the current vendor
CMake EOF. Its final SHA-256 must be
`32e596f3222f4e52a94a678f0c0b1dbad667293306f1b77d1e4dcfa3c7072acb`.
Add this sorted entry to `config/runtime_overlay.json`'s `modified` object:

```json
"src/vendor/bunker_description/CMakeLists.txt": "32e596f3222f4e52a94a678f0c0b1dbad667293306f1b77d1e4dcfa3c7072acb"
```

Add only the relative path string to the test's `EXACT_MODIFIED` frozenset:

```python
"src/vendor/bunker_description/CMakeLists.txt",
```

The frozen upstream CMake SHA-256 remains
`29479bcbbb2b268b467fba38011ca4199aa666809fa3806156832665415f58bb`.
Do not change `config/import_provenance.json` or the vendor `package.xml`
(frozen SHA-256
`539b6227d37f1b808b2e40a7f9692b518fd556911c516744d58d1a197b65c15e`).

Verify the frozen digest with:

```bash
sha256sum src/vendor/bunker_description/CMakeLists.txt
```

Expected: the exact final digest above followed by the relative file path.
The extraction test remains the authoritative closure check.

- [ ] **Step 5: Run the focused suite to GREEN**

Run the exact focused command:

```bash
env -i PATH=/usr/bin:/bin LANG=C.UTF-8 LC_ALL=C.UTF-8 \
  TMPDIR="$PWD/logs/bunker_standalone/engineering-tmp" \
  /usr/bin/python3 -B -m unittest -v \
  tests.test_bunker_build_contract \
  tests.test_runtime_source_contract \
  tests.test_imported_layout \
  tests.test_runtime_extraction
```

Expected: all selected tests pass, with 18
imported packages, 6 local declarations, 3 materialized local packages, 30
modified inherited files, 42 removed files, and an exact 25-file BUNKER asset
closure.

- [ ] **Step 6: Commit the boundary implementation**

```bash
git add config/bunker_assets.json config/runtime_sources.json \
  config/runtime_overlay.json tools/bunker_assets.py \
  src/vendor/bunker_description/CMakeLists.txt \
  src/platform/bunker_sim_runtime tests/test_bunker_build_contract.py \
  tests/test_runtime_source_contract.py tests/test_imported_layout.py \
  tests/test_runtime_extraction.py
git commit -m "build: install the frozen BUNKER asset closure"
```

---

## Task 3: Implement pure identity, sim-time, and velocity admission contracts

**Files:**

- Create: `src/platform/bunker_sim_runtime/src/bunker_sim_runtime/contracts.py`
- Create: `src/platform/bunker_sim_runtime/src/bunker_sim_runtime/sim_time.py`
- Create: `src/platform/bunker_sim_runtime/src/bunker_sim_runtime/velocity_guard.py`
- Create: `src/platform/bunker_sim_runtime/test/test_contracts.py`
- Create: `src/platform/bunker_sim_runtime/test/test_sim_time.py`
- Create: `src/platform/bunker_sim_runtime/test/test_velocity_guard.py`
- Create: `src/platform/bunker_sim_runtime/test/test_spawn_bunker_preflight.py`
- Create: `src/platform/bunker_sim_runtime/scripts/velocity_guard.py`
- Create: `src/platform/bunker_sim_runtime/scripts/spawn_bunker_preflight.py`

- [ ] **Step 1: Write failing pure command-admission tests**

In `test_velocity_guard.py`, table-drive `admit_components` with the six Twist
scalars in `(linear.x, linear.y, linear.z, angular.x, angular.y, angular.z)`
order. Assert these exact outcomes:

```python
VALID_CASES = (
    ((0.25, 0.0, 0.0, 0.0, 0.0, 0.5),
     (0.25, 0.0, 0.0, 0.0, 0.0, 0.5), None),
    ((0.75, 0.0, 0.0, 0.0, 0.0, 2.0),
     (0.5, 0.0, 0.0, 0.0, 0.0, 1.0), "clamped"),
    ((-0.75, 0.0, 0.0, 0.0, 0.0, -2.0),
     (-0.5, 0.0, 0.0, 0.0, 0.0, -1.0), "clamped"),
)
REJECT_CASES = (
    ((float("nan"), 0, 0, 0, 0, 0), "nonfinite"),
    ((0, 0, 0, 0, 0, float("inf")), "nonfinite"),
    ((0, 1.1e-9, 0, 0, 0, 0), "nonplanar"),
    ((0, 0, -1.1e-9, 0, 0, 0), "nonplanar"),
    ((0, 0, 0, 1.1e-9, 0, 0), "nonplanar"),
    ((0, 0, 0, 0, -1.1e-9, 0), "nonplanar"),
)
ZERO = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
```

Also assert that a disallowed component of exactly `1e-9` is admitted as
zero, booleans/non-numeric values are rejected as `nonfinite`, input objects
are never mutated, and this pure module imports without `rospy` or
`geometry_msgs` present.

- [ ] **Step 2: Write failing frozen-identity and spawn-preflight tests**

In `test_contracts.py`, freeze these values and grammars:

```python
MODEL_NAME = "bunker"
ROS_NAMESPACE = "/ground"
TF_PREFIX = "ground"
DEFAULT_POSE = ("0.0", "0.0", "0.36", "0.0", "0.0", "0.0")
LOCAL_NAME_RE = re.compile(r"^[A-Za-z][A-Za-z0-9_]{0,63}$")
ENTITY_NAME_RE = re.compile(r"^[a-z][a-z0-9_]{0,63}$")
DECIMAL_RE = re.compile(
    r"^[+-]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)(?:[eE][+-]?[0-9]+)?$"
)
SPAWN_EXECUTABLE = "/opt/ros/noetic/lib/gazebo_ros/spawn_model"
```

Test `validate_local_name`, `validate_entity_name`, and `parse_pose` against
empty strings, dots, `..`, slashes, whitespace, XML punctuation, 65-character
names, `nan`, `inf`, hexadecimal floats, missing fields, and extra fields.
`parse_pose` must return six finite floats without changing their order.

Test `validate_spawn_command(command, pose, ros_log_dir)` using this one
accepted vector:

```python
ros_log_dir = TEMP_ROOT / "ros-log"
ros_log_dir.mkdir(mode=0o700)
command = [
    SPAWN_EXECUTABLE,
    "-urdf", "-param", "robot_description",
    "-model", "bunker", "-b",
    "-x", pose[0], "-y", pose[1], "-z", pose[2],
    "-R", pose[3], "-P", pose[4], "-Y", pose[5],
    "__name:=spawn_bunker",
    "__log:=%s" % (ros_log_dir / "ground-spawn_bunker-1.log"),
]
```

The first 19 tokens are the frozen `spawn_model` invocation. The last two are
the only ROS remaps that local roslaunch appends. Require
`__name:=spawn_bunker`; require the `__log` value to be a canonical descendant
of the supplied mode-0700 `ROS_LOG_DIR` with basename matching
`ground-spawn_bunker-[1-9][0-9]*.log`. Reject any third remap and return all 21
tokens unchanged so `spawn_model` still receives its ROS remappings.

Reject a noncanonical executable, omitted `-b`, reordered/duplicated flags,
an absolute robot-description parameter, any model other than `bunker`, and a
pose token that differs from the validated launch-prefix pose.

- [ ] **Step 3: Write failing strict XML-RPC sim-time tests**

In `test_sim_time.py`, inject a fake XML-RPC proxy into
`require_boolean_sim_time(master_uri, caller_id, proxy_factory)`. Accept the
normal XML-RPC list `[1, message, True]` and the equivalent tuple. Reject
transport exceptions, every other response length, non-success codes,
missing values, `False`, `0`, `1`, and the strings `"true"`/`"false"`. The
return value on success is the literal `True`; failures raise
`SimTimeContractError` without importing `rospy`.

Create `test_spawn_bunker_preflight.py` with a mode-0700 `ROS_LOG_DIR` below
`logs/bunker_standalone/engineering-tmp`. Import the script by file path and
call its injected `main`. Cover the accepted path with a fake sim-time reader
and a fake `execv` that records the exact 21-token vector, then cover: wrong
argument shape (`64`), missing environment (`65`), non-boolean/false sim time
(`65`), unavailable canonical spawn executable (`66`), and `execv` failure
(`66`). Use the basename `ground-spawn_bunker-1.log`; no test may create a
file outside the SIM-local temporary root.

```python
import importlib.util
import io
import tempfile
import unittest
from pathlib import Path

from bunker_sim_runtime.contracts import DEFAULT_POSE, SPAWN_EXECUTABLE
from bunker_sim_runtime.sim_time import SimTimeContractError

ROOT = Path(__file__).resolve().parents[4]
SCRIPT = ROOT / (
    "src/platform/bunker_sim_runtime/scripts/spawn_bunker_preflight.py")
TEMP_ROOT = ROOT / "logs/bunker_standalone/engineering-tmp"

def _load_script():
    spec = importlib.util.spec_from_file_location(
        "spawn_bunker_preflight_test_target", str(SCRIPT))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

class SpawnBunkerPreflightTest(unittest.TestCase):
    def setUp(self):
        TEMP_ROOT.mkdir(mode=0o700, parents=True, exist_ok=True)
        self.temporary = tempfile.TemporaryDirectory(dir=str(TEMP_ROOT))
        self.addCleanup(self.temporary.cleanup)
        self.log_dir = Path(self.temporary.name) / "ros-log"
        self.log_dir.mkdir(mode=0o700)
        self.environment = {
            "ROS_LOG_DIR": str(self.log_dir),
            "ROS_MASTER_URI": "http://127.0.0.1:11311/",
        }
        self.command = [
            SPAWN_EXECUTABLE,
            "-urdf", "-param", "robot_description",
            "-model", "bunker", "-b",
            "-x", DEFAULT_POSE[0], "-y", DEFAULT_POSE[1],
            "-z", DEFAULT_POSE[2], "-R", DEFAULT_POSE[3],
            "-P", DEFAULT_POSE[4], "-Y", DEFAULT_POSE[5],
            "__name:=spawn_bunker",
            "__log:=%s" % (self.log_dir / "ground-spawn_bunker-1.log"),
        ]
        self.argv = list(DEFAULT_POSE) + ["--"] + self.command
        self.module = _load_script()

    @staticmethod
    def _sim_time(master_uri, caller_id):
        if master_uri != "http://127.0.0.1:11311/":
            raise AssertionError(master_uri)
        if caller_id != "/ground/spawn_bunker":
            raise AssertionError(caller_id)
        return True

    def test_delegates_the_exact_command_after_preflight(self):
        calls = []
        errors = io.StringIO()
        status = self.module.main(
            argv=self.argv, environ=self.environment, stderr=errors,
            sim_time_reader=self._sim_time,
            execv=lambda executable, command: calls.append(
                (executable, tuple(command))),
            isfile=lambda path: True, access=lambda path, mode: True)
        self.assertEqual(70, status)
        self.assertEqual(
            [(SPAWN_EXECUTABLE, tuple(self.command))], calls)
        self.assertIn("execv returned unexpectedly", errors.getvalue())

    def test_rejects_bad_arguments_and_missing_environment(self):
        self.assertEqual(64, self.module.main(argv=(), stderr=io.StringIO()))
        self.assertEqual(
            65,
            self.module.main(
                argv=self.argv, environ={}, stderr=io.StringIO(),
                isfile=lambda path: True, access=lambda path, mode: True),
        )

    def test_rejects_sim_time_failure(self):
        def reject(master_uri, caller_id):
            raise SimTimeContractError("not boolean true")

        self.assertEqual(
            65,
            self.module.main(
                argv=self.argv, environ=self.environment,
                stderr=io.StringIO(), sim_time_reader=reject,
                isfile=lambda path: True, access=lambda path, mode: True),
        )

    def test_rejects_unavailable_spawn_and_exec_failure(self):
        self.assertEqual(
            66,
            self.module.main(
                argv=self.argv, environ=self.environment,
                stderr=io.StringIO(), isfile=lambda path: False,
                access=lambda path, mode: False),
        )

        def fail_exec(executable, command):
            raise OSError("fixture exec failure")

        self.assertEqual(
            66,
            self.module.main(
                argv=self.argv, environ=self.environment,
                stderr=io.StringIO(), sim_time_reader=self._sim_time,
                execv=fail_exec, isfile=lambda path: True,
                access=lambda path, mode: True),
        )

if __name__ == "__main__":
    unittest.main()
```

Run the three files directly and verify they fail on missing modules:

```bash
scripts/with_noetic_env.bash /usr/bin/env \
  TMPDIR="$PWD/logs/bunker_standalone/engineering-tmp" \
  PYTHONPATH="$PWD/src/platform/bunker_sim_runtime/src" \
  /usr/bin/python3 -B -m unittest discover -v \
  -s src/platform/bunker_sim_runtime/test -p 'test_contracts.py'
scripts/with_noetic_env.bash /usr/bin/env \
  TMPDIR="$PWD/logs/bunker_standalone/engineering-tmp" \
  PYTHONPATH="$PWD/src/platform/bunker_sim_runtime/src" \
  /usr/bin/python3 -B -m unittest discover -v \
  -s src/platform/bunker_sim_runtime/test -p 'test_sim_time.py'
scripts/with_noetic_env.bash /usr/bin/env \
  TMPDIR="$PWD/logs/bunker_standalone/engineering-tmp" \
  PYTHONPATH="$PWD/src/platform/bunker_sim_runtime/src" \
  /usr/bin/python3 -B -m unittest discover -v \
  -s src/platform/bunker_sim_runtime/test -p 'test_velocity_guard.py'
scripts/with_noetic_env.bash /usr/bin/env \
  TMPDIR="$PWD/logs/bunker_standalone/engineering-tmp" \
  PYTHONPATH="$PWD/src/platform/bunker_sim_runtime/src" \
  /usr/bin/python3 -B -m unittest discover -v \
  -s src/platform/bunker_sim_runtime/test \
  -p 'test_spawn_bunker_preflight.py'
```

- [ ] **Step 4: Implement the three pure modules**

Create `contracts.py` with the complete frozen validators below:

```python
import decimal
import math
import os
from pathlib import Path
import re

MODEL_NAME = "bunker"
ROS_NAMESPACE = "/ground"
TF_PREFIX = "ground"
DEFAULT_POSE = ("0.0", "0.0", "0.36", "0.0", "0.0", "0.0")
LOCAL_NAME_RE = re.compile(r"^[A-Za-z][A-Za-z0-9_]{0,63}$")
ENTITY_NAME_RE = re.compile(r"^[a-z][a-z0-9_]{0,63}$")
DECIMAL_RE = re.compile(
    r"^[+-]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)(?:[eE][+-]?[0-9]+)?$")
SPAWN_EXECUTABLE = "/opt/ros/noetic/lib/gazebo_ros/spawn_model"
SPAWN_LOG_RE = re.compile(r"^ground-spawn_bunker-[1-9][0-9]*\.log$")

class RuntimeContractError(ValueError):
    pass

def validate_local_name(value):
    if type(value) is not str or LOCAL_NAME_RE.fullmatch(value) is None:
        raise RuntimeContractError("invalid local name: %r" % (value,))
    return value

def validate_entity_name(value):
    if type(value) is not str or ENTITY_NAME_RE.fullmatch(value) is None:
        raise RuntimeContractError("invalid entity name: %r" % (value,))
    return value

def parse_pose(values):
    if (not isinstance(values, (list, tuple)) or
            isinstance(values, (str, bytes)) or len(values) != 6):
        raise RuntimeContractError("pose requires six decimal scalars")
    parsed = []
    for value in values:
        if type(value) is not str or DECIMAL_RE.fullmatch(value) is None:
            raise RuntimeContractError("invalid pose scalar: %r" % (value,))
        number = decimal.Decimal(value)
        result = float(number)
        if not number.is_finite() or not math.isfinite(result):
            raise RuntimeContractError("nonfinite pose scalar: %r" % value)
        parsed.append(result)
    return tuple(parsed)

def _canonical_mode_0700_directory(value):
    try:
        path = Path(value)
    except TypeError:
        raise RuntimeContractError("ROS_LOG_DIR is not canonical")
    if not path.is_absolute() or path.is_symlink():
        raise RuntimeContractError("ROS_LOG_DIR is not canonical")
    canonical = path.resolve(strict=True)
    if canonical != path or not canonical.is_dir():
        raise RuntimeContractError("ROS_LOG_DIR is not canonical")
    if (canonical.stat().st_mode & 0o777) != 0o700:
        raise RuntimeContractError("ROS_LOG_DIR is not mode 0700")
    return canonical

def validate_spawn_command(command, pose_tokens, ros_log_dir):
    parse_pose(pose_tokens)
    if (not isinstance(command, (list, tuple)) or
            isinstance(command, (str, bytes)) or len(command) != 21):
        raise RuntimeContractError("spawn command requires 21 tokens")
    if any(type(value) is not str for value in command):
        raise RuntimeContractError("spawn command tokens must be strings")
    executable = os.path.realpath(command[0])
    if (command[0] != SPAWN_EXECUTABLE or executable != SPAWN_EXECUTABLE or
            not os.path.isfile(executable) or
            not os.access(executable, os.X_OK)):
        raise RuntimeContractError("invalid spawn executable")
    expected = (
        SPAWN_EXECUTABLE,
        "-urdf", "-param", "robot_description", "-model", MODEL_NAME, "-b",
        "-x", pose_tokens[0], "-y", pose_tokens[1], "-z", pose_tokens[2],
        "-R", pose_tokens[3], "-P", pose_tokens[4], "-Y", pose_tokens[5],
    )
    if tuple(command[:19]) != expected:
        raise RuntimeContractError("spawn command contract changed")
    if command[19] != "__name:=spawn_bunker":
        raise RuntimeContractError("spawn node remap changed")
    if not command[20].startswith("__log:="):
        raise RuntimeContractError("spawn log remap is missing")
    log_root = _canonical_mode_0700_directory(ros_log_dir)
    log_path = Path(command[20][len("__log:="):])
    if not log_path.is_absolute():
        raise RuntimeContractError("spawn log path is invalid")
    if SPAWN_LOG_RE.fullmatch(log_path.name) is None:
        raise RuntimeContractError("spawn log basename changed")
    parent = log_path.parent.resolve(strict=True)
    if parent != log_path.parent:
        raise RuntimeContractError("spawn log path is noncanonical")
    try:
        parent.relative_to(log_root)
    except ValueError:
        raise RuntimeContractError("spawn log escapes ROS_LOG_DIR")
    return tuple(command)
```

Create `sim_time.py` with this complete XML-RPC implementation:

```python
import xmlrpc.client

class SimTimeContractError(RuntimeError):
    pass

def require_boolean_sim_time(master_uri, caller_id,
                             proxy_factory=xmlrpc.client.ServerProxy):
    try:
        response = proxy_factory(master_uri).getParam(
            caller_id, "/use_sim_time")
    except Exception as error:
        raise SimTimeContractError(
            "could not read /use_sim_time: %s" % error)
    if not isinstance(response, (list, tuple)) or len(response) != 3:
        raise SimTimeContractError("invalid ROS master getParam response")
    code, message, value = response
    if type(code) is not int or code != 1 or type(message) is not str:
        raise SimTimeContractError("ROS master rejected /use_sim_time")
    if type(value) is not bool or value is not True:
        raise SimTimeContractError("/use_sim_time must be boolean true")
    return True
```

Create `velocity_guard.py` with this complete pure implementation:

```python
import math

ZERO = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

def admit_components(linear_x, linear_y, linear_z,
                     angular_x, angular_y, angular_z,
                     epsilon=1e-9):
    raw = (linear_x, linear_y, linear_z,
           angular_x, angular_y, angular_z)
    if any(type(value) is bool for value in raw):
        return ZERO, "nonfinite"
    try:
        values = tuple(float(value) for value in raw)
    except (TypeError, ValueError, OverflowError):
        return ZERO, "nonfinite"
    if not all(math.isfinite(value) for value in values):
        return ZERO, "nonfinite"
    if any(abs(value) > epsilon for value in
           (values[1], values[2], values[3], values[4])):
        return ZERO, "nonplanar"
    admitted = (
        max(-0.5, min(0.5, values[0])), 0.0, 0.0, 0.0, 0.0,
        max(-1.0, min(1.0, values[5])),
    )
    return admitted, ("clamped" if admitted[0] != values[0]
                       or admitted[5] != values[5] else None)
```

Compile the three pure modules without writing outside SIM, then run the three
pure suites before adding ROS adapters. The preflight adapter test remains RED
until Step 5:

```bash
scripts/with_noetic_env.bash /usr/bin/env \
  TMPDIR="$PWD/logs/bunker_standalone/engineering-tmp" \
  PYTHONPYCACHEPREFIX="$PWD/logs/bunker_standalone/engineering-tmp/pycache" \
  /usr/bin/python3 -B -m py_compile \
  src/platform/bunker_sim_runtime/src/bunker_sim_runtime/contracts.py \
  src/platform/bunker_sim_runtime/src/bunker_sim_runtime/sim_time.py \
  src/platform/bunker_sim_runtime/src/bunker_sim_runtime/velocity_guard.py
for bunker_pattern in \
    test_contracts.py test_sim_time.py test_velocity_guard.py; do
  scripts/with_noetic_env.bash /usr/bin/env \
    TMPDIR="$PWD/logs/bunker_standalone/engineering-tmp" \
    PYTHONPATH="$PWD/src/platform/bunker_sim_runtime/src" \
    /usr/bin/python3 -B -m unittest discover -v \
    -s src/platform/bunker_sim_runtime/test -p "$bunker_pattern"
done
```

- [ ] **Step 5: Add the thin ROS guard and spawn-prefix adapters**

Create `scripts/velocity_guard.py` with this complete ROS adapter:

```python
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

from bunker_sim_runtime.velocity_guard import admit_components

class VelocityGuard:
    def __init__(self):
        self._publisher = rospy.Publisher(
            "cmd_vel_safe", Twist, queue_size=1)
        self._subscriber = rospy.Subscriber(
            "cmd_vel", Twist, self._callback, queue_size=1)

    def _callback(self, message):
        admitted, reason = admit_components(
            message.linear.x, message.linear.y, message.linear.z,
            message.angular.x, message.angular.y, message.angular.z)
        output = Twist()
        output.linear.x, output.linear.y, output.linear.z = admitted[:3]
        output.angular.x, output.angular.y, output.angular.z = admitted[3:]
        if reason in ("nonfinite", "nonplanar"):
            rospy.logerr_throttle(
                1.0, "velocity guard rejected %s input" % reason)
        elif reason == "clamped":
            rospy.logwarn_throttle(1.0, "velocity guard clamped input")
        self._publisher.publish(output)

def main():
    rospy.init_node("velocity_guard")
    VelocityGuard()
    rospy.spin()

if __name__ == "__main__":
    main()
```

This node has no timer, lifecycle, or duplicate watchdog.

`scripts/spawn_bunker_preflight.py` accepts exactly:

```text
spawn_bunker_preflight.py X Y Z ROLL PITCH YAW -- SPAWN_COMMAND
```

`SPAWN_COMMAND` is the exact 21-token command frozen by
`validate_spawn_command`; it is not a shell string.

Create `scripts/spawn_bunker_preflight.py` with this complete adapter:

```python
#!/usr/bin/env python3
import os
import sys

from bunker_sim_runtime.contracts import (
    RuntimeContractError,
    SPAWN_EXECUTABLE,
    parse_pose,
    validate_spawn_command,
)
from bunker_sim_runtime.sim_time import (
    SimTimeContractError,
    require_boolean_sim_time,
)

def _fail(code, message, stderr):
    stderr.write("bunker-spawn-preflight: %s\n" % message)
    return code

def main(argv=None, environ=None, stderr=None,
         sim_time_reader=require_boolean_sim_time, execv=os.execv,
         isfile=os.path.isfile, access=os.access):
    arguments = list(sys.argv[1:] if argv is None else argv)
    environment = os.environ if environ is None else environ
    errors = sys.stderr if stderr is None else stderr
    if len(arguments) < 8 or arguments[6] != "--":
        return _fail(
            64, "expected six pose scalars, --, and spawn command", errors)
    pose_tokens = arguments[:6]
    command = arguments[7:]
    try:
        parse_pose(pose_tokens)
    except RuntimeContractError as error:
        return _fail(64, str(error), errors)
    if (command and command[0] == SPAWN_EXECUTABLE and
            (not isfile(SPAWN_EXECUTABLE) or
             not access(SPAWN_EXECUTABLE, os.X_OK))):
        return _fail(
            66, "frozen spawn executable is unavailable", errors)
    try:
        ros_log_dir = environment["ROS_LOG_DIR"]
        master_uri = environment["ROS_MASTER_URI"]
    except KeyError as error:
        return _fail(
            65, "missing environment variable %s" % error.args[0], errors)
    try:
        validated = validate_spawn_command(
            command, pose_tokens, ros_log_dir)
    except RuntimeContractError as error:
        return _fail(64, str(error), errors)
    try:
        sim_time_reader(master_uri, "/ground/spawn_bunker")
    except SimTimeContractError as error:
        return _fail(65, str(error), errors)
    try:
        execv(validated[0], validated)
    except OSError as error:
        return _fail(
            66, "could not exec spawn_model: %s" % error, errors)
    return _fail(70, "execv returned unexpectedly", errors)

if __name__ == "__main__":
    raise SystemExit(main())
```

Mark both adapters executable immediately:

```bash
chmod 755 src/platform/bunker_sim_runtime/scripts/velocity_guard.py \
  src/platform/bunker_sim_runtime/scripts/spawn_bunker_preflight.py
```

- [ ] **Step 6: Run the pure tests to GREEN and commit**

Run all four exact commands. Expected: all cases pass without a ROS master or
Gazebo process.

```bash
scripts/with_noetic_env.bash /usr/bin/env \
  TMPDIR="$PWD/logs/bunker_standalone/engineering-tmp" \
  PYTHONPATH="$PWD/src/platform/bunker_sim_runtime/src" \
  /usr/bin/python3 -B -m unittest discover -v \
  -s src/platform/bunker_sim_runtime/test -p test_contracts.py
scripts/with_noetic_env.bash /usr/bin/env \
  TMPDIR="$PWD/logs/bunker_standalone/engineering-tmp" \
  PYTHONPATH="$PWD/src/platform/bunker_sim_runtime/src" \
  /usr/bin/python3 -B -m unittest discover -v \
  -s src/platform/bunker_sim_runtime/test -p test_sim_time.py
scripts/with_noetic_env.bash /usr/bin/env \
  TMPDIR="$PWD/logs/bunker_standalone/engineering-tmp" \
  PYTHONPATH="$PWD/src/platform/bunker_sim_runtime/src" \
  /usr/bin/python3 -B -m unittest discover -v \
  -s src/platform/bunker_sim_runtime/test -p test_velocity_guard.py
scripts/with_noetic_env.bash /usr/bin/env \
  TMPDIR="$PWD/logs/bunker_standalone/engineering-tmp" \
  PYTHONPATH="$PWD/src/platform/bunker_sim_runtime/src" \
  /usr/bin/python3 -B -m unittest discover -v \
  -s src/platform/bunker_sim_runtime/test \
  -p test_spawn_bunker_preflight.py
```

```bash
git add src/platform/bunker_sim_runtime/src/bunker_sim_runtime \
  src/platform/bunker_sim_runtime/scripts/velocity_guard.py \
  src/platform/bunker_sim_runtime/scripts/spawn_bunker_preflight.py \
  src/platform/bunker_sim_runtime/test/test_contracts.py \
  src/platform/bunker_sim_runtime/test/test_spawn_bunker_preflight.py \
  src/platform/bunker_sim_runtime/test/test_sim_time.py \
  src/platform/bunker_sim_runtime/test/test_velocity_guard.py
git commit -m "feat: guard the BUNKER runtime inputs"
```

---

## Task 4: Render one deterministic BUNKER runtime URDF

**Files:**

- Create: `src/platform/bunker_sim_runtime/src/bunker_sim_runtime/renderer.py`
- Create: `src/platform/bunker_sim_runtime/scripts/render_bunker_runtime.py`
- Create: `src/platform/bunker_sim_runtime/test/test_renderer.py`

- [ ] **Step 1: Write the failing source-structure and name-remap tests**

Load the frozen vendor xacro through `/opt/ros/noetic/bin/xacro`, then require
the renderer to reject malformed XML, a wrong input SHA-256, a source path
other than `urdf/bunker.urdf.xacro`, any symlink in the input or referenced
mesh path, a missing/non-regular mesh, a source tree not containing exactly
17 links and 16 joints, and any input `<plugin>`.

Test `canonical_local_name` with consecutive invalid characters collapsed to
one underscore. Freeze these representative mappings and assert that two
different source names mapping to the same output are rejected:

```python
{
    "base_link": "base_link",
    "wheel1.1_Link": "wheel1_1_Link",
    "wheel1.1_jont": "wheel1_1_jont",
    "wheel4.3_joint": "wheel4_3_joint",
}
```

Assert remapping covers link/joint `name`, joint parent/child `link`,
`gazebo@reference`, and `mimic@joint` attributes. Do not silently correct the
upstream `jont` typo beyond replacing the dot.

- [ ] **Step 2: Write the failing exact output-tree tests**

For `render_runtime_urdf(frozen_input)`, parse stdout and assert:

```python
EXPECTED_COUNTS = {
    "link": 18,
    "joint": 17,
    "collision": 1,
    "sensor": 1,
    "plugin": 2,
}
EXPECTED_BOX_ORIGIN = "0.018058912 0.001357451 -0.160420741"
EXPECTED_BOX_SIZE = "1.026335219 0.782744936 0.395154782"
EXPECTED_RUNTIME_URDF_SHA256 = (
    "2b58856bed616a9e7402f7ddf4be4336f669271ca3989fdcf1d50f6939270832")
EXPECTED_PLUGIN_FILES = {
    "libgazebo_ros_planar_move.so",
    "libgazebo_ros_laser.so",
}
```

All 17 joints must be `fixed`; every `axis`, `limit`, `dynamics`,
`calibration`, `mimic`, and `safety_controller` child must be absent. Every
source visual and inertial element remains. The only collision is
`base_link_collision` under `base_link`, with one box and no mesh. The root is
`base_link`. All names satisfy the local grammar and are unique within their
entity class.

Assert this exact LiDAR subtree beneath `<gazebo
reference="lidar_2d_link">`:

```xml
<sensor name="bunker_lidar_2d" type="ray">
  <always_on>true</always_on>
  <update_rate>15.0</update_rate>
  <ray>
    <scan>
      <horizontal>
        <samples>720</samples>
        <resolution>1</resolution>
        <min_angle>-3.141592653589793</min_angle>
        <max_angle>3.141592653589793</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.12</min>
      <max>8.0</max>
      <resolution>0.01</resolution>
    </range>
    <noise>
      <type>gaussian</type>
      <mean>0</mean>
      <stddev>0.002</stddev>
    </noise>
  </ray>
  <plugin name="bunker_laser" filename="libgazebo_ros_laser.so">
    <robotNamespace>/ground</robotNamespace>
    <topicName>scan</topicName>
    <frameName>lidar_2d_link</frameName>
  </plugin>
</sensor>
```

Assert the added link is exactly `lidar_2d_link` and its fixed joint is:

```xml
<joint name="lidar_2d_joint" type="fixed">
  <origin xyz="-0.30 0.0 0.25" rpy="0 0 0"/>
  <parent link="base_link"/>
  <child link="lidar_2d_link"/>
</joint>
```

Assert the one model plugin subtree is exactly:

```xml
<gazebo>
  <plugin name="bunker_planar_move"
          filename="libgazebo_ros_planar_move.so">
    <robotNamespace>/ground</robotNamespace>
    <commandTopic>cmd_vel_safe</commandTopic>
    <odometryTopic>odom</odometryTopic>
    <odometryFrame>odom</odometryFrame>
    <robotBaseFrame>base_link</robotBaseFrame>
    <odometryRate>50.0</odometryRate>
    <cmdTimeout>0.5</cmdTimeout>
  </plugin>
</gazebo>
```

Explicitly reject `publishTF`, a plugin `tf_prefix` tag, already-prefixed
frames, unsupported update/noise tags inside the laser plugin, arm/gripper/
camera/controller/navigation/brick tokens, or any third plugin. Verify two
consecutive renders are byte-identical, end in one newline, contain no source
absolute path, hash to `EXPECTED_RUNTIME_URDF_SHA256`, and pass `check_urdf`
when written to a temporary file. This digest is the result of the complete
renderer below under the frozen Python 3.8/xacro environment; changing it
requires an explicit renderer-contract review.

- [ ] **Step 3: Run the renderer tests and verify RED**

```bash
scripts/with_noetic_env.bash /usr/bin/env \
  TMPDIR="$PWD/logs/bunker_standalone/engineering-tmp" \
  PYTHONPATH="$PWD/src/platform/bunker_sim_runtime/src" \
  /usr/bin/python3 -B -m unittest discover -v \
  -s src/platform/bunker_sim_runtime/test -p test_renderer.py
```

Expected: FAIL because `bunker_sim_runtime.renderer` and its CLI do not yet
exist.

- [ ] **Step 4: Implement the deterministic transformation pipeline**

Create `src/platform/bunker_sim_runtime/src/bunker_sim_runtime/renderer.py`
with the following complete content:

```python
import hashlib
import os
import re
import stat
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path, PurePosixPath

FROZEN_INPUT_SHA256 = (
    "41e6d862c476264dfb8d150c0f8e1780da2451867908ff6e38300813a2b54f74")
FROZEN_INPUT_RELATIVE = PurePosixPath("urdf/bunker.urdf.xacro")
XACRO_EXECUTABLE = "/opt/ros/noetic/bin/xacro"
MESH_URI_PREFIX = "package://bunker_description/"
LOCAL_NAME_RE = re.compile(r"^[A-Za-z][A-Za-z0-9_]{0,63}$")
INVALID_NAME_RUN_RE = re.compile(r"[^A-Za-z0-9_]+")
EXPECTED_BOX_ORIGIN = "0.018058912 0.001357451 -0.160420741"
EXPECTED_BOX_SIZE = "1.026335219 0.782744936 0.395154782"
INCOMPATIBLE_JOINT_TAGS = (
    "axis", "limit", "dynamics", "calibration", "mimic",
    "safety_controller")
EXPECTED_SOURCE_MESHES = frozenset({
    "meshes/BUNKER.STL",
    "meshes/wheel1.1_Link.STL", "meshes/wheel1.2_Link.STL",
    "meshes/wheel1.3_Link.STL", "meshes/wheel1_Link.STL",
    "meshes/wheel2.1_Link.STL", "meshes/wheel2.2_Link.STL",
    "meshes/wheel2.3_Link.STL", "meshes/wheel2_Link.STL",
    "meshes/wheel3.1_Link.STL", "meshes/wheel3.2_Link.STL",
    "meshes/wheel3.3_Link.STL", "meshes/wheel3_Link.STL",
    "meshes/wheel4.1_Link.STL", "meshes/wheel4.2_Link.STL",
    "meshes/wheel4.3_Link.STL", "meshes/wheel4_Link.STL",
})

COLLISION_XML = """<collision name="base_link_collision">
  <origin xyz="0.018058912 0.001357451 -0.160420741" rpy="0 0 0" />
  <geometry><box size="1.026335219 0.782744936 0.395154782" /></geometry>
</collision>"""
LIDAR_JOINT_XML = """<joint name="lidar_2d_joint" type="fixed">
  <origin xyz="-0.30 0.0 0.25" rpy="0 0 0" />
  <parent link="base_link" />
  <child link="lidar_2d_link" />
</joint>"""
LIDAR_GAZEBO_XML = """<gazebo reference="lidar_2d_link">
  <sensor name="bunker_lidar_2d" type="ray">
    <always_on>true</always_on>
    <update_rate>15.0</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>
          <resolution>1</resolution>
          <min_angle>-3.141592653589793</min_angle>
          <max_angle>3.141592653589793</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.12</min>
        <max>8.0</max>
        <resolution>0.01</resolution>
      </range>
      <noise>
        <type>gaussian</type>
        <mean>0</mean>
        <stddev>0.002</stddev>
      </noise>
    </ray>
    <plugin name="bunker_laser" filename="libgazebo_ros_laser.so">
      <robotNamespace>/ground</robotNamespace>
      <topicName>scan</topicName>
      <frameName>lidar_2d_link</frameName>
    </plugin>
  </sensor>
</gazebo>"""
PLANAR_GAZEBO_XML = """<gazebo>
  <plugin name="bunker_planar_move" filename="libgazebo_ros_planar_move.so">
    <robotNamespace>/ground</robotNamespace>
    <commandTopic>cmd_vel_safe</commandTopic>
    <odometryTopic>odom</odometryTopic>
    <odometryFrame>odom</odometryFrame>
    <robotBaseFrame>base_link</robotBaseFrame>
    <odometryRate>50.0</odometryRate>
    <cmdTimeout>0.5</cmdTimeout>
  </plugin>
</gazebo>"""

class RenderContractError(ValueError):
    pass

def sha256_file(path):
    digest = hashlib.sha256()
    try:
        with Path(path).open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as error:
        raise RenderContractError("cannot hash input: %s" % error)
    return digest.hexdigest()

def _lexical_absolute(path):
    return Path(os.path.abspath(os.fspath(path)))

def _regular_file_within(package_root, path, label):
    package_root = _lexical_absolute(package_root)
    candidate = _lexical_absolute(path)
    try:
        relative = candidate.relative_to(package_root)
    except ValueError:
        raise RenderContractError("%s escapes package root" % label)
    current = package_root
    for part in relative.parts:
        current = current / part
        try:
            mode = current.lstat().st_mode
        except OSError as error:
            raise RenderContractError("missing %s: %s" % (label, error))
        if stat.S_ISLNK(mode):
            raise RenderContractError("symlinked %s: %s" % (label, relative))
    try:
        root_resolved = package_root.resolve(strict=True)
        resolved = candidate.resolve(strict=True)
        resolved.relative_to(root_resolved)
    except (OSError, ValueError) as error:
        raise RenderContractError("%s escapes package root: %s" %
                                  (label, error))
    if not stat.S_ISREG(resolved.stat().st_mode):
        raise RenderContractError("%s is not a regular file" % label)
    return resolved

def _validated_source(source_path):
    source = _lexical_absolute(source_path)
    if (source.name != FROZEN_INPUT_RELATIVE.name or
            source.parent.name != FROZEN_INPUT_RELATIVE.parent.name or
            source.parent.parent.name != "bunker_description"):
        raise RenderContractError(
            "input must be bunker_description/urdf/bunker.urdf.xacro")
    package_root = source.parent.parent
    try:
        root_mode = package_root.lstat().st_mode
        urdf_mode = source.parent.lstat().st_mode
    except OSError as error:
        raise RenderContractError("package tree is unavailable: %s" % error)
    if (stat.S_ISLNK(root_mode) or not stat.S_ISDIR(root_mode) or
            stat.S_ISLNK(urdf_mode) or not stat.S_ISDIR(urdf_mode)):
        raise RenderContractError(
            "package and urdf directories must be real")
    source = _regular_file_within(package_root, source, "frozen xacro")
    resolved_package_root = package_root.resolve(strict=True)
    if resolved_package_root != package_root:
        raise RenderContractError("package root is noncanonical")
    package_root = resolved_package_root
    if sha256_file(source) != FROZEN_INPUT_SHA256:
        raise RenderContractError("frozen xacro SHA-256 mismatch")
    references = _source_mesh_references(source, package_root)
    if references != EXPECTED_SOURCE_MESHES:
        raise RenderContractError("frozen source mesh set changed")
    return source, package_root

def _source_mesh_references(source, package_root):
    try:
        root = ET.parse(str(source)).getroot()
    except (OSError, ET.ParseError) as error:
        raise RenderContractError("cannot parse frozen xacro: %s" % error)
    references = set()
    for mesh in root.findall(".//mesh"):
        uri = mesh.get("filename")
        if type(uri) is not str or not uri.startswith(MESH_URI_PREFIX):
            raise RenderContractError(
                "unsupported source mesh URI: %r" % uri)
        relative_text = uri[len(MESH_URI_PREFIX):]
        relative = PurePosixPath(relative_text)
        if (relative.as_posix() != relative_text or relative.is_absolute() or
                len(relative.parts) != 2 or relative.parts[0] != "meshes" or
                any(part in ("", ".", "..") for part in relative.parts)):
            raise RenderContractError("unsafe source mesh URI: %s" % uri)
        _regular_file_within(
            package_root, package_root.joinpath(*relative.parts),
            "source mesh %s" % relative_text)
        references.add(relative.as_posix())
    return frozenset(references)

def run_xacro(path, executable=XACRO_EXECUTABLE):
    if (type(executable) is not str or
            os.path.realpath(executable) != XACRO_EXECUTABLE or
            not os.path.isfile(XACRO_EXECUTABLE) or
            not os.access(XACRO_EXECUTABLE, os.X_OK)):
        raise RenderContractError("xacro executable is not canonical")
    try:
        result = subprocess.run(
            [XACRO_EXECUTABLE, str(path)], stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, check=False, timeout=10.0)
    except (OSError, subprocess.TimeoutExpired) as error:
        raise RenderContractError("xacro execution failed: %s" % error)
    if result.returncode != 0:
        detail = result.stderr.decode("utf-8", "replace").strip()
        raise RenderContractError(
            "xacro returned %d: %s" % (result.returncode, detail))
    return result.stdout

def canonical_local_name(value):
    if type(value) is not str or not value:
        raise RenderContractError("empty source name")
    mapped = INVALID_NAME_RUN_RE.sub("_", value)
    if LOCAL_NAME_RE.fullmatch(mapped) is None:
        raise RenderContractError("name cannot be canonicalized: %r" % value)
    return mapped

def _declaration_map(elements, label):
    mapping = {}
    outputs = {}
    for element in elements:
        source = element.get("name")
        if type(source) is not str or source in mapping:
            raise RenderContractError("duplicate or missing %s name" % label)
        target = canonical_local_name(source)
        if target in outputs:
            raise RenderContractError(
                "%s name collision: %s and %s" %
                (label, outputs[target], source))
        mapping[source] = target
        outputs[target] = source
    return mapping
```

Continue the same file with the second block below; the two blocks form one
module and must not be split into separate files:

```python
def validate_source_tree(root, source_path):
    _validated_source(source_path)
    if root.tag != "robot" or root.get("name") != "bunker_description":
        raise RenderContractError("unexpected source robot root")
    links = root.findall("link")
    joints = root.findall("joint")
    if len(links) != 17 or len(joints) != 16:
        raise RenderContractError("source must contain 17 links and 16 joints")
    if root.findall(".//plugin"):
        raise RenderContractError("source plugins are forbidden")
    if len(root.findall(".//visual")) != 17:
        raise RenderContractError("source visual count changed")
    if len(root.findall(".//inertial")) != 17:
        raise RenderContractError("source inertial count changed")
    if len(root.findall(".//collision")) != 17:
        raise RenderContractError("source collision count changed")
    _declaration_map(links, "link")
    _declaration_map(joints, "joint")
    return root

def remap_urdf_names(root):
    links = root.findall("link")
    joints = root.findall("joint")
    link_map = _declaration_map(links, "link")
    joint_map = _declaration_map(joints, "joint")
    if set(link_map.values()) & set(joint_map.values()):
        raise RenderContractError("mapped link and joint names collide")
    for link in links:
        link.set("name", link_map[link.get("name")])
    for joint in joints:
        joint.set("name", joint_map[joint.get("name")])
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            raise RenderContractError("joint lacks parent or child")
        for reference in (parent, child):
            source = reference.get("link")
            if source not in link_map:
                raise RenderContractError("joint references unknown link")
            reference.set("link", link_map[source])
        mimic = joint.find("mimic")
        if mimic is not None:
            source = mimic.get("joint")
            if source not in joint_map:
                raise RenderContractError("mimic references unknown joint")
            mimic.set("joint", joint_map[source])
    for gazebo in root.findall("gazebo"):
        if "reference" in gazebo.attrib:
            source = gazebo.get("reference")
            if source not in link_map:
                raise RenderContractError("gazebo references unknown link")
            gazebo.set("reference", link_map[source])
    return root

def fix_wheel_joints(root):
    joints = root.findall("joint")
    if len(joints) != 16 or any(
            not joint.get("name", "").startswith("wheel")
            for joint in joints):
        raise RenderContractError("expected exactly 16 wheel joints")
    for joint in joints:
        joint.set("type", "fixed")
        for tag in INCOMPATIBLE_JOINT_TAGS:
            for child in tuple(joint.findall(tag)):
                joint.remove(child)
    return root

def replace_collisions(root):
    links = root.findall("link")
    removed = 0
    for link in links:
        for collision in tuple(link.findall("collision")):
            link.remove(collision)
            removed += 1
    if removed != 17:
        raise RenderContractError("expected to remove 17 source collisions")
    base_link = next(
        (link for link in links if link.get("name") == "base_link"), None)
    if base_link is None:
        raise RenderContractError("base_link is missing")
    base_link.append(ET.fromstring(COLLISION_XML))
    return root

def add_lidar(root):
    if any(element.get("name") in {"lidar_2d_link", "lidar_2d_joint"}
           for element in root.findall("link") + root.findall("joint")):
        raise RenderContractError("LiDAR entity already exists")
    root.append(ET.Element("link", {"name": "lidar_2d_link"}))
    root.append(ET.fromstring(LIDAR_JOINT_XML))
    root.append(ET.fromstring(LIDAR_GAZEBO_XML))
    return root

def add_planar_plugin(root):
    plugins = root.findall(".//plugin")
    if (len(plugins) != 1 or plugins[0].get("name") != "bunker_laser" or
            plugins[0].get("filename") != "libgazebo_ros_laser.so"):
        raise RenderContractError("expected only the declared laser plugin")
    root.append(ET.fromstring(PLANAR_GAZEBO_XML))
    return root

def _signature(element):
    return (
        element.tag, tuple(sorted(element.attrib.items())),
        (element.text or "").strip(),
        tuple(_signature(child) for child in list(element)),
    )

def _require_signature(actual, expected_xml, label):
    expected = ET.fromstring(expected_xml)
    if actual is None or _signature(actual) != _signature(expected):
        raise RenderContractError("%s subtree differs" % label)

def validate_runtime_tree(root):
    if root.tag != "robot" or root.get("name") != "bunker_description":
        raise RenderContractError("runtime robot root changed")
    links = root.findall("link")
    joints = root.findall("joint")
    collisions = root.findall(".//collision")
    sensors = root.findall(".//sensor")
    plugins = root.findall(".//plugin")
    if (len(links), len(joints), len(collisions), len(sensors), len(plugins)) != (
            18, 17, 1, 1, 2):
        raise RenderContractError("runtime entity counts differ")
    if (len(root.findall(".//visual")) != 17 or
            len(root.findall(".//inertial")) != 17):
        raise RenderContractError(
            "source visuals or inertials were not preserved")
    for label, elements in (
            ("link", links), ("joint", joints),
            ("collision", collisions), ("sensor", sensors),
            ("plugin", plugins)):
        names = [element.get("name") for element in elements]
        if any(type(name) is not str or LOCAL_NAME_RE.fullmatch(name) is None
               for name in names):
            raise RenderContractError("invalid runtime %s name" % label)
        if len(names) != len(set(names)):
            raise RenderContractError("duplicate runtime %s name" % label)
    link_names = {link.get("name") for link in links}
    child_names = set()
    for joint in joints:
        if joint.get("type") != "fixed":
            raise RenderContractError("all runtime joints must be fixed")
        if any(joint.find(tag) is not None for tag in INCOMPATIBLE_JOINT_TAGS):
            raise RenderContractError(
                "fixed joint retains incompatible child")
        parent = joint.find("parent")
        child = joint.find("child")
        if (parent is None or child is None or
                parent.get("link") not in link_names or
                child.get("link") not in link_names):
            raise RenderContractError("runtime joint reference is invalid")
        child_names.add(child.get("link"))
    if link_names - child_names != {"base_link"}:
        raise RenderContractError("runtime must have one base_link root")
    base_link = next(
        link for link in links if link.get("name") == "base_link")
    _require_signature(
        base_link.find("collision"), COLLISION_XML, "base collision")
    lidar_joint = next(
        (joint for joint in joints
         if joint.get("name") == "lidar_2d_joint"), None)
    _require_signature(lidar_joint, LIDAR_JOINT_XML, "LiDAR joint")
    gazebos = root.findall("gazebo")
    if len(gazebos) != 2:
        raise RenderContractError("expected two Gazebo extension blocks")
    lidar_gazebo = next(
        (gazebo for gazebo in gazebos
         if gazebo.get("reference") == "lidar_2d_link"), None)
    planar_gazebo = next(
        (gazebo for gazebo in gazebos if not gazebo.attrib), None)
    _require_signature(lidar_gazebo, LIDAR_GAZEBO_XML, "LiDAR Gazebo")
    _require_signature(planar_gazebo, PLANAR_GAZEBO_XML, "planar Gazebo")
    if {plugin.get("filename") for plugin in plugins} != {
            "libgazebo_ros_planar_move.so", "libgazebo_ros_laser.so"}:
        raise RenderContractError("runtime plugin library set differs")
    if root.findall(".//publishTF") or root.findall(".//tf_prefix"):
        raise RenderContractError("unsupported TF plugin tag")
    serialized = ET.tostring(root, encoding="unicode").lower()
    for token in (
            "aubo", "ag95", "d435", "move_base", "controller",
            "attachment", "handoff", "navigation", "brick", "camera"):
        if token in serialized:
            raise RenderContractError("forbidden runtime token: %s" % token)
    return root

def _indent_tree(element, level=0):
    prefix = "\n" + "  " * level
    child_prefix = "\n" + "  " * (level + 1)
    children = list(element)
    if children:
        if element.text is None or not element.text.strip():
            element.text = child_prefix
        for child in children:
            _indent_tree(child, level + 1)
        if children[-1].tail is None or not children[-1].tail.strip():
            children[-1].tail = prefix
    if level and (element.tail is None or not element.tail.strip()):
        element.tail = prefix

def render_runtime_urdf(source_path):
    source, package_root = _validated_source(source_path)
    del package_root
    try:
        root = ET.fromstring(run_xacro(source))
    except ET.ParseError as error:
        raise RenderContractError(
            "expanded URDF is invalid XML: %s" % error)
    validate_source_tree(root, source)
    remap_urdf_names(root)
    fix_wheel_joints(root)
    replace_collisions(root)
    add_lidar(root)
    add_planar_plugin(root)
    validate_runtime_tree(root)
    _indent_tree(root)
    payload = ET.tostring(
        root, encoding="utf-8", xml_declaration=True,
        short_empty_elements=True).decode("utf-8")
    return payload.rstrip("\n") + "\n"
```

The module above is the entire production implementation. Add one focused
test for `_indent_tree` using a nested three-level element and freeze its exact
serialized whitespace; this proves the renderer never depends on the absent
Python 3.8 `ET.indent` API. All other public/helper behavior is exercised by
the Step 1–2 source/output tests.

Do not add `preserveFixedJoint`, friction/slip guesses, transmissions,
controllers, extra inertials, extra material, or another Gazebo plugin.

- [ ] **Step 5: Add the stdout-only renderer CLI**

Create `scripts/render_bunker_runtime.py` with the complete stdout-only CLI:

```python
#!/usr/bin/env python3
import sys

from bunker_sim_runtime.renderer import (
    RenderContractError, render_runtime_urdf)

def main(argv=None, stdout=None, stderr=None):
    arguments = tuple(sys.argv[1:] if argv is None else argv)
    output = sys.stdout if stdout is None else stdout
    errors = sys.stderr if stderr is None else stderr
    if len(arguments) != 1:
        errors.write("bunker-render: expected one xacro path\n")
        return 64
    try:
        rendered = render_runtime_urdf(arguments[0])
    except (RenderContractError, OSError, ValueError) as error:
        errors.write("bunker-render: %s\n" % error)
        return 65
    output.write(rendered)
    return 0

if __name__ == "__main__":
    sys.exit(main())
```

The script has no ROS-master dependency. Mark it executable before its first
direct invocation:

```bash
chmod 755 src/platform/bunker_sim_runtime/scripts/render_bunker_runtime.py
```

- [ ] **Step 6: Run to GREEN and commit**

Run the focused unittest command, then render the real source twice and
compare it:

```bash
scripts/with_noetic_env.bash /usr/bin/env \
  TMPDIR="$PWD/logs/bunker_standalone/engineering-tmp" \
  PYTHONPATH="$PWD/src/platform/bunker_sim_runtime/src" \
  /usr/bin/python3 -B -m unittest discover -v \
  -s src/platform/bunker_sim_runtime/test -p test_renderer.py
```

```bash
scripts/with_noetic_env.bash /bin/bash --noprofile --norc -c '
  set -euo pipefail
  export PYTHONPATH="$PWD/src/platform/bunker_sim_runtime/src"
  run_dir="$(mktemp -d "$PWD/logs/bunker-render-test.XXXXXX")"
  trap '\''rm -rf -- "$run_dir"'\'' EXIT
  src/platform/bunker_sim_runtime/scripts/render_bunker_runtime.py \
    src/vendor/bunker_description/urdf/bunker.urdf.xacro >"$run_dir/one.urdf"
  src/platform/bunker_sim_runtime/scripts/render_bunker_runtime.py \
    src/vendor/bunker_description/urdf/bunker.urdf.xacro >"$run_dir/two.urdf"
  cmp "$run_dir/one.urdf" "$run_dir/two.urdf"
  check_urdf "$run_dir/one.urdf"
'
```

```bash
git add src/platform/bunker_sim_runtime/src/bunker_sim_runtime/renderer.py \
  src/platform/bunker_sim_runtime/scripts/render_bunker_runtime.py \
  src/platform/bunker_sim_runtime/test/test_renderer.py
git commit -m "feat: render the BUNKER runtime model"
```

---

## Task 5: Define the worldless component, standalone world, and TF ownership

**Files:**

- Create: `src/platform/bunker_sim_runtime/launch/bunker_runtime.launch`
- Create: `src/platform/bunker_sim_runtime/launch/bunker_standalone.launch`
- Create: `src/platform/bunker_sim_runtime/worlds/bunker_standalone.world`
- Create: `src/platform/bunker_sim_runtime/test/test_launch_contract.py`
- Create: `src/platform/bunker_sim_runtime/test/tf_prefix_fixture.urdf`
- Create: `src/platform/bunker_sim_runtime/test/tf_prefix_contract.test`
- Create: `src/platform/bunker_sim_runtime/test/test_tf_prefix_contract.py`
- Modify: `tools/runtime_boundary.py`
- Modify: `tests/test_runtime_boundary.py`

- [ ] **Step 1: Write failing structured launch and world tests**

Parse both launch files with `xml.etree.ElementTree`, not substring-only
assertions. Freeze the only public arguments:

```python
RUNTIME_ARGS = {
    "x": "0.0", "y": "0.0", "z": "0.36",
    "roll": "0.0", "pitch": "0.0", "yaw": "0.0",
}
STANDALONE_ARGS = {"gui": "false"}
```

Require `bunker_runtime.launch` to have one `/ground` group, literal model and
namespace identity, one `tf_prefix=ground` parameter, one rendered
`robot_description`, and exactly four nodes:

```text
/ground/velocity_guard             bunker_sim_runtime/velocity_guard.py
/ground/spawn_bunker               gazebo_ros/spawn_model
/ground/robot_state_publisher      robot_state_publisher/robot_state_publisher
/ground/world_to_odom              tf2_ros/static_transform_publisher
```

The guard is `required="true"`. The spawn is also required and has both the
preflight launch-prefix and bonded `-b` argument so it remains alive after a
successful spawn. The robot-state publisher has no tuning parameters. Freeze
the static publisher's quaternion-form arguments at
`0 0 0 0 0 0 1 world ground/odom`.

Require `bunker_standalone.launch` to include Gazebo's `empty_world.launch`
once and the worldless runtime once. `use_sim_time` is a literal `true`,
`server_required` is literal `true`, `gui` is the only forwarded public
argument, and no identity or pose argument is exposed by the standalone
surface.

Parse the world and require exactly these non-world children:

```xml
<include><uri>model://ground_plane</uri></include>
<include><uri>model://sun</uri></include>
<model name="scan_obstacle">
  <static>true</static>
  <pose>2.0 0.0 0.5 0 0 0</pose>
  <link name="link">
    <collision name="collision">
      <geometry><box><size>0.5 1.0 1.0</size></box></geometry>
    </collision>
    <visual name="visual">
      <geometry><box><size>0.5 1.0 1.0</size></box></geometry>
    </visual>
  </link>
</model>
```

Reject RViz, navigation, controllers, AUBO, AG95, D435, attachment,
orchestration, and any second spawn/include of the runtime.

- [ ] **Step 2: Freeze the shared-world spawn classification in RED**

Add this tuple to production `JOINT_CANDIDATE_SPAWNS` and test
`EXPECTED_JOINT_CANDIDATE_SPAWNS`:

```python
(
    "src/platform/bunker_sim_runtime/launch/bunker_runtime.launch",
    "spawn_bunker",
),
```

Update exact repository assertions in `tests/test_runtime_boundary.py`:

```python
self.assertEqual(21, len(discover_packages(ROOT)))
self.assertEqual((4, 3, 7), (
    len(EXPECTED_JOINT_CANDIDATE_SPAWNS),
    len(EXPECTED_STANDALONE_SMOKE_SPAWNS),
    len(EXPECTED_INACTIVE_LEGACY_SPAWNS),
))
self.assertEqual(14, len(discover_startup_spawns(ROOT)))
```

Do not reclassify any legacy Ground spawn and do not modify
`sim_platform_bringup`.

- [ ] **Step 3: Write the Noetic `tf_prefix` rostest as a staged runtime test**

Create a two-link fixture with `base_link`, `lidar_2d_link`, and the fixed
mount from Task 4. The `.test` file sets `/ground/tf_prefix` to `ground`, loads
the fixture as `/ground/robot_description`, starts exactly one namespaced
`robot_state_publisher`, then runs `test_tf_prefix_contract.py`.

The test subscribes to `/tf_static`, waits at most 10 wall seconds, and
requires one edge `ground/base_link -> ground/lidar_2d_link` with translation
`[-0.30, 0.0, 0.25]`, identity quaternion, and caller ID
`/ground/robot_state_publisher`. Reject unprefixed frames,
`ground/ground/...`, duplicate authorities, or any unexpected dynamic copy on
`/tf`.

Do not invoke `rostest` in Task 5: the package has not been built yet. This is
a staged runtime test artifact whose first real execution is Task 6 after the
Catkin build. Run the two offline launch/boundary suites below and verify their
intended RED before Steps 4-6:

```bash
env -i PATH=/usr/bin:/bin LANG=C.UTF-8 LC_ALL=C.UTF-8 \
  TMPDIR="$PWD/logs/bunker_standalone/engineering-tmp" \
  /usr/bin/python3 -B -m unittest -v \
  tests.test_runtime_boundary
scripts/with_noetic_env.bash /usr/bin/env \
  TMPDIR="$PWD/logs/bunker_standalone/engineering-tmp" \
  PYTHONPATH="$PWD/src/platform/bunker_sim_runtime/src" \
  /usr/bin/python3 -B -m unittest discover -v \
  -s src/platform/bunker_sim_runtime/test -p test_launch_contract.py
```

- [ ] **Step 4: Implement the minimal standalone world**

Create `worlds/bunker_standalone.world` with this complete SDF 1.6 document:

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="bunker_standalone">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>
    <model name="scan_obstacle">
      <static>true</static>
      <pose>2.0 0.0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>0.5 1.0 1.0</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>0.5 1.0 1.0</size></box>
          </geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

Add no physics tuning, road texture, map, camera, GUI plugin, benchmark
object, or manipulation target.

- [ ] **Step 5: Implement the worldless launch**

Use this exact structure, preserving pose-token order between preflight and
spawn arguments:

```xml
<launch>
  <arg name="x" default="0.0"/>
  <arg name="y" default="0.0"/>
  <arg name="z" default="0.36"/>
  <arg name="roll" default="0.0"/>
  <arg name="pitch" default="0.0"/>
  <arg name="yaw" default="0.0"/>

  <group ns="ground">
    <param name="tf_prefix" type="str" value="ground"/>
    <param name="robot_description"
      command="$(find bunker_sim_runtime)/scripts/render_bunker_runtime.py $(find bunker_description)/urdf/bunker.urdf.xacro"/>

    <node pkg="bunker_sim_runtime" type="velocity_guard.py"
      name="velocity_guard" required="true" output="screen"/>
    <node pkg="gazebo_ros" type="spawn_model" name="spawn_bunker"
      required="true" output="screen"
      launch-prefix="$(find bunker_sim_runtime)/scripts/spawn_bunker_preflight.py $(arg x) $(arg y) $(arg z) $(arg roll) $(arg pitch) $(arg yaw) --"
      args="-urdf -param robot_description -model bunker -b -x $(arg x) -y $(arg y) -z $(arg z) -R $(arg roll) -P $(arg pitch) -Y $(arg yaw)"/>
    <node pkg="robot_state_publisher" type="robot_state_publisher"
      name="robot_state_publisher" output="screen"/>
    <node pkg="tf2_ros" type="static_transform_publisher"
      name="world_to_odom"
      args="0 0 0 0 0 0 1 world ground/odom"/>
  </group>
</launch>
```

Do not set `/use_sim_time` in this component launch; the preflight must see an
already-existing boolean `true`. Do not make namespace/model/prefix public
arguments.

- [ ] **Step 6: Implement the standalone launch**

Create `launch/bunker_standalone.launch` with the complete wrapper below. It
exposes no argument except `gui`:

```xml
<launch>
  <arg name="gui" default="false"/>

  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name"
      value="$(find bunker_sim_runtime)/worlds/bunker_standalone.world"/>
    <arg name="gui" value="$(arg gui)"/>
    <arg name="paused" value="false"/>
    <arg name="debug" value="false"/>
    <arg name="verbose" value="false"/>
    <arg name="use_sim_time" value="true"/>
    <arg name="server_required" value="true"/>
  </include>

  <include file="$(find bunker_sim_runtime)/launch/bunker_runtime.launch"/>
</launch>
```

- [ ] **Step 7: Run the offline launch checks and commit**

Mark the rostest node executable, then run both Python suites below. The
rostest itself runs for the first time in Task 6 after Catkin has built the
package. This Task ends with the offline launch sources GREEN and the staged
rostest still pending, not with a claimed runtime-test pass.

```bash
chmod 755 \
  src/platform/bunker_sim_runtime/test/test_tf_prefix_contract.py
env -i PATH=/usr/bin:/bin LANG=C.UTF-8 LC_ALL=C.UTF-8 \
  TMPDIR="$PWD/logs/bunker_standalone/engineering-tmp" \
  /usr/bin/python3 -B -m unittest -v tests.test_runtime_boundary
scripts/with_noetic_env.bash /usr/bin/env \
  TMPDIR="$PWD/logs/bunker_standalone/engineering-tmp" \
  PYTHONPATH="$PWD/src/platform/bunker_sim_runtime/src" \
  /usr/bin/python3 -B -m unittest discover -v \
  -s src/platform/bunker_sim_runtime/test -p test_launch_contract.py
```

```bash
git add src/platform/bunker_sim_runtime/launch \
  src/platform/bunker_sim_runtime/worlds \
  src/platform/bunker_sim_runtime/test/test_launch_contract.py \
  src/platform/bunker_sim_runtime/test/tf_prefix_fixture.urdf \
  src/platform/bunker_sim_runtime/test/tf_prefix_contract.test \
  src/platform/bunker_sim_runtime/test/test_tf_prefix_contract.py \
  tools/runtime_boundary.py tests/test_runtime_boundary.py
git commit -m "feat: define the BUNKER standalone launch"
```

---

## Task 6: Build and prove the install-only runtime closure

**Files:**

- Modify: `src/platform/bunker_sim_runtime/CMakeLists.txt`
- Create: `scripts/with_bunker_env.bash`
- Create: `scripts/validate_bunker_install.bash`
- Create: `tests/test_bunker_shell_contract.py`
- Modify: `tests/test_bunker_build_contract.py`

- [ ] **Step 1: Write failing build/install-surface tests**

Extend `test_bunker_build_contract.py` to require:

```cmake
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/test_contracts.py)
  catkin_add_nosetests(test/test_sim_time.py)
  catkin_add_nosetests(test/test_velocity_guard.py)
  catkin_add_nosetests(test/test_spawn_bunker_preflight.py)
  catkin_add_nosetests(test/test_renderer.py)
  catkin_add_nosetests(test/test_launch_contract.py)
  add_rostest(test/tf_prefix_contract.test)
endif()

catkin_install_python(
  PROGRAMS scripts/velocity_guard.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  PROGRAMS
    scripts/render_bunker_runtime.py
    scripts/spawn_bunker_preflight.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)

install(
  DIRECTORY launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
```

Assert that the renderer and preflight are absent from
`catkin_install_python`, the guard is absent from share scripts, and no vendor
launch/config/RViz directory is installed.

Also add `BunkerBuildProfileContractTest` using `yaml.safe_load` on
`.catkin_tools/profiles/p450-clean/config.yaml`. Before any build it must
require these exact profile invariants:

```python
EXPECTED_PROFILE = {
    "source_space": "src",
    "build_space": "build/p450-clean",
    "devel_space": "devel/p450-clean",
    "install": True,
    "install_space": "install/p450-clean",
    "isolate_install": False,
    "extend_path": "/opt/ros/noetic",
    "use_env_cache": False,
    "cmake_args": [
        "-DCMAKE_BUILD_TYPE=RelWithDebInfo",
        "-DPYTHON_EXECUTABLE=/usr/bin/python3",
    ],
}
```

Compare each named value with strict Python type checks, so `true` cannot be a
string and a second/reordered CMake argument cannot pass. Other profile keys
remain outside the BUNKER-A contract.

- [ ] **Step 2: Write failing clean-wrapper behavior tests**

Create `tests/test_bunker_shell_contract.py` using `subprocess.run` with
argument vectors and temporary mode-0700 directories. Before any build or ROS
launch, require `with_bunker_env.bash` test modes to prove:

```text
--test-run-dir PATH
  accepts one canonical existing mode-0700 directory below
  /media/lu/P450_PAPER/SIM/p450_sim_v1/logs/bunker_standalone; rejects
  symlink, noncanonical, mode 0755,
  missing, outside-SIM, and repository-root inputs

--test-path-list LABEL VALUE RUN_DIR
  accepts unique canonical absolute readable directories rooted in /opt,
  /usr, install/p450-clean, or RUN_DIR; rejects empty components, relative
  entries, duplicates after realpath, symlinks, unreadable/missing entries,
  and any source/devel/build path. Exercise ROS_PACKAGE_PATH,
  CMAKE_PREFIX_PATH, PYTHONPATH, LD_LIBRARY_PATH, GAZEBO_PLUGIN_PATH,
  GAZEBO_MODEL_PATH, GAZEBO_RESOURCE_PATH, PKG_CONFIG_PATH, and
  OGRE_RESOURCE_PATH.

--test-fixed-environment PATH LANG LC_ALL ROS_ETC_DIR ROS_ROOT ROSLISP_PACKAGE_DIRECTORIES
  accepts exactly /opt/ros/noetic/bin:/usr/bin:/bin, C.UTF-8, C.UTF-8,
  /opt/ros/noetic/etc/ros, /opt/ros/noetic/share/ros, and the empty string

--test-plugin-candidates GAZEBO_PLUGIN_PATH LD_LIBRARY_PATH
  forms one order-preserving, canonical-directory union of both lists,
  deduplicates overlapping directories before lookup, resolves exactly the
  three frozen library files once each, and rejects a same-basename shadow or
  missing/noncanonical library
```

Also test wrong argument counts and commands after `--`. Every test-only mode
must branch before setup files are sourced or state directories are created.
The normal interface is exactly:

```text
scripts/with_bunker_env.bash --run-dir RUN_DIR -- COMMAND ARGUMENTS
```

`RUN_DIR` means one fresh, existing, canonical absolute mode-0700 descendant
of the fixed BUNKER log root. Its eight state-directory names must not already
exist. `COMMAND` is a nonempty executable token and `ARGUMENTS` are passed
byte-for-byte as the remaining argv entries.

- [ ] **Step 3: Implement the clean environment wrapper**

The outer script resolves its own canonical repository root, exports it as
`BUNKER_REPO_ROOT`, resolves the real passwd home/name with `getent passwd`,
and keeps `HOME` unchanged. After validating the fresh run directory, export
its canonical path as `BUNKER_RUN_DIR` and create these mode-0700 directories
*before either setup file is sourced*: `tmp`, `ros-home`, `ros-log`,
`gazebo-log`, `ign-fuel-cache`, `xdg-cache`, `xdg-config`, and `xdg-data`.
Bind `TMPDIR`, `ROS_HOME`, `ROS_LOG_DIR`, `GAZEBO_LOG_PATH`,
`IGN_FUEL_CACHE_PATH`, `XDG_CACHE_HOME`, `XDG_CONFIG_HOME`, and
`XDG_DATA_HOME` to those paths. Test-only modes branch before this creation.

Start the child through `/usr/bin/env -i` with only the resolved identity,
locale, shell, fixed PATH, the two BUNKER paths, the eight state paths, and the
requested command encoded as positional arguments. Inside
`bash --noprofile --norc`, use this exact setup order:

```bash
set -eo pipefail
umask 077
source /opt/ros/noetic/setup.bash
source "$BUNKER_REPO_ROOT/install/p450-clean/setup.bash"

GAZEBO_RESOURCE_PATH="${GAZEBO_RESOURCE_PATH:-}"
GAZEBO_PLUGIN_PATH="/opt/ros/noetic/lib${GAZEBO_PLUGIN_PATH:+:$GAZEBO_PLUGIN_PATH}"
GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:-}"
LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}"
set -u
source /usr/share/gazebo/setup.sh
GAZEBO_MODEL_DATABASE_URI=""
ROSLISP_PACKAGE_DIRECTORIES=""
export GAZEBO_MODEL_DATABASE_URI ROSLISP_PACKAGE_DIRECTORIES
```

Do not enable nounset before the two ROS/Catkin setup files: the local Noetic
profile reads unset `ROS_DISTRO` and fails under `set -u`. The state paths
already exist in the sanitized environment when either setup file runs.
After the setup block:

1. verify both BUNKER variables and all eight state variables survived with
   their exact canonical values and remain mode-0700 descendants of the run
   directory;
2. canonicalize and deduplicate every component of `ROS_PACKAGE_PATH`,
   `CMAKE_PREFIX_PATH`, `PYTHONPATH`, `LD_LIBRARY_PATH`,
   `GAZEBO_PLUGIN_PATH`, `GAZEBO_MODEL_PATH`, and
   `GAZEBO_RESOURCE_PATH`, `PKG_CONFIG_PATH`, and `OGRE_RESOURCE_PATH`;
3. validate the final path lists and exact unshadowed plugin candidates before
   `exec` of the requested command.

After canonicalization, require these exact values, with
`INSTALL=$BUNKER_REPO_ROOT/install/p450-clean` substituted literally and no
empty path component:

```text
ROS_PACKAGE_PATH=INSTALL/share:/opt/ros/noetic/share
CMAKE_PREFIX_PATH=INSTALL:/opt/ros/noetic
PYTHONPATH=INSTALL/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages
LD_LIBRARY_PATH=INSTALL/lib:/opt/ros/noetic/lib:/opt/ros/noetic/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins
GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:/opt/ros/noetic/lib
GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models
GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11
PKG_CONFIG_PATH=INSTALL/lib/pkgconfig:/opt/ros/noetic/lib/pkgconfig:/opt/ros/noetic/lib/x86_64-linux-gnu/pkgconfig
OGRE_RESOURCE_PATH=/usr/lib/x86_64-linux-gnu/OGRE-1.9.0
ROS_ETC_DIR=/opt/ros/noetic/etc/ros
ROS_ROOT=/opt/ros/noetic/share/ros
ROSLISP_PACKAGE_DIRECTORIES=
PATH=/opt/ros/noetic/bin:/usr/bin:/bin
LANG=C.UTF-8
LC_ALL=C.UTF-8
```

Freeze these canonical plugin files:

```text
/opt/ros/noetic/lib/libgazebo_ros_planar_move.so
/opt/ros/noetic/lib/libgazebo_ros_laser.so
/usr/lib/x86_64-linux-gnu/gazebo-11/plugins/libRayPlugin.so
```

The accepted nonempty path roots are `/opt`, `/usr`, the canonical
`install/p450-clean` prefix, and the canonical run directory. No empty path
component, source/devel/build path, relative directory, symlink alias, or
duplicate canonical directory survives. `ROSLISP_PACKAGE_DIRECTORIES` is a
frozen empty scalar, not a path list. The script must not read
`P450_PX4_ROOT`, ambient ROS/Gazebo variables, or either external repository.

- [ ] **Step 4: Implement the no-Gazebo install validator**

`validate_bunker_install.bash` runs only inside the clean wrapper and exits
before starting ROS master/Gazebo. It must:

1. require both `rospack find bunker_description` and
   `rospack find bunker_sim_runtime` to equal their canonical
   `install/p450-clean/share/bunker_description` and
   `install/p450-clean/share/bunker_sim_runtime` paths;
2. call `"$BUNKER_REPO_ROOT/tools/bunker_assets.py"` against the source and
   installed vendor package and require the exact 25-file closure;
3. require the installed renderer at
   `share/bunker_sim_runtime/scripts/render_bunker_runtime.py`, the preflight
   beside it, and the guard at
   `lib/bunker_sim_runtime/velocity_guard.py`, all regular executable files;
4. run only the installed renderer against only the installed frozen xacro,
   write its stdout below the run directory, parse it, run `check_urdf`, and
   resolve all 17 package mesh URIs to regular installed files with matching
   hashes;
5. reject source/devel/build paths in the current search environment and in
   the renderer executable/input/output provenance;
6. resolve the three plugin files to the exact canonical paths, run `ldd` on
   each, and reject every `not found`; and
7. emit one JSON object containing the sanitized environment, package paths,
   renderer/input/output SHA-256 values, mesh paths/hashes, plugin paths, and
   `ldd` status. The rendered-output digest must equal
   `2b58856bed616a9e7402f7ddf4be4336f669271ca3989fdcf1d50f6939270832`.

Add `--test-environment` and `--test-plugin-ldd` branches before any rendered
output is created so the shell test can exercise missing, shadowed, and
source-path fixtures without launching Gazebo.

- [ ] **Step 5: Finish CMake and build in two explicit waves**

Append this exact install/test surface to the initial Task 2 CMake file:

```cmake
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/test_contracts.py)
  catkin_add_nosetests(test/test_sim_time.py)
  catkin_add_nosetests(test/test_velocity_guard.py)
  catkin_add_nosetests(test/test_spawn_bunker_preflight.py)
  catkin_add_nosetests(test/test_renderer.py)
  catkin_add_nosetests(test/test_launch_contract.py)
  add_rostest(test/tf_prefix_contract.test)
endif()

catkin_install_python(
  PROGRAMS scripts/velocity_guard.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  PROGRAMS
    scripts/render_bunker_runtime.py
    scripts/spawn_bunker_preflight.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)

install(
  DIRECTORY launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
```

Make the intended scripts executable, then build the vendor install repair
before the platform package:

```bash
chmod 755 scripts/with_bunker_env.bash \
  scripts/validate_bunker_install.bash \
  src/platform/bunker_sim_runtime/scripts/render_bunker_runtime.py \
  src/platform/bunker_sim_runtime/scripts/spawn_bunker_preflight.py \
  src/platform/bunker_sim_runtime/scripts/velocity_guard.py \
  src/platform/bunker_sim_runtime/test/test_tf_prefix_contract.py
env -i PATH=/usr/bin:/bin LANG=C.UTF-8 LC_ALL=C.UTF-8 \
  TMPDIR="$PWD/logs/bunker_standalone/engineering-tmp" \
  /usr/bin/python3 -B -m unittest -v \
  tests.test_bunker_build_contract.BunkerBuildProfileContractTest
scripts/with_noetic_env.bash /usr/bin/env \
  TMPDIR="$PWD/logs/bunker_standalone/engineering-tmp" \
  catkin build --workspace "$PWD" --profile p450-clean \
  bunker_description --force-cmake --no-status --summarize
scripts/with_noetic_env.bash /usr/bin/env \
  TMPDIR="$PWD/logs/bunker_standalone/engineering-tmp" \
  catkin build --workspace "$PWD" --profile p450-clean \
  bunker_sim_runtime --force-cmake --no-status --summarize
```

Do not rebuild an external checkout and do not change the profile/install
space name. The profile precondition must pass immediately before the first
Catkin build; do not let a later install-probe failure stand in for it.

- [ ] **Step 6: Run Catkin tests, the TF-prefix rostest, and shell tests**

```bash
scripts/with_noetic_env.bash /usr/bin/env \
  TMPDIR="$PWD/logs/bunker_standalone/engineering-tmp" \
  catkin test --workspace "$PWD" --profile p450-clean bunker_sim_runtime \
  --no-status --summarize
scripts/with_noetic_env.bash /usr/bin/env \
  TMPDIR="$PWD/logs/bunker_standalone/engineering-tmp" \
  catkin_test_results --all \
  build/p450-clean/bunker_sim_runtime/test_results
env -i PATH=/usr/bin:/bin LANG=C.UTF-8 LC_ALL=C.UTF-8 \
  TMPDIR="$PWD/logs/bunker_standalone/engineering-tmp" \
  /usr/bin/python3 -B -m unittest -v \
  tests.test_bunker_build_contract \
  tests.test_bunker_shell_contract \
  tests.test_runtime_boundary
```

Expected: zero test failures; the real `tf_prefix` rostest observes only the
prefixed fixed edge.

- [ ] **Step 7: Run the clean install-only probe**

```bash
set -euo pipefail
run_dir="$PWD/logs/bunker_standalone/install-probe-$(date -u +%Y%m%dT%H%M%SZ)-$$"
test ! -e "$run_dir"
install -d -m 700 -- "$run_dir"
umask 077
install_result_tmp="$run_dir/.install-contract.json.tmp"
install_result="$run_dir/install-contract.json"
scripts/with_bunker_env.bash --run-dir "$run_dir" -- \
  "$PWD/scripts/validate_bunker_install.bash" >"$install_result_tmp"
/usr/bin/python3 -I -B - "$install_result_tmp" <<'PY'
import json
import os
import sys
from pathlib import Path

path = Path(sys.argv[1])
with path.open("r+", encoding="utf-8") as stream:
    payload = json.load(stream)
    if payload.get("schema_version") != 1 or payload.get("status") != "PASS":
        raise SystemExit("install contract is not schema-1 PASS")
    stream.flush()
    os.fsync(stream.fileno())
PY
chmod 600 "$install_result_tmp"
mv -T -- "$install_result_tmp" "$install_result"
test -s "$install_result"
```

Expected: exit 0, valid JSON, both packages and every mesh below
`install/p450-clean`, the renderer path below installed share, the guard below
installed libexec, and the three exact system plugin paths. Confirm the
external `P450-PAPER` worktree/status and all external PX4 source paths are
unchanged.

- [ ] **Step 8: Commit the install-only closure**

```bash
git add src/platform/bunker_sim_runtime/CMakeLists.txt \
  scripts/with_bunker_env.bash scripts/validate_bunker_install.bash \
  tests/test_bunker_build_contract.py tests/test_bunker_shell_contract.py
git commit -m "build: prove the BUNKER install-only closure"
```

---

## Task 7: Define the ROS-independent live-evidence contract

**Files:**

- Create: `tools/bunker_live_contract.py`
- Create: `tests/test_bunker_live_contract.py`

Execute this task as the following small RED→GREEN slices; each named test is
run immediately after it is written (RED), again after only the named helper
is added (GREEN), and the full module is committed only after slice 7.17:

- [ ] **7.1:** `LivePrimitiveTest.test_exact_keys_and_finite_numbers` →
  `_require_exact_keys` and `_finite_float`.
- [ ] **7.2:** `LiveRateTest.test_median_period_boundaries` →
  `median_period`.
- [ ] **7.3:** `LiveClockTest.test_clock_is_finite_nonzero_and_increasing` →
  `validate_clock_samples`.
- [ ] **7.4:** `LiveOdomTest.test_exact_frames_count_and_rate` → odometry
  count/frame/stamp/rate checks.
- [ ] **7.5:** `LiveOdomTest.test_pose_and_twist_reject_nonfinite` → odometry
  pose/quaternion/twist checks.
- [ ] **7.6:** `LiveScanTest.test_exact_metadata_and_lengths` → scan metadata
  and vector lengths.
- [ ] **7.7:** `LiveScanTest.test_range_and_intensity_domain` → range,
  intensity, and finite-return checks.
- [ ] **7.8:** `LiveGuardTest.test_three_ordered_cases` → guard output and
  simulated-latency checks.
- [ ] **7.9:** `LiveGuardTest.test_nonfinite_input_has_json_token_encoding` →
  `nan` string-token persistence with strict JSON.
- [ ] **7.10:** `LiveMotionMathTest.test_unwrap_and_local_projection` → yaw
  unwrap and baseline-frame projection.
- [ ] **7.11:** `LiveForwardTest.test_forward_limits_and_xy_agreement` →
  forward-phase validation.
- [ ] **7.12:** `LiveWatchdogTest.test_sim_time_stop_and_stationary_window` →
  watchdog and stationary-window validation.
- [ ] **7.13:** `LiveRotationTest.test_yaw_drift_and_odom_agreement` →
  rotation-phase validation.
- [ ] **7.14:** `LiveGraphTest.test_exact_topic_types_and_owners` → exact ROS
  graph validation.
- [ ] **7.15:** `LiveTfTest.test_complete_nineteen_edge_tree` → full TF
  authority/currentness validation.
- [ ] **7.16:**
  `LiveBoundaryTest.test_plugins_collision_environment_and_shutdown` → the
  plugin, collision, environment, provenance, log, and shutdown validators.
- [ ] **7.17:**
  `LiveDocumentTest.test_probe_merge_result_and_atomic_write` → schema-1
  probe validation, schema-2 merge/final validation, CLI, and atomic writer.

After each slice, run the entire still-small pure module with this exact
command; the newly added test must change from its expected RED to GREEN and
all earlier slices must remain GREEN:

```bash
env -i PATH=/usr/bin:/bin LANG=C.UTF-8 LC_ALL=C.UTF-8 \
  TMPDIR="$PWD/logs/bunker_standalone/engineering-tmp" \
  /usr/bin/python3 -B -m unittest -v tests.test_bunker_live_contract
```

The detailed fixtures and production contracts for those slices follow.

### Contract detail 7.A: odometry, scan, and rate validators

Use plain dictionaries/numbers so these tests need no ROS installation.
`validate_odometry_samples(samples)` requires at least 20 messages. Every
sample has exact frames `ground/odom` and `ground/base_link`, a finite nonzero
strictly increasing simulation timestamp, and finite position, quaternion,
linear-twist, and angular-twist components. Require a median adjacent period
in `[0.016, 0.030]` seconds and reject a zero quaternion norm. Its returned
JSON summary contains only count, first/last stamp, median period, exact
frames, and finite extrema; it does not retain the sample array.

```python
VECTOR3_KEYS = frozenset({"x", "y", "z"})
QUATERNION_KEYS = frozenset({"x", "y", "z", "w"})
ODOMETRY_SAMPLE_KEYS = frozenset({
    "stamp", "frame_id", "child_frame_id", "position", "orientation",
    "linear_twist", "angular_twist",
})
SCAN_SAMPLE_KEYS = frozenset({
    "stamp", "frame_id", "ranges", "intensities", "angle_min",
    "angle_max", "angle_increment", "range_min", "range_max",
    "time_increment", "scan_time",
})
```

`position`, `linear_twist`, and `angular_twist` have exactly `VECTOR3_KEYS`;
`orientation` has exactly `QUATERNION_KEYS`. Samples reject every extra key.

`validate_scan_samples(samples)` must require at least eight messages and, for
every sample:

```python
SCAN_CONTRACT = {
    "frame_id": "ground/lidar_2d_link",
    "range_count": 720,
    "intensity_count": 720,
    "angle_min": -math.pi,
    "angle_max": math.pi,
    "angle_increment": 2.0 * math.pi / 719.0,
    "range_min": 0.12,
    "range_max": 8.0,
    "time_increment": 0.0,
    "scan_time": 0.0,
}
```

All timestamps are finite, nonzero, and strictly increasing. Every intensity
is finite. Each range is finite within `[0.12-1e-6, 8.0+1e-6]` or positive
infinity; reject NaN, negative infinity, and out-of-range finite values.
Require at least one finite in-range return across the set. Use `1e-6` for
metadata comparisons and `1e-9` for the two zero timing fields.

The scan validator returns a finite-only summary: replace raw positive
infinities with `positive_infinity_count` and retain only finite min/max plus
the exact count/metadata/rate fields. No raw infinity is serialized.

`median_period(stamps, minimum_count)` returns the median adjacent period only
after validating finite, strictly increasing stamps. Tests freeze accepted
period windows to `0.016-0.030` seconds for at least 20 odometry samples and
`0.050-0.090` seconds for at least eight scans.

### Contract detail 7.B: guard and planar-motion validators

Test `validate_guard_admission(records)` for exactly three ordered probes:

```text
over_limit input  (0.75, 0, 0, 0, 0, 2.0)
           output (0.5,  0, 0, 0, 0, 1.0)
nonplanar input   (0, 0.1, 0, 0, 0, 0)
           output all zero within 0.10 simulated seconds
nonfinite input   (NaN, 0, 0, 0, 0, 0)
           output all zero within 0.10 simulated seconds
```

The raw NaN exists only in the transient ROS message. The unit fixture is
exactly:

```python
{
    "case": "nonfinite",
    "input_tokens": ["nan", "0", "0", "0", "0", "0"],
    "output": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "input_stamp": 12.0,
    "output_stamp": 12.04,
    "latency": 0.04,
}
```

Production records measured finite stamps/latency in `[0.0, 0.10]`, requires
`latency == output_stamp - input_stamp` within `1e-9`, compares every output
component within `1e-9`, and never serializes raw NaN.

`unwrap_yaw(previous, current)` chooses the continuous delta in `[-pi, pi]`.
`project_local_delta(baseline_pose, final_pose)` rotates world x/y displacement
by negative baseline yaw and returns local forward/lateral/unwrapped yaw.

Test `validate_motion_phase(kind, evidence)` with boundary fixtures for:

```python
FORWARD_LIMITS = {
    "forward": (0.50, 0.70), "abs_lateral_max": 0.10,
    "model_odom_xy_max": 0.03, "model_odom_yaw_max": 0.04,
}
ROTATION_LIMITS = {
    "yaw": (0.85, 1.15), "planar_drift_max": 0.08,
    "model_odom_xy_max": 0.03, "model_odom_yaw_max": 0.04,
}
STOP_LIMITS = {
    "minimum_clock_coast": 0.5,
    "model_linear_speed_max": 0.01,
    "model_angular_speed_max": 0.01,
    "odom_linear_speed_max": 0.01,
    "odom_angular_speed_max": 0.01,
    "stationary_window": 1.0,
    "stationary_position_drift_max": 0.01,
    "stationary_yaw_drift_max": 0.01,
}
MOTION_PHASE_KEYS = frozenset({
    "kind", "command", "baseline", "final", "watchdog", "stationary",
    "metrics",
})
COMMAND_KEYS = frozenset({
    "linear_x", "angular_z", "rate_hz", "target_duration",
    "start_stamp", "last_publish_stamp", "end_stamp", "published_count",
    "actual_duration", "minimum_period", "median_period", "maximum_period",
})
WATCHDOG_KEYS = frozenset({
    "last_command_stamp", "stop_observed_stamp", "clock_coast",
    "time_to_stop", "model_linear_speed", "model_angular_speed",
    "odom_linear_speed", "odom_angular_speed",
})
STATIONARY_KEYS = frozenset({
    "start_stamp", "end_stamp", "duration", "position_drift", "yaw_drift",
})
```

All compared inputs must be finite. Model/odometry comparison uses x/y/yaw
only; model z/roll/pitch is validated separately against the settled baseline
and absolute attitude limits. Planar distance and model/odometry xy agreement
use `math.hypot(dx, dy)`; linear and angular speed each use the Euclidean norm
of all three axes. Forward requires at least 40 publishes and rotation at
least 30. `rate_hz` must equal `20.0`; the production adapter retains the
transient raw publish stamps long enough to derive the four timing summaries
but persists only the summaries. Require
`actual_duration == last_publish_stamp - start_stamp` within `1e-9`,
`end_stamp - start_stamp >= target_duration`,
`target_duration - 0.075 <= actual_duration <= target_duration + 0.025`, and
`0.0 <= end_stamp - last_publish_stamp <= 0.075`. Require
`minimum_period >= 0.025`, `0.045 <= median_period <= 0.055`, and
`maximum_period <= 0.075`; therefore a self-reported rate or a burst of
commands cannot satisfy the gate. The stationary window starts only after the
observed watchdog stop.

### Contract detail 7.C: graph, TF, plugin, and schema tests

`validate_graph(evidence)` requires the exact topic types and owners from the
spec after transient safe-topic subscribers have disconnected. It requires
one live `gzserver`, one `/gazebo`, one `/ground/velocity_guard`, one model
`bunker`, and these exact graph sets:

```python
EXPECTED_GRAPH = {
    "/ground/cmd_vel": {
        "type": "geometry_msgs/Twist",
        "publishers": frozenset(),
        "subscribers": frozenset({"/ground/velocity_guard"}),
    },
    "/ground/cmd_vel_safe": {
        "type": "geometry_msgs/Twist",
        "publishers": frozenset({"/ground/velocity_guard"}),
        "subscribers": frozenset({"/gazebo"}),
    },
    "/ground/odom": {
        "type": "nav_msgs/Odometry",
        "publishers": frozenset({"/gazebo"}),
        "subscribers": frozenset(),
    },
    "/ground/scan": {
        "type": "sensor_msgs/LaserScan",
        "publishers": frozenset({"/gazebo"}),
        "subscribers": frozenset(),
    },
}
```

The cmd input publisher set may contain the active probe only during command
phases, but must be empty in the recorded ownership snapshot. All transient
sample/TF/guard subscribers disconnect before that snapshot; only the frozen
topic records above are persisted.

`load_expected_fixed_transforms(rendered_urdf_path, expected_sha256)` first
requires the canonical run-local URDF captured from the installed
renderer/input chain to hash to
`2b58856bed616a9e7402f7ddf4be4336f669271ca3989fdcf1d50f6939270832`,
parses its 17 fixed joints, applies the frozen `ground/` prefix, and returns
their exact parent/child translations and normalized quaternions.
`validate_tf_authorities(edges, current_clock, expected_fixed_transforms)`
then requires all 19 runtime edges. Freeze the two non-RSP edges and generate
the 17 exact RSP authorities from this tuple:

```python
EXPECTED_TF_AUTHORITIES = {
    ("world", "ground/odom", "tf_static"): "/ground/world_to_odom",
    ("ground/odom", "ground/base_link", "tf"): "/gazebo",
}
EXPECTED_FIXED_CHILDREN = (
    "ground/lidar_2d_link",
    "ground/wheel1_Link", "ground/wheel1_1_Link",
    "ground/wheel1_2_Link", "ground/wheel1_3_Link",
    "ground/wheel2_Link", "ground/wheel2_1_Link",
    "ground/wheel2_2_Link", "ground/wheel2_3_Link",
    "ground/wheel3_Link", "ground/wheel3_1_Link",
    "ground/wheel3_2_Link", "ground/wheel3_3_Link",
    "ground/wheel4_Link", "ground/wheel4_1_Link",
    "ground/wheel4_2_Link", "ground/wheel4_3_Link",
)
EXPECTED_TF_AUTHORITIES.update({
    ("ground/base_link", child, "tf_static"):
        "/ground/robot_state_publisher"
    for child in EXPECTED_FIXED_CHILDREN
})

TF_EDGE_KEYS = frozenset({
    "parent", "child", "channel", "authority", "stamp",
    "translation", "rotation",
})
```

Require identity world edge, current dynamic odom edge, the exact LiDAR mount,
the 16 renderer-derived wheel transforms, one authority per edge,
connectivity, exactly 19 edges total, and no unprefixed or
`ground/ground/...` frame anywhere. Compare every fixed transform against the
hash-verified expected-transform mapping within `1e-6`, rather than accepting
child names alone. The dynamic odom edge stamp must trail the current
`/clock` by no more than `0.10` simulated seconds. Both
`validate_probe_evidence` and final schema validation obtain the canonical
model path/digest from provenance, call the loader, and cross-check the same
mapping; no caller-supplied numeric TF oracle is trusted.

`validate_plugin_maps(records)` compares resolved mapped files to exactly the
three Task 6 paths and rejects missing or same-basename shadows.

Freeze separate cleanup-before and cleanup-after schemas. The probe cannot
write a schema-2 result because `logs` and `shutdown` do not exist until the
supervisor has cleaned the process group:

```python
PROBE_SCHEMA_VERSION = 1
LAUNCHER_SCHEMA_VERSION = 1
RESULT_SCHEMA_VERSION = 2

PROBE_KEYS = frozenset({
    "schema_version", "status", "fatal_errors", "run", "environment",
    "provenance", "processes", "graph", "samples", "guard", "motion",
    "tf", "collision",
})
RESULT_KEYS = PROBE_KEYS | frozenset({"logs", "shutdown"})

PROBE_SECTION_KEYS = {
    "run": frozenset({
        "run_id", "run_dir", "started_utc", "probe_state",
    }),
    "environment": frozenset({
        "home", "ros_master_uri", "gazebo_master_uri",
        "gazebo_model_database_uri", "search_paths", "fixed_environment",
        "state_paths", "writable_fd_audit",
    }),
    "provenance": frozenset({
        "install_contract_path", "install_contract_sha256", "packages",
        "renderer", "input", "model", "meshes", "plugins",
        "external_before",
    }),
    "processes": frozenset({
        "owned_root", "gzserver", "models", "required_nodes",
    }),
    "graph": frozenset({"snapshot_stamp", "topics"}),
    "samples": frozenset({"clock", "odom", "scan"}),
    "guard": frozenset({
        "records", "probe_disconnected", "post_probe_snapshot_stamp",
    }),
    "motion": frozenset({"forward", "rotation"}),
    "tf": frozenset({"edges", "forbidden_frames"}),
    "collision": frozenset({
        "model", "link", "collisions", "mesh_collision_count",
        "baseline_height", "final_height", "roll", "pitch",
        "raw_info_path", "raw_info_sha256",
    }),
}

LAUNCHER_RUN_KEYS = frozenset({
    "run_id", "run_dir", "started_utc", "finished_utc", "wall_seconds",
    "supervisor_status",
})
RESULT_RUN_KEYS = LAUNCHER_RUN_KEYS
RESULT_PROVENANCE_KEYS = (
    PROBE_SECTION_KEYS["provenance"] |
    frozenset({"external_after", "external_unchanged"}))
LOG_KEYS = frozenset({
    "launch_path", "launch_sha256", "fatal_matches",
})
SHUTDOWN_KEYS = frozenset({
    "probe_status", "launcher_status", "cleanup_owner",
    "signal_sequence", "escalated", "remaining_pids", "remaining_ports",
})
LAUNCHER_KEYS = frozenset({
    "schema_version", "run", "logs", "shutdown", "external_after",
    "first_failure", "fatal_errors",
})
FIRST_FAILURE_KEYS = frozenset({"exit_class", "phase", "message"})
SEARCH_PATH_KEYS = frozenset({
    "ROS_PACKAGE_PATH", "CMAKE_PREFIX_PATH", "PYTHONPATH",
    "LD_LIBRARY_PATH", "GAZEBO_PLUGIN_PATH", "GAZEBO_MODEL_PATH",
    "GAZEBO_RESOURCE_PATH", "PKG_CONFIG_PATH", "OGRE_RESOURCE_PATH",
})
EXPECTED_FIXED_ENVIRONMENT = {
    "PATH": "/opt/ros/noetic/bin:/usr/bin:/bin",
    "LANG": "C.UTF-8",
    "LC_ALL": "C.UTF-8",
    "ROS_ETC_DIR": "/opt/ros/noetic/etc/ros",
    "ROS_ROOT": "/opt/ros/noetic/share/ros",
    "ROSLISP_PACKAGE_DIRECTORIES": "",
}
FIXED_ENVIRONMENT_KEYS = frozenset(EXPECTED_FIXED_ENVIRONMENT)
STATE_PATH_KEYS = frozenset({
    "TMPDIR", "ROS_HOME", "ROS_LOG_DIR", "GAZEBO_LOG_PATH",
    "IGN_FUEL_CACHE_PATH", "XDG_CACHE_HOME", "XDG_CONFIG_HOME",
    "XDG_DATA_HOME",
})
GUARD_RECORD_KEYS = frozenset({
    "case", "input_tokens", "output", "input_stamp", "output_stamp",
    "latency",
})
GUARD_CASE_TOKENS = {
    "over_limit": ("0.75", "0", "0", "0", "0", "2.0"),
    "nonplanar": ("0", "0.1", "0", "0", "0", "0"),
    "nonfinite": ("nan", "0", "0", "0", "0", "0"),
}
```

`validate_environment` requires the nine `SEARCH_PATH_KEYS` to equal the
canonical Task 6 lists. The fixed environment object must equal
`EXPECTED_FIXED_ENVIRONMENT` by key/value, independent of mapping iteration
order. The model-database URI must be empty, and every state path must be a
mode-0700 descendant of the same run directory. The empty ROSLISP scalar is
not passed through path-list validation.

`validate_probe_evidence` accepts schema 1 only. Probe PASS requires every
section non-null and valid plus `run.probe_state="completed"`; probe FAIL
retains every exact key, represents an unavailable value as JSON `null`, and
has a nonempty `fatal_errors` list. Immediately after the supervisor creates
a run directory it atomically writes a complete null-filled schema-1 FAIL
seed from `make_probe_failure_seed(...)`, with real run fields,
`run.probe_state="seed"`, and one provisional `probe not completed` fatal
error. The actual probe atomically replaces that document and sets
`probe_state="completed"`, whether its status is PASS or FAIL.

The launcher document always has the exact `LAUNCHER_KEYS`. `first_failure`
is JSON `null` only when no failure occurred; otherwise it has exactly
`FIRST_FAILURE_KEYS`, while `fatal_errors` is an ordered list of every
supervisor/launcher failure. Its version is exactly
`LAUNCHER_SCHEMA_VERSION`, its `run` object has exactly `LAUNCHER_RUN_KEYS`,
and its `run_id`, canonical `run_dir`, and `started_utc` must equal the probe
fields. `merge_result` accepts a completed probe or the seed plus a launcher
document. A seed is valid only for a final FAIL and its
provisional error is replaced by the launcher's real `first_failure`; it can
never yield PASS. For completed probes, the supervisor's globally preserved
`first_failure` remains first and probe failures are appended without
reordering. This constructs schema 2 even when install, startup, or readiness
fails before the probe starts.

Final PASS contains no `null`, has an empty fatal list, both probe and launcher
statuses zero, `cleanup_owner="inner"`, and passes every validator. Final FAIL
keeps the full schema and a nonempty fatal list; backup cleanup is recorded as
`cleanup_owner="outer-watchdog"` and cannot pass. Any missing/extra key, bool
used as a number, non-finite number, wrong type, dirty log, unexpected nonzero
status, escalation, or remaining PID/port fails closed. Usage failure before
a run directory exists returns 64 without a result; once the run directory
and seed exist, every controlled exit path must attempt a schema-2 FAIL.

Add one subtest per top-level and nested key that deletes exactly that key and
expects `LiveContractError`. All persisted numbers are finite because scan
misses and NaN probes use counts/string tokens. Serialize only with
`json.dumps(payload, allow_nan=False, sort_keys=True,
separators=(",", ":"))`.

`_atomic_write_document(path, payload, expected_basename)` implements the one
writer: it requires a canonical path below the mode-0700 run directory,
writes a mode-0600 temporary sibling, flushes and `fsync`s it, uses
`os.replace`, `fsync`s the parent directory, and leaves no temporary file.
`atomic_write_probe` freezes `expected_basename="probe-evidence.json"` and
`atomic_write_result` freezes `expected_basename="result.json"`.

### Contract detail 7.D: whole-suite RED checkpoint

```bash
env -i PATH=/usr/bin:/bin LANG=C.UTF-8 LC_ALL=C.UTF-8 \
  TMPDIR="$PWD/logs/bunker_standalone/engineering-tmp" \
  /usr/bin/python3 -B -m unittest -v tests.test_bunker_live_contract
```

Expected: FAIL because `tools.bunker_live_contract` does not exist.

### Contract detail 7.E: pure evidence library surface

Create the following public functions with `LiveContractError` as the only
expected validation exception:

```text
validate_clock_samples(stamps) -> summary dict
validate_scan_samples(samples) -> summary dict
validate_odometry_samples(samples) -> summary dict
median_period(stamps, minimum_count) -> float
unwrap_yaw(previous, current) -> float delta
project_local_delta(baseline_pose, final_pose) -> dict
validate_guard_admission(records) -> summary dict
validate_motion_phase(kind, evidence) -> summary dict
validate_graph(evidence) -> summary dict
load_expected_fixed_transforms(rendered_urdf_path, expected_sha256) -> mapping
validate_tf_authorities(edges, current_clock, expected_fixed_transforms) -> summary dict
validate_plugin_maps(records) -> summary dict
validate_collision(evidence) -> summary dict
validate_environment(evidence) -> summary dict
validate_provenance(evidence) -> summary dict
validate_logs(evidence) -> summary dict
validate_shutdown(evidence) -> summary dict
make_probe_failure_seed(run_id, run_dir, started_utc, fatal_error) -> schema-1 payload
validate_probe_evidence(payload) -> normalized schema-1 payload
merge_result(probe_payload, launcher_payload) -> schema-2 payload
validate_live_evidence(payload) -> normalized payload
atomic_write_probe(path, payload) -> canonical probe path
atomic_write_result(path, payload) -> canonical result path
```

`validate_collision` requires exactly one `base_link_collision`, box origin
and size within `1e-6`, zero mesh collisions, height within `0.03 m` of the
settled baseline, and absolute roll/pitch below `0.05 rad`. It labels this
only as proxy materialization/loading and makes no contact-safety claim.

```python
COLLISION_RECORD_KEYS = frozenset({
    "name", "link", "geometry", "origin", "size",
})
```

The sole normalized record is named `base_link_collision`, belongs to
`base_link`, has `geometry="box"`, three finite origin values equal to the
declared collision origin, and three finite size values equal to the declared
box size within `1e-6`.

Do not import `rospy`, Gazebo messages, NumPy, pandas, benchmark helpers, or
P450 modules. Sort all set-derived data before serialization so identical
evidence produces byte-identical JSON.

The same pure file exposes this exact CLI for the supervisor and offline
tests. Production always invokes it as
`/usr/bin/python3 -B "$BUNKER_REPO_ROOT/tools/bunker_live_contract.py"`;
every supplied path is canonical and absolute, every file argument is regular,
and every output is below the supplied run directory:

```text
/usr/bin/python3 -B "$BUNKER_REPO_ROOT/tools/bunker_live_contract.py" seed-probe \
  --run-id RUN_ID --run-dir RUN_DIR --started-utc UTC \
  --fatal-error "probe not completed" \
  --output RUN_DIR/probe-evidence.json
/usr/bin/python3 -B "$BUNKER_REPO_ROOT/tools/bunker_live_contract.py" validate-probe \
  --input RUN_DIR/probe-evidence.json
/usr/bin/python3 -B "$BUNKER_REPO_ROOT/tools/bunker_live_contract.py" finalize \
  --probe RUN_DIR/probe-evidence.json \
  --launcher RUN_DIR/launcher-evidence.json \
  --output RUN_DIR/result.json
/usr/bin/python3 -B "$BUNKER_REPO_ROOT/tools/bunker_live_contract.py" validate-result \
  --input RUN_DIR/result.json
```

`seed-probe` exits 0 only after a valid seed has been atomically written.
For `validate-probe`, `finalize`, and `validate-result`, exit 0 means a
structurally valid PASS, exit 1 means a structurally valid FAIL document was
read or written, and exit 65 means malformed/unreadable evidence. A valid FAIL
is still atomically persisted and is never normalized into success.

### Contract detail 7.F: whole-suite GREEN checkpoint and commit

Run this exact command twice. The second run must produce identical serialized
fixtures and leave no temporary result file:

```bash
env -i PATH=/usr/bin:/bin LANG=C.UTF-8 LC_ALL=C.UTF-8 \
  TMPDIR="$PWD/logs/bunker_standalone/engineering-tmp" \
  /usr/bin/python3 -B -m unittest -v tests.test_bunker_live_contract
```

```bash
git add tools/bunker_live_contract.py tests/test_bunker_live_contract.py
git commit -m "test: define the BUNKER live evidence contract"
```

---

## Task 8: Run one bounded BUNKER standalone live gate

**Files:**

- Create: `scripts/probe_bunker_standalone.py`
- Create: `scripts/start_bunker_runtime_group.py`
- Create: `scripts/smoke_bunker_standalone.bash`
- Create: `tests/test_bunker_probe_contract.py`
- Create: `tests/test_bunker_runtime_group.py`
- Modify: `tests/test_bunker_shell_contract.py`
- Modify: `tests/test_bunker_live_contract.py`

Execute the adapter/supervisor work in these small RED→GREEN slices. After
8.1-8.18, run the already materialized
`tests.test_bunker_live_contract` and `tests.test_bunker_probe_contract`
modules; after 8.19 add `tests.test_bunker_shell_contract`; after 8.20 the
runtime-group module exists, so run the exact four-module command under 8.23
after every remaining slice. In every case the new test turns GREEN without
regressing the previously materialized modules; never require Python to import
a future test file before its creation slice.

- [ ] **8.1:** `ProbeArgumentTest.test_exact_cli` → exact parser and
  `ProbeConfig`.
- [ ] **8.2:**
  `ProbeArgumentTest.test_rejects_bad_paths_and_process_identity` → canonical
  run/install paths and process-identity grammar.
- [ ] **8.3:** `ProbeGraphAdapterTest.test_normalizes_master_state` → typed
  system-state/topic normalization.
- [ ] **8.4:** `ProbeProcessTest.test_descendants_bind_pid_start_time` →
  descendant PID/PGID/SID/start-ticks binding.
- [ ] **8.5:** `ProbeProcessTest.test_proc_maps_deduplicates_segments` →
  canonical unique mapped-library records.
- [ ] **8.6:** `ProbeProcessTest.test_writable_fd_boundary` → writable-FD
  containment audit.
- [ ] **8.7:** `ProbeModelInfoTest.test_parses_one_box_and_no_mesh` → bounded
  `gz model -i` parser.
- [ ] **8.8:** `ProbeTfCollectorTest.test_preserves_connection_callerid` → TF
  channel/authority collection.
- [ ] **8.9:** `ProbeReadinessTest.test_requires_all_conditions_together` →
  readiness predicate.
- [ ] **8.10:** `ProbeReadinessTest.test_wall_deadline_cannot_be_extended` →
  monotonic wall deadline.
- [ ] **8.11:** `ProbeSampleTest.test_collects_clock_odom_and_scan` → initial
  clock/20-odom/8-scan collection.
- [ ] **8.12:** `ProbeGuardTest.test_clamp_lateral_and_nan` → three command
  admission probes.
- [ ] **8.13:** `ProbeGuardTest.test_disconnects_before_graph_snapshot` →
  unregister/zero/watchdog/re-snapshot sequence.
- [ ] **8.14:** `ProbeSchedulerTest.test_twenty_hertz_uses_sim_time` →
  simulation-time command scheduler.
- [ ] **8.15:**
  `ProbeForwardTest.test_forward_watchdog_and_stationary_window` → forward
  phase.
- [ ] **8.16:**
  `ProbeRotationTest.test_rotation_watchdog_and_stationary_window` → rotation
  phase.
- [ ] **8.17:**
  `ProbeFinalEvidenceTest.test_collects_tf_collision_maps_and_provenance` →
  final behavior/provenance collection.
- [ ] **8.18:**
  `ProbeFailureTest.test_every_failure_writes_probe_fail_document` and
  `LiveSchemaTest.test_seed_merges_pre_probe_failure` → completed/seed
  schema-1 FAIL evidence and valid schema-2 pre-probe FAIL.
- [ ] **8.19:** `SmokeArgumentTest` and `SmokeRunDirectoryTest` → outer
  argument/run-directory branches.
- [ ] **8.20:** `SmokeOwnershipTest`, `SmokeCleanupTest`, and
  `SmokeWatchdogTest`, plus `RuntimeGroupEntryTest` → signal reset, two-phase
  process identity, leader-directed SIGINT, outer watchdog cleanup, ports,
  and first-failure preservation.
- [ ] **8.21:** `SmokeFinalizationTest` → launcher evidence, schema merge, and
  atomic result finalization.
- [ ] **8.22:** `bash -n` plus `shellcheck` for all three production shell
  scripts.
- [ ] **8.23:** all offline probe/live/shell fixtures GREEN.
- [ ] **8.24:** install-only probe GREEN, then one fresh live gate PASS.
- [ ] **8.25:** commit only scripts and offline tests; never generated logs.

### Contract detail 8.A: probe-adapter tests

Create `tests/test_bunker_probe_contract.py`. Keep ROS collection separate
from validation. Inject fake ROS-master XML-RPC responses, Gazebo services,
topic messages, `/proc` snapshots, and a command runner to exercise
`scripts/probe_bunker_standalone.py` without a running master.
Require adapters to:

- preserve the publisher `callerid` from `/tf` and `/tf_static` connection
  headers for every edge;
- query `getSystemState` and `getTopicTypes`, normalize node/topic names, and
  disconnect the transient `/ground/cmd_vel_safe` subscriber before the final
  graph snapshot;
- locate exactly one live `gzserver` descendant of the owned process group;
- parse only canonical mapped library paths from `/proc/<pid>/maps`;
- call Gazebo model-state services and `gz model -m bunker -i` with argument
  vectors and a timeout, then parse exactly one box/no mesh collision;
- convert ROS time, odometry, model pose/twist, LaserScan, and transforms to
  plain dictionaries before calling the pure validators; only finite-only
  summaries enter persisted evidence; and
- write `probe-evidence.json`, never the final PASS result.

Use dependency injection for time, publishers/subscribers, service proxies,
XML-RPC proxies, process inspection, and command execution. No test monkeypatch
may weaken a production validator.

### Contract detail 8.B: bounded-shell test modes

Extend `tests/test_bunker_shell_contract.py` for test branches that execute
before environment setup or ROS/Gazebo launch:

```text
--test-smoke-integers ROS_PORT GAZEBO_PORT STARTUP_SECONDS TOTAL_SECONDS
  canonical ranges: ports 1024-65535 and distinct; startup 30-300;
  total 90-600 and at least startup+45

--test-private-run-directory PARENT RUN_ID
  accepts only YYYYMMDDTHHMMSSZ-PID under the canonical
  logs/bunker_standalone parent and creates mode 0700 once

--test-process-tree ROOT_PID NEEDLE SNAPSHOT
  finds only live non-zombie descendants and rejects PID 1/bad roots

--test-launch-log FILE
  rejects case-insensitively: segmentation fault, assertion failure, abort,
  core dump, boost thread trying joining itself, unexpected process death,
  plugin/dependency load failure, XML error, and mesh error

--test-plugin-maps EXPECTED_PLANAR EXPECTED_LASER EXPECTED_RAY MAPS_FILE
  accepts all three exact canonical files once and no same-basename shadow

--test-cleanup-status LAUNCH_STATUS CLEANUP_OWNER SIGNAL_SEQUENCE REMAINING_COUNT
  accepts only launch status 0, cleanup owner inner, one PID-directed SIGINT,
  and zero remaining; rejects normalized 130, outer-watchdog cleanup, or any
  group escalation
```

Test timeout and probe failure fixtures, cleanup-status preservation, runtime
identity handoff, inner death before/after that handoff, outer-watchdog
cleanup, atomic result finalization, occupied ports, stale result paths, and
hostile run IDs.

`tests/test_bunker_runtime_group.py` imports the trampoline without running
ROS and injects `setsid`, `signal`, `/proc` reading, atomic writing, and
`execv`. Freeze tests proving: exact production argv only; state-changing
calls occur in the order `setsid -> reset SIGINT -> reset SIGTERM -> write
leader-only identity -> exec timeout`; inherited ignored signals become
`SIG_DFL`; the identity binds the current PID/start ticks and mode-0700 run
directory; and any validation/write failure occurs before `execv`. A fake
`execv` must receive exactly `/usr/bin/timeout`, `--signal=INT`,
`--kill-after=30s`, the bounded seconds token, and the frozen roslaunch argv.

### Contract detail 8.C: ROS/Gazebo collection and readiness

`probe_bunker_standalone.py` accepts exactly:

```text
--run-dir RUN_DIR
--owned-root-pid PID --owned-pgid PGID --owned-sid SID
--owned-start-ticks START_TICKS --startup-seconds STARTUP_SECONDS
--install-contract RUN_DIR/install-contract.json
```

The script canonicalizes `--run-dir`, requires it to be the mode-0700 run
directory below `logs/bunker_standalone`, and fixes its output to
`RUN_DIR/probe-evidence.json`; there is no caller-selected output path.
`PID`, `PGID`, `SID`, and `START_TICKS` are canonical positive decimal
integers greater than one; `STARTUP_SECONDS` is an integer in `[30, 300]`.
Freeze the parser output and dependency boundary as:

```python
@dataclass(frozen=True)
class ProcessIdentity:
    pid: int
    pgid: int
    sid: int
    start_ticks: int

@dataclass(frozen=True)
class ProbeConfig:
    run_dir: Path
    owned_root: ProcessIdentity
    startup_seconds: int
    install_contract: Path
    output: Path

@dataclass
class ProbeDependencies:
    now_wall: Callable[[], float]
    sleep_wall: Callable[[float], None]
    sim_clock: Callable[[], float]
    read_param: Callable[[str], object]
    master_state: Callable[[], Mapping]
    topic_types: Callable[[], Mapping]
    model_names: Callable[[], Sequence[str]]
    model_state: Callable[[str], Mapping]
    create_publisher: Callable
    create_subscriber: Callable
    run_command: Callable
    process_snapshot: Callable[[], Sequence[Mapping]]
    read_text: Callable[[Path], str]
    read_link: Callable[[Path], str]
```

Production ROS imports occur only in `build_runtime_dependencies()`. The
adapter tests use a fake clock, master, publishers/subscribers, model state,
process snapshots, filesystem readers, and command runner without starting
ROS or Gazebo.

Because the clean environment forbids a source-tree `PYTHONPATH`, load the
pure validator by canonical file path without changing `sys.path`:

```python
REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
contract_path = REPOSITORY_ROOT / "tools/bunker_live_contract.py"
spec = importlib.util.spec_from_file_location(
    "bunker_live_contract", contract_path)
bunker_live_contract = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bunker_live_contract)
```

It initializes one fixed node `/bunker_standalone_probe` and uses condition
polling with wall-time deadlines; no fixed startup sleep counts as readiness.
Readiness requires all of these simultaneously:

1. advancing `/clock` and
   `type(dependencies.read_param("/use_sim_time")) is bool` with the value
   exactly `True` (integers, strings, and truthy substitutes fail);
2. exactly one model named `bunker` and exactly one owned live `gzserver`;
3. guard and Gazebo command-topic connections present;
4. one Gazebo publisher each for typed odometry and scan;
5. the three required TF edges received; and
6. canonical plugin mappings available from the live gzserver.

Collect at least 20 odometry and eight scan samples. Preserve their simulation
timestamps and collect `/clock` independently so a stamped-but-stalled stream
cannot pass.

### Contract detail 8.D: guard-admission phase

Before the motion baseline, temporarily subscribe to
`/ground/cmd_vel_safe`, publish each probe to `/ground/cmd_vel`, and require the
corresponding safe output within `0.10` simulated seconds:

```text
20 Hz over-limit: linear.x=0.75, angular.z=2.0 -> 0.5, 1.0
one nonplanar:     linear.y=0.1                  -> all zero
one nonfinite:     linear.x=NaN                  -> all zero
```

Unregister the safe subscriber and command publisher, publish an explicit zero
through a short-lived publisher, wait for watchdog stop, unregister it, then
repeat the graph ownership snapshot. Fail if the probe remains in either
publisher/subscriber set.

### Contract detail 8.E: sequential motion phases

Use simulation time for every phase deadline and publish at 20 Hz:

1. settle; record Gazebo and odometry baseline pose/yaw;
2. send `linear.x=0.25` for 2.0 simulated seconds;
3. stop publishing, require at least 0.5 simulated seconds of clock advance,
   observe both model and odometry speeds below `0.01`, then begin a separate
   1.0-simulated-second stationary window;
4. record forward/lateral displacement in the baseline-yaw local frame and
   model/odometry x/y/yaw agreement;
5. from the new settled pose, send `angular.z=0.5` for 1.5 simulated seconds;
6. repeat watchdog and stationary windows; and
7. record unwrapped yaw, planar drift, model/odometry yaw agreement, height,
   roll, and pitch.

Never compare quaternion components directly. Never use wall time as evidence
of a Gazebo-plugin timeout. Record command windows, time-to-stop, watchdog
coast, and stationary deltas for both phases.

### Contract detail 8.F: TF, collision, process, and provenance collection

After motion, collect current TF edges and connection-header authorities;
read strict-string `/ground/robot_description`, atomically persist it mode
0600 as `RUN_DIR/rendered-bunker.urdf`, require its digest and installed
renderer/input provenance to match the frozen chain, and load from it the
hash-verified 17-transform oracle. Then reject numeric mismatch, stale dynamic
transforms, and every unprefixed/double-prefixed frame.
Capture `gz model -m bunker -i` output below the run directory and parse the
single exact box collision. Record sanitized search variables, canonical
package/input/renderer/model/mesh/plugin paths and hashes, gzserver PID/maps,
all graph owners, sample/rate summaries, and the owned process-group identity.

Poll `/proc/<pid>/fd` for every owned live descendant during readiness and
each phase. Classify access mode from `/proc/PID/fdinfo/FD`. For writable
regular filesystem targets, resolve the canonical path and fail unless it is
a descendant of SIM (normally the run-local state tree); a deleted regular
file is always fatal. `/dev/null` is the sole allowed writable device target.

Pipes, TCP/UDP/Unix sockets, `anon_inode`, `memfd`, eventfd, timerfd, and
similar kernel IPC descriptors are non-filesystem endpoints: allow and record
them by class without applying path containment. In particular, do not treat
the `(deleted)` suffix of a `memfd` as a deleted regular file. Independently
prove owned ports/process identities, so allowing an IPC descriptor cannot
hide a foreign listener or process. Snapshot the known login-home
ROS/Gazebo/XDG state paths and the read-only external repository status before
and after the run; any detected external write/change is fatal.

Validate the complete plain evidence through `bunker_live_contract`, then
write only `probe-evidence.json`. On any exception, write the same schema with
`status="FAIL"` and a nonempty `fatal_errors` list before returning nonzero.

### Contract detail 8.G: process-owning smoke supervisor

`smoke_bunker_standalone.bash` defaults to ROS port `11371`, Gazebo port
`11372`, startup deadline `120` seconds, and total deadline `300` seconds.
It must:

1. validate all knobs and confirm both loopback ports are free;
2. create one mode-0700 run directory at
   `logs/bunker_standalone/YYYYMMDDTHHMMSSZ-PID`, where the timestamp is the
   current UTC second and `PID` is the outer supervisor's canonical positive
   decimal process ID. Require `test ! -e` before creation, then immediately
   atomically write the schema-1 FAIL seed before setup, installation, or any
   process launch. If seed creation fails, start nothing;
3. create a 128-bit hex token with
   `/usr/bin/od -An -N16 -tx1 /dev/urandom`, write it mode 0600 below the run
   directory, and start the clean inner supervisor under an outer wall-time
   watchdog using this exact boundary:

   ```bash
   bunker_outer_seconds=$((bunker_total_seconds + 60))
   /usr/bin/timeout --signal=INT --kill-after=30s \
       "${bunker_outer_seconds}s" \
     "$bunker_repo_root/scripts/with_bunker_env.bash" \
       --run-dir "$bunker_run_dir" -- \
       /usr/bin/env \
         ROS_MASTER_URI="http://127.0.0.1:${bunker_ros_port}/" \
         GAZEBO_MASTER_URI="http://127.0.0.1:${bunker_gazebo_port}" \
       "$bunker_repo_root/scripts/smoke_bunker_standalone.bash" \
         --inner "$bunker_run_dir" "$bunker_run_token" &
   bunker_clean_pid=$!
   ```

   The inner branch compares the token and exact `BUNKER_REPO_ROOT` /
   `BUNKER_RUN_DIR` values before doing work. The outer branch remains alive
   as a watchdog; an inner timeout or crash does not make the runtime group
   unreachable;
4. inside the clean supervisor, run
   `"$BUNKER_REPO_ROOT/scripts/validate_bunker_install.bash"` to a mode-0600
   temporary sibling, accept only schema-1 PASS, `fsync`/rename it to
   `RUN_DIR/install-contract.json`, then start the sole runtime group exactly
   as follows:

   ```bash
   /usr/bin/python3 -B \
     "$BUNKER_REPO_ROOT/scripts/start_bunker_runtime_group.py" \
       --run-dir "$BUNKER_RUN_DIR" \
       --total-seconds "$bunker_total_seconds" -- \
       /opt/ros/noetic/bin/roslaunch \
         --screen --sigint-timeout=10 --sigterm-timeout=5 \
         bunker_sim_runtime bunker_standalone.launch gui:=false \
     >"$BUNKER_RUN_DIR/roslaunch.log" 2>&1 &
   bunker_launch_pid=$!
   ```

   The 3.8-compatible trampoline validates the exact command, calls
   `os.setsid()`, explicitly restores SIGINT/SIGTERM to `SIG_DFL`, and before
   `execv` atomically writes a mode-0600 *leader-only*
   `runtime-identity.json` containing phase, PID, PGID, SID, and start ticks.
   It then becomes `/usr/bin/timeout --signal=INT --kill-after=30s` and runs
   roslaunch without changing PID. Thus even an inner crash immediately after
   launch leaves enough verified identity for outer group cleanup.

   Freeze identity schema 1 with exact top-level keys `schema_version`,
   `phase`, `run_dir`, `leader`, and `roslaunch`; each non-null identity has
   exactly `pid`, `pgid`, `sid`, and `start_ticks`. Phase `leader-only`
   requires `roslaunch=null`; phase `full` requires both identities in the
   same PGID/SID. Each atomic replacement must preserve the leader fields
   byte-for-byte.

   `roslaunch` itself starts the one ROS master; do not launch a separate
   `roscore`. Require `bunker_launch_pid` to match the leader-only identity,
   then resolve exactly one actual `/opt/ros/noetic/bin/roslaunch` descendant
   by canonical executable and NUL-delimited argv. Atomically upgrade the
   document to phase `full` with both leader and roslaunch PID/PGID/SID/start
   ticks. Require one shared non-caller PGID/SID, then pass the leader identity
   and exact install-contract path to the probe. The outer watchdog accepts a
   leader-only identity for fail-safe group TERM/KILL, but a live gate cannot
   pass until the full identity has been recorded;
5. preserve the first nonzero install/readiness/probe/launcher/log/cleanup
   result. Every failure after seed creation continues through cleanup,
   launcher-evidence generation, merge, and atomic schema-2 FAIL writing;
6. after probe completion or a deadline, revalidate the exact roslaunch
   identity and send SIGINT to that PID only. The supervisor shell and timeout
   leader are not signaled. Wait up to 20 wall seconds for roslaunch and the
   group to exit, then `wait "$bunker_launch_pid"`; PASS requires the actual
   wait status to be 0. Never translate 130 or another nonzero status to 0.
   Only after the grace period may TERM and then KILL target the verified
   whole PGID, and either escalation is fatal;
7. prove no owned descendant, model process, ROS/Gazebo node, or owned port
   remains, then atomically write an inner-cleanup marker. If the clean inner
   supervisor times out/crashes or returns without that valid marker, the
   still-running outer watchdog reads `runtime-identity.json`, revalidates
   the available identities, applies the same PID-directed SIGINT when phase
   is `full`, and uses verified group-directed TERM/KILL for a leader-only
   handoff or after the grace period. It records
   `cleanup_owner="outer-watchdog"`. This backup path is always a valid FAIL,
   never PASS; and
8. merge the seed/completed probe with exact launcher/log/shutdown evidence,
   validate it, and atomically write the only final `result.json` after
   cleanup. The outer watchdog performs this finalization itself if the inner
   cannot, using absolute paths below `BUNKER_REPO_ROOT` and the existing seed.

Use traps for EXIT/INT/TERM, but disable recursive traps during final cleanup.
Never signal an unresolved PID, PID 1, the caller's process group, or a
process outside the recorded session. Treat unexpected roslaunch/group exit
before controlled SIGINT as fatal. A probe PASS cannot override a dirty log,
nonzero launcher, escalation, outer-watchdog cleanup, remaining process/port,
or missing evidence. This explicitly avoids treating the clean-supervisor
timeout as containment for a separately created runtime session.

Freeze the supervisor state machine and exit classes:

```text
OUTER_VALIDATE -> CREATE_RUN_DIR -> WRITE_PROBE_FAIL_SEED
-> START_CLEAN_SUPERVISOR_WITH_WATCHDOG
-> INNER_VALIDATE_TOKEN_AND_ENV -> WRITE_PRE_RUN_SNAPSHOTS
-> RUN_INSTALL_CONTRACT -> START_OWNED_ROSLAUNCH_GROUP
-> WRITE_RUNTIME_IDENTITY -> WAIT_ROS_MASTER -> RUN_PROBE
-> CAPTURE_FIRST_FAILURE -> SIGINT_ROSLAUNCH_PID
-> WAIT_20_SECONDS -> OPTIONAL_GROUP_SIGTERM -> OPTIONAL_GROUP_SIGKILL
-> WAIT_AND_COLLECT_LAUNCHER_STATUS -> CHECK_PIDS_AND_PORTS -> SCAN_LOG
-> WRITE_POST_RUN_SNAPSHOTS -> WRITE_INNER_CLEANUP_MARKER
-> WRITE_LAUNCHER_EVIDENCE -> MERGE_AND_VALIDATE_RESULT
-> ATOMIC_WRITE_RESULT -> OUTER_VERIFY_OR_BACKUP_CLEANUP
-> EXIT_WITH_PRESERVED_STATUS

0 PASS
64 usage or argument contract
65 environment or install preflight
70 startup or readiness
71 behavioral probe
72 launcher or fatal log
73 cleanup, escalation, or remnant
74 evidence merge, validation, or atomic finalization
```

Before signaling, re-read PID/PGID/SID/start ticks and fail rather than signal
if any identity changed, is zero/one, equals the supervisor's group, or no
longer belongs to the recorded session. `/proc/<pid>/maps` validation first
deduplicates canonical real paths; “one plugin” means one unique canonical
library, not one raw memory-map segment.

### Contract detail 8.H: offline smoke GREEN checkpoint

```bash
chmod 755 scripts/probe_bunker_standalone.py \
  scripts/start_bunker_runtime_group.py \
  scripts/smoke_bunker_standalone.bash
env -i PATH=/usr/bin:/bin LANG=C.UTF-8 LC_ALL=C.UTF-8 \
  TMPDIR="$PWD/logs/bunker_standalone/engineering-tmp" \
  /usr/bin/python3 -B -m unittest -v \
  tests.test_bunker_live_contract \
  tests.test_bunker_probe_contract \
  tests.test_bunker_runtime_group \
  tests.test_bunker_shell_contract
bash -n scripts/with_bunker_env.bash \
  scripts/validate_bunker_install.bash \
  scripts/smoke_bunker_standalone.bash
shellcheck scripts/with_bunker_env.bash \
  scripts/validate_bunker_install.bash \
  scripts/smoke_bunker_standalone.bash
```

Expected: every success and injected-failure branch passes without starting a
ROS master or Gazebo.

### Contract detail 8.I: fresh standalone live gate

First rerun the install validator. Then execute:

```bash
scripts/smoke_bunker_standalone.bash
```

Expected: exit 0 and one new mode-0700 run directory whose atomic
`result.json` has `schema_version=2`, `status="PASS"`, empty fatal errors,
exact graph/plugin/TF/collision evidence, accepted guard/forward/rotation
phases, `shutdown.signal_sequence=["SIGINT"]`, no escalation, no remaining
process/port, and no external write/change.

If the live gate fails, invoke `superpowers:systematic-debugging`, diagnose the
first failing contract, add or tighten a reproducing offline test, make only
the smallest BUNKER-A change, rebuild the affected package, rerun the
install-only probe, and repeat one fresh live run. Do not relax numeric,
ownership, provenance, log, or cleanup gates merely to obtain PASS.

### Contract detail 8.J: commit the bounded live gate

Do not commit generated logs/results. Commit only implementation/tests:

```bash
git add scripts/probe_bunker_standalone.py \
  scripts/start_bunker_runtime_group.py \
  scripts/smoke_bunker_standalone.bash \
  tests/test_bunker_live_contract.py tests/test_bunker_probe_contract.py \
  tests/test_bunker_runtime_group.py \
  tests/test_bunker_shell_contract.py
git commit -m "test: gate the BUNKER standalone runtime"
```

---

## Task 9: Regress Air runtime and close BUNKER-A

**Files:**

- Modify only if a test exposes a BUNKER-A defect: files already named in
  Tasks 1-8
- Do not create benchmark, report, Pilot/T2/Formal, or paper-evidence files

- [ ] **Step 1: Run the complete repository offline suite from a blank env**

```bash
env -i PATH=/usr/bin:/bin LANG=C.UTF-8 LC_ALL=C.UTF-8 \
  TMPDIR="$PWD/logs/bunker_standalone/engineering-tmp" \
  /usr/bin/python3 -B -m unittest discover -s tests -v
scripts/with_noetic_env.bash /usr/bin/env \
  TMPDIR="$PWD/logs/bunker_standalone/engineering-tmp" \
  /usr/bin/python3 -B \
  src/p450/prometheus_gazebo/test/test_p450_sensor_profiles.py -v
```

Expected: zero failures/errors/skips not already explicitly accepted by the
repository. Record the actual test count from this fresh run; do not copy the
historical `175/175` count into the result.

- [ ] **Step 2: Rebuild and run both Catkin package suites**

```bash
scripts/with_noetic_env.bash /usr/bin/env \
  TMPDIR="$PWD/logs/bunker_standalone/engineering-tmp" \
  catkin build --workspace "$PWD" --profile p450-clean \
  bunker_description bunker_sim_runtime sim_platform_bringup \
  --force-cmake --no-status --summarize
scripts/with_noetic_env.bash /usr/bin/env \
  TMPDIR="$PWD/logs/bunker_standalone/engineering-tmp" \
  catkin test --workspace "$PWD" --profile p450-clean \
  bunker_sim_runtime sim_platform_bringup --no-status --summarize
scripts/with_noetic_env.bash /usr/bin/env \
  TMPDIR="$PWD/logs/bunker_standalone/engineering-tmp" \
  catkin_test_results --all \
  build/p450-clean/bunker_sim_runtime/test_results
scripts/with_noetic_env.bash /usr/bin/env \
  TMPDIR="$PWD/logs/bunker_standalone/engineering-tmp" \
  catkin_test_results --all \
  build/p450-clean/sim_platform_bringup/test_results
```

Expected: build and all package tests pass, including the real Noetic
`tf_prefix` rostest and the existing P450 launch contracts.

- [ ] **Step 3: Rerun the install-only and BUNKER live gates from scratch**

Create a new install-probe directory; do not reuse Task 6 output. Run:

```bash
set -euo pipefail
run_dir="$PWD/logs/bunker_standalone/final-install-$(date -u +%Y%m%dT%H%M%SZ)-$$"
test ! -e "$run_dir"
install -d -m 700 -- "$run_dir"
umask 077
final_install_tmp="$run_dir/.install-contract.json.tmp"
scripts/with_bunker_env.bash --run-dir "$run_dir" -- \
  "$PWD/scripts/validate_bunker_install.bash" >"$final_install_tmp"
/usr/bin/python3 -I -B - "$final_install_tmp" <<'PY'
import json
import os
import sys
from pathlib import Path

path = Path(sys.argv[1])
with path.open("r+", encoding="utf-8") as stream:
    payload = json.load(stream)
    assert payload["schema_version"] == 1
    assert payload["status"] == "PASS"
    stream.flush()
    os.fsync(stream.fileno())
PY
chmod 600 "$final_install_tmp"
mv -T -- "$final_install_tmp" "$run_dir/install-contract.json"
scripts/smoke_bunker_standalone.bash
```

Expected: both exit zero; the newest BUNKER result is schema-2 PASS with clean
SIGINT-only shutdown. Keep logs untracked.

- [ ] **Step 4: Run fresh P450 D435 then MID360 standalone regressions**

Only after the BUNKER PASS, use the already pinned external checkout as a
read-only runtime dependency:

```bash
P450_PX4_ROOT=/media/lu/P450_PAPER/P450-PAPER/workspaces/dependencies/px4 \
  scripts/smoke_p450_standalone.bash
P450_PX4_ROOT=/media/lu/P450_PAPER/P450-PAPER/workspaces/dependencies/px4 \
  scripts/smoke_p450_mid360.bash
```

Expected: D435 then MID360 each exit zero with their existing accepted result
contracts. Do not modify P450 runtime JSON, launches, smoke scripts, PX4, or
the external repository to accommodate BUNKER.

- [ ] **Step 5: Run boundary/provenance and workspace-cleanliness checks**

```bash
set -euo pipefail
env -i PATH=/usr/bin:/bin LANG=C.UTF-8 LC_ALL=C.UTF-8 \
  TMPDIR="$PWD/logs/bunker_standalone/engineering-tmp" \
  /usr/bin/python3 -B tools/runtime_boundary.py "$PWD"
git diff --check
git status --short
GIT_OPTIONAL_LOCKS=0 git -C /media/lu/P450_PAPER/P450-PAPER/source/P450-SIM \
  rev-parse HEAD | cmp - \
  logs/bunker_standalone/bunker-a-baseline/paper-head.txt
GIT_OPTIONAL_LOCKS=0 git -C /media/lu/P450_PAPER/P450-PAPER/source/P450-SIM \
  status --porcelain=v1 --untracked-files=all -z | cmp - \
  logs/bunker_standalone/bunker-a-baseline/paper-status.z
GIT_OPTIONAL_LOCKS=0 git \
  -C /media/lu/P450_PAPER/P450-PAPER/workspaces/dependencies/px4 \
  rev-parse HEAD | cmp - \
  logs/bunker_standalone/bunker-a-baseline/px4-head.txt
GIT_OPTIONAL_LOCKS=0 git \
  -C /media/lu/P450_PAPER/P450-PAPER/workspaces/dependencies/px4 \
  status --porcelain=v1 --untracked-files=all -z | cmp - \
  logs/bunker_standalone/bunker-a-baseline/px4-status.z
```

Expected: boundary check exits zero; no source/devel path appears in accepted
install/live evidence; the SIM worktree has only deliberate BUNKER-A source
changes/commits and untracked ignored logs; external status is byte-for-byte
the same as the pre-implementation snapshot.

- [ ] **Step 6: Request independent implementation review**

Invoke `superpowers:requesting-code-review` against the approved specification
and the Task 1-8 commit range. Require reviewers to check, independently:

1. scope/write-boundary and dependency closure;
2. renderer, launch, TF/plugin ownership, and install provenance; and
3. live numeric evidence, fail-closed paths, and cleanup.

Resolve Critical and Important findings with a reproducing test and rerun the
smallest affected verification loop. Re-run the fresh BUNKER live gate after
any runtime change and both P450 smokes after any install-space change. Do not
expand scope to AUBO/AG95 or paper infrastructure during review.

- [ ] **Step 7: Commit final review fixes, if any, and report the gate**

If review caused source changes, commit them in one focused commit after all
affected tests pass:

First run `git status --short` and reject every changed path outside the
following allowlist, then stage the allowlist explicitly:

```bash
git add config/bunker_assets.json config/runtime_overlay.json \
  config/runtime_sources.json scripts/probe_bunker_standalone.py \
  scripts/start_bunker_runtime_group.py \
  scripts/smoke_bunker_standalone.bash scripts/validate_bunker_install.bash \
  scripts/with_bunker_env.bash src/platform/bunker_sim_runtime \
  src/vendor/bunker_description/CMakeLists.txt \
  tests/test_bunker_build_contract.py tests/test_bunker_live_contract.py \
  tests/test_bunker_probe_contract.py tests/test_bunker_runtime_group.py \
  tests/test_bunker_shell_contract.py \
  tests/test_imported_layout.py \
  tests/test_runtime_boundary.py tests/test_runtime_extraction.py \
  tests/test_runtime_source_contract.py tools/bunker_assets.py \
  tools/bunker_live_contract.py tools/runtime_boundary.py
git diff --cached --check
git commit -m "fix: close the BUNKER-A runtime review"
```

Report exact commit IDs, fresh offline/Catkin counts, install-probe path, newest
BUNKER result path, D435/MID360 result paths, review finding counts, and the
external-repository unchanged check. State only the proven claim:
“BUNKER-A is an independently runnable, bounded-command,
collision-proxy-enabled kinematic planar Gazebo runtime.” Do not claim tracked
vehicle dynamics, collision safety, navigation readiness, shared-world
integration, or Air-Ground Pick completion.

---

## Stop condition

BUNKER-A is complete only when Tasks 1-9 are checked, the install-only probe
passes, one fresh BUNKER standalone run passes every schema-2 contract and
cleans up with SIGINT only, fresh P450 D435 and MID360 regressions pass, no
Critical/Important review finding remains, and no file outside `SIM` changed.
The next milestone is a separate user-approved decision; this plan must not
continue into AUBO/AG95, D435-on-Ground, MoveIt, shared-world bringup, mission
orchestration, benchmark, or method optimization.
