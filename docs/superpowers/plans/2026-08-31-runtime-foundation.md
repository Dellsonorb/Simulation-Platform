# Simulation Platform V1 Runtime Foundation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a tested, manifest-driven, read-only import boundary that creates the exact non-paper ROS runtime source snapshot for Simulation Platform V1 inside `/media/lu/P450_PAPER/SIM/p450_sim_v1`.

**Architecture:** The upstream P450-PAPER repository is immutable input. A JSON contract records the exact upstream commit, selected packages, destination paths, forbidden research packages, and auxiliary source files. A standard-library Python importer is dry-run by default, rejects path escapes, duplicate destinations, external symlinks, and overwrites, then records file hashes after an explicit apply. Offline `unittest` gates verify the contract and imported tree without sourcing ROS, Gazebo, PX4, Conda, or any parent workspace.

**Tech Stack:** Git 2.25, Python 3.8 standard library, `unittest`, JSON, ROS1 catkin metadata, Ubuntu 20.04 filesystem tools.

---

## Fixed scope and constraints

- Writable project root: `/media/lu/P450_PAPER/SIM/p450_sim_v1` only.
- Read-only upstream root: `/media/lu/P450_PAPER/P450-PAPER/source/P450-SIM`.
- Read-only PX4 source candidate: `/media/lu/P450_PAPER/P450-PAPER/workspaces/dependencies/px4`.
- Do not import `paper_benchmark`, `ground_aerial_benchmark`, `system_baseline_freeze`, `task_aware_approach`, `aerial_traversability`, `irm_base_placement`, Pilot, T2, Formal, results, artifacts, build, or devel trees.
- Do not use the ambient Conda Python, inherited `CMAKE_PREFIX_PATH`, inherited `PYTHONPATH`, or inherited `LD_LIBRARY_PATH` for foundation tests.
- This plan does not build or launch ROS. P450 build/runtime, Ground build/runtime, shared-world/TF, and Air-Ground Pick are separate follow-on plans.

## Frozen package set

Inherited packages:

```text
prometheus_msgs
prometheus_gazebo
prometheus_uav_control
realsense_ros_gazebo
livox_laser_gazebo_plugins
brick_aerial_perception
bunker_aubo_description
bunker_aubo_gazebo
bunker_aubo_moveit_config
bunker_navigation
brick_rgbd_perception
brick_visual_pick
brick_pick_demo
ground_pick_orchestrator
aubo_description
bunker_description
dh_ag95_description
roboticsgroup_gazebo_plugins
```

New neutral packages, created in later tasks/plans:

```text
sim_platform_bringup
ground_runtime_compat
air_ground_pose_bridge
air_ground_pick_demo
```

### Task 1: Manifest parser and validation API

**Files:**
- Create: `tools/__init__.py`
- Create: `tools/runtime_manifest.py`
- Create: `tests/__init__.py`
- Create: `tests/test_runtime_manifest.py`

- [ ] **Step 1: Write the failing parser tests**

```python
# tests/test_runtime_manifest.py
import json
import tempfile
import unittest
from pathlib import Path

from tools.runtime_manifest import ManifestError, RuntimeManifest


class RuntimeManifestTest(unittest.TestCase):
    def write_manifest(self, payload):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        path = Path(temporary.name) / "runtime_sources.json"
        path.write_text(json.dumps(payload), encoding="utf-8")
        return path

    def minimal_payload(self):
        return {
            "schema_version": 1,
            "upstream": {"expected_commit": "a" * 40},
            "layout": {"required_paths": ["src/p450"]},
            "packages": {
                "demo": {
                    "role": "p450",
                    "source": "Modules/demo",
                    "destination": "src/p450/demo",
                    "imported": True,
                }
            },
            "auxiliary_imports": [],
            "forbidden": {"package_names": [], "runtime_tokens": []},
        }

    def test_loads_valid_manifest(self):
        manifest = RuntimeManifest.load(self.write_manifest(self.minimal_payload()))
        self.assertEqual(1, manifest.schema_version)
        self.assertEqual("src/p450/demo", manifest.packages["demo"].destination)

    def test_rejects_parent_path_escape(self):
        payload = self.minimal_payload()
        payload["packages"]["demo"]["source"] = "../P450-PAPER"
        with self.assertRaisesRegex(ManifestError, "relative path escape"):
            RuntimeManifest.load(self.write_manifest(payload))

    def test_rejects_duplicate_destination(self):
        payload = self.minimal_payload()
        payload["packages"]["other"] = dict(payload["packages"]["demo"])
        with self.assertRaisesRegex(ManifestError, "duplicate destination"):
            RuntimeManifest.load(self.write_manifest(payload))

    def test_rejects_unknown_role(self):
        payload = self.minimal_payload()
        payload["packages"]["demo"]["role"] = "paper"
        with self.assertRaisesRegex(ManifestError, "unsupported role"):
            RuntimeManifest.load(self.write_manifest(payload))


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run the test and verify RED**

Run:

```bash
env -i \
  /usr/bin/python3 -B -m unittest -v tests.test_runtime_manifest
```

Expected: import failure for `tools.runtime_manifest` because it does not exist.

- [ ] **Step 3: Implement the minimum parser**

```python
# tools/runtime_manifest.py
import json
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from typing import Dict, Optional, Tuple


class ManifestError(ValueError):
    pass


def _relative_path(value: str) -> str:
    path = PurePosixPath(value)
    if path.is_absolute() or ".." in path.parts:
        raise ManifestError("relative path escape: {}".format(value))
    return path.as_posix()


@dataclass(frozen=True)
class PackageSpec:
    name: str
    role: str
    source: Optional[str]
    destination: str
    imported: bool


@dataclass(frozen=True)
class AuxiliaryImport:
    name: str
    source: str
    destination: str


@dataclass(frozen=True)
class RuntimeManifest:
    schema_version: int
    expected_commit: str
    required_paths: Tuple[str, ...]
    packages: Dict[str, PackageSpec]
    auxiliary_imports: Tuple[AuxiliaryImport, ...]
    forbidden_packages: Tuple[str, ...]
    forbidden_tokens: Tuple[str, ...]

    @classmethod
    def load(cls, path: Path):
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        if data.get("schema_version") != 1:
            raise ManifestError("unsupported schema_version")
        packages = {}
        destinations = set()
        for name, raw in data["packages"].items():
            role = raw["role"]
            if role not in {"p450", "ground", "vendor", "platform", "demo"}:
                raise ManifestError("unsupported role: {}".format(role))
            destination = _relative_path(raw["destination"])
            if destination in destinations:
                raise ManifestError("duplicate destination: {}".format(destination))
            destinations.add(destination)
            source = raw.get("source")
            if source is not None:
                source = _relative_path(source)
            packages[name] = PackageSpec(
                name=name,
                role=role,
                source=source,
                destination=destination,
                imported=bool(raw["imported"]),
            )
        auxiliary = tuple(
            AuxiliaryImport(
                name=item["name"],
                source=_relative_path(item["source"]),
                destination=_relative_path(item["destination"]),
            )
            for item in data.get("auxiliary_imports", [])
        )
        all_destinations = destinations | {item.destination for item in auxiliary}
        if len(all_destinations) != len(destinations) + len(auxiliary):
            raise ManifestError("duplicate destination in auxiliary imports")
        return cls(
            schema_version=1,
            expected_commit=data["upstream"]["expected_commit"],
            required_paths=tuple(
                _relative_path(item) for item in data["layout"]["required_paths"]
            ),
            packages=packages,
            auxiliary_imports=auxiliary,
            forbidden_packages=tuple(data["forbidden"]["package_names"]),
            forbidden_tokens=tuple(data["forbidden"]["runtime_tokens"]),
        )
```

Create empty `tools/__init__.py` and `tests/__init__.py`.

- [ ] **Step 4: Run the test and verify GREEN**

Run the Step 2 command. Expected: four tests pass.

- [ ] **Step 5: Commit**

```bash
git add tools tests
git commit -m "test: define runtime source manifest contract"
```

### Task 2: Freeze the exact V1 source contract

**Files:**
- Create: `config/runtime_sources.json`
- Create: `tests/test_runtime_source_contract.py`

- [ ] **Step 1: Write the failing frozen-contract test**

```python
# tests/test_runtime_source_contract.py
import unittest
from pathlib import Path

from tools.runtime_manifest import RuntimeManifest

ROOT = Path(__file__).resolve().parents[1]
EXPECTED_PACKAGES = {
    "prometheus_msgs", "prometheus_gazebo", "prometheus_uav_control",
    "realsense_ros_gazebo", "livox_laser_gazebo_plugins",
    "brick_aerial_perception", "bunker_aubo_description",
    "bunker_aubo_gazebo", "bunker_aubo_moveit_config",
    "bunker_navigation", "brick_rgbd_perception", "brick_visual_pick",
    "brick_pick_demo", "ground_pick_orchestrator", "aubo_description",
    "bunker_description", "dh_ag95_description",
    "roboticsgroup_gazebo_plugins", "sim_platform_bringup",
    "ground_runtime_compat", "air_ground_pose_bridge",
    "air_ground_pick_demo",
}
FORBIDDEN_PACKAGES = {
    "paper_benchmark", "ground_aerial_benchmark", "system_baseline_freeze",
    "task_aware_approach", "aerial_traversability", "irm_base_placement",
    "aerial_ground_bridge",
}


class FrozenSourceContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.manifest = RuntimeManifest.load(ROOT / "config/runtime_sources.json")

    def test_exact_package_allowlist(self):
        self.assertEqual(EXPECTED_PACKAGES, set(self.manifest.packages))

    def test_exact_forbidden_packages(self):
        self.assertEqual(FORBIDDEN_PACKAGES, set(self.manifest.forbidden_packages))
        self.assertFalse(EXPECTED_PACKAGES & FORBIDDEN_PACKAGES)

    def test_upstream_commit_is_actual_audited_checkout(self):
        self.assertEqual(
            "6809c15e3919d1aa3acb6518ad61c49e4150435f",
            self.manifest.expected_commit,
        )

    def test_imported_packages_have_sources(self):
        for package in self.manifest.packages.values():
            if package.imported:
                self.assertIsNotNone(package.source, package.name)
            else:
                self.assertIsNone(package.source, package.name)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run and verify RED**

```bash
env -i \
  /usr/bin/python3 -B -m unittest -v tests.test_runtime_source_contract
```

Expected: `config/runtime_sources.json` is missing.

- [ ] **Step 3: Add the exact JSON contract**

Create `config/runtime_sources.json` with schema version 1, audited commit `6809c15e3919d1aa3acb6518ad61c49e4150435f`, these required paths:

```json
[
  "dependencies", "src/p450", "src/ground", "src/vendor",
  "src/platform", "src/demos", "config", "tools", "tests", "docs"
]
```

For avoidance of ambiguity, the complete file content is:

~~~json
{
  "schema_version": 1,
  "upstream": {
    "expected_commit": "6809c15e3919d1aa3acb6518ad61c49e4150435f"
  },
  "layout": {
    "required_paths": [
      "dependencies",
      "src/p450",
      "src/ground",
      "src/vendor",
      "src/platform",
      "src/demos",
      "config",
      "tools",
      "tests",
      "docs"
    ]
  },
  "packages": {
    "prometheus_msgs": {"role": "p450", "source": "Modules/common/prometheus_msgs", "destination": "src/p450/prometheus_msgs", "imported": true},
    "prometheus_gazebo": {"role": "p450", "source": "Simulator/gazebo_simulator", "destination": "src/p450/prometheus_gazebo", "imported": true},
    "prometheus_uav_control": {"role": "p450", "source": "Modules/uav_control", "destination": "src/p450/prometheus_uav_control", "imported": true},
    "realsense_ros_gazebo": {"role": "p450", "source": "Simulator/realsense_gazebo_plugin", "destination": "src/p450/realsense_ros_gazebo", "imported": true},
    "livox_laser_gazebo_plugins": {"role": "p450", "source": "Simulator/livox_laser_gazebo_plugins", "destination": "src/p450/livox_laser_gazebo_plugins", "imported": true},
    "brick_aerial_perception": {"role": "p450", "source": "Modules/brick_aerial_perception", "destination": "src/p450/brick_aerial_perception", "imported": true},
    "bunker_aubo_description": {"role": "ground", "source": "Ground/src/bunker_aubo_project/bunker_aubo_description", "destination": "src/ground/bunker_aubo_description", "imported": true},
    "bunker_aubo_gazebo": {"role": "ground", "source": "Ground/src/bunker_aubo_project/bunker_aubo_gazebo", "destination": "src/ground/bunker_aubo_gazebo", "imported": true},
    "bunker_aubo_moveit_config": {"role": "ground", "source": "Ground/src/bunker_aubo_project/bunker_aubo_moveit_config", "destination": "src/ground/bunker_aubo_moveit_config", "imported": true},
    "bunker_navigation": {"role": "ground", "source": "Ground/src/bunker_aubo_project/bunker_navigation", "destination": "src/ground/bunker_navigation", "imported": true},
    "brick_rgbd_perception": {"role": "ground", "source": "Ground/src/bunker_aubo_project/brick_rgbd_perception", "destination": "src/ground/brick_rgbd_perception", "imported": true},
    "brick_visual_pick": {"role": "ground", "source": "Ground/src/bunker_aubo_project/brick_visual_pick", "destination": "src/ground/brick_visual_pick", "imported": true},
    "brick_pick_demo": {"role": "ground", "source": "Ground/src/bunker_aubo_project/brick_pick_demo", "destination": "src/ground/brick_pick_demo", "imported": true},
    "ground_pick_orchestrator": {"role": "ground", "source": "Ground/src/bunker_aubo_project/ground_pick_orchestrator", "destination": "src/ground/ground_pick_orchestrator", "imported": true},
    "aubo_description": {"role": "vendor", "source": "Ground/src/third_party/aubo_robot_melodic/aubo_description", "destination": "src/vendor/aubo_description", "imported": true},
    "bunker_description": {"role": "vendor", "source": "Ground/src/third_party/ugv_gazebo_sim/bunker/bunker_description", "destination": "src/vendor/bunker_description", "imported": true},
    "dh_ag95_description": {"role": "vendor", "source": "Ground/src/third_party/scout_cobot_sim/dh_ag95_description", "destination": "src/vendor/dh_ag95_description", "imported": true},
    "roboticsgroup_gazebo_plugins": {"role": "vendor", "source": "Ground/src/third_party/roboticsgroup_gazebo_plugins", "destination": "src/vendor/roboticsgroup_gazebo_plugins", "imported": true},
    "sim_platform_bringup": {"role": "platform", "source": null, "destination": "src/platform/sim_platform_bringup", "imported": false},
    "ground_runtime_compat": {"role": "platform", "source": null, "destination": "src/platform/ground_runtime_compat", "imported": false},
    "air_ground_pose_bridge": {"role": "platform", "source": null, "destination": "src/platform/air_ground_pose_bridge", "imported": false},
    "air_ground_pick_demo": {"role": "demo", "source": null, "destination": "src/demos/air_ground_pick_demo", "imported": false}
  },
  "auxiliary_imports": [
    {"name": "geometry_utils", "source": "Modules/common/include/geometry_utils.h", "destination": "src/p450/prometheus_uav_control/vendor_upstream/common/include/geometry_utils.h"},
    {"name": "math_utils", "source": "Modules/common/include/math_utils.h", "destination": "src/p450/prometheus_uav_control/vendor_upstream/common/include/math_utils.h"},
    {"name": "printf_utils", "source": "Modules/common/include/printf_utils.h", "destination": "src/p450/prometheus_uav_control/vendor_upstream/common/include/printf_utils.h"},
    {"name": "param_manager_header", "source": "Modules/communication/include/param_manager.hpp", "destination": "src/p450/prometheus_uav_control/vendor_upstream/communication/include/param_manager.hpp"},
    {"name": "message_convert_header", "source": "Modules/communication/include/message_convert.hpp", "destination": "src/p450/prometheus_uav_control/vendor_upstream/communication/include/message_convert.hpp"},
    {"name": "param_manager_source", "source": "Modules/communication/src/param_manager.cpp", "destination": "src/p450/prometheus_uav_control/vendor_upstream/communication/src/param_manager.cpp"},
    {"name": "upstream_license", "source": "LICENSE", "destination": "docs/upstream/P450-SIM-LICENSE"},
    {"name": "third_party_licenses", "source": "THIRD_PARTY_LICENSES.md", "destination": "docs/upstream/THIRD_PARTY_LICENSES.md"},
    {"name": "ground_upstreams", "source": "Ground/upstream.repos", "destination": "docs/upstream/Ground-upstream.repos"},
    {"name": "source_provenance", "source": "docs/SOURCE_PROVENANCE.md", "destination": "docs/upstream/SOURCE_PROVENANCE.md"},
    {"name": "historical_source_lock", "source": "manifests/source-lock.json", "destination": "docs/upstream/historical-source-lock.json"}
  ],
  "forbidden": {
    "package_names": [
      "paper_benchmark",
      "ground_aerial_benchmark",
      "system_baseline_freeze",
      "task_aware_approach",
      "aerial_traversability",
      "irm_base_placement",
      "aerial_ground_bridge"
    ],
    "runtime_tokens": ["/m5/", "m5_", "rbp_", "task_aware", "task-aware"]
  }
}
~~~

- [ ] **Step 4: Run and verify GREEN**

Run the Step 2 command. Expected: four tests pass.

- [ ] **Step 5: Commit**

```bash
git add config tests/test_runtime_source_contract.py
git commit -m "build: freeze V1 runtime source allowlist"
```

### Task 3: Safe dry-run importer and provenance writer

**Files:**
- Create: `tools/import_runtime.py`
- Create: `tests/test_import_runtime.py`

- [ ] **Step 1: Write failing behavior tests**

Create `tests/test_import_runtime.py`:

~~~python
import json
import subprocess
import tempfile
import unittest
from pathlib import Path

from tools.import_runtime import (
    RuntimeImportError,
    apply_import_plan,
    build_import_plan,
    validate_source_tree,
    verify_upstream_commit,
)
from tools.runtime_manifest import RuntimeManifest


class ImportRuntimeTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.base = Path(self.temporary.name)
        self.upstream = self.base / "upstream"
        self.destination = self.base / "destination"
        self.source_tree = self.upstream / "Modules/demo"
        self.source_tree.mkdir(parents=True)
        self.destination.mkdir()
        (self.source_tree / "package.xml").write_text(
            "<package><name>demo</name></package>\n", encoding="utf-8"
        )
        (self.source_tree / "live.txt").write_text("runtime\n", encoding="utf-8")
        self.git("init")
        self.git("add", ".")
        self.git(
            "-c", "user.name=Platform Test",
            "-c", "user.email=platform-test@example.invalid",
            "commit", "-m", "fixture",
        )
        self.commit = self.git("rev-parse", "HEAD").strip()
        self.payload = {
            "schema_version": 1,
            "upstream": {"expected_commit": self.commit},
            "layout": {"required_paths": ["src/p450"]},
            "packages": {
                "demo": {
                    "role": "p450",
                    "source": "Modules/demo",
                    "destination": "src/p450/demo",
                    "imported": True,
                }
            },
            "auxiliary_imports": [],
            "forbidden": {"package_names": [], "runtime_tokens": []},
        }
        config = self.destination / "config"
        config.mkdir()
        self.manifest_path = config / "runtime_sources.json"
        self.manifest_path.write_text(
            json.dumps(self.payload), encoding="utf-8"
        )

    def git(self, *arguments):
        return subprocess.run(
            ["/usr/bin/git", "-C", str(self.upstream), *arguments],
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        ).stdout

    def manifest(self):
        return RuntimeManifest.load(self.manifest_path)

    def test_dry_plan_does_not_copy(self):
        plan = build_import_plan(
            self.manifest(), self.upstream, self.destination
        )
        self.assertFalse((self.destination / "src/p450/demo").exists())
        self.assertEqual(
            "Modules/demo",
            plan[0].source.relative_to(self.upstream).as_posix(),
        )

    def test_rejects_commit_mismatch(self):
        with self.assertRaisesRegex(RuntimeImportError, "commit mismatch"):
            verify_upstream_commit(self.upstream, "b" * 40)

    def test_rejects_external_symlink(self):
        outside = self.base / "outside.txt"
        outside.write_text("outside\n", encoding="utf-8")
        (self.source_tree / "external").symlink_to(outside)
        with self.assertRaisesRegex(RuntimeImportError, "external symlink"):
            validate_source_tree(self.upstream, self.source_tree)

    def test_rejects_existing_destination_before_copy(self):
        plan = build_import_plan(
            self.manifest(), self.upstream, self.destination
        )
        plan[0].destination.mkdir(parents=True)
        with self.assertRaisesRegex(RuntimeImportError, "destination exists"):
            apply_import_plan(plan, self.destination, self.commit)

    def test_apply_copies_runtime_and_ignores_generated_trees(self):
        for relative in (
            ".git/marker", "build/marker", "devel/marker",
            "docs/results/marker", "__pycache__/marker",
        ):
            path = self.source_tree / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text("ignored\n", encoding="utf-8")
        plan = build_import_plan(
            self.manifest(), self.upstream, self.destination
        )
        provenance = apply_import_plan(plan, self.destination, self.commit)
        imported = self.destination / "src/p450/demo"
        self.assertEqual("runtime\n", (imported / "live.txt").read_text())
        for relative in (
            ".git", "build", "devel", "docs/results", "__pycache__",
        ):
            self.assertFalse((imported / relative).exists(), relative)
        self.assertEqual(self.commit, provenance["upstream_commit"])
        self.assertIn("src/p450/demo/live.txt", provenance["files"])
        self.assertTrue(
            (self.destination / "config/import_provenance.json").is_file()
        )


if __name__ == "__main__":
    unittest.main()
~~~

- [ ] **Step 2: Run and verify RED**

```bash
env -i \
  /usr/bin/python3 -B -m unittest -v tests.test_import_runtime
```

Expected: import failure because `tools.import_runtime` does not exist.

- [ ] **Step 3: Implement the minimum importer**

Create `tools/import_runtime.py` with this implementation:

~~~python
import argparse
import hashlib
import json
import shutil
import subprocess
from dataclasses import dataclass
from pathlib import Path

from tools.runtime_manifest import RuntimeManifest


class RuntimeImportError(RuntimeError):
    pass


@dataclass(frozen=True)
class ImportItem:
    name: str
    source: Path
    destination: Path


def _is_within(path: Path, root: Path) -> bool:
    try:
        path.relative_to(root)
        return True
    except ValueError:
        return False


def verify_upstream_commit(upstream: Path, expected: str) -> None:
    result = subprocess.run(
        ["/usr/bin/git", "-C", str(upstream), "rev-parse", "HEAD"],
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    actual = result.stdout.strip()
    if actual != expected:
        raise RuntimeImportError(
            "commit mismatch: expected {}, got {}".format(expected, actual)
        )


def validate_source_tree(upstream: Path, source: Path) -> None:
    upstream = upstream.resolve(strict=True)
    resolved_source = source.resolve(strict=True)
    if not _is_within(resolved_source, upstream):
        raise RuntimeImportError("source escapes upstream: {}".format(source))
    candidates = (source,) if source.is_file() else source.rglob("*")
    for candidate in candidates:
        if candidate.is_symlink():
            resolved = candidate.resolve(strict=True)
            if not _is_within(resolved, upstream):
                raise RuntimeImportError(
                    "external symlink: {} -> {}".format(candidate, resolved)
                )


def build_import_plan(manifest, upstream: Path, destination: Path) -> tuple:
    upstream = upstream.resolve(strict=True)
    destination = destination.resolve(strict=True)
    items = []
    for package in manifest.packages.values():
        if not package.imported:
            continue
        items.append(
            ImportItem(
                package.name,
                upstream / package.source,
                destination / package.destination,
            )
        )
    for auxiliary in manifest.auxiliary_imports:
        items.append(
            ImportItem(
                auxiliary.name,
                upstream / auxiliary.source,
                destination / auxiliary.destination,
            )
        )
    for item in items:
        validate_source_tree(upstream, item.source)
        if not _is_within(item.destination.parent.resolve(), destination):
            raise RuntimeImportError(
                "destination escapes project: {}".format(item.destination)
            )
    return tuple(items)


def _copy_ignore(directory, names):
    ignored = {
        name
        for name in names
        if name in {".git", "build", "devel", "__pycache__", ".pytest_cache"}
    }
    if Path(directory).name == "docs" and "results" in names:
        ignored.add("results")
    return ignored


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def apply_import_plan(plan, destination: Path, upstream_commit: str) -> dict:
    destination = destination.resolve(strict=True)
    existing = [item.destination for item in plan if item.destination.exists()]
    if existing:
        raise RuntimeImportError(
            "destination exists: {}".format(
                ", ".join(str(path) for path in existing)
            )
        )
    for item in plan:
        item.destination.parent.mkdir(parents=True, exist_ok=True)
        if item.source.is_dir():
            shutil.copytree(
                str(item.source),
                str(item.destination),
                symlinks=False,
                ignore=_copy_ignore,
            )
        else:
            shutil.copy2(str(item.source), str(item.destination))
    files = {}
    for item in plan:
        candidates = (
            tuple(item.destination.rglob("*"))
            if item.destination.is_dir()
            else (item.destination,)
        )
        for path in candidates:
            if path.is_file():
                relative = path.relative_to(destination).as_posix()
                files[relative] = sha256_file(path)
    provenance = {
        "schema_version": 1,
        "upstream_commit": upstream_commit,
        "files": dict(sorted(files.items())),
    }
    provenance_path = destination / "config/import_provenance.json"
    provenance_path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = destination / "config/.import_provenance.json.tmp"
    temporary_path.write_text(
        json.dumps(provenance, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    temporary_path.replace(provenance_path)
    return provenance


def main(argv=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--source-root", required=True, type=Path)
    parser.add_argument("--destination-root", required=True, type=Path)
    parser.add_argument("--apply", action="store_true")
    args = parser.parse_args(argv)
    manifest = RuntimeManifest.load(args.manifest)
    verify_upstream_commit(args.source_root, manifest.expected_commit)
    plan = build_import_plan(manifest, args.source_root, args.destination_root)
    if not args.apply:
        for item in plan:
            print("COPY {} -> {}".format(item.source, item.destination))
        return 0
    apply_import_plan(plan, args.destination_root, manifest.expected_commit)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
~~~

- [ ] **Step 4: Run and verify GREEN**

Run the Step 2 command. Expected: all importer tests pass.

- [ ] **Step 5: Verify the real import plan without writing**

```bash
env -i \
  /usr/bin/python3 -B -m tools.import_runtime \
  --manifest config/runtime_sources.json \
  --source-root /media/lu/P450_PAPER/P450-PAPER/source/P450-SIM \
  --destination-root /media/lu/P450_PAPER/SIM/p450_sim_v1
```

Expected: one COPY line for every inherited package and auxiliary file, no filesystem changes, exit zero.

- [ ] **Step 6: Commit**

```bash
git add tools/import_runtime.py tests/test_import_runtime.py
git commit -m "feat: add safe runtime source importer"
```

### Task 4: Import the selected source snapshot

**Files:**
- Create: imported package trees declared by `config/runtime_sources.json`
- Create: `config/import_provenance.json`
- Create: `tests/test_imported_layout.py`

- [ ] **Step 1: Write the failing imported-layout test**

Create `tests/test_imported_layout.py`:

~~~python
import json
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from tools.runtime_manifest import RuntimeManifest


ROOT = Path(__file__).resolve().parents[1]


class ImportedLayoutTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.manifest = RuntimeManifest.load(ROOT / "config/runtime_sources.json")

    def test_all_inherited_packages_exist_with_expected_names(self):
        for name, spec in self.manifest.packages.items():
            if not spec.imported:
                continue
            package_root = ROOT / spec.destination
            self.assertTrue(package_root.is_dir(), spec.destination)
            actual_name = ET.parse(
                str(package_root / "package.xml")
            ).getroot().findtext("name")
            self.assertEqual(name, actual_name, spec.destination)

    def test_auxiliary_imports_exist(self):
        for item in self.manifest.auxiliary_imports:
            self.assertTrue((ROOT / item.destination).is_file(), item.destination)

    def test_imports_contain_no_generated_or_nested_git_trees(self):
        forbidden_names = {".git", "build", "devel", "__pycache__"}
        for spec in self.manifest.packages.values():
            if not spec.imported:
                continue
            package_root = ROOT / spec.destination
            for path in package_root.rglob("*"):
                relative = path.relative_to(package_root)
                self.assertFalse(
                    forbidden_names & set(relative.parts),
                    relative.as_posix(),
                )
                self.assertNotIn(
                    ("docs", "results"),
                    tuple(zip(relative.parts, relative.parts[1:])),
                    relative.as_posix(),
                )

    def test_imported_symlinks_are_internal_and_resolvable(self):
        root = ROOT.resolve()
        for spec in self.manifest.packages.values():
            if not spec.imported:
                continue
            for path in (ROOT / spec.destination).rglob("*"):
                if not path.is_symlink():
                    continue
                self.assertFalse(Path(path.readlink()).is_absolute(), str(path))
                resolved = path.resolve(strict=True)
                try:
                    resolved.relative_to(root)
                except ValueError:
                    self.fail("symlink escapes project: {} -> {}".format(path, resolved))

    def test_provenance_covers_all_imported_regular_files(self):
        provenance = json.loads(
            (ROOT / "config/import_provenance.json").read_text(encoding="utf-8")
        )
        self.assertEqual(self.manifest.expected_commit, provenance["upstream_commit"])
        recorded = set(provenance["files"])
        for spec in self.manifest.packages.values():
            if not spec.imported:
                continue
            for path in (ROOT / spec.destination).rglob("*"):
                if path.is_file():
                    self.assertIn(path.relative_to(ROOT).as_posix(), recorded)
        for item in self.manifest.auxiliary_imports:
            self.assertIn(item.destination, recorded)


if __name__ == "__main__":
    unittest.main()
~~~

- [ ] **Step 2: Run and verify RED**

```bash
env -i \
  /usr/bin/python3 -B -m unittest -v tests.test_imported_layout
```

Expected: selected destination directories are missing.

- [ ] **Step 3: Apply the audited import once**

```bash
env -i \
  /usr/bin/python3 -B -m tools.import_runtime \
  --manifest config/runtime_sources.json \
  --source-root /media/lu/P450_PAPER/P450-PAPER/source/P450-SIM \
  --destination-root /media/lu/P450_PAPER/SIM/p450_sim_v1 \
  --apply
```

Expected: 18 catkin packages and eleven auxiliary files copied, provenance written, parent repository unchanged.

- [ ] **Step 4: Run and verify GREEN**

Run the Step 2 command, then run:

```bash
git -C /media/lu/P450_PAPER/P450-PAPER/source/P450-SIM status --short
```

Expected: imported-layout tests pass and upstream status is empty.

- [ ] **Step 5: Commit**

```bash
git add src config/import_provenance.json tests/test_imported_layout.py
git commit -m "build: import minimal V1 runtime sources"
```

### Task 5: Offline research-boundary gate

**Files:**
- Create: `tools/runtime_boundary.py`
- Create: `tests/test_runtime_boundary.py`
- Create: `.gitignore`
- Create: `README.md`

- [ ] **Step 1: Write failing boundary tests**

Create `tests/test_runtime_boundary.py`:

~~~python
import tempfile
import unittest
from pathlib import Path

from tools.runtime_boundary import (
    active_runtime_files,
    discover_packages,
    scan_forbidden_dependencies,
    scan_forbidden_model_lifecycle,
    scan_forbidden_tokens,
)
from tools.runtime_manifest import RuntimeManifest


ROOT = Path(__file__).resolve().parents[1]
NEW_PACKAGES = {
    "sim_platform_bringup",
    "ground_runtime_compat",
    "air_ground_pose_bridge",
    "air_ground_pick_demo",
}


class RuntimeBoundaryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.manifest = RuntimeManifest.load(ROOT / "config/runtime_sources.json")
        cls.package_index = discover_packages(ROOT, cls.manifest)
        cls.runtime_files = active_runtime_files(ROOT, cls.manifest)

    def test_only_inherited_allowlist_packages_are_present(self):
        expected = set(self.manifest.packages) - NEW_PACKAGES
        self.assertEqual(expected, set(self.package_index))
        self.assertFalse(
            set(self.package_index) & set(self.manifest.forbidden_packages)
        )

    def test_no_runtime_package_depends_on_research_packages(self):
        self.assertEqual(
            [],
            scan_forbidden_dependencies(self.package_index, self.manifest),
        )

    def test_no_active_runtime_file_contains_method_tokens(self):
        self.assertEqual(
            [],
            scan_forbidden_tokens(self.runtime_files, self.manifest),
        )

    def test_no_active_runtime_file_deletes_or_respawns_models(self):
        self.assertEqual(
            [],
            scan_forbidden_model_lifecycle(self.runtime_files),
        )

    def test_lifecycle_scanner_ignores_comments_but_finds_calls(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        path = Path(temporary.name) / "node.py"
        path.write_text(
            "# /gazebo/delete_model is forbidden\n"
            "service = '/gazebo/delete_model'\n",
            encoding="utf-8",
        )
        findings = scan_forbidden_model_lifecycle((path,))
        self.assertEqual(1, len(findings))
        self.assertEqual(2, findings[0].line)


if __name__ == "__main__":
    unittest.main()
~~~

The scanner must:

- Inspect active `package.xml`, `CMakeLists.txt`, `.launch`, `.xml`, `.xacro`, `.sdf`, `.jinja`, `.py`, `.cpp`, `.hpp`, `.h`, `.yaml`, and `.json` files.
- Exclude `docs`, `tests`, licenses, provenance, generated caches, and comments when checking model lifecycle calls.
- Match forbidden package names as identifier boundaries.
- Match `/m5/`, `m5_`, `rbp_`, `task_aware`, and `task-aware` exactly as configured.
- Reject `/gazebo/delete_model`, `DeleteModel`, runtime `/gazebo/spawn_urdf_model`, runtime `/gazebo/spawn_sdf_model`, and model-respawn helper identifiers.
- Allow `gazebo_ros/spawn_model` only in later manifest-declared startup launch files; no startup launches exist in this foundation milestone.

- [ ] **Step 2: Run and verify RED**

```bash
env -i \
  /usr/bin/python3 -B -m unittest -v tests.test_runtime_boundary
```

Expected: import failure because `tools.runtime_boundary` does not exist.

- [ ] **Step 3: Implement the minimal scanner**

Create `tools/runtime_boundary.py` with this implementation:

~~~python
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path


RUNTIME_SUFFIXES = {
    ".launch", ".xml", ".xacro", ".sdf", ".jinja", ".py", ".cpp",
    ".hpp", ".h", ".yaml", ".yml", ".json",
}
DEPENDENCY_TAGS = {
    "depend", "build_depend", "build_export_depend", "exec_depend",
    "run_depend", "test_depend",
}
LIFECYCLE_TOKENS = {
    "/gazebo/delete_model", "DeleteModel", "/gazebo/spawn_urdf_model",
    "/gazebo/spawn_sdf_model", "respawn_model", "delete_respawn",
}


@dataclass(frozen=True)
class Finding:
    path: Path
    line: int
    token: str


def discover_packages(root: Path, manifest) -> dict:
    packages = {}
    for spec in manifest.packages.values():
        if not spec.imported:
            continue
        package_root = root / spec.destination
        package_xml = package_root / "package.xml"
        document = ET.parse(str(package_xml))
        actual_name = document.getroot().findtext("name")
        if actual_name != spec.name:
            raise ValueError(
                "package name mismatch at {}: expected {}, got {}".format(
                    package_xml, spec.name, actual_name
                )
            )
        if actual_name in packages:
            raise ValueError("duplicate package: {}".format(actual_name))
        packages[actual_name] = package_root
    return packages


def active_runtime_files(root: Path, manifest) -> tuple:
    paths = []
    for spec in manifest.packages.values():
        if not spec.imported:
            continue
        package_root = root / spec.destination
        for path in package_root.rglob("*"):
            if not path.is_file():
                continue
            relative_parts = path.relative_to(package_root).parts
            if any(
                part in {"docs", "test", "tests", "results", "__pycache__"}
                for part in relative_parts
            ):
                continue
            if path.name == "CMakeLists.txt" or path.suffix in RUNTIME_SUFFIXES:
                paths.append(path)
    return tuple(sorted(paths))


def _line_findings(path: Path, tokens) -> list:
    findings = []
    text = path.read_text(encoding="utf-8", errors="replace")
    for number, line in enumerate(text.splitlines(), start=1):
        for token in tokens:
            if token in line:
                findings.append(Finding(path, number, token))
    return findings


def scan_forbidden_dependencies(package_index, manifest) -> list:
    findings = []
    forbidden = set(manifest.forbidden_packages)
    for package_root in package_index.values():
        package_xml = package_root / "package.xml"
        document = ET.parse(str(package_xml))
        dependencies = {
            element.text.strip()
            for element in document.getroot()
            if element.tag in DEPENDENCY_TAGS and element.text
        }
        matches = dependencies & forbidden
        for token in sorted(matches):
            findings.extend(_line_findings(package_xml, (token,)))
    return sorted(findings, key=lambda item: (str(item.path), item.line, item.token))


def scan_forbidden_tokens(paths, manifest) -> list:
    findings = []
    for path in paths:
        for number, line in enumerate(_without_comments(path), start=1):
            for token in manifest.forbidden_tokens:
                if token.startswith("/") or token.endswith("_") or "-" in token:
                    matched = token in line
                else:
                    pattern = r"(?<![A-Za-z0-9_]){}(?![A-Za-z0-9_])".format(
                        re.escape(token)
                    )
                    matched = re.search(pattern, line) is not None
                if matched:
                    findings.append(Finding(path, number, token))
    return sorted(findings, key=lambda item: (str(item.path), item.line, item.token))


def _without_comments(path: Path) -> tuple:
    text = path.read_text(encoding="utf-8", errors="replace")
    text = re.sub(r"<!--.*?-->", "", text, flags=re.DOTALL)
    lines = []
    for line in text.splitlines():
        stripped = line.lstrip()
        if stripped.startswith(("#", "//", "*")):
            lines.append("")
            continue
        line = re.sub(r"//.*$", "", line)
        line = re.sub(r"#.*$", "", line)
        lines.append(line)
    return tuple(lines)


def scan_forbidden_model_lifecycle(paths) -> list:
    findings = []
    for path in paths:
        for number, line in enumerate(_without_comments(path), start=1):
            for token in LIFECYCLE_TOKENS:
                if token in line:
                    findings.append(Finding(path, number, token))
    return sorted(findings, key=lambda item: (str(item.path), item.line, item.token))
~~~

The code uses `xml.etree.ElementTree` for dependency elements and deterministic line-oriented findings for runtime files.

- [ ] **Step 4: Run RED against imported sources and classify findings**

Run the Step 2 command. If the real-tree assertions report inherited research coupling, keep the test RED and remove that coupling in this task by narrowing the import to runtime files or by applying a minimal copied-tree patch under SIM. Do not weaken forbidden tokens, do not defer a failing boundary test, and do not import a forbidden package to satisfy a dependency.

- [ ] **Step 5: Add project boundary documentation and ignore rules**

`.gitignore` must contain only generated/local outputs:

```gitignore
build/
devel/
install/
logs/
.catkin_tools/
__pycache__/
*.pyc
.import-staging-*/
```

Create `README.md` exactly as follows:

~~~~markdown
# P450 Simulation Platform V1

This repository is the clean runtime workspace for a stable P450 + BUNKER +
AUBO i5 + AG95 Gazebo platform and its minimal Air-Ground Pick demo.

## Source boundary

`/media/lu/P450_PAPER/P450-PAPER` is immutable upstream material. Source enters
this repository only through `config/runtime_sources.json` and the dry-run-first
`tools.import_runtime` importer. The import provenance file binds every copied
regular file to the audited upstream commit.

Paper benchmark, Ground/Aerial benchmark, M5, RBP/IRM, Task-aware, Pilot, T2,
Formal, paper lifecycle, metrics, evidence, and result artifacts are outside the
V1 runtime boundary and are not built or launched here.

## Target stack

- Ubuntu 20.04
- ROS Noetic
- Gazebo Classic 11
- PX4 SITL at commit `713814f4eea5990e49dd776a38a36ad53e171f60`
- MAVROS 1.20.1
- MoveIt 1.1.16

## Offline foundation gate

~~~bash
env -i /usr/bin/python3 -B -m unittest discover -s tests -v
~~~

Passing the foundation gate proves only source selection, provenance, and the
non-paper boundary. It does not claim that ROS packages build, robots launch,
the combined world is stable, or the pick demo succeeds; those claims require
their later fresh build and runtime verification gates.
~~~~

- [ ] **Step 6: Run the full foundation gate**

```bash
env -i \
  /usr/bin/python3 -B -m unittest discover -s tests -v
```

Expected: all foundation tests pass with no cache files created.

- [ ] **Step 7: Commit**

```bash
git add .gitignore README.md tools/runtime_boundary.py tests/test_runtime_boundary.py
git commit -m "test: enforce non-paper runtime boundary"
```

## Foundation completion gate

Before starting the P450 clean-build plan, independently verify:

```bash
git status --short
env -i \
  /usr/bin/python3 -B -m unittest discover -s tests -v
git -C /media/lu/P450_PAPER/P450-PAPER/source/P450-SIM status --short
```

Required evidence:

- Feature branch is clean.
- All offline foundation tests pass.
- Upstream repository remains clean.
- No imported symlink resolves outside `p450_sim_v1`.
- No forbidden package is present or referenced by active runtime files.
- `config/import_provenance.json` binds every copied file to upstream commit `6809c15e3919d1aa3acb6518ad61c49e4150435f`.

After this gate, write and execute `2026-08-31-p450-clean-build.md`; do not begin Ground or shared-world implementation in the same unreviewed change.
