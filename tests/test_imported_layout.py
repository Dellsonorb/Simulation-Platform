import hashlib
import json
import stat
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from tools.runtime_manifest import RuntimeManifest


ROOT = Path(__file__).resolve().parents[1]
EXPECTED_COMMIT = "6809c15e3919d1aa3acb6518ad61c49e4150435f"
FORBIDDEN_PATH_PARTS = {
    ".git",
    "build",
    "devel",
    "__pycache__",
    ".pytest_cache",
}


def _is_regular_file(path):
    return stat.S_ISREG(path.lstat().st_mode)


def _sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


class ImportedLayoutTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.manifest = RuntimeManifest.load(
            ROOT / "config/runtime_sources.json"
        )
        cls.imported_packages = tuple(
            spec
            for spec in cls.manifest.packages.values()
            if spec.imported
        )
        cls.neutral_packages = tuple(
            spec
            for spec in cls.manifest.packages.values()
            if not spec.imported
        )

    def test_manifest_declares_exact_import_counts(self):
        self.assertEqual(18, len(self.imported_packages))
        self.assertEqual(11, len(self.manifest.auxiliary_imports))
        self.assertEqual(4, len(self.neutral_packages))

    def test_all_required_paths_are_directories(self):
        self.assertEqual(10, len(self.manifest.required_paths))
        for relative in self.manifest.required_paths:
            self.assertTrue((ROOT / relative).is_dir(), relative)

    def test_imported_packages_exist_with_exact_package_names(self):
        for spec in self.imported_packages:
            package_root = ROOT / spec.destination
            self.assertTrue(package_root.is_dir(), spec.destination)
            package_xml = package_root / "package.xml"
            self.assertTrue(package_xml.is_file(), package_xml.as_posix())
            actual_name = ET.parse(str(package_xml)).getroot().findtext(
                "name"
            )
            self.assertEqual(spec.name, actual_name, spec.destination)

    def test_neutral_packages_are_not_materialized(self):
        for spec in self.neutral_packages:
            self.assertFalse((ROOT / spec.destination).exists(), spec.name)

    def test_auxiliary_destinations_are_regular_files(self):
        for item in self.manifest.auxiliary_imports:
            path = ROOT / item.destination
            self.assertTrue(path.is_file(), item.destination)
            self.assertTrue(_is_regular_file(path), item.destination)

    def test_imported_trees_exclude_generated_and_research_results(self):
        for spec in self.imported_packages:
            package_root = ROOT / spec.destination
            for path in package_root.rglob("*"):
                relative = path.relative_to(package_root)
                self.assertTrue(
                    FORBIDDEN_PATH_PARTS.isdisjoint(relative.parts),
                    relative.as_posix(),
                )
                adjacent_parts = tuple(
                    zip(relative.parts, relative.parts[1:])
                )
                self.assertNotIn(
                    ("docs", "results"),
                    adjacent_parts,
                    relative.as_posix(),
                )

    def test_imported_packages_and_auxiliaries_contain_no_symlinks(self):
        for spec in self.imported_packages:
            package_root = ROOT / spec.destination
            self.assertFalse(package_root.is_symlink(), spec.destination)
            for path in package_root.rglob("*"):
                self.assertFalse(
                    path.is_symlink(),
                    path.relative_to(ROOT).as_posix(),
                )
        for item in self.manifest.auxiliary_imports:
            path = ROOT / item.destination
            self.assertFalse(path.is_symlink(), item.destination)

    def test_provenance_exactly_covers_and_hashes_imported_files(self):
        provenance_path = ROOT / "config/import_provenance.json"
        self.assertTrue(provenance_path.is_file(), provenance_path.as_posix())
        provenance = json.loads(provenance_path.read_text(encoding="utf-8"))

        self.assertEqual(1, provenance["schema_version"])
        self.assertEqual(EXPECTED_COMMIT, provenance["upstream_commit"])
        self.assertEqual(
            EXPECTED_COMMIT,
            self.manifest.expected_commit,
        )
        recorded_files = provenance["files"]
        self.assertEqual(
            list(recorded_files),
            sorted(recorded_files),
            "provenance file keys are not sorted",
        )

        expected_files = set()
        for spec in self.imported_packages:
            package_root = ROOT / spec.destination
            for path in package_root.rglob("*"):
                if _is_regular_file(path):
                    expected_files.add(path.relative_to(ROOT).as_posix())
        for item in self.manifest.auxiliary_imports:
            path = ROOT / item.destination
            if _is_regular_file(path):
                expected_files.add(item.destination)

        self.assertEqual(expected_files, set(recorded_files))
        self.assertNotIn(
            "config/import_provenance.json",
            recorded_files,
        )
        for marker in (
            "dependencies/.gitkeep",
            "src/platform/.gitkeep",
            "src/demos/.gitkeep",
        ):
            self.assertNotIn(marker, recorded_files)
        for relative, expected_digest in recorded_files.items():
            self.assertEqual(
                expected_digest,
                _sha256(ROOT / relative),
                relative,
            )


if __name__ == "__main__":
    unittest.main()
