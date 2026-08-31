import contextlib
import hashlib
import io
import json
import subprocess
import tempfile
import unittest
from pathlib import Path

from tools.import_runtime import (
    ImportItem,
    RuntimeImportError,
    apply_import_plan,
    build_import_plan,
    main,
    sha256_file,
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
        (self.source_tree / "live.txt").write_text(
            "runtime\n", encoding="utf-8"
        )
        self.git("init")
        self.git("add", ".")
        self.git(
            "-c",
            "user.name=Platform Test",
            "-c",
            "user.email=platform-test@example.invalid",
            "commit",
            "-m",
            "fixture",
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
        (self.destination / "config").mkdir()
        self.manifest_path = (
            self.destination / "config/runtime_sources.json"
        )
        self.write_manifest()

    def git(self, *arguments):
        return subprocess.run(
            ["/usr/bin/git", "-C", str(self.upstream), *arguments],
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        ).stdout

    def write_manifest(self):
        self.manifest_path.write_text(
            json.dumps(self.payload), encoding="utf-8"
        )

    def manifest(self):
        return RuntimeManifest.load(self.manifest_path)

    def test_default_cli_is_a_dry_run(self):
        before = sorted(
            path.relative_to(self.destination).as_posix()
            for path in self.destination.rglob("*")
        )
        output = io.StringIO()

        with contextlib.redirect_stdout(output):
            result = main(
                [
                    "--manifest",
                    str(self.manifest_path),
                    "--source-root",
                    str(self.upstream),
                    "--destination-root",
                    str(self.destination),
                ]
            )

        after = sorted(
            path.relative_to(self.destination).as_posix()
            for path in self.destination.rglob("*")
        )
        self.assertEqual(0, result)
        self.assertEqual(before, after)
        self.assertEqual(1, output.getvalue().count("COPY "))
        self.assertIn("Modules/demo", output.getvalue())

    def test_build_plan_preserves_package_order_then_appends_auxiliary(self):
        second_source = self.upstream / "Modules/second"
        second_source.mkdir()
        (second_source / "package.xml").write_text(
            "<package><name>second</name></package>\n", encoding="utf-8"
        )
        helper = self.upstream / "Modules/common/helper.h"
        helper.parent.mkdir()
        helper.write_text("helper\n", encoding="utf-8")
        self.payload["packages"]["second"] = {
            "role": "p450",
            "source": "Modules/second",
            "destination": "src/p450/second",
            "imported": True,
        }
        self.payload["auxiliary_imports"] = [
            {
                "name": "helper",
                "source": "Modules/common/helper.h",
                "destination": "src/p450/demo/vendor/helper.h",
            }
        ]
        self.write_manifest()

        plan = build_import_plan(
            self.manifest(), self.upstream, self.destination
        )

        self.assertEqual(["demo", "second", "helper"], [x.name for x in plan])
        self.assertFalse((self.destination / "src").exists())

    def test_rejects_commit_mismatch(self):
        with self.assertRaisesRegex(RuntimeImportError, "commit mismatch"):
            verify_upstream_commit(self.upstream, "b" * 40)

    def test_rejects_external_symlink(self):
        outside = self.base / "outside.txt"
        outside.write_text("outside\n", encoding="utf-8")
        (self.source_tree / "external").symlink_to(outside)

        with self.assertRaisesRegex(RuntimeImportError, "external symlink"):
            validate_source_tree(self.upstream, self.source_tree)

    def test_rejects_dangling_source_symlink(self):
        (self.source_tree / "dangling").symlink_to(
            self.base / "missing-source"
        )

        with self.assertRaisesRegex(RuntimeImportError, "external symlink"):
            validate_source_tree(self.upstream, self.source_tree)

    def test_rejects_external_symlink_even_in_generated_tree(self):
        outside = self.base / "outside.txt"
        outside.write_text("outside\n", encoding="utf-8")
        generated = self.source_tree / "build"
        generated.mkdir()
        (generated / "external").symlink_to(outside)

        with self.assertRaisesRegex(RuntimeImportError, "external symlink"):
            validate_source_tree(self.upstream, self.source_tree)

    def test_existing_later_destination_prevents_all_copying(self):
        second_source = self.upstream / "Modules/second"
        second_source.mkdir()
        (second_source / "live.txt").write_text("second\n", encoding="utf-8")
        self.payload["packages"]["second"] = {
            "role": "p450",
            "source": "Modules/second",
            "destination": "src/p450/second",
            "imported": True,
        }
        self.write_manifest()
        plan = build_import_plan(
            self.manifest(), self.upstream, self.destination
        )
        plan[1].destination.mkdir(parents=True)

        with self.assertRaisesRegex(RuntimeImportError, "destination exists"):
            apply_import_plan(plan, self.destination, self.commit)

        self.assertFalse(plan[0].destination.exists())

    def test_existing_provenance_prevents_all_copying(self):
        provenance_path = self.destination / "config/import_provenance.json"
        provenance_path.write_text("do not replace\n", encoding="utf-8")
        plan = build_import_plan(
            self.manifest(), self.upstream, self.destination
        )

        with self.assertRaisesRegex(RuntimeImportError, "destination exists"):
            apply_import_plan(plan, self.destination, self.commit)

        self.assertEqual(
            "do not replace\n", provenance_path.read_text(encoding="utf-8")
        )
        self.assertFalse(plan[0].destination.exists())

    def test_plan_cannot_claim_reserved_provenance_destination(self):
        provenance_path = self.destination / "config/import_provenance.json"
        plan = (
            ImportItem(
                "reserved", self.source_tree / "live.txt", provenance_path
            ),
        )

        with self.assertRaisesRegex(RuntimeImportError, "destination exists"):
            apply_import_plan(plan, self.destination, self.commit)

        self.assertFalse(provenance_path.exists())

    def test_external_config_symlink_is_rejected_before_copy(self):
        project = self.base / "symlink-config-destination"
        project.mkdir()
        external_config = self.base / "external-config"
        external_config.mkdir()
        (project / "config").symlink_to(
            external_config, target_is_directory=True
        )
        first_destination = project / "src/first.txt"
        plan = (
            ImportItem(
                "first",
                self.source_tree / "live.txt",
                first_destination,
            ),
        )

        with self.assertRaisesRegex(RuntimeImportError, "escapes project"):
            apply_import_plan(plan, project, self.commit)

        self.assertFalse(first_destination.exists())
        self.assertFalse((external_config / "import_provenance.json").exists())

    def test_obstructed_later_parent_prevents_all_copying(self):
        second_source = self.upstream / "Modules/second"
        second_source.mkdir()
        (second_source / "live.txt").write_text("second\n", encoding="utf-8")
        self.payload["packages"]["second"] = {
            "role": "p450",
            "source": "Modules/second",
            "destination": "blocked/second",
            "imported": True,
        }
        self.write_manifest()
        blocked_parent = self.destination / "blocked"
        blocked_parent.write_text("not a directory\n", encoding="utf-8")
        plan = build_import_plan(
            self.manifest(), self.upstream, self.destination
        )

        with self.assertRaisesRegex(RuntimeImportError, "destination exists"):
            apply_import_plan(plan, self.destination, self.commit)

        self.assertFalse(plan[0].destination.exists())
        self.assertEqual(
            "not a directory\n",
            blocked_parent.read_text(encoding="utf-8"),
        )

    def test_dangling_symlink_parent_prevents_all_copying(self):
        second_source = self.upstream / "Modules/second"
        second_source.mkdir()
        (second_source / "live.txt").write_text("second\n", encoding="utf-8")
        self.payload["packages"]["second"] = {
            "role": "p450",
            "source": "Modules/second",
            "destination": "dangling/second",
            "imported": True,
        }
        self.write_manifest()
        dangling_parent = self.destination / "dangling"
        dangling_parent.symlink_to(
            self.destination / "missing-target", target_is_directory=True
        )
        plan = build_import_plan(
            self.manifest(), self.upstream, self.destination
        )

        with self.assertRaisesRegex(RuntimeImportError, "destination exists"):
            apply_import_plan(plan, self.destination, self.commit)

        self.assertFalse(plan[0].destination.exists())
        self.assertTrue(dangling_parent.is_symlink())

    def test_apply_copies_live_files_and_ignores_generated_trees(self):
        for relative in (
            ".git/marker",
            "build/marker",
            "devel/marker",
            "docs/results/marker",
            "nested/docs/results/marker",
            "__pycache__/marker",
            ".pytest_cache/marker",
        ):
            path = self.source_tree / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text("ignored\n", encoding="utf-8")
        internal_link = self.source_tree / "internal-link.txt"
        internal_link.symlink_to(self.source_tree / "live.txt")
        plan = build_import_plan(
            self.manifest(), self.upstream, self.destination
        )

        provenance = apply_import_plan(plan, self.destination, self.commit)

        imported = self.destination / "src/p450/demo"
        self.assertEqual(
            "runtime\n", (imported / "live.txt").read_text(encoding="utf-8")
        )
        self.assertFalse((imported / "internal-link.txt").is_symlink())
        self.assertEqual(
            "runtime\n",
            (imported / "internal-link.txt").read_text(encoding="utf-8"),
        )
        for relative in (
            ".git",
            "build",
            "devel",
            "docs/results",
            "nested/docs/results",
            "__pycache__",
            ".pytest_cache",
        ):
            self.assertFalse((imported / relative).exists(), relative)
        self.assertEqual(self.commit, provenance["upstream_commit"])

    def test_nested_auxiliary_destination_is_supported(self):
        helper = self.upstream / "Modules/common/include/helper.h"
        helper.parent.mkdir(parents=True)
        helper.write_text("helper\n", encoding="utf-8")
        self.payload["auxiliary_imports"] = [
            {
                "name": "helper",
                "source": "Modules/common/include/helper.h",
                "destination": (
                    "src/p450/demo/vendor_upstream/common/include/helper.h"
                ),
            }
        ]
        self.write_manifest()
        plan = build_import_plan(
            self.manifest(), self.upstream, self.destination
        )

        apply_import_plan(plan, self.destination, self.commit)

        imported_helper = self.destination / plan[1].destination.relative_to(
            self.destination
        )
        self.assertEqual(
            "helper\n", imported_helper.read_text(encoding="utf-8")
        )

    def test_nested_auxiliary_leaf_conflict_is_preflighted(self):
        conflicting = self.source_tree / "vendor_upstream/common/helper.h"
        conflicting.parent.mkdir(parents=True)
        conflicting.write_text("package copy\n", encoding="utf-8")
        helper = self.upstream / "Modules/common/helper.h"
        helper.parent.mkdir()
        helper.write_text("auxiliary copy\n", encoding="utf-8")
        self.payload["auxiliary_imports"] = [
            {
                "name": "helper",
                "source": "Modules/common/helper.h",
                "destination": "src/p450/demo/vendor_upstream/common/helper.h",
            }
        ]
        self.write_manifest()
        plan = build_import_plan(
            self.manifest(), self.upstream, self.destination
        )

        with self.assertRaisesRegex(RuntimeImportError, "destination exists"):
            apply_import_plan(plan, self.destination, self.commit)

        self.assertFalse(plan[0].destination.exists())
        self.assertFalse(
            (self.destination / "config/import_provenance.json").exists()
        )

    def test_file_before_ignored_nested_suffix_is_preflighted(self):
        conflicting_prefix = self.source_tree / "foo"
        conflicting_prefix.write_text("not a directory\n", encoding="utf-8")
        helper = self.upstream / "Modules/common/helper.h"
        helper.parent.mkdir()
        helper.write_text("auxiliary copy\n", encoding="utf-8")
        self.payload["auxiliary_imports"] = [
            {
                "name": "helper",
                "source": "Modules/common/helper.h",
                "destination": "src/p450/demo/foo/build/helper.h",
            }
        ]
        self.write_manifest()
        plan = build_import_plan(
            self.manifest(), self.upstream, self.destination
        )

        with self.assertRaisesRegex(RuntimeImportError, "destination exists"):
            apply_import_plan(plan, self.destination, self.commit)

        self.assertFalse(plan[0].destination.exists())
        self.assertFalse(
            (self.destination / "config/import_provenance.json").exists()
        )

    def test_apply_rejects_manually_constructed_destination_escape(self):
        outside = self.destination.parent / "escape.txt"
        plan = (
            ImportItem("escape", self.source_tree / "live.txt", outside),
        )

        with self.assertRaisesRegex(RuntimeImportError, "escapes project"):
            apply_import_plan(plan, self.destination, self.commit)

        self.assertFalse(outside.exists())

    def test_apply_rejects_external_alias_back_into_project(self):
        alias = self.base / "project-alias"
        alias.symlink_to(self.destination, target_is_directory=True)
        copied = self.destination / "copied.txt"
        provenance = self.destination / "config/import_provenance.json"
        plan = (
            ImportItem(
                "alias",
                self.source_tree / "live.txt",
                alias / "copied.txt",
            ),
        )

        with self.assertRaisesRegex(RuntimeImportError, "escapes project"):
            apply_import_plan(plan, self.destination, self.commit)

        self.assertFalse(copied.exists())
        self.assertFalse(provenance.exists())

    def test_provenance_records_exact_sorted_hashes_and_commit(self):
        plan = build_import_plan(
            self.manifest(), self.upstream, self.destination
        )

        provenance = apply_import_plan(plan, self.destination, self.commit)

        imported = self.destination / "src/p450/demo"
        expected_files = {
            "src/p450/demo/live.txt": hashlib.sha256(
                b"runtime\n"
            ).hexdigest(),
            "src/p450/demo/package.xml": sha256_file(
                imported / "package.xml"
            ),
        }
        self.assertEqual(1, provenance["schema_version"])
        self.assertEqual(self.commit, provenance["upstream_commit"])
        self.assertEqual(expected_files, provenance["files"])
        self.assertEqual(
            sorted(provenance["files"]), list(provenance["files"])
        )
        provenance_path = self.destination / "config/import_provenance.json"
        self.assertEqual(
            provenance,
            json.loads(provenance_path.read_text(encoding="utf-8")),
        )
        self.assertNotIn(
            "config/import_provenance.json", provenance["files"]
        )


if __name__ == "__main__":
    unittest.main()
