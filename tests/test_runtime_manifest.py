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

    def test_rejects_non_empty_relative_posix_path_violations(self):
        cases = (
            ("empty package source", "", "package_source"),
            ("dot package destination", ".", "package_destination"),
            ("backslash required path", "a\\b", "required_path"),
            ("Windows drive auxiliary path", "C:/temp/x", "auxiliary_path"),
        )
        for label, value, location in cases:
            with self.subTest(label=label):
                payload = self.minimal_payload()
                if location == "package_source":
                    payload["packages"]["demo"]["source"] = value
                elif location == "package_destination":
                    payload["packages"]["demo"]["destination"] = value
                elif location == "required_path":
                    payload["layout"]["required_paths"] = [value]
                else:
                    payload["auxiliary_imports"] = [
                        {
                            "name": "model-assets",
                            "source": "assets/model",
                            "destination": value,
                        }
                    ]
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

    def test_records_package_name(self):
        manifest = RuntimeManifest.load(self.write_manifest(self.minimal_payload()))
        self.assertEqual("demo", manifest.packages["demo"].name)

    def test_records_auxiliary_import_name(self):
        payload = self.minimal_payload()
        payload["auxiliary_imports"] = [
            {
                "name": "model-assets",
                "source": "assets/model",
                "destination": "src/vendor/model",
            }
        ]
        manifest = RuntimeManifest.load(self.write_manifest(payload))
        self.assertEqual("model-assets", manifest.auxiliary_imports[0].name)

    def test_allows_omitted_package_source(self):
        payload = self.minimal_payload()
        del payload["packages"]["demo"]["source"]
        manifest = RuntimeManifest.load(self.write_manifest(payload))
        self.assertIsNone(manifest.packages["demo"].source)

    def test_defaults_auxiliary_imports_to_empty(self):
        payload = self.minimal_payload()
        del payload["auxiliary_imports"]
        manifest = RuntimeManifest.load(self.write_manifest(payload))
        self.assertEqual((), manifest.auxiliary_imports)

    def test_rejects_missing_schema_version(self):
        payload = self.minimal_payload()
        del payload["schema_version"]
        with self.assertRaisesRegex(ManifestError, "unsupported schema_version"):
            RuntimeManifest.load(self.write_manifest(payload))


if __name__ == "__main__":
    unittest.main()
