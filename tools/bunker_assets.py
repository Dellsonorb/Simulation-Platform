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
        if (type(payload["schema_version"]) is not int or
                payload["schema_version"] != SUPPORTED_SCHEMA_VERSION):
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
    manifest = (manifest if isinstance(manifest, AssetManifest)
                else AssetManifest.load(manifest))
    root = _canonical_directory(package_root)
    observed = _scan_regular_files(root, ("urdf", "meshes"))
    expected = {item.path for item in manifest.files}
    if observed != expected:
        missing = sorted(expected - observed)
        extra = sorted(observed - expected)
        raise AssetValidationError(
            "asset closure mismatch missing=%r extra=%r" % (missing, extra))
    for item in manifest.files:
        if _sha256(_regular_file_within(root / item.path, root)) != item.sha256:
            raise AssetValidationError("asset hash mismatch: %s" % item.path)
    package_xml = _regular_file_within(root / "package.xml", root)
    license_values = [
        item.text
        for item in ET.parse(package_xml).getroot().findall("license")
    ]
    if license_values != [EXPECTED_LICENSE_VALUE]:
        raise AssetValidationError("vendor package license changed: package.xml")
    frozen = manifest.source["frozen_renderer_input"]
    digest_by_path = {item.path: item.sha256 for item in manifest.files}
    if digest_by_path.get(frozen) != manifest.source[
            "frozen_renderer_input_sha256"]:
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
