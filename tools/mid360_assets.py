#!/usr/bin/env python3
"""Validate the independently pinned MID360 model asset closure."""

import argparse
import hashlib
import json
import os
import posixpath
import re
import stat
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path, PurePosixPath, PureWindowsPath
from typing import Tuple


SUPPORTED_SCHEMA_VERSION = 1
UNRESOLVED_LICENSE_STATUS = "redistribution-unresolved"
MID360_TARGET_PREFIX = "models/MID360/"
REQUIRED_SDF_REFERENCES = frozenset({
    "models/MID360/meshes/MID360.dae",
    "models/MID360/scan_mode/mid360.csv",
})
SOURCE_PACKAGE_METADATA = frozenset({
    "CMakeLists.txt",
    "package.xml",
})
INSTALL_PACKAGE_METADATA = frozenset({
    "package.xml",
    "cmake/sim_platform_assetsConfig.cmake",
    "cmake/sim_platform_assetsConfig-version.cmake",
})
PACKAGE_METADATA_MODES = frozenset({
    frozenset(),
    SOURCE_PACKAGE_METADATA,
    INSTALL_PACKAGE_METADATA,
})
KNOWN_PACKAGE_METADATA = SOURCE_PACKAGE_METADATA | INSTALL_PACKAGE_METADATA
SHA256_PATTERN = re.compile(r"^[0-9a-f]{64}$")
URI_SCHEME_PATTERN = re.compile(r"^[A-Za-z][A-Za-z0-9+.-]*:")


class AssetValidationError(ValueError):
    """Raised when the manifest or an asset tree violates the contract."""


def _exact_keys(payload, expected, label):
    if not isinstance(payload, dict):
        raise AssetValidationError("{} must be an object".format(label))
    observed = set(payload)
    expected = set(expected)
    if observed != expected:
        missing = sorted(expected - observed)
        extra = sorted(observed - expected)
        details = []
        if missing:
            details.append("missing {}".format(", ".join(missing)))
        if extra:
            details.append("unexpected {}".format(", ".join(extra)))
        raise AssetValidationError(
            "invalid {} keys: {}".format(label, "; ".join(details))
        )


def _root_relative_path(value, field):
    if not isinstance(value, str):
        raise AssetValidationError("{} path must be a string".format(field))
    raw_parts = value.split("/")
    try:
        path = PurePosixPath(value)
    except (TypeError, ValueError) as error:
        raise AssetValidationError(
            "{} path escapes root: {}".format(field, value)
        ) from error
    if (
        not value
        or "\x00" in value
        or "\\" in value
        or path.is_absolute()
        or PureWindowsPath(value).drive
        or any(part in {"", ".", ".."} for part in raw_parts)
        or not path.parts
    ):
        raise AssetValidationError(
            "{} path escapes root: {}".format(field, value)
        )
    return path.as_posix()


def _nonempty_string(payload, key, label):
    value = payload[key]
    if not isinstance(value, str) or not value.strip():
        raise AssetValidationError("{}.{} must be a string".format(label, key))
    return value


@dataclass(frozen=True)
class SourceIdentity:
    remote: str
    pinned_commit: str
    introduction_commit: str
    author: str
    license_status: str
    license_note: str


@dataclass(frozen=True)
class AssetFile:
    source: str
    target: str
    sha256: str


@dataclass(frozen=True)
class AssetManifest:
    schema_version: int
    source: SourceIdentity
    files: Tuple[AssetFile, ...]
    excluded: Tuple[str, ...]

    @classmethod
    def load(cls, path):
        manifest_path = Path(path)
        try:
            manifest_text = manifest_path.read_text(encoding="utf-8")
        except UnicodeError as error:
            raise AssetValidationError(
                "manifest is not valid UTF-8: {}".format(manifest_path)
            ) from error
        try:
            payload = json.loads(manifest_text)
        except json.JSONDecodeError as error:
            raise AssetValidationError(
                "invalid JSON: {}".format(manifest_path)
            ) from error

        if not isinstance(payload, dict):
            raise AssetValidationError("manifest must be an object")
        schema_version = payload.get("schema_version")
        if (
            type(schema_version) is not int
            or schema_version != SUPPORTED_SCHEMA_VERSION
        ):
            raise AssetValidationError(
                "unsupported schema_version: {}".format(schema_version)
            )
        _exact_keys(
            payload,
            {"schema_version", "source", "files", "excluded"},
            "manifest",
        )

        source_payload = payload["source"]
        _exact_keys(
            source_payload,
            {
                "remote",
                "pinned_commit",
                "introduction_commit",
                "author",
                "license_status",
                "license_note",
            },
            "source",
        )
        license_status = _nonempty_string(
            source_payload, "license_status", "source"
        )
        if license_status != UNRESOLVED_LICENSE_STATUS:
            raise AssetValidationError(
                "license_status must be redistribution-unresolved"
            )
        license_note = _nonempty_string(source_payload, "license_note", "source")
        if "no tracked license" not in license_note.casefold():
            raise AssetValidationError(
                "license_note must record that no tracked license was found"
            )
        source = SourceIdentity(
            remote=_nonempty_string(source_payload, "remote", "source"),
            pinned_commit=_nonempty_string(
                source_payload, "pinned_commit", "source"
            ),
            introduction_commit=_nonempty_string(
                source_payload, "introduction_commit", "source"
            ),
            author=_nonempty_string(source_payload, "author", "source"),
            license_status=license_status,
            license_note=license_note,
        )

        files_payload = payload["files"]
        if not isinstance(files_payload, list) or not files_payload:
            raise AssetValidationError("files must be a nonempty array")
        files = []
        sources = set()
        targets = set()
        for entry in files_payload:
            _exact_keys(entry, {"source", "target", "sha256"}, "file entry")
            source_path = _root_relative_path(entry["source"], "source")
            target_path = _root_relative_path(entry["target"], "target")
            digest = entry["sha256"]
            if not isinstance(digest, str) or not SHA256_PATTERN.fullmatch(digest):
                raise AssetValidationError(
                    "invalid sha256 for target: {}".format(target_path)
                )
            if source_path in sources:
                raise AssetValidationError(
                    "duplicate source path: {}".format(source_path)
                )
            if target_path in targets:
                raise AssetValidationError(
                    "duplicate target path: {}".format(target_path)
                )
            sources.add(source_path)
            targets.add(target_path)
            files.append(AssetFile(source_path, target_path, digest))

        excluded_payload = payload["excluded"]
        if not isinstance(excluded_payload, list):
            raise AssetValidationError("excluded must be an array")
        excluded = tuple(
            _root_relative_path(value, "excluded") for value in excluded_payload
        )
        if len(set(excluded)) != len(excluded):
            raise AssetValidationError("duplicate excluded path")
        overlap = set(excluded) & targets
        if overlap:
            raise AssetValidationError(
                "excluded path is allowlisted: {}".format(sorted(overlap)[0])
            )

        return cls(schema_version, source, tuple(files), excluded)


def _relative(path, root):
    return path.relative_to(root).as_posix()


def _scan_tree(root):
    observed = set()

    def fail_closed(error):
        failed_path = Path(error.filename) if error.filename else root
        try:
            context = failed_path.relative_to(root).as_posix()
        except ValueError:
            context = str(failed_path)
        if context == ".":
            context = "asset root"
        reason = error.strerror or str(error)
        raise AssetValidationError(
            "asset tree scan failed at {}: {}".format(context, reason)
        ) from error

    for current, directories, filenames in os.walk(
        str(root), followlinks=False, onerror=fail_closed
    ):
        current_path = Path(current)
        for name in sorted(directories + filenames):
            path = current_path / name
            relative = _relative(path, root)
            mode = path.lstat().st_mode
            if stat.S_ISLNK(mode):
                raise AssetValidationError(
                    "symlink is forbidden: {}".format(relative)
                )
            if stat.S_ISDIR(mode):
                continue
            if not stat.S_ISREG(mode):
                raise AssetValidationError(
                    "non-regular asset is forbidden: {}".format(relative)
                )
            observed.add(relative)
    return observed


def _sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _local_tag(element):
    return element.tag.rsplit("}", 1)[-1]


def _parse_xml(root, relative):
    try:
        return ET.parse(root / relative).getroot()
    except (ET.ParseError, OSError) as error:
        raise AssetValidationError("invalid XML: {}".format(relative)) from error


def _normalise_local_reference(base, value):
    if (
        not value
        or "\x00" in value
        or "\\" in value
        or value.startswith("/")
        or PureWindowsPath(value).drive
    ):
        raise AssetValidationError(
            "asset reference escapes root: {}".format(value)
        )
    combined = posixpath.normpath(posixpath.join(base, value))
    if combined == ".." or combined.startswith("../"):
        raise AssetValidationError(
            "asset reference escapes root: {}".format(value)
        )
    return _root_relative_path(combined, "asset reference")


def _sdf_references(root, sdf_relative):
    document = _parse_xml(root, sdf_relative)
    references = set()
    base = PurePosixPath(sdf_relative).parent.as_posix()
    for element in document.iter():
        tag = _local_tag(element)
        value = (element.text or "").strip()
        if tag == "uri" and value:
            scheme_match = URI_SCHEME_PATTERN.match(value)
            if scheme_match is not None:
                scheme = scheme_match.group(0)[:-1].casefold()
                remainder = value[scheme_match.end():]
                if scheme != "model" or not remainder.startswith("//MID360/"):
                    raise AssetValidationError(
                        "asset URI scheme is forbidden: {}".format(value)
                    )
                references.add(
                    _normalise_local_reference(
                        MID360_TARGET_PREFIX,
                        remainder[len("//MID360/"):],
                    )
                )
            else:
                references.add(_normalise_local_reference(base, value))
        elif tag == "csv_file_name" and value:
            references.add(
                _normalise_local_reference(
                    "models/MID360/scan_mode", value
                )
            )
    return references


def _dae_references(root, dae_relative):
    document = _parse_xml(root, dae_relative)
    references = set()
    base = PurePosixPath(dae_relative).parent.as_posix()

    def add_reference(value):
        value = value.strip()
        if not value or value.startswith("#"):
            return
        if URI_SCHEME_PATTERN.match(value):
            raise AssetValidationError(
                "asset URI scheme is forbidden: {}".format(value)
            )
        relative = value.split("#", 1)[0]
        if relative:
            references.add(_normalise_local_reference(base, relative))

    for element in document.iter():
        for attribute_name, value in element.attrib.items():
            local_name = attribute_name.rsplit("}", 1)[-1]
            if local_name in {"url", "source"}:
                add_reference(value)
    for image in document.iter():
        if _local_tag(image) != "image":
            continue
        for element in image.iter():
            if _local_tag(element) == "init_from":
                add_reference(element.text or "")
    return references


def _validate_references(root, allowlist):
    sdf_relative = "models/MID360/MID360.sdf"
    dae_relative = "models/MID360/meshes/MID360.dae"
    for required in (sdf_relative, dae_relative):
        if required not in allowlist:
            raise AssetValidationError(
                "required closure file is not allowlisted: {}".format(required)
            )

    sdf_references = _sdf_references(root, sdf_relative)
    references = sdf_references | _dae_references(root, dae_relative)
    for reference in sorted(references):
        if reference not in allowlist:
            raise AssetValidationError(
                "asset reference is not allowlisted: {}".format(reference)
            )
    missing_references = REQUIRED_SDF_REFERENCES - sdf_references
    if missing_references:
        raise AssetValidationError(
            "missing required asset reference: {}".format(
                sorted(missing_references)[0]
            )
        )


def validate_asset_tree(manifest, asset_root):
    """Validate an exact source/install asset tree and return its real root."""
    if not isinstance(manifest, AssetManifest):
        manifest = AssetManifest.load(manifest)
    requested_root = Path(asset_root)
    try:
        root_mode = requested_root.lstat().st_mode
    except OSError as error:
        raise AssetValidationError(
            "asset root is not a directory: {}".format(requested_root)
        ) from error
    if stat.S_ISLNK(root_mode):
        raise AssetValidationError("asset root must not be a symlink")
    if not stat.S_ISDIR(root_mode):
        raise AssetValidationError(
            "asset root is not a directory: {}".format(requested_root)
        )
    root = requested_root.resolve()

    observed = _scan_tree(root)
    allowlist = {entry.target: entry for entry in manifest.files}
    missing = set(allowlist) - observed
    if missing:
        raise AssetValidationError("missing asset: {}".format(sorted(missing)[0]))
    extra = observed - set(allowlist) - KNOWN_PACKAGE_METADATA
    if extra:
        raise AssetValidationError(
            "unexpected asset: {}".format(sorted(extra)[0])
        )
    observed_metadata = frozenset(observed & KNOWN_PACKAGE_METADATA)
    if observed_metadata not in PACKAGE_METADATA_MODES:
        raise AssetValidationError(
            "invalid package metadata mode: {}".format(
                ", ".join(sorted(observed_metadata))
            )
        )

    for relative in sorted(allowlist):
        if _sha256(root / relative) != allowlist[relative].sha256:
            raise AssetValidationError("sha256 mismatch: {}".format(relative))
    _validate_references(root, set(allowlist))
    return root


def validate_assets(config_path, asset_root):
    """Load ``config_path`` and validate ``asset_root``."""
    return validate_asset_tree(AssetManifest.load(config_path), asset_root)


def _parser():
    parser = argparse.ArgumentParser(
        description="Validate the exact pinned MID360 model asset closure."
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "config/mid360_assets.json",
    )
    parser.add_argument("--asset-root", type=Path, required=True)
    return parser


def main(argv=None):
    arguments = _parser().parse_args(argv)
    try:
        canonical_root = validate_assets(arguments.config, arguments.asset_root)
    except (AssetValidationError, OSError) as error:
        print("mid360-assets: {}".format(error), file=sys.stderr)
        return 1
    print(canonical_root)
    return 0


ManifestError = AssetValidationError
Mid360AssetManifest = AssetManifest


if __name__ == "__main__":
    sys.exit(main())
