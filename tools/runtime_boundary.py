#!/usr/bin/env python3
"""Offline source-boundary gate for Simulation Platform V1.0."""

import argparse
import ast
import io
import json
import re
import sys
import tokenize
import xml.etree.ElementTree as ET
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path


ACTIVE_TEXT_NAMES = frozenset({"CMakeLists.txt"})
ACTIVE_TEXT_SUFFIXES = frozenset({
    ".cmake",
    ".launch", ".xml", ".xacro", ".urdf", ".sdf", ".world",
    ".jinja", ".config",
    ".py", ".cpp", ".cc", ".c", ".hpp", ".hh", ".h",
    ".yaml", ".yml", ".json", ".rviz",
    ".msg", ".srv", ".action",
    ".bash", ".sh",
})

EXCLUDED_COMPONENTS = frozenset({
    "docs", "test", "tests", "results", "generated", "__pycache__",
    ".pytest_cache", ".mypy_cache", ".ruff_cache", ".cache", "cache",
    "caches", "license", "licenses",
})

DYNAMIC_LIFECYCLE_TOKENS = (
    "/gazebo/delete_model",
    "DeleteModel",
    "SpawnModel",
    "/gazebo/spawn_urdf_model",
    "/gazebo/spawn_sdf_model",
    "respawn_model",
    "delete_respawn",
)

JOINT_CANDIDATE_SPAWNS = frozenset({
    ("src/platform/sim_platform_bringup/launch/p450_runtime.launch",
     "p450_D435i_1_spawn"),
    ("src/platform/bunker_sim_runtime/launch/bunker_runtime.launch",
     "spawn_bunker"),
    ("src/ground/bunker_aubo_gazebo/launch/combined_robot.launch",
     "spawn_bunker_aubo"),
    ("src/ground/bunker_aubo_gazebo/launch/combined_robot.launch",
     "spawn_bunker_aubo_startup_pose"),
})

STANDALONE_SMOKE_SPAWNS = frozenset({
    ("src/ground/bunker_aubo_gazebo/launch/ag95_only.launch", "spawn_ag95"),
    ("src/ground/bunker_aubo_gazebo/launch/aubo_only.launch", "spawn_aubo_i5"),
    ("src/ground/bunker_aubo_gazebo/launch/bunker_only.launch", "spawn_bunker"),
})

INACTIVE_LEGACY_SPAWNS = frozenset({
    ("src/ground/bunker_aubo_gazebo/launch/brick_world.launch", "spawn_brick"),
    ("src/ground/ground_pick_orchestrator/launch/ground_pick_demo.launch",
     "spawn_ground_pick_brick"),
    ("src/ground/ground_pick_orchestrator/launch/ground_pick_demo.launch",
     "spawn_ground_pick_obstacle"),
    ("src/p450/prometheus_gazebo/launch_basic/sitl_px4_indoor.launch",
     "$(arg vehicle)_$(arg uav_id)_spawn"),
    ("src/p450/prometheus_gazebo/launch_basic/sitl_px4_outdoor.launch",
     "$(arg vehicle)_$(arg uav_id)_spawn"),
    ("src/vendor/aubo_description/launch/gazebo.launch", "spawn_gazebo_model"),
    ("src/vendor/dh_ag95_description/launch/gazebo.launch",
     "spawn_gazebo_model"),
})

M5_PATTERN = re.compile(
    r"(?<![A-Za-z0-9])m5(?:[_/-]|(?![A-Za-z0-9]))", re.IGNORECASE)


@dataclass(frozen=True, order=True)
class Finding:
    path: str
    line: int
    token: str


@dataclass(frozen=True, order=True)
class PackageRecord:
    path: str
    name: str


def _relative(path, root):
    return path.relative_to(root).as_posix()


def _is_license_or_boundary_metadata(path):
    lower = path.name.lower()
    if lower in {"import_provenance.json", "runtime_overlay.json"}:
        return True
    return (lower == "license" or lower.startswith("license.") or
            lower == "copying" or lower.startswith("copying.") or
            lower == "notice" or lower.startswith("notice."))


def _is_active_text_path(path, source_root):
    if path.is_symlink() or not path.is_file():
        return False
    relative = path.relative_to(source_root)
    if any(part.lower() in EXCLUDED_COMPONENTS
           for part in relative.parts[:-1]):
        return False
    if _is_license_or_boundary_metadata(path):
        return False
    return (path.name in ACTIVE_TEXT_NAMES or
            path.suffix.lower() in ACTIVE_TEXT_SUFFIXES)


def active_text_paths(root):
    root = Path(root)
    source_root = root / "src"
    if not source_root.is_dir():
        return tuple()
    return tuple(sorted(
        path for path in source_root.rglob("*")
        if _is_active_text_path(path, source_root)
    ))


def _blank_range(characters, start, end):
    for index in range(start, end):
        if characters[index] not in "\r\n":
            characters[index] = " "


def _strip_block_comments(text, opening, closing):
    characters = list(text)
    cursor = 0
    while True:
        start = text.find(opening, cursor)
        if start < 0:
            break
        stop = text.find(closing, start + len(opening))
        end = len(text) if stop < 0 else stop + len(closing)
        _blank_range(characters, start, end)
        cursor = end
    return "".join(characters)


def _strip_jinja_block_comments(text, opening, closing):
    """Blank template comments while preserving delimiters in Jinja strings."""
    characters = list(text)
    cursor = 0
    code_closer = None
    quote = None
    while cursor < len(text):
        character = text[cursor]
        if code_closer is not None:
            if quote is not None:
                if character == "\\":
                    cursor += 2
                    continue
                if character == quote:
                    quote = None
                cursor += 1
                continue
            if character in ("'", '"'):
                quote = character
                cursor += 1
                continue
            if text.startswith(code_closer, cursor):
                cursor += len(code_closer)
                code_closer = None
                continue
            if text.startswith(opening, cursor):
                stop = text.find(closing, cursor + len(opening))
                end = len(text) if stop < 0 else stop + len(closing)
                _blank_range(characters, cursor, end)
                cursor = end
                continue
            cursor += 1
            continue

        if text.startswith("{{", cursor):
            code_closer = "}}"
            cursor += 2
            continue
        if text.startswith("{%", cursor):
            code_closer = "%}"
            cursor += 2
            continue
        if text.startswith(opening, cursor):
            stop = text.find(closing, cursor + len(opening))
            end = len(text) if stop < 0 else stop + len(closing)
            _blank_range(characters, cursor, end)
            cursor = end
            continue
        cursor += 1
    return "".join(characters)


def _strip_c_comments(text):
    characters = list(text)
    cursor = 0
    quote = None
    while cursor < len(text):
        character = text[cursor]
        if quote is not None:
            if character == "\\":
                cursor += 2
                continue
            if character == quote:
                quote = None
            cursor += 1
            continue
        if character in ("'", '"'):
            quote = character
            cursor += 1
            continue
        if text.startswith("/*", cursor):
            stop = text.find("*/", cursor + 2)
            end = len(text) if stop < 0 else stop + 2
            _blank_range(characters, cursor, end)
            cursor = end
            continue
        if text.startswith("//", cursor):
            end = cursor + 2
            while end < len(text) and text[end] not in "\r\n":
                end += 1
            _blank_range(characters, cursor, end)
            cursor = end
            continue
        cursor += 1
    return "".join(characters)


def _strip_hash_comments(text):
    characters = list(text)
    cursor = 0
    quote = None
    while cursor < len(text):
        character = text[cursor]
        if quote is not None:
            if character == "\\" and quote == '"':
                cursor += 2
                continue
            if character == quote:
                if quote == "'" and cursor + 1 < len(text) \
                        and text[cursor + 1] == "'":
                    cursor += 2
                    continue
                quote = None
            cursor += 1
            continue
        if character in ("'", '"'):
            quote = character
            cursor += 1
            continue
        if character == "#":
            end = cursor + 1
            while end < len(text) and text[end] not in "\r\n":
                end += 1
            _blank_range(characters, cursor, end)
            cursor = end
            continue
        cursor += 1
    return "".join(characters)


def _strip_python_comments(text):
    characters = list(text)
    lines = text.splitlines(keepends=True)
    offsets = []
    offset = 0
    for line in lines:
        offsets.append(offset)
        offset += len(line)
    if not lines or offset < len(text):
        offsets.append(offset)
    comments = []
    try:
        for token in tokenize.generate_tokens(io.StringIO(text).readline):
            if token.type == tokenize.COMMENT:
                comments.append((token.start, token.end))
    except (IndentationError, tokenize.TokenError):
        pass
    for start, end in comments:
        start_offset = offsets[start[0] - 1] + start[1]
        end_offset = offsets[end[0] - 1] + end[1]
        _blank_range(characters, start_offset, end_offset)
    return "".join(characters)


def strip_comments(path, text):
    path = Path(path)
    suffix = path.suffix.lower()
    if suffix == ".jinja":
        return _strip_jinja_block_comments(
            _strip_jinja_block_comments(text, "<!--", "-->"),
            "{#", "#}")
    if suffix in {".launch", ".xml", ".xacro", ".urdf", ".sdf",
                  ".world", ".config"}:
        return _strip_block_comments(text, "<!--", "-->")
    if suffix in {".cpp", ".cc", ".c", ".hpp", ".hh", ".h"}:
        return _strip_c_comments(text)
    if suffix == ".py":
        return _strip_python_comments(text)
    if (path.name == "CMakeLists.txt" or suffix in {
            ".cmake", ".yaml", ".yml", ".rviz", ".msg", ".srv",
            ".action", ".bash", ".sh"}):
        return _strip_hash_comments(text)
    return text


def _line_number(text, offset):
    return text.count("\n", 0, offset) + 1


def _add_matches(findings, relative, text, pattern, token, flags=0):
    for match in re.finditer(pattern, text, flags):
        findings.add(Finding(relative, _line_number(text, match.start()), token))


def scan_active_content(root, runtime_tokens):
    root = Path(root)
    findings = set()
    method_tokens = tuple(runtime_tokens)
    for path in active_text_paths(root):
        relative = _relative(path, root)
        try:
            text = path.read_bytes().decode("utf-8")
        except UnicodeDecodeError:
            findings.add(Finding(relative, 0, "decode:utf-8"))
            continue
        active = strip_comments(path, text)
        for token in DYNAMIC_LIFECYCLE_TOKENS:
            _add_matches(findings, relative, active, re.escape(token), token)

        if any(token.casefold() in {"/m5/", "m5_"}
               for token in method_tokens):
            for match in M5_PATTERN.finditer(active):
                findings.add(Finding(
                    relative, _line_number(active, match.start()),
                    "method:m5"))
        for token in method_tokens:
            canonical = token.casefold()
            if canonical in {"/m5/", "m5_"}:
                continue
            _add_matches(
                findings, relative, active, re.escape(token),
                "method:%s" % canonical, re.IGNORECASE)
    return tuple(sorted(findings))


def _local_tag(element):
    return element.tag.rsplit("}", 1)[-1]


def _package_files(root):
    source = Path(root) / "src"
    if not source.is_dir():
        return tuple()
    return tuple(sorted(path for path in source.rglob("package.xml")
                        if path.is_file() and not path.is_symlink()))


def _parse_package(path, root):
    relative = _relative(path, root)
    try:
        text = path.read_bytes().decode("utf-8")
    except UnicodeDecodeError:
        return None, (Finding(relative, 0, "package:decode:utf-8"),)
    try:
        package = ET.fromstring(text)
    except ET.ParseError as error:
        line = error.position[0] if error.position else 0
        return None, (Finding(relative, line, "package:malformed"),)
    name_element = next(
        (item for item in package if _local_tag(item) == "name"), None)
    name = "" if name_element is None or name_element.text is None \
        else name_element.text.strip()
    if not name:
        return None, (Finding(relative, 0, "package:missing-name"),)
    return (PackageRecord(relative, name), package, text), tuple()


def discover_packages(root):
    records = []
    for path in _package_files(root):
        parsed, _findings = _parse_package(path, Path(root))
        if parsed is not None:
            records.append(parsed[0])
    return tuple(sorted(records))


def _find_dependency_line(text, tag, name):
    pattern = (r"<%s\b[^>]*>\s*%s\s*</%s\s*>" %
               (re.escape(tag), re.escape(name), re.escape(tag)))
    match = re.search(pattern, text, re.IGNORECASE | re.DOTALL)
    if match is None:
        match = re.search(re.escape(name), text, re.IGNORECASE)
    return 0 if match is None else _line_number(text, match.start())


def validate_packages(root, expected_packages, forbidden_packages,
                      optional_packages=None):
    """Validate mandatory imports and any materialized declared local package."""
    root = Path(root)
    required = {name: Path(destination).as_posix()
                for name, destination in expected_packages.items()}
    optional = {
        name: Path(destination).as_posix()
        for name, destination in (optional_packages or {}).items()
    }
    declared = dict(optional)
    declared.update(required)
    forbidden = {name.casefold() for name in forbidden_packages}
    findings = set()
    parsed_records = []

    for path in _package_files(root):
        parsed, parse_findings = _parse_package(path, root)
        findings.update(parse_findings)
        if parsed is None:
            continue
        record, package, raw = parsed
        parsed_records.append(record)

        for element in package.iter():
            tag = _local_tag(element)
            if tag != "depend" and not tag.endswith("_depend"):
                continue
            dependency = "" if element.text is None else element.text.strip()
            if dependency.casefold() in forbidden:
                findings.add(Finding(
                    record.path,
                    _find_dependency_line(raw, tag, dependency),
                    "dependency:%s" % dependency.casefold()))

    by_name = defaultdict(list)
    by_directory = {}
    for record in parsed_records:
        by_name[record.name].append(record)
        by_directory[Path(record.path).parent.as_posix()] = record

    package_directories = [Path(record.path).parent
                           for record in parsed_records]
    for record in parsed_records:
        directory = Path(record.path).parent
        if any(other != directory and other in directory.parents
               for other in package_directories):
            findings.add(Finding(record.path, 0, "package:nested:%s" %
                                 record.name))
        if record.name not in declared:
            findings.add(Finding(record.path, 0, "package:unlisted:%s" %
                                 record.name))
        elif directory.as_posix() != declared[record.name]:
            findings.add(Finding(record.path, 0, "package:wrong-path:%s" %
                                 record.name))

    for name, records in by_name.items():
        if len(records) > 1:
            for record in records:
                findings.add(Finding(
                    record.path, 0, "package:duplicate:%s" % name))

    for name, destination in declared.items():
        expected_file = "%s/package.xml" % destination
        at_destination = by_directory.get(destination)
        if at_destination is not None and at_destination.name != name:
            findings.add(Finding(
                at_destination.path, 0,
                "package:wrong-name:%s:%s" % (name, at_destination.name)))
        exact_record = any(
            record.name == name and record.path == expected_file
            for record in parsed_records
        )
        destination_path = root / destination
        materialized = (destination_path.exists() or
                        destination_path.is_symlink())
        if not exact_record and (name in required or materialized):
            findings.add(Finding(
                expected_file, 0, "package:missing:%s" % name))

    return tuple(sorted(findings))


def _startup_spawn_entries(root):
    root = Path(root)
    entries = []
    parse_findings = []
    for path in active_text_paths(root):
        if path.suffix.lower() != ".launch":
            continue
        relative = _relative(path, root)
        try:
            text = path.read_bytes().decode("utf-8")
        except UnicodeDecodeError:
            parse_findings.append(Finding(
                relative, 0, "launch:decode:utf-8"))
            continue
        try:
            launch = ET.fromstring(text)
        except ET.ParseError as error:
            line = error.position[0] if error.position else 0
            parse_findings.append(Finding(relative, line, "launch:malformed"))
            continue
        cursor = 0
        for node in launch.iter("node"):
            if (node.attrib.get("pkg") != "gazebo_ros" or
                    node.attrib.get("type") != "spawn_model"):
                continue
            name = node.attrib.get("name", "")
            marker_double = 'name="%s"' % name
            marker_single = "name='%s'" % name
            positions = [position for position in (
                text.find(marker_double, cursor),
                text.find(marker_single, cursor),
            ) if position >= 0]
            position = min(positions) if positions else 0
            cursor = position + 1
            entries.append(((relative, name), _line_number(text, position)))
    return entries, parse_findings


def discover_startup_spawns(root):
    entries, _findings = _startup_spawn_entries(root)
    return tuple(identity for identity, _line in entries)


def validate_startup_spawns(root, joint_candidates=None,
                            standalone_smoke=None, inactive_legacy=None):
    joint = JOINT_CANDIDATE_SPAWNS if joint_candidates is None \
        else frozenset(joint_candidates)
    standalone = STANDALONE_SMOKE_SPAWNS if standalone_smoke is None \
        else frozenset(standalone_smoke)
    inactive = INACTIVE_LEGACY_SPAWNS if inactive_legacy is None \
        else frozenset(inactive_legacy)
    expected = joint | standalone | inactive
    entries, parse_findings = _startup_spawn_entries(root)
    findings = set(parse_findings)
    counts = Counter(identity for identity, _line in entries)
    lines = {identity: line for identity, line in entries}

    overlaps = ((joint & standalone) | (joint & inactive) |
                (standalone & inactive))
    for path, name in overlaps:
        findings.add(Finding(
            path, lines.get((path, name), 0),
            "startup-spawn:classification-overlap:%s" % name))

    for identity, count in counts.items():
        path, name = identity
        if identity not in expected:
            findings.add(Finding(
                path, lines[identity], "startup-spawn:unclassified:%s" % name))
        if count > 1:
            findings.add(Finding(
                path, lines[identity], "startup-spawn:duplicate:%s" % name))
    for path, name in expected:
        if counts[(path, name)] == 0:
            findings.add(Finding(
                path, 0, "startup-spawn:missing:%s" % name))
    return tuple(sorted(findings))


def _removed_python_module(removed_path):
    if (removed_path.suffix != ".py" or len(removed_path.parts) <= 5 or
            removed_path.parts[3] != "src"):
        return None
    parts = list(removed_path.parts[4:])
    parts[-1] = Path(parts[-1]).stem
    return ".".join(parts)


def _candidate_package_root(candidate, package_roots):
    matches = []
    for package_root in package_roots:
        try:
            candidate.relative_to(package_root)
        except ValueError:
            continue
        matches.append(package_root)
    return max(matches, key=lambda item: len(item.parts)) if matches else None


def _python_import_lines(candidate, package_root, text):
    try:
        tree = ast.parse(text, filename=str(candidate))
    except SyntaxError:
        return {}
    package_context = None
    if package_root is not None:
        relative = candidate.relative_to(package_root)
        if len(relative.parts) >= 3 and relative.parts[0] == "src":
            package_context = list(relative.parts[1:-1])

    imported = {}
    for node in ast.walk(tree):
        line = getattr(node, "lineno", 0)
        if isinstance(node, ast.Import):
            for alias in node.names:
                imported.setdefault(alias.name, line)
            continue
        if not isinstance(node, ast.ImportFrom):
            continue
        if node.level:
            if package_context is None or node.level > len(package_context):
                continue
            keep = len(package_context) - (node.level - 1)
            base_parts = package_context[:keep]
            if node.module:
                base_parts.extend(node.module.split("."))
            base = ".".join(base_parts)
        else:
            base = node.module or ""
        if base:
            imported.setdefault(base, line)
        for alias in node.names:
            if alias.name != "*":
                imported.setdefault(".".join(
                    part for part in (base, alias.name) if part), line)
    return imported


def scan_removed_references(root, removed_paths):
    root = Path(root)
    removed_records = []
    for removed in removed_paths:
        removed_path = Path(removed)
        removed_records.append((
            removed,
            (
                removed,
                Path(*removed_path.parts[3:]).as_posix(),
                Path(*removed_path.parts[2:]).as_posix(),
                removed_path.name,
            ),
            _removed_python_module(removed_path),
        ))

    package_roots = tuple(path.parent for path in _package_files(root))
    findings = set()
    for candidate in active_text_paths(root):
        relative = _relative(candidate, root)
        try:
            text = candidate.read_bytes().decode("utf-8")
        except UnicodeDecodeError:
            continue
        active = strip_comments(candidate, text)
        imports = {}
        if candidate.suffix.lower() == ".py":
            package_root = _candidate_package_root(candidate, package_roots)
            imports = _python_import_lines(candidate, package_root, active)
        for removed, needles, module in removed_records:
            for needle in needles:
                boundary = r"[A-Za-z0-9_.-]"
                pattern = (r"(?<!%s)%s(?!%s)" %
                           (boundary, re.escape(needle), boundary))
                for match in re.finditer(pattern, active):
                    findings.add(Finding(
                        relative, _line_number(active, match.start()),
                        "removed:%s" % removed))
            if module is not None and module in imports:
                findings.add(Finding(
                    relative, imports[module], "removed:%s" % removed))
    return tuple(sorted(findings))


def _is_string_list(value):
    return (isinstance(value, list) and
            all(isinstance(item, str) for item in value))


def _valid_manifest_boundary_schema(manifest):
    if not isinstance(manifest, dict):
        return False
    packages = manifest.get("packages")
    forbidden = manifest.get("forbidden")
    if not isinstance(packages, dict) or not isinstance(forbidden, dict):
        return False
    if not _is_string_list(forbidden.get("package_names")):
        return False
    if not _is_string_list(forbidden.get("runtime_tokens")):
        return False
    destinations = set()
    for name, item in packages.items():
        if not isinstance(name, str) or not name or not isinstance(item, dict):
            return False
        if not isinstance(item.get("imported"), bool):
            return False
        destination = item.get("destination")
        if not isinstance(destination, str) or not destination:
            return False
        path = Path(destination)
        if (path.is_absolute() or path.as_posix() != destination or
                len(path.parts) < 2 or path.parts[0] != "src" or
                any(part in ("", ".", "..") for part in path.parts) or
                destination in destinations):
            return False
        destinations.add(destination)
    return True


def _valid_overlay_boundary_schema(overlay):
    if not isinstance(overlay, dict):
        return False
    removed = overlay.get("removed")
    if not _is_string_list(removed):
        return False
    for item in removed:
        path = Path(item)
        if (not item or path.is_absolute() or
                any(part in ("", ".", "..") for part in path.parts)):
            return False
    return True


def validate_repository(root):
    root = Path(root)
    manifest_path = root / "config/runtime_sources.json"
    overlay_path = root / "config/runtime_overlay.json"
    findings = set()
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, ValueError):
        return (Finding("config/runtime_sources.json", 0,
                        "boundary:manifest-unreadable"),)
    try:
        overlay = json.loads(overlay_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, ValueError):
        return (Finding("config/runtime_overlay.json", 0,
                        "boundary:overlay-unreadable"),)

    if not _valid_manifest_boundary_schema(manifest):
        return (Finding("config/runtime_sources.json", 0,
                        "boundary:manifest-schema"),)
    if not _valid_overlay_boundary_schema(overlay):
        return (Finding("config/runtime_overlay.json", 0,
                        "boundary:overlay-schema"),)

    expected_packages = {
        name: item["destination"]
        for name, item in manifest["packages"].items()
        if item["imported"]
    }
    optional_packages = {
        name: item["destination"]
        for name, item in manifest["packages"].items()
        if not item["imported"]
    }
    forbidden = manifest["forbidden"]
    findings.update(scan_active_content(
        root, forbidden["runtime_tokens"]))
    findings.update(validate_packages(
        root, expected_packages, forbidden["package_names"],
        optional_packages))
    findings.update(validate_startup_spawns(root))
    findings.update(scan_removed_references(root, overlay["removed"]))
    return tuple(sorted(findings))


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Validate the Simulation Platform V1.0 source boundary")
    parser.add_argument(
        "root", nargs="?", type=Path,
        default=Path(__file__).resolve().parents[1])
    arguments = parser.parse_args(argv)
    findings = validate_repository(arguments.root.resolve())
    for finding in findings:
        print("%s:%d: %s" % (finding.path, finding.line, finding.token))
    return 1 if findings else 0


if __name__ == "__main__":
    sys.exit(main())
