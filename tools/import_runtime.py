import argparse
import hashlib
import json
import os
import shutil
import subprocess
import tempfile
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


_IGNORED_NAMES = {
    ".git",
    "build",
    "devel",
    "__pycache__",
    ".pytest_cache",
}


def _is_within(path: Path, root: Path) -> bool:
    try:
        path.relative_to(root)
        return True
    except ValueError:
        return False


def _lexists(path: Path) -> bool:
    return os.path.lexists(str(path))


def verify_upstream_commit(upstream: Path, expected: str) -> None:
    try:
        result = subprocess.run(
            ["/usr/bin/git", "-C", str(upstream), "rev-parse", "HEAD"],
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
    except (OSError, subprocess.CalledProcessError) as error:
        raise RuntimeImportError(
            "unable to read upstream commit: {}".format(upstream)
        ) from error
    actual = result.stdout.strip()
    if actual != expected:
        raise RuntimeImportError(
            "commit mismatch: expected {}, got {}".format(expected, actual)
        )


def _resolved_symlink(candidate: Path) -> Path:
    try:
        return candidate.resolve(strict=True)
    except (OSError, RuntimeError) as error:
        raise RuntimeImportError(
            "external symlink: {} has an unresolved target".format(candidate)
        ) from error


def validate_source_tree(upstream: Path, source: Path) -> None:
    try:
        upstream = Path(upstream).resolve(strict=True)
    except (OSError, RuntimeError) as error:
        raise RuntimeImportError(
            "source root does not exist: {}".format(upstream)
        ) from error

    source = Path(source)
    if not _lexists(source):
        raise RuntimeImportError("source does not exist: {}".format(source))
    if source.is_symlink():
        resolved_source = _resolved_symlink(source)
        if not _is_within(resolved_source, upstream):
            raise RuntimeImportError(
                "external symlink: {} -> {}".format(source, resolved_source)
            )
    else:
        try:
            resolved_source = source.resolve(strict=True)
        except (OSError, RuntimeError) as error:
            raise RuntimeImportError(
                "source does not exist: {}".format(source)
            ) from error
    if not _is_within(resolved_source, upstream):
        raise RuntimeImportError("source escapes upstream: {}".format(source))
    if not resolved_source.is_dir():
        return

    visited = set()
    pending = [resolved_source]
    while pending:
        directory = pending.pop()
        try:
            directory = directory.resolve(strict=True)
            stat_result = directory.stat()
            identity = (stat_result.st_dev, stat_result.st_ino)
            if identity in visited:
                continue
            visited.add(identity)
            children = tuple(directory.iterdir())
        except (OSError, RuntimeError) as error:
            raise RuntimeImportError(
                "cannot inspect source tree: {}".format(directory)
            ) from error
        for candidate in children:
            if candidate.is_symlink():
                resolved = _resolved_symlink(candidate)
                if not _is_within(resolved, upstream):
                    raise RuntimeImportError(
                        "external symlink: {} -> {}".format(
                            candidate, resolved
                        )
                    )
                if resolved.is_dir():
                    pending.append(resolved)
            elif candidate.is_dir():
                pending.append(candidate)


def _validated_destination(destination: Path, root: Path) -> Path:
    try:
        lexical = Path(os.path.abspath(str(destination)))
        resolved = lexical.resolve(strict=False)
    except (OSError, RuntimeError) as error:
        raise RuntimeImportError(
            "destination escapes project: {}".format(destination)
        ) from error
    if (
        lexical == root
        or not _is_within(lexical, root)
        or resolved == root
        or not _is_within(resolved, root)
    ):
        raise RuntimeImportError(
            "destination escapes project: {}".format(destination)
        )
    return resolved


def build_import_plan(manifest, upstream: Path, destination: Path) -> tuple:
    try:
        upstream = Path(upstream).resolve(strict=True)
        destination = Path(destination).resolve(strict=True)
    except (OSError, RuntimeError) as error:
        raise RuntimeImportError("import root does not exist") from error

    items = []
    for package in manifest.packages.values():
        if not package.imported:
            continue
        if package.source is None:
            raise RuntimeImportError(
                "imported package has no source: {}".format(package.name)
            )
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
        _validated_destination(item.destination, destination)
    return tuple(items)


def _copy_ignore(directory, names):
    ignored = {name for name in names if name in _IGNORED_NAMES}
    if Path(directory).name == "docs" and "results" in names:
        ignored.add("results")
    return ignored


def _raise_planned_collision(path: Path) -> None:
    raise RuntimeImportError("destination exists: {}".format(path))


def _obstructed_parent(path: Path, root: Path):
    for parent in Path(path).parents:
        if parent == root:
            return None
        if _lexists(parent) and not parent.is_dir():
            return parent
    return None


def _preflight_nested_pair(
    parent_item: ImportItem,
    parent_destination: Path,
    parent_index: int,
    nested_destination: Path,
    nested_index: int,
) -> None:
    relative = nested_destination.relative_to(parent_destination)

    # A descendant copied first necessarily creates the later ancestor target.
    if nested_index < parent_index:
        _raise_planned_collision(parent_destination)
    if not parent_item.source.is_dir():
        _raise_planned_collision(nested_destination)

    mapped = parent_item.source
    target = parent_item.source / relative
    parent_name = parent_item.source.name
    for part in relative.parts:
        if part in _IGNORED_NAMES or (
            parent_name == "docs" and part == "results"
        ):
            return
        mapped = mapped / part
        if not _lexists(mapped):
            return
        if mapped != target and not mapped.is_dir():
            _raise_planned_collision(nested_destination)
        parent_name = part
    if _lexists(mapped):
        _raise_planned_collision(nested_destination)


def _preflight_plan(plan, destination: Path) -> tuple:
    resolved_destinations = []
    existing = []
    provenance_path = destination / "config/import_provenance.json"
    resolved_provenance = provenance_path.resolve(strict=False)
    if not _is_within(resolved_provenance, destination):
        raise RuntimeImportError(
            "provenance destination escapes project: {}".format(
                provenance_path
            )
        )
    for item in plan:
        if not _lexists(item.source):
            raise RuntimeImportError(
                "source does not exist: {}".format(item.source)
            )
        resolved = _validated_destination(item.destination, destination)
        resolved_destinations.append(resolved)
        if _is_within(resolved, resolved_provenance) or _is_within(
            resolved_provenance, resolved
        ):
            existing.append(item.destination)
        if _lexists(item.destination):
            existing.append(item.destination)
        obstructed_parent = _obstructed_parent(
            item.destination, destination
        )
        if obstructed_parent is not None:
            existing.append(obstructed_parent)

    if _lexists(provenance_path):
        existing.append(provenance_path)
    config_path = destination / "config"
    if _lexists(config_path) and not config_path.is_dir():
        existing.append(config_path)
    if existing:
        raise RuntimeImportError(
            "destination exists: {}".format(
                ", ".join(str(path) for path in existing)
            )
        )

    for left_index, left_item in enumerate(plan):
        left_destination = resolved_destinations[left_index]
        for right_index in range(left_index + 1, len(plan)):
            right_item = plan[right_index]
            right_destination = resolved_destinations[right_index]
            if left_destination == right_destination:
                _raise_planned_collision(right_destination)
            if _is_within(right_destination, left_destination):
                _preflight_nested_pair(
                    left_item,
                    left_destination,
                    left_index,
                    right_destination,
                    right_index,
                )
            elif _is_within(left_destination, right_destination):
                _preflight_nested_pair(
                    right_item,
                    right_destination,
                    right_index,
                    left_destination,
                    left_index,
                )
    return tuple(
        ImportItem(item.name, item.source, resolved_destinations[index])
        for index, item in enumerate(plan)
    )


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _copy_item(item: ImportItem) -> None:
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


def _write_provenance_atomic(path: Path, provenance: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_name = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=str(path.parent),
            prefix=".import_provenance.",
            suffix=".tmp",
            delete=False,
        ) as stream:
            temporary_name = stream.name
            json.dump(provenance, stream, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_name, str(path))
        temporary_name = None
    finally:
        if temporary_name is not None and os.path.exists(temporary_name):
            os.unlink(temporary_name)


def apply_import_plan(plan, destination: Path, upstream_commit: str) -> dict:
    try:
        destination = Path(destination).resolve(strict=True)
    except (OSError, RuntimeError) as error:
        raise RuntimeImportError(
            "destination root does not exist: {}".format(destination)
        ) from error
    plan = tuple(plan)
    plan = _preflight_plan(plan, destination)

    for item in plan:
        _copy_item(item)

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
    _write_provenance_atomic(
        destination / "config/import_provenance.json", provenance
    )
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
    plan = build_import_plan(
        manifest, args.source_root, args.destination_root
    )
    if not args.apply:
        for item in plan:
            print("COPY {} -> {}".format(item.source, item.destination))
        return 0
    apply_import_plan(plan, args.destination_root, manifest.expected_commit)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
