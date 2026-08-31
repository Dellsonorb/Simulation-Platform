import json
from dataclasses import dataclass
from pathlib import Path, PurePosixPath, PureWindowsPath
from typing import Dict, Optional, Tuple


class ManifestError(ValueError):
    pass


def _relative_path(value: str) -> str:
    try:
        path = PurePosixPath(value)
    except TypeError as error:
        raise ManifestError("relative path escape") from error
    if (
        not path.parts
        or path.is_absolute()
        or ".." in path.parts
        or "\\" in str(value)
        or PureWindowsPath(value).drive
    ):
        raise ManifestError("relative path escape")
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
    def load(cls, path: Path) -> "RuntimeManifest":
        payload = json.loads(Path(path).read_text(encoding="utf-8"))
        schema_version = payload.get("schema_version")
        if schema_version != 1:
            raise ManifestError("unsupported schema_version")

        required_paths = tuple(
            _relative_path(required_path)
            for required_path in payload["layout"]["required_paths"]
        )

        package_destinations = set()
        packages = {}
        supported_roles = {"p450", "ground", "vendor", "platform", "demo"}
        for name, package in payload["packages"].items():
            role = package["role"]
            if role not in supported_roles:
                raise ManifestError("unsupported role")

            source_value = package.get("source")
            source = (
                None if source_value is None else _relative_path(source_value)
            )
            destination = _relative_path(package["destination"])
            if destination in package_destinations:
                raise ManifestError("duplicate destination")
            package_destinations.add(destination)
            packages[name] = PackageSpec(
                name=name,
                role=role,
                source=source,
                destination=destination,
                imported=bool(package["imported"]),
            )

        auxiliary_imports = []
        all_destinations = set(package_destinations)
        for auxiliary in payload.get("auxiliary_imports", []):
            source = _relative_path(auxiliary["source"])
            destination = _relative_path(auxiliary["destination"])
            if destination in all_destinations:
                raise ManifestError(
                    "duplicate destination in auxiliary imports"
                )
            all_destinations.add(destination)
            auxiliary_imports.append(
                AuxiliaryImport(
                    name=auxiliary["name"],
                    source=source,
                    destination=destination,
                )
            )

        forbidden = payload["forbidden"]
        return cls(
            schema_version=schema_version,
            expected_commit=payload["upstream"]["expected_commit"],
            required_paths=required_paths,
            packages=packages,
            auxiliary_imports=tuple(auxiliary_imports),
            forbidden_packages=tuple(forbidden["package_names"]),
            forbidden_tokens=tuple(forbidden["runtime_tokens"]),
        )
