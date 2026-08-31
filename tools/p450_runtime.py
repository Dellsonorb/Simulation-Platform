#!/usr/bin/python3
"""Validate the explicit read-only PX4 runtime dependency for P450 SITL."""

import argparse
import hashlib
import json
import os
import pwd
import re
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path, PurePosixPath


EXPECTED_REQUIRED_PACKAGES = (
    "prometheus_msgs",
    "realsense_ros_gazebo",
    "prometheus_gazebo",
    "prometheus_uav_control",
    "brick_aerial_perception",
    "sim_platform_assets",
    "livox_laser_gazebo_plugins",
)
EXPECTED_MODEL_FILES = (
    "Tools/sitl_gazebo/models/gps/model.config",
    "Tools/sitl_gazebo/models/gps/gps.sdf",
)
EXPECTED_PLUGINS = (
    "libgazebo_barometer_plugin.so",
    "libgazebo_gps_plugin.so",
    "libgazebo_groundtruth_plugin.so",
    "libgazebo_imu_plugin.so",
    "libgazebo_magnetometer_plugin.so",
    "libgazebo_mavlink_interface.so",
    "libgazebo_motor_model.so",
    "libgazebo_multirotor_base_plugin.so",
)
EXPECTED_SUPPORT_LIBRARIES = (
    "libmav_msgs.so",
    "libnav_msgs.so",
    "libphysics_msgs.so",
    "libsensor_msgs.so",
    "libstd_msgs.so",
)
EXPECTED_LOCAL_MODEL_ROOTS = (
    "gazebo_models/uav_models",
    "gazebo_models/sensor_models",
    "gazebo_models/scene_models",
    "gazebo_models/r200_models",
    "gazebo_models/texture",
)


class RuntimeConfigError(ValueError):
    """The checked-in runtime configuration is malformed or unsafe."""


class RuntimeValidationError(RuntimeError):
    """The caller-supplied PX4 checkout does not meet the runtime contract."""


@dataclass(frozen=True)
class PluginOverlaySpec:
    plugin: str
    upstream_source: str
    upstream_source_sha256: str
    patched_source_sha256: str
    upstream_header: str
    upstream_header_sha256: str
    generated_header: str
    generated_header_sha256: str


@dataclass(frozen=True)
class RuntimeConfig:
    schema_version: int
    ros_distribution: str
    ros_root: str
    install_space: str
    required_packages: tuple
    px4_commit: str
    sitl_gazebo_path: str
    sitl_gazebo_commit: str
    binary: str
    romfs: str
    startup_script: str
    jinja_generator: str
    model_root: str
    required_model_files: tuple
    plugin_root: str
    plugins: tuple
    support_libraries: tuple
    local_model_roots: tuple
    local_plugin_root: str
    local_plugin_specs: tuple
    overlay_support_library: str
    overlay_support_library_sha256: str
    overlay_common_header: str
    overlay_common_header_sha256: str


@dataclass(frozen=True)
class ResolvedPluginInput:
    spec: PluginOverlaySpec
    upstream_source: Path
    upstream_header: Path
    generated_header: Path


@dataclass(frozen=True)
class ResolvedPx4Checkout:
    root: Path
    binary: Path
    romfs: Path
    startup_script: Path
    jinja_generator: Path
    model_root: Path
    plugin_root: Path
    plugins: tuple
    support_libraries: tuple
    overlay_plugin_inputs: tuple
    overlay_support_library: Path
    overlay_common_header: Path


@dataclass(frozen=True)
class ResolvedPluginOverlay:
    root: Path
    plugins: tuple


class _DuplicateJsonKey(ValueError):
    pass


def _closed_object(value, keys, location):
    if not isinstance(value, dict) or set(value) != set(keys):
        raise RuntimeConfigError(
            "%s must contain exactly: %s" %
            (location, ", ".join(sorted(keys))))
    return value


def _json_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise _DuplicateJsonKey("duplicate JSON key: %s" % key)
        result[key] = value
    return result


def _tuple_of_strings(value, location):
    if (not isinstance(value, list) or not value or
            any(not isinstance(item, str) or not item for item in value)):
        raise RuntimeConfigError("%s must be a non-empty string list" % location)
    result = tuple(value)
    if len(result) != len(set(result)):
        raise RuntimeConfigError("%s must not contain duplicates" % location)
    return result


def _relative_path(value, location):
    if not isinstance(value, str) or not value or "\\" in value:
        raise RuntimeConfigError("%s must be a safe POSIX relative path" % location)
    path = PurePosixPath(value)
    if (path.is_absolute() or path.as_posix() != value or
            any(part in ("", ".", "..") for part in path.parts)):
        raise RuntimeConfigError("%s must be a safe POSIX relative path" % location)
    return value


def _commit(value, location):
    if not isinstance(value, str) or re.fullmatch(r"[0-9a-f]{40}", value) is None:
        raise RuntimeConfigError("%s must be a lowercase 40-character commit" % location)
    return value


def _sha256(value, location):
    if not isinstance(value, str) or re.fullmatch(r"[0-9a-f]{64}", value) is None:
        raise RuntimeConfigError("%s must be a lowercase SHA-256 digest" % location)
    return value


def _expect(value, expected, location):
    if value != expected:
        raise RuntimeConfigError("%s is not the supported value" % location)
    return value


def load_runtime_config(path):
    """Load a closed, deterministic runtime configuration."""
    config_path = Path(path)
    try:
        with config_path.open("r", encoding="utf-8") as stream:
            payload = json.load(stream, object_pairs_hook=_json_object)
    except (OSError, UnicodeError, json.JSONDecodeError, _DuplicateJsonKey) as error:
        raise RuntimeConfigError("unable to load %s: %s" % (config_path, error))

    top = _closed_object(payload, ("schema_version", "ros", "px4", "gazebo"), "root")
    if top["schema_version"] != 2:
        raise RuntimeConfigError("schema_version must be 2")

    ros = _closed_object(
        top["ros"],
        ("distribution", "root", "install_space", "required_packages"),
        "ros",
    )
    _expect(ros["distribution"], "noetic", "ros.distribution")
    _expect(ros["root"], "/opt/ros/noetic", "ros.root")
    _expect(ros["install_space"], "install/p450-clean", "ros.install_space")
    required_packages = _tuple_of_strings(
        ros["required_packages"], "ros.required_packages")
    _expect(required_packages, EXPECTED_REQUIRED_PACKAGES, "ros.required_packages")

    px4 = _closed_object(top["px4"], ("commit", "sitl_gazebo", "artifacts"), "px4")
    sitl = _closed_object(px4["sitl_gazebo"], ("path", "commit"), "px4.sitl_gazebo")
    artifacts = _closed_object(
        px4["artifacts"],
        (
            "binary", "romfs", "startup_script", "jinja_generator",
            "model_root", "required_model_files", "plugin_root", "plugins",
            "support_libraries",
        ),
        "px4.artifacts",
    )

    paths = {
        "sitl_gazebo_path": _relative_path(sitl["path"], "px4.sitl_gazebo.path"),
        "binary": _relative_path(artifacts["binary"], "px4.artifacts.binary"),
        "romfs": _relative_path(artifacts["romfs"], "px4.artifacts.romfs"),
        "startup_script": _relative_path(
            artifacts["startup_script"], "px4.artifacts.startup_script"),
        "jinja_generator": _relative_path(
            artifacts["jinja_generator"], "px4.artifacts.jinja_generator"),
        "model_root": _relative_path(
            artifacts["model_root"], "px4.artifacts.model_root"),
        "plugin_root": _relative_path(
            artifacts["plugin_root"], "px4.artifacts.plugin_root"),
    }
    expected_paths = {
        "sitl_gazebo_path": "Tools/sitl_gazebo",
        "binary": "build/amovlab_sitl_default/bin/px4",
        "romfs": "ROMFS/px4fmu_common",
        "startup_script": "ROMFS/px4fmu_common/init.d-posix/rcS",
        "jinja_generator": "Tools/sitl_gazebo/scripts/jinja_gen.py",
        "model_root": "Tools/sitl_gazebo/models",
        "plugin_root": "build/amovlab_sitl_default/build_gazebo",
    }
    for name, expected in expected_paths.items():
        _expect(paths[name], expected, name)

    required_model_files = _tuple_of_strings(
        artifacts["required_model_files"], "px4.artifacts.required_model_files")
    for index, value in enumerate(required_model_files):
        _relative_path(value, "px4.artifacts.required_model_files[%d]" % index)
    _expect(required_model_files, EXPECTED_MODEL_FILES, "px4.artifacts.required_model_files")

    plugins = _tuple_of_strings(artifacts["plugins"], "px4.artifacts.plugins")
    support_libraries = _tuple_of_strings(
        artifacts["support_libraries"], "px4.artifacts.support_libraries")
    _expect(plugins, EXPECTED_PLUGINS, "px4.artifacts.plugins")
    _expect(support_libraries, EXPECTED_SUPPORT_LIBRARIES, "px4.artifacts.support_libraries")
    for name in plugins + support_libraries:
        if PurePosixPath(name).name != name or not name.endswith(".so"):
            raise RuntimeConfigError("plugin/library names must be .so basenames")

    gazebo = _closed_object(
        top["gazebo"],
        ("local_model_roots", "local_plugin_overlay"),
        "gazebo",
    )
    local_model_roots = _tuple_of_strings(
        gazebo["local_model_roots"], "gazebo.local_model_roots")
    for index, value in enumerate(local_model_roots):
        _relative_path(value, "gazebo.local_model_roots[%d]" % index)
    _expect(local_model_roots, EXPECTED_LOCAL_MODEL_ROOTS, "gazebo.local_model_roots")

    overlay = _closed_object(
        gazebo["local_plugin_overlay"],
        (
            "plugin_root", "support_library", "support_library_sha256",
            "common_header", "common_header_sha256", "plugins",
        ),
        "gazebo.local_plugin_overlay",
    )
    local_plugin_root = _relative_path(
        overlay["plugin_root"], "gazebo.local_plugin_overlay.plugin_root")
    overlay_support_library = _relative_path(
        overlay["support_library"],
        "gazebo.local_plugin_overlay.support_library",
    )
    overlay_common_header = _relative_path(
        overlay["common_header"],
        "gazebo.local_plugin_overlay.common_header",
    )
    _expect(
        local_plugin_root,
        "install/p450-runtime-overlays/lib",
        "gazebo.local_plugin_overlay.plugin_root",
    )
    _expect(
        overlay_support_library,
        "build/amovlab_sitl_default/build_gazebo/libsensor_msgs.so",
        "gazebo.local_plugin_overlay.support_library",
    )
    _expect(
        overlay_common_header,
        "Tools/sitl_gazebo/include/common.h",
        "gazebo.local_plugin_overlay.common_header",
    )

    raw_plugin_specs = overlay["plugins"]
    if not isinstance(raw_plugin_specs, list) or not raw_plugin_specs:
        raise RuntimeConfigError(
            "gazebo.local_plugin_overlay.plugins must be a non-empty list")
    local_plugin_specs = []
    for index, raw_spec in enumerate(raw_plugin_specs):
        location = "gazebo.local_plugin_overlay.plugins[%d]" % index
        spec = _closed_object(
            raw_spec,
            (
                "plugin", "upstream_source", "upstream_source_sha256",
                "patched_source_sha256", "upstream_header",
                "upstream_header_sha256", "generated_header",
                "generated_header_sha256",
            ),
            location,
        )
        plugin = spec["plugin"]
        if (not isinstance(plugin, str) or
                PurePosixPath(plugin).name != plugin or
                not plugin.endswith(".so")):
            raise RuntimeConfigError(
                "%s.plugin must be a .so basename" % location)
        local_plugin_specs.append(PluginOverlaySpec(
            plugin=plugin,
            upstream_source=_relative_path(
                spec["upstream_source"], location + ".upstream_source"),
            upstream_source_sha256=_sha256(
                spec["upstream_source_sha256"],
                location + ".upstream_source_sha256",
            ),
            patched_source_sha256=_sha256(
                spec["patched_source_sha256"],
                location + ".patched_source_sha256",
            ),
            upstream_header=_relative_path(
                spec["upstream_header"], location + ".upstream_header"),
            upstream_header_sha256=_sha256(
                spec["upstream_header_sha256"],
                location + ".upstream_header_sha256",
            ),
            generated_header=_relative_path(
                spec["generated_header"], location + ".generated_header"),
            generated_header_sha256=_sha256(
                spec["generated_header_sha256"],
                location + ".generated_header_sha256",
            ),
        ))
    local_plugin_specs = tuple(local_plugin_specs)
    plugin_names = tuple(spec.plugin for spec in local_plugin_specs)
    if len(plugin_names) != len(set(plugin_names)):
        raise RuntimeConfigError(
            "gazebo.local_plugin_overlay.plugins must not contain duplicates")
    expected_plugin_inputs = (
        (
            "libgazebo_gps_plugin.so",
            "Tools/sitl_gazebo/src/gazebo_gps_plugin.cpp",
            "Tools/sitl_gazebo/include/gazebo_gps_plugin.h",
            "build/amovlab_sitl_default/build_gazebo/SITLGps.pb.h",
        ),
        (
            "libgazebo_groundtruth_plugin.so",
            "Tools/sitl_gazebo/src/gazebo_groundtruth_plugin.cpp",
            "Tools/sitl_gazebo/include/gazebo_groundtruth_plugin.h",
            "build/amovlab_sitl_default/build_gazebo/Groundtruth.pb.h",
        ),
    )
    actual_plugin_inputs = tuple(
        (
            spec.plugin, spec.upstream_source, spec.upstream_header,
            spec.generated_header,
        )
        for spec in local_plugin_specs
    )
    _expect(
        actual_plugin_inputs,
        expected_plugin_inputs,
        "gazebo.local_plugin_overlay.plugins",
    )

    return RuntimeConfig(
        schema_version=2,
        ros_distribution=ros["distribution"],
        ros_root=ros["root"],
        install_space=ros["install_space"],
        required_packages=required_packages,
        px4_commit=_commit(px4["commit"], "px4.commit"),
        sitl_gazebo_path=paths["sitl_gazebo_path"],
        sitl_gazebo_commit=_commit(sitl["commit"], "px4.sitl_gazebo.commit"),
        binary=paths["binary"],
        romfs=paths["romfs"],
        startup_script=paths["startup_script"],
        jinja_generator=paths["jinja_generator"],
        model_root=paths["model_root"],
        required_model_files=required_model_files,
        plugin_root=paths["plugin_root"],
        plugins=plugins,
        support_libraries=support_libraries,
        local_model_roots=local_model_roots,
        local_plugin_root=local_plugin_root,
        local_plugin_specs=local_plugin_specs,
        overlay_support_library=overlay_support_library,
        overlay_support_library_sha256=_sha256(
            overlay["support_library_sha256"],
            "gazebo.local_plugin_overlay.support_library_sha256",
        ),
        overlay_common_header=overlay_common_header,
        overlay_common_header_sha256=_sha256(
            overlay["common_header_sha256"],
            "gazebo.local_plugin_overlay.common_header_sha256",
        ),
    )


def _clean_process_environment(extra=None):
    identity = pwd.getpwuid(os.getuid())
    environment = {
        "HOME": identity.pw_dir,
        "USER": identity.pw_name,
        "LOGNAME": identity.pw_name,
        "LANG": "C.UTF-8",
        "LC_ALL": "C.UTF-8",
        "PATH": "/usr/bin:/bin:/usr/sbin:/sbin",
        "GIT_CONFIG_NOSYSTEM": "1",
        "GIT_CONFIG_GLOBAL": "/dev/null",
        "GIT_OPTIONAL_LOCKS": "0",
    }
    if extra:
        environment.update(extra)
    return environment


def _run_git(root, arguments):
    result = subprocess.run(
        ["/usr/bin/git"] + list(arguments),
        cwd=str(root),
        env=_clean_process_environment(),
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        detail = result.stderr.strip() or result.stdout.strip()
        raise RuntimeValidationError(
            "git validation failed at %s: %s" % (root, detail))
    return result.stdout.strip()


def _is_within(path, parent):
    try:
        path.relative_to(parent)
    except ValueError:
        return False
    return True


def _require_path(root, relative, kind, executable=False):
    path = root / relative
    try:
        resolved = path.resolve(strict=True)
    except OSError as error:
        raise RuntimeValidationError("required artifact is missing: %s (%s)" % (path, error))
    if not _is_within(resolved, root):
        raise RuntimeValidationError("required artifact escapes PX4 root: %s" % path)
    if kind == "file" and not resolved.is_file():
        raise RuntimeValidationError("required artifact is not a file: %s" % path)
    if kind == "directory" and not resolved.is_dir():
        raise RuntimeValidationError("required artifact is not a directory: %s" % path)
    if executable and not os.access(str(resolved), os.X_OK):
        raise RuntimeValidationError("required executable is not executable: %s" % path)
    return resolved


def _default_ldd_runner(path, environment):
    result = subprocess.run(
        ["/usr/bin/ldd", str(path)],
        env=environment,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        raise RuntimeValidationError(
            "ldd failed for %s: %s" % (path, result.stdout.strip()))
    return result.stdout


def _default_dynamic_runner(path):
    result = subprocess.run(
        ["/usr/bin/readelf", "-d", str(path)],
        env=_clean_process_environment(),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        raise RuntimeValidationError(
            "readelf failed for %s: %s" % (path, result.stdout.strip()))
    return result.stdout


def _default_header_runner(path):
    result = subprocess.run(
        ["/usr/bin/readelf", "-h", str(path)],
        env=_clean_process_environment(),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        raise RuntimeValidationError(
            "readelf header inspection failed for %s: %s" %
            (path, result.stdout.strip()))
    return result.stdout


def _default_symbol_runner(path):
    result = subprocess.run(
        ["/usr/bin/readelf", "-WsW", str(path)],
        env=_clean_process_environment(),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        raise RuntimeValidationError(
            "readelf symbol inspection failed for %s: %s" %
            (path, result.stdout.strip()))
    return result.stdout


def _file_sha256(path):
    digest = hashlib.sha256()
    try:
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as error:
        raise RuntimeValidationError(
            "unable to hash required artifact %s: %s" % (path, error))
    return digest.hexdigest()


def _require_sha256(path, expected, label):
    observed = _file_sha256(path)
    if observed != expected:
        raise RuntimeValidationError(
            "%s SHA-256 mismatch: expected %s, observed %s" %
            (label, expected, observed))


def _validate_ldd(plugin, output, plugin_root, support_libraries):
    if not isinstance(output, str):
        raise RuntimeValidationError("ldd runner returned non-text output for %s" % plugin)
    supported = set(support_libraries)
    resolutions = {}
    for line in output.splitlines():
        library, separator, resolution = line.partition("=>")
        library = library.strip()
        if not separator:
            continue
        resolution = re.sub(
            r"\s+\(0x[0-9a-fA-F]+\)\s*$", "", resolution).strip()
        if resolution.lower() == "not found":
            raise RuntimeValidationError(
                "unresolved library dependency for %s" % plugin)
        if library not in supported:
            continue
        resolved = Path(resolution).resolve(strict=False)
        expected = (plugin_root / library).resolve(strict=True)
        if resolved != expected:
            raise RuntimeValidationError(
                "%s resolved outside the pinned plugin root: %s" %
                (library, resolved))
        resolutions[library] = resolved
    return resolutions


def validate_px4_checkout(config, px4_root, repository_root, ldd_runner=None):
    """Validate a caller-owned checkout without changing it."""
    supplied = Path(px4_root)
    if not supplied.is_absolute():
        raise RuntimeValidationError("P450_PX4_ROOT must be absolute")
    try:
        root = supplied.resolve(strict=True)
    except OSError as error:
        raise RuntimeValidationError("P450_PX4_ROOT is unavailable: %s" % error)
    if not root.is_dir():
        raise RuntimeValidationError("P450_PX4_ROOT is not a directory: %s" % root)
    if ":" in str(root) or "\n" in str(root) or "\r" in str(root):
        raise RuntimeValidationError(
            "P450_PX4_ROOT cannot contain path-list delimiters or newlines")

    repository = Path(repository_root).resolve(strict=False)
    if _is_within(root, repository) or _is_within(repository, root):
        raise RuntimeValidationError(
            "P450_PX4_ROOT and the SIM repository must not overlap")

    top_level = Path(_run_git(root, ("rev-parse", "--show-toplevel"))).resolve(strict=True)
    if top_level != root:
        raise RuntimeValidationError("P450_PX4_ROOT is not the PX4 Git root")
    if _run_git(root, ("rev-parse", "HEAD")) != config.px4_commit:
        raise RuntimeValidationError("PX4 checkout commit does not match p450_runtime.json")
    parent_dirty = _run_git(
        root,
        ("status", "--porcelain=v1", "--untracked-files=no", "--ignore-submodules=dirty"),
    )
    if parent_dirty:
        raise RuntimeValidationError("PX4 checkout has tracked modifications")

    sitl_root = _require_path(root, config.sitl_gazebo_path, "directory")
    child_top = Path(_run_git(sitl_root, ("rev-parse", "--show-toplevel"))).resolve(strict=True)
    if child_top != sitl_root:
        raise RuntimeValidationError("SITL Gazebo path is not its Git root")
    if _run_git(sitl_root, ("rev-parse", "HEAD")) != config.sitl_gazebo_commit:
        raise RuntimeValidationError("SITL Gazebo checkout commit does not match p450_runtime.json")
    child_dirty = _run_git(
        sitl_root,
        ("status", "--porcelain=v1", "--untracked-files=no", "--ignore-submodules=dirty"),
    )
    if child_dirty:
        raise RuntimeValidationError("SITL Gazebo checkout has tracked modifications")

    gitlink = _run_git(root, ("ls-tree", "HEAD", "--", config.sitl_gazebo_path))
    expected_gitlink = "160000 commit %s\t%s" % (
        config.sitl_gazebo_commit, config.sitl_gazebo_path)
    if gitlink != expected_gitlink:
        raise RuntimeValidationError("PX4 Git tree does not pin the expected SITL Gazebo gitlink")

    binary = _require_path(root, config.binary, "file", executable=True)
    romfs = _require_path(root, config.romfs, "directory")
    startup_script = _require_path(root, config.startup_script, "file")
    jinja_generator = _require_path(root, config.jinja_generator, "file")
    model_root = _require_path(root, config.model_root, "directory")
    for relative in config.required_model_files:
        _require_path(root, relative, "file")
    plugin_root = _require_path(root, config.plugin_root, "directory")
    plugins = tuple(
        _require_path(plugin_root, name, "file") for name in config.plugins)
    support_libraries = tuple(
        _require_path(plugin_root, name, "file")
        for name in config.support_libraries)
    overlay_plugin_inputs = []
    for spec in config.local_plugin_specs:
        upstream_source = _require_path(
            root, spec.upstream_source, "file")
        upstream_header = _require_path(
            root, spec.upstream_header, "file")
        generated_header = _require_path(
            root, spec.generated_header, "file")
        _require_sha256(
            upstream_source,
            spec.upstream_source_sha256,
            "%s overlay upstream source" % spec.plugin,
        )
        _require_sha256(
            upstream_header,
            spec.upstream_header_sha256,
            "%s overlay upstream header" % spec.plugin,
        )
        _require_sha256(
            generated_header,
            spec.generated_header_sha256,
            "%s overlay generated header" % spec.plugin,
        )
        overlay_plugin_inputs.append(ResolvedPluginInput(
            spec=spec,
            upstream_source=upstream_source,
            upstream_header=upstream_header,
            generated_header=generated_header,
        ))
    overlay_support_library = _require_path(
        root, config.overlay_support_library, "file")
    overlay_common_header = _require_path(
        root, config.overlay_common_header, "file")
    _require_sha256(
        overlay_support_library,
        config.overlay_support_library_sha256,
        "PX4 Gazebo overlay support library",
    )
    _require_sha256(
        overlay_common_header,
        config.overlay_common_header_sha256,
        "PX4 Gazebo overlay common header",
    )

    environment = _clean_process_environment({"LD_LIBRARY_PATH": str(plugin_root)})
    runner = ldd_runner or _default_ldd_runner
    for plugin in plugins:
        try:
            output = runner(plugin, dict(environment))
        except RuntimeValidationError:
            raise
        except Exception as error:
            raise RuntimeValidationError("ldd failed for %s: %s" % (plugin, error))
        _validate_ldd(plugin, output, plugin_root, config.support_libraries)

    return ResolvedPx4Checkout(
        root=root,
        binary=binary,
        romfs=romfs,
        startup_script=startup_script,
        jinja_generator=jinja_generator,
        model_root=model_root,
        plugin_root=plugin_root,
        plugins=plugins,
        support_libraries=support_libraries,
        overlay_plugin_inputs=tuple(overlay_plugin_inputs),
        overlay_support_library=overlay_support_library,
        overlay_common_header=overlay_common_header,
    )


def validate_runtime_plugin_overlay(
        config, repository_root, px4_checkout, ldd_runner=None,
        header_runner=None, dynamic_runner=None, symbol_runner=None):
    """Validate the SIM-owned plugin override and its dynamic boundary."""
    supplied_repository = Path(repository_root)
    if not supplied_repository.is_absolute():
        raise RuntimeValidationError("repository root must be absolute")
    try:
        repository = supplied_repository.resolve(strict=True)
    except OSError as error:
        raise RuntimeValidationError("repository root is unavailable: %s" % error)
    if not repository.is_dir():
        raise RuntimeValidationError(
            "repository root is not a directory: %s" % repository)
    if ":" in str(repository) or "\n" in str(repository) or "\r" in str(repository):
        raise RuntimeValidationError(
            "repository root cannot contain path-list delimiters or newlines")

    candidate_root = repository / config.local_plugin_root
    try:
        root = candidate_root.resolve(strict=True)
    except OSError as error:
        raise RuntimeValidationError(
            "local plugin overlay root is missing: %s (%s)" %
            (candidate_root, error))
    if not _is_within(root, repository) or not root.is_dir():
        raise RuntimeValidationError(
            "local plugin overlay root must be a directory inside SIM: %s" %
            candidate_root)

    environment = _clean_process_environment({
        "LD_LIBRARY_PATH": "%s:%s" % (root, px4_checkout.plugin_root),
    })
    runner = ldd_runner or _default_ldd_runner
    required_support = Path(config.overlay_support_library).name
    inspect_header = header_runner or _default_header_runner
    inspect_dynamic = dynamic_runner or _default_dynamic_runner
    inspect_symbols = symbol_runner or _default_symbol_runner
    plugins = []
    for spec in config.local_plugin_specs:
        candidate_plugin = candidate_root / spec.plugin
        if candidate_plugin.is_symlink():
            raise RuntimeValidationError(
                "local plugin overlay must be a materialized file: %s" %
                candidate_plugin)
        try:
            plugin = candidate_plugin.resolve(strict=True)
        except OSError as error:
            raise RuntimeValidationError(
                "local plugin overlay is missing: %s (%s)" %
                (candidate_plugin, error))
        if not _is_within(plugin, root) or not plugin.is_file():
            raise RuntimeValidationError(
                "local plugin overlay must remain inside its plugin root: %s" %
                candidate_plugin)

        try:
            ldd_output = runner(plugin, dict(environment))
        except RuntimeValidationError:
            raise
        except Exception as error:
            raise RuntimeValidationError(
                "ldd failed for %s: %s" % (plugin, error))
        resolutions = _validate_ldd(
            plugin,
            ldd_output,
            px4_checkout.plugin_root,
            config.support_libraries,
        )
        if required_support not in resolutions:
            raise RuntimeValidationError(
                "local plugin overlay does not dynamically resolve %s" %
                required_support)
        if resolutions[required_support] != px4_checkout.overlay_support_library:
            raise RuntimeValidationError(
                "local plugin overlay resolved the wrong %s" % required_support)

        try:
            header_output = inspect_header(plugin)
        except RuntimeValidationError:
            raise
        except Exception as error:
            raise RuntimeValidationError(
                "readelf header inspection failed for %s: %s" %
                (plugin, error))
        if not isinstance(header_output, str):
            raise RuntimeValidationError(
                "readelf header runner returned non-text output for %s" %
                plugin)
        required_header_fields = (
            r"(?m)^\s*Class:\s+ELF64\s*$",
            r"(?m)^\s*Type:\s+DYN \(Shared object file\)\s*$",
            r"(?m)^\s*Machine:\s+Advanced Micro Devices X86-64\s*$",
        )
        if any(re.search(pattern, header_output) is None
               for pattern in required_header_fields):
            raise RuntimeValidationError(
                "local plugin overlay has an unsupported ELF header: %s" %
                plugin)

        try:
            dynamic_output = inspect_dynamic(plugin)
        except RuntimeValidationError:
            raise
        except Exception as error:
            raise RuntimeValidationError(
                "readelf failed for %s: %s" % (plugin, error))
        if not isinstance(dynamic_output, str):
            raise RuntimeValidationError(
                "readelf runner returned non-text output for %s" % plugin)
        if re.search(r"\((?:RPATH|RUNPATH)\)", dynamic_output):
            raise RuntimeValidationError(
                "local plugin overlay must not embed RPATH or RUNPATH: %s" %
                plugin)
        needed = re.findall(r"\(NEEDED\).*?\[([^\]]+)\]", dynamic_output)
        if required_support not in needed:
            raise RuntimeValidationError(
                "local plugin overlay DT_NEEDED omits %s" % required_support)
        sonames = re.findall(r"\(SONAME\).*?\[([^\]]+)\]", dynamic_output)
        if sonames != [spec.plugin]:
            raise RuntimeValidationError(
                "local plugin overlay has an invalid DT_SONAME: %s" % sonames)

        try:
            symbol_output = inspect_symbols(plugin)
        except RuntimeValidationError:
            raise
        except Exception as error:
            raise RuntimeValidationError(
                "readelf symbol inspection failed for %s: %s" %
                (plugin, error))
        if not isinstance(symbol_output, str):
            raise RuntimeValidationError(
                "readelf symbol runner returned non-text output for %s" % plugin)
        if re.search(
                r"(?m)^\s*\d+:\s+[0-9a-fA-F]+\s+\d+\s+FUNC\s+"
                r"(?:GLOBAL|WEAK)\s+\S+\s+(?!UND\b)\S+\s+"
                r"RegisterPlugin\s*$",
                symbol_output) is None:
            raise RuntimeValidationError(
                "local plugin overlay does not export RegisterPlugin")
        if ("gazebo::physics::World::Reset()" in symbol_output or
                "_ZN6gazebo7physics5World5ResetEv" in symbol_output):
            raise RuntimeValidationError(
                "local plugin overlay still references World::Reset()")
        plugins.append(plugin)

    return ResolvedPluginOverlay(root=root, plugins=tuple(plugins))


def _parse_arguments(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--px4-root", required=True)
    parser.add_argument("--repository-root", required=True)
    parser.add_argument(
        "--external-only",
        action="store_true",
        help="validate only the read-only PX4 checkout before building overlays",
    )
    return parser.parse_args(argv)


def main(argv=None):
    arguments = _parse_arguments(argv)
    try:
        config = load_runtime_config(arguments.config)
        resolved = validate_px4_checkout(
            config, arguments.px4_root, arguments.repository_root)
        if not arguments.external_only:
            validate_runtime_plugin_overlay(
                config, arguments.repository_root, resolved)
    except (RuntimeConfigError, RuntimeValidationError) as error:
        print("P450 runtime validation failed: %s" % error, file=sys.stderr)
        return 65
    print(resolved.root)
    return 0


if __name__ == "__main__":
    sys.exit(main())
