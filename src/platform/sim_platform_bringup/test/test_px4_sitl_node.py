#!/usr/bin/env python3
"""Offline security contract for the platform PX4 SITL node wrapper."""

import os
import subprocess
import tempfile
import unittest
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WRAPPER = PACKAGE_ROOT / "scripts" / "px4_sitl_node.bash"


class Px4SitlNodeWorkdirContractTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.px4_root = self.root / "px4 root"
        self.ros_home = self.root / "ros home"
        self.caller_cwd = self.root / "caller cwd"
        self.ros_home.mkdir()
        self.caller_cwd.mkdir()

        binary = self.px4_root / "build/amovlab_sitl_default/bin/px4"
        binary.parent.mkdir(parents=True)
        binary.write_text(
            "#!/bin/bash\n"
            "set -euo pipefail\n"
            "printf 'cwd=<%s>\\n' \"$PWD\"\n"
            "printf 'arg=<%s>\\n' \"$@\"\n",
            encoding="utf-8",
        )
        binary.chmod(0o755)

    def run_wrapper(self, arguments, include_ros_home=True):
        environment = {
            "P450_PX4_ROOT": str(self.px4_root.resolve()),
            "PATH": "/usr/bin:/bin",
            "LANG": "C.UTF-8",
            "LC_ALL": "C.UTF-8",
        }
        if include_ros_home:
            environment["ROS_HOME"] = str(self.ros_home.resolve())
        return subprocess.run(
            [str(WRAPPER), *arguments],
            cwd=str(self.caller_cwd),
            env=environment,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=3.0,
            check=False,
        )

    def assert_rejected(self, arguments, expected_error):
        result = self.run_wrapper(arguments)
        self.assertNotEqual(
            0,
            result.returncode,
            "unexpected success: stdout={!r} stderr={!r}".format(
                result.stdout, result.stderr),
        )
        self.assertIn(expected_error, result.stderr)

    def test_valid_component_resolves_from_ros_home_and_preserves_arguments(self):
        arguments = [
            "ROMFS path",
            "-s",
            "etc/init.d-posix/rcS",
            "-i",
            "0",
            "-w",
            "sitl_smoke_20260831T181144Z-634853",
            "-d",
            "two words",
        ]

        result = self.run_wrapper(arguments)

        self.assertEqual(0, result.returncode, result.stderr)
        expected_lines = ["cwd=<%s>" % self.ros_home.resolve()]
        expected_lines.extend("arg=<%s>" % argument for argument in arguments)
        self.assertEqual("\n".join(expected_lines) + "\n", result.stdout)

    def test_requires_exactly_one_workdir_option_with_a_value(self):
        cases = (
            ["ROMFS", "-d"],
            ["ROMFS", "-w"],
            ["ROMFS", "-w", "first", "-w", "second", "-d"],
        )
        for arguments in cases:
            with self.subTest(arguments=arguments):
                self.assert_rejected(arguments, "exactly one -w WORKDIR")

    def test_rejects_unsafe_workdir_components(self):
        cases = (
            "/tmp/outside",
            "../outside",
            "nested/workdir",
            ".",
            "..",
            ".hidden",
            "-another-option",
            "two words",
            "back\\slash",
        )
        for component in cases:
            with self.subTest(component=component):
                self.assert_rejected(
                    ["ROMFS", "-w", component, "-d"],
                    "safe single path component",
                )

    def test_rejects_workdir_symlink_escape(self):
        outside = self.root / "outside"
        outside.mkdir()
        (self.ros_home / "escape").symlink_to(
            outside, target_is_directory=True)

        self.assert_rejected(
            ["ROMFS", "-w", "escape", "-d"],
            "strictly inside ROS_HOME",
        )

    def test_requires_ros_home(self):
        result = self.run_wrapper(
            ["ROMFS", "-w", "sitl_amov_0", "-d"],
            include_ros_home=False,
        )

        self.assertNotEqual(0, result.returncode)
        self.assertIn("ROS_HOME", result.stderr)


if __name__ == "__main__":
    unittest.main()
