import tempfile
import unittest
from pathlib import Path

from bunker_sim_runtime.contracts import (
    DECIMAL_RE,
    DEFAULT_POSE,
    ENTITY_NAME_RE,
    LOCAL_NAME_RE,
    MODEL_NAME,
    ROS_NAMESPACE,
    SPAWN_EXECUTABLE,
    TF_PREFIX,
    RuntimeContractError,
    parse_pose,
    validate_entity_name,
    validate_local_name,
    validate_spawn_command,
)


ROOT = Path(__file__).resolve().parents[4]
TEMP_ROOT = ROOT / "logs/bunker_standalone/engineering-tmp"


class RuntimeContractsTest(unittest.TestCase):
    def setUp(self):
        TEMP_ROOT.mkdir(mode=0o700, parents=True, exist_ok=True)
        self.temporary = tempfile.TemporaryDirectory(dir=str(TEMP_ROOT))
        self.addCleanup(self.temporary.cleanup)
        self.log_dir = Path(self.temporary.name) / "ros-log"
        self.log_dir.mkdir(mode=0o700)
        self.command = [
            SPAWN_EXECUTABLE,
            "-urdf", "-param", "robot_description",
            "-model", "bunker", "-b",
            "-x", DEFAULT_POSE[0], "-y", DEFAULT_POSE[1],
            "-z", DEFAULT_POSE[2], "-R", DEFAULT_POSE[3],
            "-P", DEFAULT_POSE[4], "-Y", DEFAULT_POSE[5],
            "__name:=spawn_bunker",
            "__log:=%s" % (self.log_dir / "ground-spawn_bunker-1.log"),
        ]

    def test_frozen_identity_and_grammars(self):
        self.assertEqual("bunker", MODEL_NAME)
        self.assertEqual("/ground", ROS_NAMESPACE)
        self.assertEqual("ground", TF_PREFIX)
        self.assertEqual(
            ("0.0", "0.0", "0.36", "0.0", "0.0", "0.0"),
            DEFAULT_POSE,
        )
        self.assertEqual(
            r"^[A-Za-z][A-Za-z0-9_]{0,63}$", LOCAL_NAME_RE.pattern)
        self.assertEqual(
            r"^[a-z][a-z0-9_]{0,63}$", ENTITY_NAME_RE.pattern)
        self.assertEqual(
            r"^[+-]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)(?:[eE][+-]?[0-9]+)?$",
            DECIMAL_RE.pattern,
        )
        self.assertEqual(
            "/opt/ros/noetic/lib/gazebo_ros/spawn_model",
            SPAWN_EXECUTABLE,
        )

    def test_validates_local_and_entity_names(self):
        for value in ("base_link", "Wheel_1", "A"):
            self.assertEqual(value, validate_local_name(value))
        for value in ("bunker", "bunker_1", "b"):
            self.assertEqual(value, validate_entity_name(value))
        invalid = ("", ".", "..", "a/b", "a b", "<a>", "a" * 65)
        for value in invalid:
            with self.subTest(kind="local", value=value):
                with self.assertRaises(RuntimeContractError):
                    validate_local_name(value)
            with self.subTest(kind="entity", value=value):
                with self.assertRaises(RuntimeContractError):
                    validate_entity_name(value)
        for value in ("Bunker", "1bunker"):
            with self.assertRaises(RuntimeContractError):
                validate_entity_name(value)

    def test_parses_exactly_six_finite_decimal_tokens_in_order(self):
        tokens = ("-1", "+2.5", ".3", "4.", "5e-2", "6E+1")
        self.assertEqual(
            (-1.0, 2.5, 0.3, 4.0, 0.05, 60.0), parse_pose(tokens))
        self.assertEqual(tokens, ("-1", "+2.5", ".3", "4.", "5e-2", "6E+1"))
        for values in (
                (), DEFAULT_POSE[:-1], DEFAULT_POSE + ("0",),
                ("nan",) + DEFAULT_POSE[1:],
                ("inf",) + DEFAULT_POSE[1:],
                ("0x1p0",) + DEFAULT_POSE[1:],
                ("1e9999",) + DEFAULT_POSE[1:],
                "0 0 0 0 0 0"):
            with self.subTest(values=values):
                with self.assertRaises(RuntimeContractError):
                    parse_pose(values)

    def test_accepts_only_the_frozen_spawn_vector(self):
        self.assertEqual(
            tuple(self.command),
            validate_spawn_command(
                self.command, DEFAULT_POSE, str(self.log_dir)),
        )
        self.assertEqual(21, len(self.command))
        self.assertEqual(19, len(self.command[:19]))

    def test_rejects_spawn_command_mutations(self):
        mutations = []
        noncanonical = list(self.command)
        noncanonical[0] = "/opt/ros/noetic/lib/gazebo_ros/../gazebo_ros/spawn_model"
        mutations.append(noncanonical)
        omitted_background = list(self.command)
        omitted_background.pop(6)
        mutations.append(omitted_background)
        reordered = list(self.command)
        reordered[7:11] = ["-y", DEFAULT_POSE[1], "-x", DEFAULT_POSE[0]]
        mutations.append(reordered)
        duplicated = list(self.command)
        duplicated[9] = "-x"
        mutations.append(duplicated)
        absolute_param = list(self.command)
        absolute_param[3] = "/robot_description"
        mutations.append(absolute_param)
        wrong_model = list(self.command)
        wrong_model[5] = "bunker_2"
        mutations.append(wrong_model)
        wrong_pose = list(self.command)
        wrong_pose[8] = "0.1"
        mutations.append(wrong_pose)
        wrong_name = list(self.command)
        wrong_name[19] = "__name:=other"
        mutations.append(wrong_name)
        third_remap = list(self.command) + ["foo:=bar"]
        mutations.append(third_remap)
        for command in mutations:
            with self.subTest(command=command):
                with self.assertRaises(RuntimeContractError):
                    validate_spawn_command(
                        command, DEFAULT_POSE, str(self.log_dir))

    def test_rejects_log_paths_outside_canonical_private_root(self):
        outside = list(self.command)
        outside[20] = "__log:=%s" % (
            Path(self.temporary.name) / "ground-spawn_bunker-1.log")
        bad_name = list(self.command)
        bad_name[20] = "__log:=%s" % (self.log_dir / "spawn.log")
        for command in (outside, bad_name):
            with self.assertRaises(RuntimeContractError):
                validate_spawn_command(
                    command, DEFAULT_POSE, str(self.log_dir))
        self.log_dir.chmod(0o755)
        with self.assertRaises(RuntimeContractError):
            validate_spawn_command(
                self.command, DEFAULT_POSE, str(self.log_dir))


if __name__ == "__main__":
    unittest.main()
