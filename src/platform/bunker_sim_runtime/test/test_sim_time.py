import importlib.util
import sys
import unittest
from pathlib import Path
from unittest import mock

from bunker_sim_runtime.sim_time import (
    SimTimeContractError,
    require_boolean_sim_time,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = PACKAGE_ROOT / "src/bunker_sim_runtime/sim_time.py"


class _Proxy:
    def __init__(self, response):
        self.response = response
        self.calls = []

    def getParam(self, caller_id, key):
        self.calls.append((caller_id, key))
        if isinstance(self.response, Exception):
            raise self.response
        return self.response


class SimTimeContractTest(unittest.TestCase):
    def _read(self, response):
        proxy = _Proxy(response)
        result = require_boolean_sim_time(
            "http://127.0.0.1:11311/", "/ground/spawn_bunker",
            proxy_factory=lambda uri: proxy,
        )
        self.assertEqual(
            [("/ground/spawn_bunker", "/use_sim_time")], proxy.calls)
        return result

    def test_accepts_only_literal_boolean_true_in_normal_response(self):
        for response in (
                [1, "parameter value", True],
                (1, "parameter value", True)):
            with self.subTest(response=response):
                self.assertIs(True, self._read(response))

    def test_rejects_transport_failures_and_invalid_responses(self):
        invalid = (
            OSError("offline"), (), [1], [1, "message"],
            [1, "message", True, "extra"],
            [0, "message", True], [-1, "message", True],
            [True, "message", True], [1, None, True],
            [1, "message", False], [1, "message", 0],
            [1, "message", 1], [1, "message", "true"],
            [1, "message", "false"], {"value": True},
        )
        for response in invalid:
            with self.subTest(response=response):
                with self.assertRaises(SimTimeContractError):
                    self._read(response)

    def test_pure_module_imports_without_rospy(self):
        name = "bunker_sim_time_no_rospy_fixture"
        spec = importlib.util.spec_from_file_location(name, str(MODULE_PATH))
        module = importlib.util.module_from_spec(spec)
        with mock.patch.dict(sys.modules, {"rospy": None}):
            spec.loader.exec_module(module)
        self.assertTrue(callable(module.require_boolean_sim_time))


if __name__ == "__main__":
    unittest.main()
