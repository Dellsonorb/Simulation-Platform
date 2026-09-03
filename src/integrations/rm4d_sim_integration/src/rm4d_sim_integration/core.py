"""Read-only invocation boundary for the frozen RM4D Python API."""

from dataclasses import dataclass
from pathlib import Path
import subprocess
import sys

from .geometry import (
    IntegrationError,
    build_rm4d_request,
    ordered_candidates,
)


FROZEN_RM4D_COMMIT = "e9d431299053f38a4a4319aed3dfeccc261b9fac"


@dataclass(frozen=True)
class IntegrationResult:
    status: str
    candidates: tuple


def require_external_revision(actual_revision):
    actual = str(actual_revision).strip()
    if actual != FROZEN_RM4D_COMMIT:
        raise IntegrationError(
            "RM4D external revision mismatch: expected %s, got %s" %
            (FROZEN_RM4D_COMMIT, actual or "<empty>"))


def read_external_revision(root):
    resolved = Path(root).expanduser().resolve()
    if not resolved.is_dir():
        raise IntegrationError("RM4D external root is not a directory")
    completed = subprocess.run(
        ["git", "-C", str(resolved), "rev-parse", "HEAD"],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        universal_newlines=True, check=False)
    if completed.returncode != 0:
        detail = completed.stderr.strip() or "git rev-parse failed"
        raise IntegrationError("cannot read RM4D external revision: %s" % detail)
    revision = completed.stdout.strip()
    require_external_revision(revision)
    return revision


def load_external_api(root, map_path, config_path=None):
    resolved_root = Path(root).expanduser().resolve()
    read_external_revision(resolved_root)
    resolved_map = Path(map_path).expanduser().resolve()
    resolved_config = Path(config_path).expanduser().resolve() \
        if config_path else \
        resolved_root / "configs/mr4_offline_base_placement.json"
    if not resolved_map.is_file():
        raise IntegrationError("RM4D map does not exist")
    if not resolved_config.is_file():
        raise IntegrationError("RM4D planner config does not exist")
    root_text = str(resolved_root)
    if root_text not in sys.path:
        sys.path.insert(0, root_text)
    try:
        from rm4d import BasePlacementAPI
    except ImportError as error:
        raise IntegrationError("cannot import frozen RM4D BasePlacementAPI") \
            from error
    return BasePlacementAPI.from_files(
        config_path=str(resolved_config), map_path=str(resolved_map))


class IntegrationCore:
    def __init__(self, api):
        if api is None or not callable(getattr(api, "plan", None)):
            raise IntegrationError("RM4D API must provide plan()")
        self._api = api

    def plan(
            self, frame_id, exact_pose, current_bunker_pose,
            grasp_id, top_k):
        if isinstance(top_k, bool) or not isinstance(top_k, int) or top_k <= 0:
            raise IntegrationError("top_k must be a positive integer")
        request = build_rm4d_request(
            frame_id, exact_pose, current_bunker_pose, grasp_id)
        raw_result = self._api.plan(request, top_k=top_k)
        if not isinstance(raw_result, dict):
            raise IntegrationError("RM4D result must be a dictionary")
        if raw_result.get("frame_id") != "map":
            raise IntegrationError("RM4D result frame must be map")
        status = raw_result.get("status")
        if status not in ("ok", "no_feasible_candidate"):
            raise IntegrationError("RM4D result status is unsupported")
        candidates = ordered_candidates(raw_result)
        if status == "ok" and not candidates:
            raise IntegrationError("RM4D ok result has no candidates")
        if status == "no_feasible_candidate" and candidates:
            raise IntegrationError(
                "RM4D no-feasible result contains candidates")
        return IntegrationResult(status=status, candidates=candidates)
