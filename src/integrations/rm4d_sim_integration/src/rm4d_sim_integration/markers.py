"""Pure RViz marker descriptions for RM4D base candidates."""

from dataclasses import dataclass
import math


BUNKER_FOOTPRINT = (
    (-0.52, -0.39),
    (-0.52, 0.39),
    (0.52, 0.39),
    (0.52, -0.39),
)


@dataclass(frozen=True)
class MarkerSpec:
    kind: str
    rank: int
    marker_id: int
    pose: tuple
    color: tuple
    points: tuple = ()
    text: str = ""


def _world_footprint(candidate):
    cosine = math.cos(candidate.yaw)
    sine = math.sin(candidate.yaw)
    points = tuple(
        (
            candidate.x + cosine * x - sine * y,
            candidate.y + sine * x + cosine * y,
            0.02,
        )
        for x, y in BUNKER_FOOTPRINT
    )
    return points + (points[0],)


def marker_specs(candidates):
    """Preserve RM4D order while describing three markers per candidate."""
    specs = []
    for rank, candidate in enumerate(candidates):
        color = (0.10, 0.95, 0.25, 0.95) if rank == 0 else \
            (0.20, 0.55, 1.00, 0.75)
        base_id = rank * 3
        pose = (candidate.x, candidate.y, candidate.yaw)
        specs.extend((
            MarkerSpec("arrow", rank, base_id, pose, color),
            MarkerSpec(
                "footprint", rank, base_id + 1, pose, color,
                points=_world_footprint(candidate)),
            MarkerSpec(
                "text", rank, base_id + 2, pose, color,
                text="#%d %s score=%.6f" %
                (rank + 1, candidate.candidate_id, candidate.score)),
        ))
    return tuple(specs)
