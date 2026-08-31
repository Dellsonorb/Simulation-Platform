"""Pure helpers for the lightweight M1 Gazebo validation report."""

import math

import numpy as np


def _stats(values):
    values = np.asarray(values, dtype=np.float64)
    if values.size == 0:
        return {"mean": None, "median": None, "max": None}
    return {
        "mean": float(np.mean(values)),
        "median": float(np.median(values)),
        "max": float(np.max(values)),
    }


def summarize_results(results, expected_count):
    successful = [item for item in results if item.get("success", False)]
    expected_count = int(expected_count)
    summary = {
        "expected_count": expected_count,
        "result_count": len(results),
        "success_count": len(successful),
        "success_rate": (float(len(successful)) / expected_count
                         if expected_count > 0 else 0.0),
        "position_error_m": _stats(
            [item["position_error"] for item in successful]
        ),
        "xy_error_m": _stats([item["xy_error"] for item in successful]),
        "z_error_m": _stats([item["z_error"] for item in successful]),
        "yaw_error_deg": _stats([
            math.degrees(item["yaw_error"]) for item in successful
        ]),
    }
    return summary
