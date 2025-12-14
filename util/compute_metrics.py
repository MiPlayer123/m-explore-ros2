#!/usr/bin/env python3
"""Compute coverage metrics from run logs.

Inputs:
  - One or more JSON files containing a list of data points with fields: {"time": float, "coverage": float}
  - Optional baseline JSON to compute speedup/efficiency

Outputs (printed):
  - time_to_{50,70,90,95}
  - final_coverage
  - speedup vs baseline (if provided)
  - parallel efficiency vs baseline (speedup / num_robots)
  - placeholder SLAM integrity hook

This is a lightweight helper; tailor parsing to your log schema as needed.
"""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
from statistics import mean, pstdev
from typing import List, Dict, Any, Tuple

THRESHOLDS = [50, 70, 90, 95]


def load_series(path: Path) -> List[Dict[str, Any]]:
    data = json.loads(path.read_text())
    if isinstance(data, dict) and "data_points" in data:
        data = data["data_points"]
    if not isinstance(data, list):
        raise ValueError(f"Unexpected JSON structure in {path}")
    return data


def time_to_threshold(series: List[Dict[str, Any]], threshold: float) -> float:
    for pt in series:
        if float(pt.get("coverage", 0.0)) >= threshold:
            return float(pt.get("time", math.nan))
    return math.nan


def summarize(files: List[Path]) -> Dict[str, Any]:
    per_run = []
    for f in files:
        series = load_series(f)
        times = {f"t{thr}": time_to_threshold(series, thr) for thr in THRESHOLDS}
        final_cov = float(series[-1].get("coverage", 0.0)) if series else math.nan
        per_run.append({"file": str(f), **times, "final": final_cov})
    # aggregate
    def agg(key: str) -> Tuple[float, float]:
        vals = [r[key] for r in per_run if not math.isnan(r[key])]
        return (mean(vals), pstdev(vals)) if vals else (math.nan, math.nan)

    summary = {"runs": per_run}
    for thr in THRESHOLDS:
        m, s = agg(f"t{thr}")
        summary[f"time_to_{thr}"] = {"mean": m, "std": s}
    m, s = agg("final")
    summary["final_coverage"] = {"mean": m, "std": s}
    return summary


def compute_speedup(coordinated: Dict[str, Any], baseline: Dict[str, Any], num_robots: int) -> Dict[str, Any]:
    res = {}
    for thr in THRESHOLDS:
        t_c = coordinated.get(f"time_to_{thr}", {}).get("mean", math.nan)
        t_b = baseline.get(f"time_to_{thr}", {}).get("mean", math.nan)
        if not (math.isnan(t_c) or math.isnan(t_b) or t_c == 0):
            spd = t_b / t_c
            res[f"speedup_{thr}"] = spd
            res[f"efficiency_{thr}"] = spd / max(num_robots, 1)
    return res


def main():
    ap = argparse.ArgumentParser(description="Compute coverage metrics from logs")
    ap.add_argument("files", nargs="+", type=Path, help="Run JSON files for coordinated/target config")
    ap.add_argument("--baseline", type=Path, help="Baseline single-robot JSON file")
    ap.add_argument("--num-robots", type=int, default=2, help="Number of robots in coordinated run")
    args = ap.parse_args()

    summary = summarize(args.files)
    print("=== Coordinated/Target Summary ===")
    print(json.dumps(summary, indent=2))

    if args.baseline:
        base_summary = summarize([args.baseline])
        print("=== Baseline Summary ===")
        print(json.dumps(base_summary, indent=2))
        spd = compute_speedup(summary, base_summary, args.num_robots)
        print("=== Speedup / Efficiency ===")
        print(json.dumps(spd, indent=2))

    # Placeholder for SLAM integrity: integrate your map-consistency metric here.


if __name__ == "__main__":
    main()
