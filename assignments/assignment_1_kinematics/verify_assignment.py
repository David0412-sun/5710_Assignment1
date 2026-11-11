"""Run all tests and full FK/IK comparisons, then publish verified measurements.

The complete experiment is staged under .tmp. A failed test or comparison
leaves the existing results untouched. Run this file from any directory.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
from importlib.metadata import version
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
from uuid import uuid4

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from assignments.assignment_1_kinematics.build_report import (
    REQUIRED_SOURCES, RESULT_FILES, load_results, source_sha256,
)


def source_hashes():
    """Include the experiment helpers and model dependencies in provenance."""
    sources = set(REQUIRED_SOURCES)
    sources.update({
        "assignments/assignment_1_kinematics/verify_assignment.py",
        "playground/open_duck_mini_v2/constants.py",
        "playground/open_duck_mini_v2/mujoco_infer_base.py",
    })
    return {name: source_sha256(ROOT / name) for name in sorted(sources)}


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def run_python(*arguments):
    subprocess.run([sys.executable, "-u", *map(str, arguments)], cwd=ROOT, check=True)


def main():
    argparse.ArgumentParser(description=__doc__).parse_args()
    hashes_before = source_hashes()
    started_utc = datetime.now(timezone.utc).isoformat()
    staging_root = ROOT / ".tmp" / "verification"
    staging_root.mkdir(parents=True, exist_ok=True)
    stage = staging_root / ("run-" + uuid4().hex)
    stage.mkdir()
    print(f"Staging experiment in {stage}", flush=True)

    # Separate processes load exactly the current on-disk implementation.
    run_python("-m", "unittest", "discover", "-s", HERE, "-p", "test_*.py", "-v")
    run_python(HERE / "forward_kinematics.py", "--output-json", stage / "fk_results.json")
    run_python(HERE / "inverse_kinematics.py", "--headless", "--cycles", "5", "--samples", "101",
               "--parts", "left", "right", "head", "--output-json", stage / "ik_results.json")
    if source_hashes() != hashes_before:
        raise RuntimeError("Source changed during verification; existing results were not replaced.")

    environment = {
        "python": platform.python_version(),
        "numpy": version("numpy"),
        "scipy": version("scipy"),
        "mujoco": version("mujoco"),
        "etils": version("etils"),
        "platform": platform.system(),
        "platform_details": platform.platform(),
        "machine": platform.machine(),
        "started_utc": started_utc,
        "finished_utc": datetime.now(timezone.utc).isoformat(),
        "fk_seed": 0,
        "fk_samples_per_chain": 100,
        "ik": {
            "difference_scheme": "central", "h_rad": 1e-6, "alpha": 0.1,
            "max_iterations_per_target": 5000, "stop_cost": 1e-8,
            "position_scale_per_m": 10.0, "cycles": 5,
            "samples_per_cycle": 101, "headless": True,
        },
        "source_hash_scheme": "sha256-lf-v1",
        "source_sha256": hashes_before,
    }
    write_json(stage / "environment.json", environment)
    # Apply the same full data/provenance audit used by the report builder.
    load_results(stage)
    results = HERE / "results"
    results.mkdir(parents=True, exist_ok=True)
    previous = {name: (results / name).read_bytes() if (results / name).is_file() else None
                for name in RESULT_FILES}
    replaced = []
    try:
        for name in RESULT_FILES:
            # Each replacement is atomic; rollback handles ordinary write errors.
            temporary = results / (name + ".new")
            temporary.write_bytes((stage / name).read_bytes())
            os.replace(temporary, results / name)
            replaced.append(name)
    except OSError:
        for name in reversed(replaced):
            if previous[name] is None:
                (results / name).unlink(missing_ok=True)
            else:
                (results / name).write_bytes(previous[name])
        raise
    print(f"PASS: tests, 300 FK poses and 1,515 IK poses; published results to {results}")
    print("Run build_report.py to refresh the PDF, figures and submission archives.")


if __name__ == "__main__":
    main()
