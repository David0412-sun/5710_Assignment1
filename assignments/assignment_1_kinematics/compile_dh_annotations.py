"""Compile the vector annotations on the original assignment figures.

The original figures are included as PDF backgrounds. All new marks are
editable TeX/TikZ objects. Recompilation never changes the supplied handout.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
REPORT = HERE / "report"
ASSETS = REPORT / "assets"
ANNOTATIONS = REPORT / "annotations"
WORK = ROOT / ".tmp" / "dh-annotations"
CHAINS = ("left", "right", "head")
METADATA = ASSETS / "assignment_annotation_metadata.json"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def dependencies():
    return [*(ANNOTATIONS / f"{chain}_overlay.tex" for chain in CHAINS),
            *(ASSETS / f"assignment_{chain}_original.pdf" for chain in CHAINS),
            ANNOTATIONS / "annotation_pages.tex", REPORT / "annotated_figures.tex"]


def verify_annotations():
    from assignments.assignment_1_kinematics.build_report import verify_source_hashes

    metadata = json.loads(METADATA.read_text(encoding="utf-8"))
    verify_source_hashes({"source_hash_scheme": "sha256-lf-v1",
                          "source_sha256": metadata["source_sha256_lf"]})
    for path in dependencies():
        key = path.relative_to(REPORT).as_posix()
        if digest(path) != metadata["input_sha256"][key]:
            raise ValueError(f"Annotation input changed: {key}; run compile_dh_annotations.py")
    for chain in CHAINS:
        name = f"assignment_{chain}_annotated.pdf"
        if digest(ASSETS / name) != metadata["output_sha256"][name]:
            raise ValueError(f"Annotation output changed: {name}; run compile_dh_annotations.py")
    combined = ROOT / "output" / "pdf" / "assignment_1_annotated_figures.pdf"
    if not combined.is_file() or digest(combined) != metadata["standalone_pdf_sha256"]:
        raise ValueError("Standalone annotation PDF changed or missing; run compile_dh_annotations.py")
    return metadata


def run(command, cwd):
    result = subprocess.run(command, cwd=cwd, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, encoding="utf-8",
                            errors="replace", check=False)
    if result.returncode:
        raise RuntimeError(result.stdout)
    return result.stdout


def main():
    sys.path.insert(0, str(ROOT))
    from assignments.assignment_1_kinematics.build_report import (
        check_latex_log, source_sha256, REQUIRED_SOURCES,
    )
    from assignments.assignment_1_kinematics.draw_dh_diagrams import frames, validate_geometry
    from assignments.assignment_1_kinematics.forward_kinematics import parameters
    import numpy as np

    executable = shutil.which("pdflatex")
    renderer = shutil.which("pdftoppm")
    if not executable or not renderer:
        raise RuntimeError("pdfLaTeX and pdftoppm are required to compile and preview the annotation overlays")
    WORK.mkdir(parents=True, exist_ok=True)
    geometry = validate_geometry()
    construction = {}
    for chain in CHAINS:
        poses = frames(chain)
        rows = parameters(chain)[0]
        points = []
        for i, row in enumerate(rows, 1):
            a, d = row[1], row[3]
            previous, current = poses[i - 1], poses[i]
            p = previous[:3, 3] + a * previous[:3, 0]
            reconstructed = p + d * current[:3, 2]
            if not np.allclose(reconstructed, current[:3, 3], rtol=0, atol=1e-12):
                raise ValueError(f"Invalid MDH construction for {chain} row {i}")
            points.append({"row": i, "a_previous_mm": float(a * 1000),
                           "d_i_mm": float(d * 1000),
                           "P_i_mm": np.round(p * 1000, 7).tolist(),
                           "O_i_mm": np.round(current[:3, 3] * 1000, 7).tolist()})
        construction[chain] = points
        source = ANNOTATIONS / f"{chain}_overlay.tex"
        command = [executable, "-interaction=nonstopmode", "-halt-on-error",
                   "-no-shell-escape", f"-output-directory={WORK}", source.name]
        transcript = run(command, ANNOTATIONS)
        transcript += "\n" + run(command, ANNOTATIONS)
        (WORK / f"{chain}.stdout.log").write_text(transcript, encoding="utf-8")
        check_latex_log(WORK / f"{chain}_overlay.log")
        shutil.copy2(WORK / f"{chain}_overlay.pdf", ASSETS / f"assignment_{chain}_annotated.pdf")
        run([renderer, "-singlefile", "-r", "180", "-png",
             str(ASSETS / f"assignment_{chain}_annotated.pdf"),
             str(ASSETS / f"assignment_{chain}_annotated")], REPORT)
    run([executable, "-interaction=nonstopmode", "-halt-on-error", "-no-shell-escape",
         f"-output-directory={WORK}", "annotated_figures.tex"], REPORT)
    check_latex_log(WORK / "annotated_figures.log")
    combined = ROOT / "output" / "pdf" / "assignment_1_annotated_figures.pdf"
    combined.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(WORK / "annotated_figures.pdf", combined)
    metadata = {
        "method": "Native TeX/TikZ overlays on the original vector PDF crops",
        "interpretation": "Source placements show parameter association, not a calibrated projection. Exact signed axes and distances are expressed in B.",
        "input_sha256": {p.relative_to(REPORT).as_posix(): digest(p) for p in dependencies()},
        "output_sha256": {f"assignment_{c}_annotated.pdf": digest(ASSETS / f"assignment_{c}_annotated.pdf") for c in CHAINS},
        "geometry_checks": geometry,
        "construction_mm": construction,
        "source_sha256_lf": {name: source_sha256(ROOT / name) for name in REQUIRED_SOURCES},
        "standalone_pdf_sha256": digest(combined),
    }
    METADATA.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    verify_annotations()
    print("Compiled and validated all three annotated originals.")
    print(combined)


if __name__ == "__main__":
    main()
