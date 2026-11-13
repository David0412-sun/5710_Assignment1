"""Add missing displacement arrowheads over the user's original vector figures.

The handwritten source figures remain unchanged. Editable TikZ overlays live
in report/annotations/handdrawn_*_arrows.tex and retain each source page's size.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import shutil
import sys


ASSIGNMENT_DIR = Path(__file__).resolve().parent
ROOT = ASSIGNMENT_DIR.parents[1]
REPORT = ASSIGNMENT_DIR / "report"
ASSETS = REPORT / "assets"
ANNOTATIONS = REPORT / "annotations"
METADATA = ASSETS / "handdrawn_arrow_metadata.json"
CHAINS = ("left", "right", "head")
sys.path.insert(0, str(ROOT))

from assignments.assignment_1_kinematics.extract_handdrawn_dh_figures import (
    CROPS, page_size, run, sha256, verify_figures,
)


def prepare_figures() -> dict:
    """Compile the additive vector overlays and render inspection previews."""
    verify_figures()
    executables = {name: shutil.which(name) for name in ("pdflatex", "pdfinfo", "pdftoppm")}
    missing = [name for name, executable in executables.items() if not executable]
    if missing:
        raise RuntimeError("Add these MiKTeX/Poppler tools to PATH: " + ", ".join(missing))
    work = ROOT / "tmp" / "pdfs" / "handdrawn_arrows"
    work.mkdir(parents=True, exist_ok=True)
    entries = []
    for chain in CHAINS:
        source = ASSETS / f"dh_{chain}_handdrawn.pdf"
        tex = ANNOTATIONS / f"handdrawn_{chain}_arrows.tex"
        input_hashes = {path: sha256(path) for path in (source, tex)}
        run([
            executables["pdflatex"], "-interaction=nonstopmode", "-halt-on-error",
            "-no-shell-escape", f"-output-directory={work}", tex.name,
        ], cwd=ANNOTATIONS)
        log = (work / f"{tex.stem}.log").read_text(encoding="utf-8", errors="replace")
        if re.search(r"Overfull\s+\\[hv]box|Missing (?:character|glyph)\s*:|^!", log, re.MULTILINE):
            raise RuntimeError(f"Arrow overlay LaTeX log failed validation: {chain}")
        output_pdf = ASSETS / f"dh_{chain}_handdrawn_arrows.pdf"
        shutil.copyfile(work / f"{tex.stem}.pdf", output_pdf)
        left, top, right, bottom = CROPS[chain]["bounds"]
        expected_size = (right - left, bottom - top)
        actual_size = page_size(output_pdf, executables["pdfinfo"])
        if any(abs(a - b) > 0.03 for a, b in zip(actual_size, expected_size)):
            raise RuntimeError(f"Arrow overlay changed page dimensions: {chain}, {actual_size}")
        output_png = output_pdf.with_suffix(".png")
        run([
            executables["pdftoppm"], "-f", "1", "-singlefile", "-r", "180", "-png",
            str(output_pdf), str(output_png.with_suffix("")),
        ])
        if any(sha256(path) != digest for path, digest in input_hashes.items()):
            raise RuntimeError(f"Arrow overlay inputs changed during compilation: {chain}")
        entry = {"chain": chain, "page_size_pt": list(expected_size), "rasterized": False}
        for key, path in (("source_pdf", source), ("overlay_tex", tex),
                          ("output_pdf", output_pdf), ("output_png", output_png)):
            entry[key] = path.relative_to(REPORT).as_posix()
            entry[f"{key}_sha256"] = sha256(path)
        entries.append(entry)
        print(f"Created {output_pdf.name}: original ink retained, missing arrows added")
    METADATA.write_text(json.dumps({
        "schema_version": 1,
        "method": "Native TikZ arrow overlays over unchanged handwritten PDF figures.",
        "figures": entries,
    }, indent=2) + "\n", encoding="utf-8")
    return verify_arrow_figures()


def verify_arrow_figures() -> dict:
    """Validate cached vector figures and unchanged handwritten inputs."""
    verify_figures()
    if not METADATA.is_file():
        raise RuntimeError("Missing arrow overlays; run add_handdrawn_dh_arrows.py.")
    metadata = json.loads(METADATA.read_text(encoding="utf-8"))
    entries = metadata.get("figures", [])
    if (metadata.get("schema_version") != 1 or len(entries) != len(CHAINS)
            or {entry["chain"] for entry in entries} != set(CHAINS)):
        raise RuntimeError("Invalid arrow overlay metadata; rebuild the overlays.")
    for entry in entries:
        chain = entry["chain"]
        left, top, right, bottom = CROPS[chain]["bounds"]
        if entry["page_size_pt"] != [right - left, bottom - top] or entry["rasterized"] is not False:
            raise RuntimeError(f"Stale arrow overlay dimensions: {chain}")
        expected_paths = {
            "source_pdf": ASSETS / f"dh_{chain}_handdrawn.pdf",
            "overlay_tex": ANNOTATIONS / f"handdrawn_{chain}_arrows.tex",
            "output_pdf": ASSETS / f"dh_{chain}_handdrawn_arrows.pdf",
            "output_png": ASSETS / f"dh_{chain}_handdrawn_arrows.png",
        }
        for key, path in expected_paths.items():
            if (entry[key] != path.relative_to(REPORT).as_posix() or not path.is_file()
                    or sha256(path) != entry[f"{key}_sha256"]):
                raise RuntimeError(f"Arrow overlay dependency changed: {path}; rebuild the overlays.")
    return metadata


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify", action="store_true", help="Check cached overlays without TeX")
    arguments = parser.parse_args()
    if arguments.verify:
        verify_arrow_figures()
        print("All handwritten figures, additive arrow overlays and source files verified.")
    else:
        prepare_figures()
