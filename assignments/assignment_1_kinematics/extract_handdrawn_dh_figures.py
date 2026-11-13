"""Extract the user's handwritten D-H figures as native vector PDF panels.

Only page cropping is performed: every retained source stroke and label stays
unchanged. Run this script to rebuild from Assignment 1(3).pdf, or use --verify
to check the cached figures without requiring TeX or Poppler.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess


ASSIGNMENT_DIR = Path(__file__).resolve().parent
REPOSITORY_ROOT = ASSIGNMENT_DIR.parents[1]
ASSETS_DIR = ASSIGNMENT_DIR / "report" / "assets"
SOURCE = REPOSITORY_ROOT / "Assignment 1(3).pdf"
METADATA_PATH = ASSETS_DIR / "handdrawn_dh_metadata.json"
PAGE_SIZE_PT = (595.276, 841.890)
# PDF points from the source page's upper-left corner. Include handwriting
# above the original rotation arrows as well as all base/site axis labels.
CROPS = {
    "left": {"page": 3, "figure": 2, "bounds": (50.0, 108.0, 305.0, 493.0)},
    "right": {"page": 4, "figure": 3, "bounds": (190.0, 106.0, 406.0, 434.0)},
    "head": {"page": 5, "figure": 4, "bounds": (48.0, 112.0, 326.0, 404.0)},
}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run(command: list[str], cwd: Path | None = None) -> str:
    result = subprocess.run(
        command, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        encoding="utf-8", errors="replace", check=False,
    )
    if result.returncode:
        raise RuntimeError(f"Command failed ({result.returncode}): {command[0]}\n{result.stdout}")
    return result.stdout


def page_size(path: Path, pdfinfo: str) -> tuple[float, float]:
    information = run([pdfinfo, str(path)])
    match = re.search(r"Page size:\s+([\d.]+)\s+x\s+([\d.]+)\s+pts", information)
    if not match:
        raise RuntimeError(f"Cannot read PDF page size: {path}")
    return float(match.group(1)), float(match.group(2))


def extract_figures() -> dict:
    executables = {name: shutil.which(name) for name in ("pdflatex", "pdfinfo", "pdftoppm")}
    missing = [name for name, executable in executables.items() if not executable]
    if missing:
        raise RuntimeError("Add these MiKTeX/Poppler tools to PATH: " + ", ".join(missing))
    if not SOURCE.is_file():
        raise FileNotFoundError(SOURCE)
    source_hash = sha256(SOURCE)
    actual_size = page_size(SOURCE, executables["pdfinfo"])
    if any(abs(a - b) > 0.01 for a, b in zip(actual_size, PAGE_SIZE_PT)):
        raise ValueError(f"Unexpected handwritten source page size: {actual_size}")
    work_dir = REPOSITORY_ROOT / "tmp" / "pdfs" / "handdrawn" / "crops"
    work_dir.mkdir(parents=True, exist_ok=True)
    ASSETS_DIR.mkdir(parents=True, exist_ok=True)
    entries = []
    for chain, crop in CROPS.items():
        left, top, right, bottom = crop["bounds"]
        width, height = right - left, bottom - top
        trim = (left, actual_size[1] - bottom, actual_size[0] - right, top)
        trim_tex = " ".join(f"{value:.3f}bp" for value in trim)
        stem = f"dh_{chain}_handdrawn"
        tex = (
            "\\documentclass[border=0pt]{standalone}\n"
            "\\usepackage{graphicx}\n"
            "\\begin{document}%\n"
            f"\\includegraphics[page={crop['page']},trim={{{trim_tex}}},clip]"
            f"{{\\detokenize{{{SOURCE.as_posix()}}}}}%\n"
            "\\end{document}\n"
        )
        tex_path = work_dir / f"{stem}.tex"
        tex_path.write_text(tex, encoding="utf-8")
        run([
            executables["pdflatex"], "-interaction=nonstopmode", "-halt-on-error",
            "-no-shell-escape", tex_path.name,
        ], cwd=work_dir)
        output_pdf = ASSETS_DIR / f"{stem}.pdf"
        shutil.copyfile(work_dir / output_pdf.name, output_pdf)
        actual_crop_size = page_size(output_pdf, executables["pdfinfo"])
        if any(abs(a - b) > 0.03 for a, b in zip(actual_crop_size, (width, height))):
            raise RuntimeError(f"Unexpected crop size for {chain}: {actual_crop_size}")
        output_png = output_pdf.with_suffix(".png")
        run([
            executables["pdftoppm"], "-f", "1", "-singlefile", "-r", "180", "-png",
            str(output_pdf), str(output_png.with_suffix("")),
        ])
        entries.append({
            "chain": chain,
            "source_page_1_based": crop["page"],
            "source_figure": crop["figure"],
            "crop_bounds_pt": dict(zip(("left", "top", "right", "bottom"), crop["bounds"])),
            "output_page_size_pt": {"width": width, "height": height},
            "output_pdf": output_pdf.name,
            "output_pdf_sha256": sha256(output_pdf),
            "output_png": output_png.name,
            "output_png_sha256": sha256(output_png),
            "rasterized": False,
        })
        print(f"Created {output_pdf.name}: source page {crop['page']}, {width:g} x {height:g} pt")
    if sha256(SOURCE) != source_hash:
        raise RuntimeError("The handwritten source PDF changed during extraction.")
    metadata = {
        "schema_version": 1,
        "source": SOURCE.relative_to(REPOSITORY_ROOT).as_posix(),
        "source_sha256": source_hash,
        "source_page_size_pt": dict(zip(("width", "height"), actual_size)),
        "coordinate_system": "PDF points from the page's upper-left corner; x right, y down",
        "method": "Native PDF inclusion using graphicx trim/clip; no handwriting is redrawn or rasterized.",
        "retained_content": "Complete original robot drawings and all user hand-drawn axes, origins, arrows and labels.",
        "excluded_content": "Printed instructions, adjacent dimension tables, figure captions, blank D-H tables and page numbers.",
        "figures": entries,
    }
    METADATA_PATH.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    return verify_figures()


def verify_figures() -> dict:
    """Check source provenance and every cached crop without TeX or Poppler."""
    if not METADATA_PATH.is_file():
        raise RuntimeError("Missing handwritten D-H figures; run extract_handdrawn_dh_figures.py.")
    metadata = json.loads(METADATA_PATH.read_text(encoding="utf-8"))
    entries = metadata.get("figures", [])
    if (metadata.get("schema_version") != 1
            or metadata.get("source") != SOURCE.relative_to(REPOSITORY_ROOT).as_posix()
            or len(entries) != len(CROPS)
            or {entry["chain"] for entry in entries} != set(CROPS)):
        raise RuntimeError("Invalid handwritten D-H metadata; rebuild the figures.")
    if SOURCE.is_file() and sha256(SOURCE) != metadata["source_sha256"]:
        raise RuntimeError("Handwritten source PDF changed; run extract_handdrawn_dh_figures.py.")
    for entry in entries:
        chain = entry["chain"]
        crop = CROPS[chain]
        expected_bounds = dict(zip(("left", "top", "right", "bottom"), crop["bounds"]))
        if (entry["crop_bounds_pt"] != expected_bounds
                or entry["source_page_1_based"] != crop["page"]
                or entry["rasterized"] is not False):
            raise RuntimeError(f"Stale handwritten crop metadata for {chain}; rebuild the figures.")
        for key, extension in (("output_pdf", "pdf"), ("output_png", "png")):
            expected_name = f"dh_{chain}_handdrawn.{extension}"
            path = ASSETS_DIR / expected_name
            if (entry[key] != expected_name or not path.is_file()
                    or sha256(path) != entry[f"{key}_sha256"]):
                raise RuntimeError(f"Missing or changed handwritten crop: {path}")
    return metadata


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify", action="store_true", help="Verify cached figures without recompiling")
    arguments = parser.parse_args()
    if arguments.verify:
        verify_figures()
        print("All three handwritten D-H figures and their provenance verified.")
    else:
        extract_figures()
