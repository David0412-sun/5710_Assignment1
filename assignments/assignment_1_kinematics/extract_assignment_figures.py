"""Extract the assignment's original robot drawings without rasterizing them.

Uses the installed pdfLaTeX/graphicx tools and Python's standard library only.
Run from any directory; the source PDF is never modified. The crop rectangles
use PDF points (1/72 inch), measured from the page's upper-left corner.
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
PAGE_SIZE_PT = (595.276, 841.890)
CROPS = {
    "left": {"page": 3, "figure": 2, "bounds": (50.0, 114.0, 305.0, 493.0)},
    "right": {"page": 4, "figure": 3, "bounds": (190.0, 115.0, 406.0, 434.0)},
    "head": {"page": 5, "figure": 4, "bounds": (48.0, 114.0, 326.0, 404.0)},
}


def run(command: list[str], cwd: Path | None = None) -> str:
    result = subprocess.run(
        command, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        encoding="utf-8", errors="replace", check=False,
    )
    if result.returncode:
        raise RuntimeError(f"Command failed ({result.returncode}): {command[0]}\n{result.stdout}")
    return result.stdout


def page_size(pdf: Path, pdfinfo: str) -> tuple[float, float]:
    information = run([pdfinfo, str(pdf)])
    match = re.search(r"Page size:\s+([\d.]+)\s+x\s+([\d.]+)\s+pts", information)
    if not match:
        raise RuntimeError(f"Cannot determine page size for {pdf}")
    return float(match.group(1)), float(match.group(2))


def extract(source: Path, output_dir: Path, work_dir: Path) -> None:
    pdflatex = shutil.which("pdflatex")
    pdfinfo = shutil.which("pdfinfo")
    if not pdflatex or not pdfinfo:
        raise RuntimeError("Add pdfLaTeX and pdfinfo (MiKTeX on Windows) to PATH.")
    if not source.is_file():
        raise FileNotFoundError(source)
    actual_size = page_size(source, pdfinfo)
    if any(abs(actual - expected) > 0.01 for actual, expected in zip(actual_size, PAGE_SIZE_PT)):
        raise ValueError(f"The source has page size {actual_size}; these crops require {PAGE_SIZE_PT}.")
    source_hash = hashlib.sha256(source.read_bytes()).hexdigest()
    output_dir.mkdir(parents=True, exist_ok=True)
    work_dir.mkdir(parents=True, exist_ok=True)
    entries = []
    for chain, crop in CROPS.items():
        left, top, right, bottom = crop["bounds"]
        width, height = right - left, bottom - top
        trim = tuple(round(value, 3) for value in (left, actual_size[1] - bottom, actual_size[0] - right, top))
        stem = f"assignment_{chain}_original"
        trim_tex = " ".join(f"{value:.3f}bp" for value in trim)
        # graphicx embeds the source PDF as a vector form and applies clipping.
        # standalone sets the output MediaBox to exactly that clipped diagram.
        tex = (
            "\\documentclass[border=0pt]{standalone}\n"
            "\\usepackage{graphicx}\n"
            "\\begin{document}%\n"
            f"\\includegraphics[page={crop['page']},trim={{{trim_tex}}},clip]"
            f"{{\\detokenize{{{source.as_posix()}}}}}%\n"
            "\\end{document}\n"
        )
        tex_path = work_dir / f"{stem}.tex"
        tex_path.write_text(tex, encoding="utf-8")
        run([pdflatex, "-interaction=nonstopmode", "-halt-on-error", "-no-shell-escape", tex_path.name], cwd=work_dir)
        generated = work_dir / f"{stem}.pdf"
        actual_crop_size = page_size(generated, pdfinfo)
        if any(abs(actual - expected) > 0.03 for actual, expected in zip(actual_crop_size, (width, height))):
            raise RuntimeError(f"Unexpected crop page size: {actual_crop_size}, expected {(width, height)}")
        destination = output_dir / generated.name
        shutil.copyfile(generated, destination)
        entries.append({
            "chain": chain,
            "output": destination.name,
            "output_sha256": hashlib.sha256(destination.read_bytes()).hexdigest(),
            "source_page_1_based": crop["page"],
            "source_figure": crop["figure"],
            "crop_bounds_pt": {"left": left, "top": top, "right": right, "bottom": bottom},
            "graphicx_trim_pt": dict(zip(("left", "bottom", "right", "top"), trim)),
            "output_page_size_pt": {"width": actual_crop_size[0], "height": actual_crop_size[1]},
            "rasterized": False,
        })
        print(f"Created {destination.name}: source page {crop['page']}, {width:g} x {height:g} pt")
    if hashlib.sha256(source.read_bytes()).hexdigest() != source_hash:
        raise RuntimeError("Source PDF changed during extraction.")
    try:
        source_name = source.relative_to(REPOSITORY_ROOT).as_posix()
    except ValueError:
        source_name = source.as_posix()
    metadata = {
        "source": source_name,
        "source_sha256": source_hash,
        "source_page_size_pt": {"width": actual_size[0], "height": actual_size[1]},
        "coordinate_system": "PDF points (1/72 inch), origin at upper-left; x increases right, y increases down",
        "method": "Lossless source-PDF inclusion with pdfLaTeX graphicx page/trim/clip and standalone; no drawing is reconstructed or rasterized.",
        "retained_content": "Original chain, joint and dimension labels, rotation arrows, base/site coordinate axes, and Foot site or Head site label.",
        "excluded_content": "Task instructions, dimension tables, figure captions, blank D-H tables, and page numbers.",
        "figures": entries,
    }
    (output_dir / "assignment_original_source.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=REPOSITORY_ROOT / "Assignment 1.pdf")
    parser.add_argument("--output-dir", type=Path, default=ASSIGNMENT_DIR / "report" / "assets")
    parser.add_argument("--work-dir", type=Path, default=REPOSITORY_ROOT / ".tmp" / "assignment-original-crops")
    args = parser.parse_args()
    extract(args.source.resolve(), args.output_dir.resolve(), args.work_dir.resolve())


if __name__ == "__main__":
    main()
