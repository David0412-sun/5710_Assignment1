"""Prepare compact, vector-only D-H diagram panels for the main report.

The full annotated guide remains unchanged. Each standalone TeX source clips
only the drawing and its on-site coordinate keys from that guide; its side
tables and page furniture are omitted. Cached verification needs only Python.
Run this script to rebuild, or use ``--verify`` to check the cached assets.
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
REPORT_DIR = ASSIGNMENT_DIR / "report"
ASSETS_DIR = REPORT_DIR / "assets"
ANNOTATIONS_DIR = REPORT_DIR / "annotations"
METADATA_PATH = ASSETS_DIR / "report_dh_panel_metadata.json"
PAGE_SIZE_MM = (297.0, 210.0)
# Bounds are measured from the top-left corner of the landscape guide page.
# Margins retain the complete base/site labels and every boxed axis key.
CROPS = {
    "left": {"bounds_mm": (13.5, 24.5, 135.5, 158.0), "display_height_mm": 125.0},
    "right": {"bounds_mm": (9.5, 32.2, 135.0, 178.0), "display_height_mm": 130.0},
    "head": {"bounds_mm": (4.5, 41.5, 122.5, 166.5), "display_height_mm": 125.0},
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


def page_size_pt(path: Path, pdfinfo: str) -> tuple[float, float]:
    info = run([pdfinfo, str(path)])
    match = re.search(r"Page size:\s+([\d.]+)\s+x\s+([\d.]+)\s+pts", info)
    if not match:
        raise RuntimeError(f"Could not read page size: {path}")
    return float(match.group(1)), float(match.group(2))


def panel_tex(chain: str) -> str:
    left, top, right, bottom = CROPS[chain]["bounds_mm"]
    trim = (left, PAGE_SIZE_MM[1] - bottom, PAGE_SIZE_MM[0] - right, top)
    trim_tex = " ".join(f"{value:.3f}mm" for value in trim)
    return (
        "% Compile from report/annotations; paths remain portable in the source bundle.\n"
        "% graphicx clipping retains the source PDF's vector artwork and text.\n"
        "\\documentclass[border=0pt]{standalone}\n"
        "\\usepackage{graphicx}\n"
        "\\begin{document}%\n"
        f"\\includegraphics[page=1,trim={{{trim_tex}}},clip]"
        f"{{../assets/assignment_{chain}_annotated.pdf}}%\n"
        "\\end{document}\n"
    )


def prepare_panels() -> dict:
    executables = {name: shutil.which(name) for name in ("pdflatex", "pdfinfo", "pdftoppm")}
    missing = [name for name, executable in executables.items() if not executable]
    if missing:
        raise RuntimeError("Add these MiKTeX/Poppler tools to PATH: " + ", ".join(missing))
    work_dir = ASSIGNMENT_DIR.parents[1] / "tmp" / "pdfs" / "report_dh_panels"
    work_dir.mkdir(parents=True, exist_ok=True)
    ANNOTATIONS_DIR.mkdir(parents=True, exist_ok=True)
    entries = []
    for chain, settings in CROPS.items():
        source = ASSETS_DIR / f"assignment_{chain}_annotated.pdf"
        source_hash = sha256(source)
        source_size = page_size_pt(source, executables["pdfinfo"])
        expected_source = tuple(value * 72 / 25.4 for value in PAGE_SIZE_MM)
        if any(abs(actual - expected) > 0.02 for actual, expected in zip(source_size, expected_source)):
            raise ValueError(f"Unexpected guide page size for {chain}: {source_size}")
        tex_path = ANNOTATIONS_DIR / f"panel_{chain}.tex"
        tex_path.write_text(panel_tex(chain), encoding="utf-8")
        run([
            executables["pdflatex"], "-interaction=nonstopmode", "-halt-on-error",
            "-no-shell-escape", f"-output-directory={work_dir}", tex_path.name,
        ], cwd=ANNOTATIONS_DIR)
        output_pdf = ASSETS_DIR / f"dh_{chain}_annotated_panel.pdf"
        shutil.copyfile(work_dir / f"panel_{chain}.pdf", output_pdf)
        left, top, right, bottom = settings["bounds_mm"]
        width, height = right - left, bottom - top
        actual_size = page_size_pt(output_pdf, executables["pdfinfo"])
        expected_size = (width * 72 / 25.4, height * 72 / 25.4)
        if any(abs(actual - expected) > 0.03 for actual, expected in zip(actual_size, expected_size)):
            raise RuntimeError(f"Unexpected crop size for {chain}: {actual_size}, expected {expected_size}")
        output_png = output_pdf.with_suffix(".png")
        run([
            executables["pdftoppm"], "-f", "1", "-singlefile", "-r", "180", "-png",
            str(output_pdf), str(output_png.with_suffix("")),
        ])
        if sha256(source) != source_hash:
            raise RuntimeError(f"Source guide changed while cropping: {source}")
        entries.append({
            "chain": chain,
            "source_pdf": source.relative_to(REPORT_DIR).as_posix(),
            "source_pdf_sha256": source_hash,
            "source_tex": tex_path.relative_to(REPORT_DIR).as_posix(),
            "source_tex_sha256": sha256(tex_path),
            "output_pdf": output_pdf.relative_to(REPORT_DIR).as_posix(),
            "output_pdf_sha256": sha256(output_pdf),
            "output_png": output_png.relative_to(REPORT_DIR).as_posix(),
            "output_png_sha256": sha256(output_png),
            "crop_bounds_mm": dict(zip(("left", "top", "right", "bottom"), settings["bounds_mm"])),
            "output_page_size_mm": {"width": width, "height": height},
            "recommended_display_height_mm": settings["display_height_mm"],
            "rasterized": False,
        })
        print(f"Created {output_pdf.name}: {width:g} x {height:g} mm, vector PDF")
    metadata = {
        "schema_version": 1,
        "method": "Native graphicx PDF clipping; original annotated guide PDFs are unchanged.",
        "crop_coordinates": "millimetres from the upper-left corner of the source page",
        "panels": entries,
    }
    METADATA_PATH.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    return verify_report_panels()


def verify_report_panels() -> dict:
    """Validate the cached source and panel files without needing TeX or Poppler."""
    if not METADATA_PATH.is_file():
        raise RuntimeError("Missing D-H panels; run prepare_report_dh_panels.py first.")
    metadata = json.loads(METADATA_PATH.read_text(encoding="utf-8"))
    entries = metadata.get("panels", [])
    if metadata.get("schema_version") != 1 or {entry["chain"] for entry in entries} != set(CROPS):
        raise RuntimeError("Invalid D-H panel metadata; rebuild with prepare_report_dh_panels.py.")
    for entry in entries:
        chain = entry["chain"]
        expected_bounds = dict(zip(("left", "top", "right", "bottom"), CROPS[chain]["bounds_mm"]))
        if entry["crop_bounds_mm"] != expected_bounds or entry["rasterized"] is not False:
            raise RuntimeError(f"Stale crop metadata for {chain}; rebuild the D-H panels.")
        for key in ("source_pdf", "source_tex", "output_pdf", "output_png"):
            path = REPORT_DIR / entry[key]
            if not path.is_file() or sha256(path) != entry[f"{key}_sha256"]:
                raise RuntimeError(f"Missing or changed D-H panel dependency: {path}. Run prepare_report_dh_panels.py.")
        if (REPORT_DIR / entry["source_tex"]).read_text(encoding="utf-8") != panel_tex(chain):
            raise RuntimeError(f"Stale native TeX crop source for {chain}; rebuild the D-H panels.")
    return metadata


verify_panels = verify_report_panels


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify", action="store_true", help="Verify existing panels without recompiling")
    arguments = parser.parse_args()
    if arguments.verify:
        verify_report_panels()
        print("All three D-H report panels and their source files verified.")
    else:
        prepare_panels()
