"""Build the IEEE LaTeX report from the existing, verified FK/IK measurements.

This script never runs a simulation or changes the measured JSON files. Use
--prepare-only to regenerate tables and the vector figure without a TeX engine.
All paths are relative to this script, so it can run from any directory.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
from zipfile import ZIP_DEFLATED, ZipFile

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
RESULTS = HERE / "results"
REPORT = HERE / "report"
GENERATED = REPORT / "generated"
ASSETS = REPORT / "assets"
SOURCE = REPORT / "assignment_1_report.tex"
BUILD = ROOT / ".tmp" / "latex"
OUTPUT = ROOT / "output"
LOG_OUTPUT = OUTPUT / "latex"
CHAINS = ("left", "right", "head")
CHAIN_LABELS = {"left": "Left leg", "right": "Right leg", "head": "Head"}
RESULT_FILES = ("fk_results.json", "ik_results.json", "environment.json")
REQUIRED_SOURCES = (
    "assignments/assignment_1_kinematics/forward_kinematics.py",
    "assignments/assignment_1_kinematics/inverse_kinematics.py",
    "playground/open_duck_mini_v2/xmls/open_duck_mini_v2.xml",
    "playground/open_duck_mini_v2/xmls/scene_flat_terrain.xml",
)
sys.path.insert(0, str(ROOT))


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_json(path):
    def invalid_constant(value):
        raise ValueError(f"Non-finite JSON constant {value} in {path}")
    return json.loads(Path(path).read_text(encoding="utf-8"),
                      parse_constant=invalid_constant)


def finite_number(value, context):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{context} must be a number")
    if not math.isfinite(value):
        raise ValueError(f"{context} must be finite")
    return float(value)


def verify_source_hashes(environment):
    recorded = environment.get("source_sha256", {})
    if not isinstance(recorded, dict) or not set(REQUIRED_SOURCES) <= recorded.keys():
        raise ValueError("environment.json must record the FK, IK and model source hashes")
    for name, expected in recorded.items():
        path = (ROOT / name).resolve()
        if not path.is_relative_to(ROOT) or not path.is_file():
            raise ValueError(f"Recorded source is missing or outside the project: {name}")
        if not isinstance(expected, str) or not re.fullmatch(r"[0-9a-f]{64}", expected):
            raise ValueError(f"Invalid recorded SHA256 for {name}")
        if sha256(path) != expected:
            raise ValueError(f"Source changed since the measurements were recorded: {name}")


def validate_errors(rows, summary, position_limit, orientation_limit, context):
    if summary.get("passed") is not True:
        raise ValueError(f"{context} is not recorded as passing")
    for key, summary_key, limit in (
            ("position_error_m", "max_position_error_m", position_limit),
            ("orientation_error_rad", "max_orientation_error_rad", orientation_limit)):
        values = [finite_number(row[key], f"{context}: {key}") for row in rows]
        if not values or any(value < 0 or value >= limit for value in values):
            raise ValueError(f"{context}: every {key} must be nonnegative and below {limit}")
        recorded = finite_number(summary[summary_key], f"{context}: {summary_key}")
        if recorded != max(values):
            raise ValueError(f"{context}: {summary_key} disagrees with the measurements")


def load_results():
    """Validate the complete recorded experiment and its source provenance."""
    fk, ik, environment = [read_json(RESULTS / name) for name in RESULT_FILES]
    verify_source_hashes(environment)
    if fk.get("passed") is not True or ik.get("passed") is not True:
        raise ValueError("Both recorded FK and IK comparisons must pass")
    if set(fk.get("chains", {})) != set(CHAINS) or set(ik.get("chains", {})) != set(CHAINS):
        raise ValueError("Both comparisons must contain exactly left, right and head")
    for result, p_limit, r_limit in ((fk, 1e-6, 1e-5), (ik, 1e-5, 1e-4)):
        if (result.get("position_tolerance_m"), result.get("orientation_tolerance_rad")) != (
                p_limit, r_limit):
            raise ValueError("Recorded tolerances differ from the assignment limits")
    if environment.get("fk_seed") != 0 or environment.get("fk_samples_per_chain") != 100:
        raise ValueError("FK environment must specify seed 0 and 100 poses per chain")
    for chain in CHAINS:
        summary = fk["chains"][chain]
        rows = summary.get("measurements", [])
        if summary.get("samples") != 100 or summary.get("seed") != 0 or len(rows) != 100:
            raise ValueError(f"FK {chain} requires exactly 100 measurements with seed 0")
        for row in rows:
            q = row.get("joint_angles_rad", [])
            if len(q) != (4 if chain == "head" else 5):
                raise ValueError(f"FK {chain} has an incorrect joint-vector length")
            for value in q:
                finite_number(value, f"FK {chain} joint angle")
        validate_errors(rows, summary, 1e-6, 1e-5, f"FK {chain}")
    if sum(len(fk["chains"][chain]["measurements"]) for chain in CHAINS) != 300:
        raise ValueError("FK must contain exactly 300 measured configurations")
    if (ik.get("completed") is not True or ik.get("headless") is not True or
            (ik.get("cycles_requested"), ik.get("cycles_completed"),
             ik.get("samples_per_cycle")) != (5, 5, 101)):
        raise ValueError("IK requires the completed headless 5-cycle, 101-phase comparison")
    recorded_ik = environment.get("ik", {})
    if (recorded_ik.get("cycles"), recorded_ik.get("samples_per_cycle"),
            recorded_ik.get("headless")) != (5, 101, True):
        raise ValueError("IK environment metadata disagrees with the complete comparison")
    rows = ik.get("measurements", [])
    if len(rows) != 1515 or {row.get("chain") for row in rows} != set(CHAINS):
        raise ValueError("IK must contain exactly 1,515 measured chain poses")
    for chain in CHAINS:
        chain_rows = [row for row in rows if row["chain"] == chain]
        if len(chain_rows) != 505:
            raise ValueError(f"IK {chain} requires exactly 505 measurements")
        for index, row in enumerate(chain_rows):
            cycle, phase_index = divmod(index, 101)
            phase = finite_number(row["phase"], f"IK {chain} phase")
            if row.get("cycle") != cycle + 1 or not math.isclose(
                    phase, phase_index / 100, rel_tol=0, abs_tol=1e-12):
                raise ValueError(f"IK {chain} has an incomplete or reordered cycle/phase sequence")
        validate_errors(chain_rows, ik["chains"][chain], 1e-5, 1e-4, f"IK {chain}")
    return fk, ik, environment


def scientific(value):
    """Return a complete math-mode scientific-notation string."""
    if value == 0:
        return "$0$"
    mantissa, exponent = f"{value:.6e}".split("e")
    return rf"${mantissa}\times 10^{{{int(exponent)}}}$"


def angle(value):
    beta = math.atan2(19, 35)
    for actual, label in ((0, "0"), (math.pi / 2, r"\pi/2"),
                          (-math.pi / 2, r"-\pi/2"), (math.pi, r"\pi"),
                          (-beta, r"-\beta"), (math.pi + beta, r"\pi+\beta")):
        if math.isclose(value, actual, rel_tol=0, abs_tol=1e-14):
            return label
    return f"{value:.10f}".rstrip("0").rstrip(".")


def distance(value):
    if math.isclose(value, 0, rel_tol=0, abs_tol=1e-15):
        return "0"
    if math.isclose(value, math.hypot(.019, .035), rel_tol=0, abs_tol=1e-15):
        return "r"
    return f"{value:.7f}".rstrip("0").rstrip(".")


def dh_table(chain, parameters):
    dh, signs, _ = parameters(chain)
    lines = [
        r"\begin{table}[t]", r"\centering", r"\footnotesize",
        r"\setlength{\tabcolsep}{4pt}",
        rf"\caption{{Modified D--H parameters for the {CHAIN_LABELS[chain].lower()}. "
        r"Lengths $a,d$ are in metres; angles $\alpha,\theta,q$ are in radians. "
        r"The starred final row is the fixed site transform.}",
        rf"\label{{tab:dh-{chain}}}", r"\begin{tabular}{ccccc}", r"\toprule",
        r"$i$ & $\alpha_{i-1}$ & $a_{i-1}$ & $d_i$ & $\theta_i$ \\",
        r"\midrule",
    ]
    for index, (alpha, a, offset, d) in enumerate(dh):
        theta = angle(offset)
        if index < len(signs):
            term = ("-" if signs[index] < 0 else "") + rf"q_{{{index + 1}}}"
            theta = term if theta == "0" else (
                term + "-" + theta[1:] if theta.startswith("-") else term + "+" + theta)
            row_number = str(index + 1)
        else:
            row_number = rf"{index + 1}^{{*}}"
        cells = (row_number, angle(alpha), distance(a), distance(d), theta)
        lines.append(" & ".join(f"${cell}$" for cell in cells) + r" \\")
    lines.extend([r"\bottomrule", r"\end{tabular}", r"\end{table}", ""])
    return "\n".join(lines)


def result_table(label, result):
    samples = "300 random configurations" if label == "FK" else "1,515 trajectory poses"
    lines = [
        r"\begin{table}[t]", r"\centering", r"\footnotesize",
        r"\setlength{\tabcolsep}{3.5pt}",
        rf"\caption{{Measured {label} errors over {samples}. "
        r"Maxima are computed independently for each chain.}",
        rf"\label{{tab:{label.lower()}}}", r"\begin{tabular}{lccc}", r"\toprule",
        r"Chain & \shortstack{Max. position\\(m)} & "
        r"\shortstack{Max. orientation\\(rad)} & Result \\", r"\midrule",
    ]
    for chain in CHAINS:
        summary = result["chains"][chain]
        lines.append(" & ".join((CHAIN_LABELS[chain],
                                 scientific(summary["max_position_error_m"]),
                                 scientific(summary["max_orientation_error_rad"]), "PASS")) + r" \\")
    lines.extend([r"\bottomrule", r"\end{tabular}", r"\end{table}", ""])
    return "\n".join(lines)


def prepare_sources(fk, ik):
    # Importing definitions does not call either experiment's command-line entry point.
    os.environ.setdefault("MPLCONFIGDIR", str(ROOT / ".tmp" / "matplotlib"))
    from assignments.assignment_1_kinematics.forward_kinematics import parameters
    from assignments.assignment_1_kinematics.plot_paper_results import generate_plot

    GENERATED.mkdir(parents=True, exist_ok=True)
    ASSETS.mkdir(parents=True, exist_ok=True)
    lines = ["% Generated from validated recorded measurements; do not edit."]
    for label, result in (("FK", fk), ("IK", ik)):
        for suffix, key in (("Position", "max_position_error_m"),
                            ("Orientation", "max_orientation_error_rad")):
            maximum = max(result["chains"][chain][key] for chain in CHAINS)
            lines.append(rf"\newcommand{{\{label}Max{suffix}}}{{{scientific(maximum)}}}")
    (GENERATED / "data.tex").write_text("\n".join(lines) + "\n", encoding="utf-8")
    for chain in CHAINS:
        (GENERATED / f"dh_{chain}.tex").write_text(dh_table(chain, parameters), encoding="utf-8")
    for label, result in (("FK", fk), ("IK", ik)):
        (GENERATED / f"{label.lower()}_results.tex").write_text(result_table(label, result), encoding="utf-8")
    pdf, _ = generate_plot(RESULTS / "ik_results.json", ROOT / ".tmp" / "report-plots")
    shutil.copy2(pdf, ASSETS / "ik_error_plot.pdf")


def executable_path(candidate):
    if not candidate:
        return None
    found = shutil.which(str(candidate))
    if found:
        return str(Path(found).resolve())
    path = Path(candidate).expanduser()
    return str(path.resolve()) if path.is_file() and os.access(path, os.X_OK) else None


def find_engine(requested, explicit_tectonic=None):
    if explicit_tectonic and requested not in ("auto", "tectonic"):
        raise ValueError("--tectonic is only valid with --engine auto or --engine tectonic")
    if explicit_tectonic:
        tectonic = executable_path(explicit_tectonic)
        if not tectonic:
            raise ValueError(f"Explicit Tectonic executable unavailable: {explicit_tectonic}")
    else:
        candidates = [os.environ.get("CODEX_TECTONIC_PATH"), "tectonic"]
        candidates.extend(str(path) for path in sorted(
            (Path.home() / ".local" / "opt" / "chatgpt").glob(
                "*/resources/tectonic/tectonic"), reverse=True))
        tectonic = next((path for item in candidates if (path := executable_path(item))), None)
    choices = {"tectonic": tectonic, "latexmk": executable_path("latexmk"),
               "pdflatex": executable_path("pdflatex")}
    for name in (("tectonic", "latexmk", "pdflatex") if requested == "auto" else (requested,)):
        if choices[name]:
            return name, choices[name]
    raise RuntimeError("No usable TeX engine found. Install Tectonic/TeX Live, pass --tectonic, "
                       "set CODEX_TECTONIC_PATH, or use --prepare-only.")


LOG_FAILURES = (
    ("overfull box", r"Overfull\s+\\[hv]box"),
    ("undefined reference or citation", r"(?:Reference|Citation)[^\n]*undefined"),
    ("undefined reference or citation", r"(?:undefined references|undefined citations)"),
    ("missing glyph", r"Missing (?:character|glyph)\s*:"),
    ("unavailable font shape", r"Font shape[^\n]*undefined|Some font shapes were not available"),
    ("LaTeX error", r"(?:LaTeX|Package\s+\S+)\s+Error\s*:|^!"),
)


def check_latex_log(path):
    if not path.is_file():
        raise RuntimeError("The TeX engine did not produce its required final .log file")
    content = path.read_text(encoding="utf-8", errors="replace")
    failures = sorted({label for label, pattern in LOG_FAILURES
                       if re.search(pattern, content, re.IGNORECASE | re.MULTILINE)})
    if failures:
        raise RuntimeError("Final LaTeX log failed validation: " + ", ".join(failures))
    return content


def source_files():
    allowed = {".tex", ".bib", ".pdf", ".png", ".jpg", ".jpeg", ".svg", ".md", ".txt",
               ".cls", ".bst", ".sty"}
    return sorted(path for path in REPORT.rglob("*")
                  if path.is_file() and path.suffix.lower() in allowed
                  and not any(part.startswith(".") or part == "__pycache__"
                              for part in path.relative_to(REPORT).parts))


def write_source_archive():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    path = OUTPUT / "assignment_1_latex_source.zip"
    with ZipFile(path, "w", compression=ZIP_DEFLATED) as archive:
        for source in source_files():
            archive.write(source, source.relative_to(REPORT).as_posix())
    return path


def package_submission(pdf_path, source_archive):
    path = OUTPUT / "assignment_1_submission.zip"
    with ZipFile(path, "w", compression=ZIP_DEFLATED) as archive:
        for source in (HERE / "forward_kinematics.py", HERE / "inverse_kinematics.py",
                       pdf_path, *(RESULTS / name for name in RESULT_FILES), source_archive):
            archive.write(source, source.name)
        for source in source_files():
            archive.write(source, "latex/" + source.relative_to(REPORT).as_posix())
        archive.write(LOG_OUTPUT / "compile_manifest.json", "latex/compile_manifest.json")
        archive.writestr("README.txt", (
            "ROSE5710 Assignment 1: Kinematics\n\n"
            "Contains both completed Python files, the IEEE LaTeX report PDF,\n"
            "original measured FK/IK JSON and recorded environment, and the\n"
            "complete LaTeX sources (also supplied as a separate ZIP).\n\n"
            "The Python files run inside Open_Duck_Playground under\n"
            "assignments/assignment_1_kinematics/. The runnable project,\n"
            "dependencies and tests are available at:\n"
            "https://github.com/David0412-sun/5710_Assignment1\n\n"
            "Follow the repository README for the Python 3.11 environment and\n"
            "LaTeX rebuild commands. No simulations run during report building.\n"
        ))
    return path


def compile_report(engine, executable, environment, input_hashes):
    if not SOURCE.is_file() or not (REPORT / "references.bib").is_file():
        raise FileNotFoundError("The main report .tex and references.bib must exist before compilation")
    BUILD.mkdir(parents=True, exist_ok=True)
    LOG_OUTPUT.mkdir(parents=True, exist_ok=True)
    stem = SOURCE.stem
    for suffix in (".pdf", ".log", ".aux", ".bbl", ".blg", ".toc", ".out", ".fls",
                   ".fdb_latexmk", ".synctex.gz", ".xdv", ".bcf", ".run.xml"):
        (BUILD / (stem + suffix)).unlink(missing_ok=True)
    env = os.environ.copy()
    env.setdefault("XDG_CACHE_HOME", "/tmp/5710-tex-cache")
    env.setdefault("XDG_CONFIG_HOME", "/tmp/5710-tex-config")
    env["PATH"] = str(Path(executable).parent) + os.pathsep + env.get("PATH", "")
    env["BIBINPUTS"] = str(REPORT) + os.pathsep + env.get("BIBINPUTS", "")
    version = subprocess.run([executable, "--version"], env=env, text=True,
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                             check=True).stdout.strip().splitlines()[0]
    manifest = {
        "status": "compiling", "started_utc": datetime.now(timezone.utc).isoformat(),
        "engine": engine, "engine_version": version, "executable": executable,
        "source": str(SOURCE.relative_to(ROOT)),
        "result_sha256": input_hashes, "recorded_source_sha256": environment["source_sha256"],
        "latex_source_sha256": {str(path.relative_to(REPORT)): sha256(path) for path in source_files()},
        "cache_directory": env["XDG_CACHE_HOME"], "commands": [],
        "log_checks": sorted({label for label, _ in LOG_FAILURES}),
    }
    transcript = []
    (LOG_OUTPUT / "compile.stdout.log").write_text("", encoding="utf-8")

    def run(command):
        process = subprocess.run(command, cwd=REPORT, env=env, text=True,
                                 encoding="utf-8", errors="replace", stdout=subprocess.PIPE,
                                 stderr=subprocess.STDOUT, check=False)
        manifest["commands"].append({"argv": command, "cwd": str(REPORT),
                                     "returncode": process.returncode})
        transcript.append("$ " + " ".join(command) + "\n" + process.stdout)
        (LOG_OUTPUT / "compile.stdout.log").write_text("\n\n".join(transcript), encoding="utf-8")
        if process.returncode:
            raise RuntimeError(f"TeX command failed ({process.returncode}); see "
                               f"{LOG_OUTPUT / 'compile.stdout.log'}")

    log = BUILD / f"{stem}.log"
    try:
        if engine == "tectonic":
            run([executable, "--keep-logs", "--keep-intermediates", "--outdir", str(BUILD), SOURCE.name])
        elif engine == "latexmk":
            run([executable, "-norc", "-pdf", "-halt-on-error", "-interaction=nonstopmode",
                 f"-outdir={BUILD}", SOURCE.name])
        else:
            bibtex = shutil.which("bibtex", path=env["PATH"])
            if not bibtex:
                raise RuntimeError("pdflatex also requires BibTeX on PATH")
            command = [executable, "-halt-on-error", "-interaction=nonstopmode",
                       f"-output-directory={BUILD}", SOURCE.name]
            run(command)
            run([bibtex, str(BUILD / stem)])
            run(command)
            run(command)
            for _ in range(2):
                if not re.search(r"Rerun to get|Label\(s\) may have changed", log.read_text(
                        encoding="utf-8", errors="replace"), re.IGNORECASE):
                    break
                run(command)
        check_latex_log(log)
        verify_source_hashes(environment)
        if any(sha256(RESULTS / name) != digest for name, digest in input_hashes.items()):
            raise ValueError("A measured JSON file changed during the report build")
        current_source_hashes = {str(path.relative_to(REPORT)): sha256(path) for path in source_files()}
        if current_source_hashes != manifest["latex_source_sha256"]:
            raise ValueError("A LaTeX source or figure changed during compilation; rebuild the report")
        built_pdf = BUILD / f"{stem}.pdf"
        if not built_pdf.is_file() or built_pdf.stat().st_size == 0:
            raise RuntimeError("The TeX engine did not produce a nonempty PDF")
        pdf = OUTPUT / "pdf" / "assignment_1_report.pdf"
        pdf.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(built_pdf, pdf)
        manifest.update(status="passed", pdf=str(pdf.relative_to(ROOT)), pdf_sha256=sha256(pdf))
    except Exception as error:
        manifest.update(status="failed", error=str(error))
        raise
    finally:
        manifest["finished_utc"] = datetime.now(timezone.utc).isoformat()
        for suffix in (".log", ".blg"):
            path = BUILD / f"{stem}{suffix}"
            if path.is_file():
                shutil.copy2(path, LOG_OUTPUT / path.name)
        (LOG_OUTPUT / "compile_manifest.json").write_text(
            json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return pdf


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--engine", choices=("auto", "tectonic", "latexmk", "pdflatex"), default="auto")
    parser.add_argument("--tectonic", help="Explicit Tectonic executable (also supports CODEX_TECTONIC_PATH)")
    parser.add_argument("--prepare-only", action="store_true", help="Prepare validated tables and figure without compiling")
    args = parser.parse_args()
    fk, ik, environment = load_results()
    input_hashes = {name: sha256(RESULTS / name) for name in RESULT_FILES}
    prepare_sources(fk, ik)
    if args.prepare_only:
        print(GENERATED)
        print(ASSETS / "ik_error_plot.pdf")
        return
    engine, executable = find_engine(args.engine, args.tectonic)
    pdf = compile_report(engine, executable, environment, input_hashes)
    source_archive = write_source_archive()
    package = package_submission(pdf, source_archive)
    for path in (pdf, source_archive, package, LOG_OUTPUT / "compile_manifest.json"):
        print(path)


if __name__ == "__main__":
    main()
