# LaTeX report source

This is a course assignment report in a single-column A4 layout with 11 pt
text, a simple title, and page numbers. Its sections cover the task, forward
kinematics, inverse kinematics, verification results, and conclusion.
The D-H tables, necessary equations, and measured error plot are retained,
with numbered cross-references to figures, tables, and equations. The report
has no bibliography.

## Editable files

- `assignment_1_report.tex`: the report text, layout, equations, and
  figure/table/equation cross-references.
- `generated/`: D-H tables, result tables and numerical macros prepared from
  the completed implementation and recorded measurements.
- `assets/ik_error_plot.pdf`: embedded-font vector figure, regenerated from
  all 1,515 recorded IK measurements.
- `assets/assignment_left_original.pdf`, `assignment_right_original.pdf`,
  `assignment_head_original.pdf`: lossless vector crops of Figs. 2--4 in
  the supplied root-level `Assignment 1.pdf`. Original labels and axes are retained.
- `assets/assignment_original_source.json`: original PDF and crop SHA-256
  fingerprints, page/figure numbers, and exact crop bounds.
- `assets/dh_left.pdf`, `dh_right.pdf`, `dh_head.pdf`: retained simplified
  zero-pose joint/link models for reference.
  Selected origins use `O_i`, joints use `J_i`, rigid links use `ell_i`,
  and all x/y/z axes are shown.
  Blue/red/green match the original drawings' x/y/z colours. Dots are
  selected D-H origins on joint-axis lines; connectors show link topology.
- `assets/assignment_{left,right,head}_annotated.pdf`: original vector
  figures with native overlays marking `O_i`, intermediate `P_i`, and
  orange `a_(i-1)` / purple `d_i` construction routes. Colour-coded axis
  keys and tables specify exact positive directions in the base frame.
  The source sketch is not a calibrated projection: on-picture routes
  show parameter association, while signed tables define the geometry.
- `annotations/*_overlay.tex`: editable annotation sources; the large
  sheets are supplied as a separate PDF guide.
- `assets/dh_{left,right,head}_annotated_panel.pdf`: figure-only vector
  panels used directly in the report's D-H sections. They retain the
  detailed origin, distance and axis callouts, while the report's original
  D-H and axis tables retain their existing format and values.
- `assets/report_dh_panel_metadata.json`: panel crop/source/output records.
- `assets/assignment_annotation_metadata.json`: annotation input/output
  hashes, numerical intermediate points, and axis/geometry checks.
- `assets/fk_robot.png`: actual MuJoCo rendering at zero joint angles,
  beside the FK explanation, with base axes and site labels.
- `assets/ik_start.png`, `ik_solved.png`: initial and solved robot poses;
  coloured dots are actual sites and orange diamonds are phase-0.5 targets.
- `assets/illustration_metadata.json`: joint values, target/actual poses,
  errors and source hashes for the robot illustrations.

No personal name or student number was supplied. The title identifies the
course and assignment. Add the required name/student-number details in the
title area before submission if the course requires them.

## Rebuild within the repository

From the `Open_Duck_Playground` root:

On Windows (PowerShell):

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\setup_windows.ps1 -Report
powershell -NoProfile -ExecutionPolicy Bypass -File .\run_assignment.ps1 verify
powershell -NoProfile -ExecutionPolicy Bypass -File .\run_assignment.ps1 report
```

`verify` refreshes the measurements and environment together after all checks
pass. `report` reuses those results. Windows auto detection prefers pdfLaTeX
over latexmk after Tectonic, so a MiKTeX installation does not require Perl.
Source fingerprints normalize CRLF to LF and still reject content changes.

On Linux/macOS:

```bash
.venv/bin/python -m pip install -r assignments/assignment_1_kinematics/requirements-report.txt
.venv/bin/python assignments/assignment_1_kinematics/build_report.py
```

The builder checks measured maxima, sample coverage and the stored FK/IK/XML
fingerprints, prepares tables and the vector figure, and runs a real LaTeX
engine. It does not rerun or alter the simulation measurements.

The builder regenerates the analytical D-H diagrams and checks their axes,
origins and site poses against MuJoCo. Original vector crops are reused after
checking their hashes and the original PDF hash when that PDF is available.
Robot renders are reused after checking their source hashes.
To recreate the original crops (requires pdfLaTeX and pdfinfo on PATH):

```powershell
.\.venv\Scripts\python.exe assignments/assignment_1_kinematics/extract_assignment_figures.py
```

To regenerate those images explicitly from the repository root:

```powershell
.\.venv\Scripts\python.exe assignments/assignment_1_kinematics/render_robot_figures.py
powershell -NoProfile -ExecutionPolicy Bypass -File .\run_assignment.ps1 report
```

After editing an original-figure overlay, rebuild the annotated sheets first:

```powershell
.\.venv\Scripts\python.exe assignments/assignment_1_kinematics/compile_dh_annotations.py
.\.venv\Scripts\python.exe assignments/assignment_1_kinematics/prepare_report_dh_panels.py
powershell -NoProfile -ExecutionPolicy Bypass -File .\run_assignment.ps1 report
```

This writes `output/pdf/assignment_1_annotated_figures.pdf` as a standalone
three-page landscape guide. Original PDF data remains vector-based; the
source handout is never modified. Directly compiling `annotated_figures.tex`
inside the source package also produces the standalone guide from its
included annotated PDFs.

The rendering script follows the first 51 phases of the original IK
trajectory (0 through 0.5) for illustration only; it does not replace the
full five-cycle experiment. `--reuse-state` checks the saved source hashes
and redraws the saved poses without solving again. These scripts render
offscreen and require a working MuJoCo OpenGL context. All images are
already included in the portable source ZIP, so compiling that ZIP does
not require MuJoCo or OpenGL.

You can select an engine explicitly:

```bash
.venv/bin/python assignments/assignment_1_kinematics/build_report.py --engine tectonic
.venv/bin/python assignments/assignment_1_kinematics/build_report.py --engine latexmk
```

For Tectonic outside `PATH`, set `CODEX_TECTONIC_PATH` to its executable or
pass `--tectonic /absolute/path/to/tectonic`. The first Tectonic build may
need Internet access to download the class, fonts and packages; later builds
reuse the local cache.

## Compile the extracted source package

The generated source ZIP already contains the data tables and vector figure.
Extract it, enter its source directory, then use either:

```bash
tectonic --keep-logs --keep-intermediates assignment_1_report.tex
```

or a standard TeX Live / MiKTeX installation:

```bash
latexmk -norc -pdf -interaction=nonstopmode -halt-on-error assignment_1_report.tex
```

Overleaf can also open the source ZIP; choose `assignment_1_report.tex` as
the main file and use its pdfLaTeX compiler. The generated tables and asset
paths are relative and do not depend on this computer's absolute paths.

## Build checks and outputs

The repository builder rejects LaTeX errors, unresolved figure/table/equation
cross-references, missing glyphs and overfull boxes in the final compile log.
Visual inspection should still be used to confirm the page layout.

- `output/pdf/assignment_1_report.pdf`: the compiled course report.
- `output/pdf/assignment_1_annotated_figures.pdf`: annotated original figures.
- `output/assignment_1_latex_source.zip`: portable LaTeX source project.
- `output/assignment_1_submission.zip`: updated submission files.
- `output/latex/`: retained compilation log, engine output and build manifest.

Scientific tables use metres and radians. The figure normalizes errors by
their required limits and preserves every sample, including zeros.
