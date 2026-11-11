# LaTeX report source

This report uses the standard `IEEEtran` conference class in A4, two-column
mode. The class controls the margins, text size, columns, title, headings,
captions and bibliography. The document does not override the class geometry
or simulate a paper layout in a drawing library.

T1 encoding is selected before the class is loaded so IEEEtran's standard
Times-family text fonts are available under both pdfLaTeX and
XeTeX/Tectonic, without silent substitution by a different text family.

## Editable files

- `assignment_1_report.tex`: the paper text, numbered equations and references.
- `references.bib`: bibliography, formatted by `IEEEtran.bst` and BibTeX.
- `generated/`: D-H tables, result tables and numerical macros prepared from
  the completed implementation and recorded measurements.
- `assets/ik_error_plot.pdf`: embedded-font vector figure, regenerated from
  all 1,515 recorded IK measurements.

No personal name or student number was supplied. The title block identifies
the course and assignment. Replace its `\author{...}` block with the required
name/student-number details before submission if the course requires them.

## Rebuild within the repository

From the `Open_Duck_Playground` root:

```bash
.venv/bin/python -m pip install -r assignments/assignment_1_kinematics/requirements-report.txt
.venv/bin/python assignments/assignment_1_kinematics/build_report.py
```

The builder checks measured maxima, sample coverage and the stored FK/IK/XML
fingerprints, prepares tables and the vector figure, and runs a real LaTeX
engine. It does not rerun or alter the simulation measurements.

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

or a standard TeX Live / MiKTeX installation with `IEEEtran`:

```bash
latexmk -norc -pdf -interaction=nonstopmode -halt-on-error assignment_1_report.tex
```

Overleaf can also open the source ZIP; choose `assignment_1_report.tex` as
the main file and use its pdfLaTeX compiler. The generated tables and asset
paths are relative and do not depend on this computer's absolute paths.

## Build checks and outputs

The repository builder rejects LaTeX errors, unresolved references or
citations, missing glyphs and overfull boxes in the final compile log.
Visual inspection should still be used to confirm the page layout.

- `output/pdf/assignment_1_report.pdf`: the compiled paper.
- `output/assignment_1_latex_source.zip`: portable LaTeX source project.
- `output/assignment_1_submission.zip`: updated submission files.
- `output/latex/`: retained compilation log, engine output and build manifest.

Scientific tables use metres and radians. The figure normalizes errors by
their required limits and preserves every sample, including zeros.
