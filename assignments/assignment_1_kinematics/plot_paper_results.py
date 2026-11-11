"""Plot measured IK errors as a vector figure for the coursework report.

The plot uses every recorded sample without smoothing or interpolation. The
ordinate is error divided by its required tolerance, so one marks the passing
boundary. Run this file directly, or call ``generate_plot`` from a report
builder. Neither entry point modifies the source measurements.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


HERE = Path(__file__).resolve().parent
DEFAULT_RESULTS = HERE / "results" / "ik_results.json"
CHAIN_STYLES = {
    "left": {"label": "Left leg", "color": "#173F66",
             "linestyle": "-", "marker": "o"},
    "right": {"label": "Right leg", "color": "#7B3342",
              "linestyle": (0, (5, 2.2)), "marker": "s"},
    "head": {"label": "Head", "color": "#2B6654",
             "linestyle": (0, (1.1, 1.7)), "marker": "^"},
}


def _read_measurements(path):
    """Validate and group the complete five-cycle, 101-phase comparison."""
    report = json.loads(Path(path).read_text(encoding="utf-8"))
    if not report.get("completed"):
        raise ValueError("A completed IK comparison is required")
    cycles, samples = report["cycles_completed"], report["samples_per_cycle"]
    if cycles != 5 or samples != 101:
        raise ValueError("The report figure requires five cycles of 101 phases")

    records = report["measurements"]
    if len(records) != cycles * samples * len(CHAIN_STYLES):
        raise ValueError("The IK comparison must contain all 1515 measurements")
    grouped = {}
    for chain in CHAIN_STYLES:
        rows = [record for record in records if record["chain"] == chain]
        if len(rows) != cycles * samples:
            raise ValueError(f"Incomplete measurements for {chain}")
        for cycle in range(1, cycles + 1):
            phases = [row["phase"] for row in rows if row["cycle"] == cycle]
            if len(phases) != samples or not np.allclose(
                    phases, np.linspace(0, 1, samples), rtol=0, atol=1e-12):
                raise ValueError(f"Incorrect phase sequence for {chain}, cycle {cycle}")
        grouped[chain] = {
            "x": np.array([row["cycle"] - 1 + row["phase"] for row in rows]),
            "position": np.array([row["position_error_m"] for row in rows]),
            "orientation": np.array([row["orientation_error_rad"] for row in rows]),
        }
        for key in ("position", "orientation"):
            values = grouped[chain][key]
            if not np.all(np.isfinite(values)) or np.any(values < 0):
                raise ValueError(f"Invalid {key} errors for {chain}")
    return report, grouped


def generate_plot(input_path=None, output_dir=None):
    """Return PDF and PNG paths after plotting the existing measured errors.

    The saved figure is exactly 7.16 inches wide. Its PDF contains vector lines
    and embedded TrueType fonts, and its PNG is rendered at 300 dpi.
    """
    input_path = Path(input_path) if input_path is not None else DEFAULT_RESULTS
    output_dir = Path(output_dir) if output_dir is not None else HERE / "results"
    report, data = _read_measurements(input_path)
    tolerances = {
        "position": float(report["position_tolerance_m"]),
        "orientation": float(report["orientation_tolerance_rad"]),
    }
    if tolerances != {"position": 1e-5, "orientation": 1e-4}:
        raise ValueError("The measured comparison must use the assignment tolerances")

    style = {
        "font.family": "serif",
        "font.serif": ["STIXGeneral", "Nimbus Roman", "Times New Roman"],
        "mathtext.fontset": "stix",
        "font.size": 8.5,
        "axes.labelsize": 9,
        "axes.titlesize": 9,
        "axes.linewidth": 0.6,
        "xtick.labelsize": 8,
        "ytick.labelsize": 8,
        "legend.fontsize": 8.5,
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
        "savefig.facecolor": "white",
    }
    with plt.rc_context(style):
        fig, axes = plt.subplots(2, 1, figsize=(7.16, 3.85), sharex=True)
        fig.subplots_adjust(left=0.105, right=0.986, bottom=0.125,
                            top=0.84, hspace=0.36)
        handles = []
        for axis, key, title, ylabel in zip(
                axes, ("position", "orientation"),
                ("(a) Position", "(b) Orientation"),
                (r"$\|e_p\|_2 / \tau_p$",
                 r"$\|e_R\|_2 / \tau_R$")):
            for chain, line_style in CHAIN_STYLES.items():
                values = data[chain][key] / tolerances[key]
                if np.any(values > 1.08):
                    raise ValueError("Measured errors exceed the specified axis limits")
                # Preserve zero-valued initial observations and every other sample.
                line, = axis.plot(
                    data[chain]["x"], values, linewidth=0.95,
                    markersize=2.6, markerfacecolor="white", markeredgewidth=0.6,
                    markevery=50, zorder=3, **line_style)
                if axis is axes[0]:
                    handles.append(line)
            tolerance_line = axis.axhline(
                1.0, color="0.15", linewidth=0.9,
                linestyle=(0, (7, 2, 1.2, 2)), label="Tolerance", zorder=4)
            if axis is axes[0]:
                handles.append(tolerance_line)
            for boundary in range(1, report["cycles_completed"]):
                axis.axvline(boundary, color="0.70", linewidth=0.55,
                             linestyle=(0, (2, 2)), zorder=1)
            axis.set_xlim(0, 5)
            axis.set_ylim(0, 1.08)
            axis.set_yticks([0, 0.25, 0.5, 0.75, 1.0],
                           ["0", "0.25", "0.50", "0.75", "1.00"])
            axis.set_ylabel(ylabel, labelpad=5)
            axis.set_title(title, loc="left", pad=5)
            axis.grid(axis="y", color="0.90", linewidth=0.45, zorder=0)
            axis.tick_params(direction="out", length=2.7, width=0.6,
                             top=False, right=False)
            axis.spines[["top", "right"]].set_visible(False)

        axes[-1].set_xticks(range(6))
        axes[-1].set_xlabel("Cycle coordinate", labelpad=4)
        fig.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.54, 0.995),
                   ncol=4, frameon=False, handlelength=3.1, columnspacing=1.7,
                   handletextpad=0.7)
        output_dir.mkdir(parents=True, exist_ok=True)
        pdf_path = output_dir / "ik_error_plot.pdf"
        png_path = output_dir / "ik_error_plot.png"
        fig.savefig(pdf_path, metadata={
            "Title": "Normalized measured IK tracking errors",
            "Creator": "plot_paper_results.py / Matplotlib",
            "CreationDate": None,
            "ModDate": None,
        })
        fig.savefig(png_path, dpi=300)
        plt.close(fig)
    return pdf_path, png_path


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=DEFAULT_RESULTS)
    parser.add_argument("--output-dir", type=Path, default=HERE / "results")
    args = parser.parse_args()
    for output_path in generate_plot(args.input, args.output_dir):
        print(output_path)
