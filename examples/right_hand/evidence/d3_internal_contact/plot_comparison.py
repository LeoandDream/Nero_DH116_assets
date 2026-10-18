#!/usr/bin/env python3
"""Plot finalized saved traces only; no simulator or project-runtime imports."""

import argparse
import gzip
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


HERE = Path(__file__).resolve().parent
ACTIVE = ("finger11", "finger12", "finger21", "finger31", "finger41", "finger51")
EXCLUDED_PAIRS = {
    frozenset(("gripper0_base_link", "gripper0_finger12_link")),
    frozenset(("gripper0_finger12_link", "gripper0_finger42_link")),
}


def read_rows(path):
    with gzip.open(path, "rt", encoding="utf-8") as stream:
        rows = [json.loads(line) for line in stream]
    if len(rows) != 7000 or [row["step"] for row in rows] != list(range(1, 7001)):
        raise ValueError(f"Expected a finalized 7000-step trace: {path}")
    return rows


def remaining_overlap_mm(rows):
    """Use recorded actual-body pairs, correcting the raw weld-based annotation.

    The recorder falsely marked finger12/finger43 excluded via finger43's weld
    body finger42. The engine's exclusions match actual body pairs instead.
    All other recorded robot self-contacts in this no-object run are retained.
    """
    return np.array([
        max((max(0.0, -contact["distance_m"]) * 1000.0
             for contact in row["contacts"]
             if all(name.startswith(("robot0_", "gripper0_", "mount0_"))
                    for name in contact["body_names"])
             and frozenset(contact["body_names"]) not in EXCLUDED_PAIRS), default=0.0)
        for row in rows
    ])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True, help="New PNG image path")
    args = parser.parse_args()
    output = args.output.expanduser().resolve()
    if output.suffix.lower() != ".png":
        raise ValueError("Output must be a PNG image")
    if output.exists():
        raise FileExistsError(f"Choose a new output path: {output}")

    cases = []
    for name, color, directory, prefix in (
        ("Baseline", "tab:blue", HERE.parent, "hand-motion"),
        ("A+B exclusions", "tab:orange", HERE, "filtered"),
    ):
        states = read_rows(directory / f"{prefix}-state.jsonl.gz")
        contacts = read_rows(directory / f"{prefix}-contacts.jsonl.gz")
        time = np.array([row["time_s"] for row in states])
        if not np.array_equal(time, [row["time_s"] for row in contacts]):
            raise ValueError(f"State/contact times differ: {name}")
        cases.append({"name": name, "color": color, "states": states,
                      "contacts": contacts, "time": time})
    if not np.array_equal(cases[0]["time"], cases[1]["time"]):
        raise ValueError("Baseline/candidate times differ")
    for field in ("phase", "steady_window", "robot_action_13", "hand_target_rad"):
        if [row[field] for row in cases[0]["states"]] != [row[field] for row in cases[1]["states"]]:
            raise ValueError(f"Frozen baseline/candidate protocol differs: {field}")

    fig, axes = plt.subplots(4, 2, figsize=(15, 15), sharex=True)
    plotted_summary = {}
    for case in cases:
        time, states = case["time"], case["states"]
        steady = np.array([row["steady_window"] for row in states], dtype=bool)
        errors = np.array([row["active_tracking_error_rad"] for row in states])
        for index, ax in enumerate(axes.flat[:6]):
            for mask, label, alpha, width in (
                (~steady, "transient", 0.45, 0.8),
                (steady, "steady", 1.0, 1.8),
            ):
                ax.plot(time, np.where(mask, errors[:, index], np.nan),
                        color=case["color"], alpha=alpha, lw=width,
                        label=f"{case['name']} {label}")
        overlap = remaining_overlap_mm(case["contacts"])
        axes[3, 0].plot(time, overlap, color=case["color"], lw=1.1,
                        label=f"{case['name']} (peak {overlap.max():.4f} mm)")
        overshoot = [max(abs(value) for value in row["joint_limit_overshoot_rad"].values())
                     for row in states]
        axes[3, 1].plot(time, overshoot, color=case["color"], lw=1.1,
                        label=case["name"])
        plotted_summary[case["name"]] = {
            "steps": len(states), "steady_samples": int(steady.sum()),
            "max_remaining_overlap_mm": float(overlap.max()),
            "steady_tracking": {
                name: {"max_abs_error_rad": float(np.abs(errors[steady, index]).max()),
                       "exceed_samples": int((np.abs(errors[steady, index]) > 0.02).sum())}
                for index, name in enumerate(ACTIVE)
            },
        }

    for name, ax in zip(ACTIVE, axes.flat):
        ax.set(title=f"{name}: signed active tracking error", ylabel="Error / rad")
        ax.axhline(0.02, color="crimson", ls="--", lw=0.8,
                   label="Original steady bound: +/-0.02 rad")
        ax.axhline(-0.02, color="crimson", ls="--", lw=0.8)
    axes[3, 0].set(title="Remaining recorded robot self-contact (actual A/B removed)",
                   ylabel="Max cached overlap / mm")
    axes[3, 0].axhline(2.0, color="crimson", ls="--", lw=0.8,
                       label="Original overlap bound: 2 mm")
    axes[3, 1].set(title="Maximum joint-limit overshoot across all robot joints",
                   ylabel="Overshoot / rad")
    axes[3, 1].axhline(0.01, color="crimson", ls="--", lw=0.8,
                       label="Original joint-limit bound: 0.01 rad")
    for ax in axes.flat:
        ax.grid(alpha=0.2)
        ax.legend(fontsize=7, loc="best")
    for ax in axes[-1]:
        ax.set_xlabel("Recorded simulation time / s")
    fig.suptitle("Internal-contact exclusion diagnostic: saved baseline vs A+B exclusions\n"
                 "Frozen commands and steady windows; filtering does not establish geometry validity",
                 fontsize=13)
    fig.text(0.5, 0.012,
             "A: base_link / finger12_link; B: finger12_link / finger42_link. "
             "Contact overlap uses saved RK4 caches; 0 means no remaining recorded overlap.\n"
             "Raw contact annotation corrected by actual body-pair semantics. "
             "Tracking / joint limits use post-step states; tracking bounds apply only to steady samples.",
             ha="center", fontsize=8)
    fig.tight_layout(rect=(0, 0.045, 1, 0.955))
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=160)
    plt.close(fig)
    print(output)
    print(json.dumps(plotted_summary, indent=2))


if __name__ == "__main__":
    main()
