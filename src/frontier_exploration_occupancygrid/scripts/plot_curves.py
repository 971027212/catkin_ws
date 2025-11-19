#!/usr/bin/env python3
"""Plot coverage and cumulative distance curves from exploration CSV output."""
import argparse
import csv
from pathlib import Path
from typing import List, Tuple

import matplotlib.pyplot as plt


def read_metrics(csv_path: Path) -> Tuple[List[int], List[float], List[float]]:
    iterations: List[int] = []
    coverage: List[float] = []
    distance: List[float] = []
    with csv_path.open("r", encoding="utf-8") as csv_file:
        reader = csv.DictReader(csv_file)
        for row in reader:
            iterations.append(int(row["iteration"]))
            coverage.append(float(row["coverage"]))
            distance.append(float(row["distance"]))
    return iterations, coverage, distance


def plot_curves(csv_path: Path, output_path: Path) -> None:
    iterations, coverage, distance = read_metrics(csv_path)
    if not iterations:
        raise RuntimeError("No data found in metrics CSV")

    fig, (ax_cov, ax_dist) = plt.subplots(2, 1, sharex=True, figsize=(8, 8))
    ax_cov.plot(iterations, coverage, marker="o", color="tab:blue")
    ax_cov.set_ylabel("Coverage")
    ax_cov.set_ylim(0.0, 1.05)
    ax_cov.grid(True, linestyle=":", alpha=0.5)

    ax_dist.plot(iterations, distance, marker="o", color="tab:orange")
    ax_dist.set_xlabel("Iteration")
    ax_dist.set_ylabel("Cumulative distance (m)")
    ax_dist.grid(True, linestyle=":", alpha=0.5)

    fig.suptitle("Exploration progress")
    fig.tight_layout(rect=[0, 0.03, 1, 0.95])
    fig.savefig(str(output_path), dpi=200)
    print(f"Saved plot to {output_path}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_path", type=Path, help="Path to exploration_metrics CSV")
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("exploration_curves.png"),
        help="Output PNG path"
    )
    args = parser.parse_args()
    plot_curves(args.csv_path, args.output)


if __name__ == "__main__":
    main()
