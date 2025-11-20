#!/usr/bin/env python3
"""Plot coverage and cumulative distance curves from exploration CSV output.

支持：
- 单个或多个 CSV 文件叠加显示；
- 自定义图例标签（labels）；
- 自定义图标题（title）和导出清晰度（dpi）；
- 自动创建输出目录。
"""

import argparse
import csv
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import matplotlib.pyplot as plt


def read_metrics(csv_path: Path) -> Tuple[List[int], List[float], List[float]]:
    """从单个 CSV 文件读取迭代次数、覆盖率和累计距离。"""
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


def plot_curves(
    csv_paths: Sequence[Path],
    output_path: Path,
    labels: Optional[Sequence[str]] = None,
    title: str = "Exploration progress",
    dpi: int = 200,
) -> None:
    """从一个或多个 CSV 文件绘制覆盖率和累计距离曲线。"""
    if not csv_paths:
        raise RuntimeError("No CSV paths provided")

    if labels is not None and len(labels) != len(csv_paths):
        raise RuntimeError(
            "The number of labels must match the number of CSV files provided"
        )

    datasets = []
    for csv_path in csv_paths:
        iterations, coverage, distance = read_metrics(csv_path)
        if not iterations:
            raise RuntimeError(f"No data found in metrics CSV: {csv_path}")
        datasets.append((csv_path, iterations, coverage, distance))

    # 创建两行子图，共享 x 轴
    fig, (ax_cov, ax_dist) = plt.subplots(2, 1, sharex=True, figsize=(8, 8))

    # 使用 Matplotlib 默认色板，方便多条曲线区分
    color_cycle = plt.rcParams["axes.prop_cycle"].by_key().get("color", [])

    for idx, (csv_path, iterations, coverage, distance) in enumerate(datasets):
        color = color_cycle[idx % len(color_cycle)] if color_cycle else None
        label = labels[idx] if labels is not None else csv_path.stem

        ax_cov.plot(iterations, coverage, marker="o", label=label, color=color)
        ax_dist.plot(iterations, distance, marker="o", label=label, color=color)

    # 覆盖率子图设置
    ax_cov.set_ylabel("Coverage")
    ax_cov.set_ylim(0.0, 1.05)
    ax_cov.grid(True, linestyle=":", alpha=0.5)
    ax_cov.legend()

    # 距离子图设置
    ax_dist.set_xlabel("Iteration")
    ax_dist.set_ylabel("Cumulative distance (m)")
    ax_dist.grid(True, linestyle=":", alpha=0.5)
    ax_dist.legend()

    fig.suptitle(title)
    fig.tight_layout(rect=[0, 0.03, 1, 0.95])

    # 确保输出目录存在
    output_dir = output_path.parent
    if output_dir and not output_dir.exists():
        output_dir.mkdir(parents=True, exist_ok=True)

    if dpi <= 0:
        raise RuntimeError("DPI must be a positive integer")

    fig.savefig(str(output_path), dpi=dpi)
    print(f"Saved plot to {output_path}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)

    # 支持传入 1 个或多个 CSV
    parser.add_argument(
        "csv_paths",
        type=Path,
        nargs="+",
        help="One or more exploration_metrics CSV files to plot",
    )

    parser.add_argument(
        "--output",
        type=Path,
        default=Path("exploration_curves.png"),
        help="Output PNG path",
    )

    parser.add_argument(
        "--labels",
        type=str,
        nargs="*",
        help="Optional legend labels (must match the number of CSV files)",
    )

    parser.add_argument(
        "--title",
        type=str,
        default="Exploration progress",
        help="Custom figure title",
    )

    parser.add_argument(
        "--dpi",
        type=int,
        default=200,
        help="Image DPI when saving the figure",
    )

    args = parser.parse_args()
    plot_curves(args.csv_paths, args.output, args.labels, args.title, args.dpi)


if __name__ == "__main__":
    main()

