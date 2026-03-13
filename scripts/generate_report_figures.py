#!/usr/bin/env python3
"""
Generate report figures for ORB-SLAM3 monocular VO evaluation.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def load_tum_trajectory(path: str):
    data = np.loadtxt(path)
    timestamps = data[:, 0]
    positions = data[:, 1:4]
    quaternions = data[:, 4:8]
    return timestamps, positions, quaternions


def associate_trajectories(t_gt, t_est, max_time_diff=0.1):
    matches = []
    for est_index, timestamp_est in enumerate(t_est):
        diffs = np.abs(t_gt - timestamp_est)
        gt_index = int(np.argmin(diffs))
        if diffs[gt_index] < max_time_diff:
            matches.append((gt_index, est_index))
    return matches


def load_ate_rmse(results_file: str) -> float:
    content = Path(results_file).read_text(encoding="utf-8")
    for line in content.splitlines():
        if line.startswith("ATE RMSE:"):
            value = line.split(":")[1].strip().split()[0]
            return float(value)
    raise ValueError("ATE RMSE not found in evaluation_results.txt")


def compute_ate_errors(P_gt, P_est):
    return np.linalg.norm(P_gt - P_est, axis=1)


def generate_figure(
    groundtruth_file: str,
    aligned_file: str,
    results_file: str,
    output_file: str,
    max_time_diff: float = 0.1,
):
    t_gt, P_gt, _ = load_tum_trajectory(groundtruth_file)
    t_est, P_est, _ = load_tum_trajectory(aligned_file)

    matches = associate_trajectories(t_gt, t_est, max_time_diff=max_time_diff)
    if len(matches) == 0:
        raise ValueError("No matched poses found between ground truth and aligned trajectory.")

    gt_indices = [item[0] for item in matches]
    est_indices = [item[1] for item in matches]

    P_gt_matched = P_gt[gt_indices]
    P_est_matched = P_est[est_indices]

    ate_errors = compute_ate_errors(P_gt_matched, P_est_matched)
    ate_rmse = load_ate_rmse(results_file)

    plt.figure(figsize=(14, 10))

    # Top-left: XY trajectory
    plt.subplot(2, 2, 1)
    plt.plot(
        P_gt_matched[:, 0],
        P_gt_matched[:, 1],
        "g-",
        linewidth=2,
        label="Ground Truth",
        alpha=0.8,
    )
    plt.plot(
        P_est_matched[:, 0],
        P_est_matched[:, 1],
        "b-",
        linewidth=1.5,
        label="Aligned VO",
        alpha=0.8,
    )
    plt.xlabel("X (East) [m]")
    plt.ylabel("Y (North) [m]")
    plt.title("2D Trajectory After Alignment")
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.axis("equal")

    # Top-right: XZ trajectory
    plt.subplot(2, 2, 2)
    plt.plot(
        P_gt_matched[:, 0],
        P_gt_matched[:, 2],
        "g-",
        linewidth=2,
        label="Ground Truth",
        alpha=0.8,
    )
    plt.plot(
        P_est_matched[:, 0],
        P_est_matched[:, 2],
        "b-",
        linewidth=1.5,
        label="Aligned VO",
        alpha=0.8,
    )
    plt.xlabel("X (East) [m]")
    plt.ylabel("Z (Up) [m]")
    plt.title("XZ Trajectory After Alignment")
    plt.legend()
    plt.grid(True, alpha=0.3)

    # Bottom-left: error histogram
    plt.subplot(2, 2, 3)
    plt.hist(ate_errors, bins=40, color="steelblue", edgecolor="black", alpha=0.75)
    plt.axvline(np.mean(ate_errors), color="red", linestyle="--", linewidth=2, label=f"Mean: {np.mean(ate_errors):.3f} m")
    plt.axvline(np.median(ate_errors), color="orange", linestyle="--", linewidth=2, label=f"Median: {np.median(ate_errors):.3f} m")
    plt.xlabel("ATE Error [m]")
    plt.ylabel("Frequency")
    plt.title("ATE Error Distribution")
    plt.legend()
    plt.grid(True, alpha=0.3)

    # Bottom-right: error over index
    plt.subplot(2, 2, 4)
    indices = np.arange(len(ate_errors))
    plt.plot(indices, ate_errors, color="purple", linewidth=1.2)
    plt.fill_between(indices, 0, ate_errors, color="mediumpurple", alpha=0.3)
    plt.xlabel("Matched Pose Index")
    plt.ylabel("ATE Error [m]")
    plt.title(f"ATE Along Trajectory (RMSE = {ate_rmse:.4f} m)")
    plt.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_file, dpi=300, bbox_inches="tight")
    plt.close()

    print(f"Figure saved to: {output_file}")


def main():
    parser = argparse.ArgumentParser(description="Generate trajectory evaluation figures for report.")
    parser.add_argument(
        "--groundtruth",
        default="data/rtk_groundtruth.txt",
        help="Ground truth trajectory in TUM format.",
    )
    parser.add_argument(
        "--aligned",
        default="evaluation_results/aligned_trajectory.txt",
        help="Aligned estimated trajectory in TUM format.",
    )
    parser.add_argument(
        "--results",
        default="evaluation_results/evaluation_results.txt",
        help="Evaluation summary text file.",
    )
    parser.add_argument(
        "--output",
        default="evaluation_results/trajectory_evaluation.png",
        help="Output image path.",
    )
    parser.add_argument(
        "--max-time-diff",
        type=float,
        default=0.1,
        help="Maximum timestamp difference for association.",
    )
    args = parser.parse_args()

    generate_figure(
        groundtruth_file=args.groundtruth,
        aligned_file=args.aligned,
        results_file=args.results,
        output_file=args.output,
        max_time_diff=args.max_time_diff,
    )


if __name__ == "__main__":
    main()
