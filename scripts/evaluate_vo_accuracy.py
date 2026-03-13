#!/usr/bin/env python3
"""
Visual Odometry Accuracy Evaluation Script

This script evaluates VO trajectory accuracy using 3 standard academic metrics:
1. ATE (Absolute Trajectory Error) - RMSE of aligned trajectory
2. RPE (Relative Pose Error) - Local consistency metric
3. Scale Error - Monocular VO scale drift evaluation

References:
- Sturm et al. "A Benchmark for the Evaluation of RGB-D SLAM Systems" (IROS 2012)
- Geiger et al. "Vision meets Robotics: The KITTI Dataset" (IJRR 2013)
"""

import numpy as np
import argparse
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend
import matplotlib.pyplot as plt
import sys


def load_trajectory_tum(filename):
    """
    Load trajectory from TUM format file
    Format: timestamp x y z qx qy qz qw

    Returns:
        timestamps: Nx1 array of timestamps
        positions: Nx3 array of positions (x, y, z)
        orientations: Nx4 array of quaternions (qx, qy, qz, qw)
    """
    data = np.loadtxt(filename)
    timestamps = data[:, 0]
    positions = data[:, 1:4]
    orientations = data[:, 4:8]

    return timestamps, positions, orientations


def associate_trajectories(t_gt, t_est, max_time_diff=0.02):
    """
    Associate ground truth and estimated trajectories by timestamp

    Args:
        t_gt: Ground truth timestamps
        t_est: Estimated timestamps
        max_time_diff: Maximum time difference for association (seconds)

    Returns:
        matches: List of (gt_idx, est_idx) pairs
    """
    matches = []

    for i, t_e in enumerate(t_est):
        # Find closest ground truth timestamp
        time_diffs = np.abs(t_gt - t_e)
        min_idx = np.argmin(time_diffs)

        if time_diffs[min_idx] < max_time_diff:
            matches.append((min_idx, i))

    return matches


def align_trajectories_sim3(P_gt, P_est):
    """
    Align estimated trajectory to ground truth using Sim(3) alignment
    (7-DOF: translation, rotation, scale)

    Uses Horn's method for optimal alignment

    Args:
        P_gt: Nx3 ground truth positions
        P_est: Nx3 estimated positions

    Returns:
        P_aligned: Nx3 aligned estimated positions
        scale: Estimated scale factor
        R: 3x3 rotation matrix
        t: 3x1 translation vector
    """
    # Center the point clouds
    centroid_gt = np.mean(P_gt, axis=0)
    centroid_est = np.mean(P_est, axis=0)

    P_gt_centered = P_gt - centroid_gt
    P_est_centered = P_est - centroid_est

    # Compute scale
    scale = np.sqrt(np.sum(P_gt_centered ** 2) / np.sum(P_est_centered ** 2))

    # Apply scale
    P_est_scaled = P_est_centered * scale

    # Compute rotation using SVD
    H = P_est_scaled.T @ P_gt_centered
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    # Ensure proper rotation matrix (det(R) = 1)
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    # Compute translation
    t = centroid_gt - scale * (R @ centroid_est)

    # Apply full transformation
    P_aligned = scale * (P_est @ R.T) + t

    return P_aligned, scale, R, t


def compute_ate(P_gt, P_aligned):
    """
    Compute Absolute Trajectory Error (ATE)

    ATE measures the RMSE of the aligned trajectory, representing overall
    trajectory accuracy including scale, rotation, and translation errors.

    Args:
        P_gt: Nx3 ground truth positions
        P_aligned: Nx3 aligned estimated positions

    Returns:
        ate_rmse: Root Mean Square Error (meters)
        ate_mean: Mean error (meters)
        ate_median: Median error (meters)
        ate_std: Standard deviation (meters)
        ate_min: Minimum error (meters)
        ate_max: Maximum error (meters)
        errors: Nx1 array of pointwise errors
    """
    # Compute Euclidean distance errors
    errors = np.linalg.norm(P_gt - P_aligned, axis=1)

    ate_rmse = np.sqrt(np.mean(errors ** 2))
    ate_mean = np.mean(errors)
    ate_median = np.median(errors)
    ate_std = np.std(errors)
    ate_min = np.min(errors)
    ate_max = np.max(errors)

    return ate_rmse, ate_mean, ate_median, ate_std, ate_min, ate_max, errors


def compute_rpe(P_gt, P_est, delta=1):
    """
    Compute Relative Pose Error (RPE)

    RPE measures the local consistency of the trajectory by comparing
    relative transformations over a fixed time interval delta.

    Args:
        P_gt: Nx3 ground truth positions
        P_est: Nx3 estimated positions (same scale as ground truth)
        delta: Frame interval for relative pose computation

    Returns:
        rpe_trans_rmse: Translational RPE RMSE (meters)
        rpe_trans_mean: Mean translational RPE (meters)
        rpe_trans_median: Median translational RPE (meters)
        rpe_trans_std: Standard deviation (meters)
        trans_errors: Array of translational errors
    """
    n = len(P_gt)
    trans_errors = []

    for i in range(n - delta):
        # Ground truth relative motion
        gt_rel = P_gt[i + delta] - P_gt[i]

        # Estimated relative motion
        est_rel = P_est[i + delta] - P_est[i]

        # Compute difference
        error = np.linalg.norm(gt_rel - est_rel)
        trans_errors.append(error)

    trans_errors = np.array(trans_errors)

    rpe_trans_rmse = np.sqrt(np.mean(trans_errors ** 2))
    rpe_trans_mean = np.mean(trans_errors)
    rpe_trans_median = np.median(trans_errors)
    rpe_trans_std = np.std(trans_errors)

    return rpe_trans_rmse, rpe_trans_mean, rpe_trans_median, rpe_trans_std, trans_errors


def compute_scale_error(P_gt, P_est_unaligned):
    """
    Compute Scale Error

    For monocular VO, scale is unobservable and drifts over time.
    This metric computes the ratio between estimated and ground truth scales.

    Args:
        P_gt: Nx3 ground truth positions
        P_est_unaligned: Nx3 estimated positions (before alignment)

    Returns:
        scale_error: Scale ratio (estimated/ground truth)
        scale_drift_percent: Scale drift percentage
    """
    # Compute total path lengths
    gt_segments = np.diff(P_gt, axis=0)
    est_segments = np.diff(P_est_unaligned, axis=0)

    gt_length = np.sum(np.linalg.norm(gt_segments, axis=1))
    est_length = np.sum(np.linalg.norm(est_segments, axis=1))

    # Scale ratio
    scale_ratio = est_length / gt_length

    # Scale drift (percentage difference from 1.0)
    scale_drift_percent = abs(1.0 - scale_ratio) * 100.0

    return scale_ratio, scale_drift_percent


def plot_results(P_gt, P_est_unaligned, P_aligned, ate_errors, output_dir='.'):
    """
    Generate visualization plots
    """
    import os
    os.makedirs(output_dir, exist_ok=True)

    # Plot 1: 2D Trajectory Comparison (Top view: X-Y)
    plt.figure(figsize=(12, 10))

    plt.subplot(2, 2, 1)
    plt.plot(P_gt[:, 0], P_gt[:, 1], 'g-', linewidth=2, label='Ground Truth (RTK)', alpha=0.7)
    plt.plot(P_est_unaligned[:, 0], P_est_unaligned[:, 1], 'r--', linewidth=1.5,
             label='VO (Unaligned)', alpha=0.7)
    plt.xlabel('X (East) [m]', fontsize=12)
    plt.ylabel('Y (North) [m]', fontsize=12)
    plt.title('2D Trajectory - Before Alignment', fontsize=14, fontweight='bold')
    plt.legend(fontsize=10)
    plt.grid(True, alpha=0.3)
    plt.axis('equal')

    plt.subplot(2, 2, 2)
    plt.plot(P_gt[:, 0], P_gt[:, 1], 'g-', linewidth=2, label='Ground Truth (RTK)', alpha=0.7)
    plt.plot(P_aligned[:, 0], P_aligned[:, 1], 'b-', linewidth=1.5,
             label='VO (Aligned)', alpha=0.7)
    plt.xlabel('X (East) [m]', fontsize=12)
    plt.ylabel('Y (North) [m]', fontsize=12)
    plt.title('2D Trajectory - After Sim(3) Alignment', fontsize=14, fontweight='bold')
    plt.legend(fontsize=10)
    plt.grid(True, alpha=0.3)
    plt.axis('equal')

    # Plot 2: ATE Error Distribution
    plt.subplot(2, 2, 3)
    plt.hist(ate_errors, bins=50, color='steelblue', alpha=0.7, edgecolor='black')
    plt.xlabel('ATE Error [m]', fontsize=12)
    plt.ylabel('Frequency', fontsize=12)
    plt.title('Absolute Trajectory Error Distribution', fontsize=14, fontweight='bold')
    plt.axvline(np.mean(ate_errors), color='r', linestyle='--', linewidth=2, label=f'Mean: {np.mean(ate_errors):.3f}m')
    plt.axvline(np.median(ate_errors), color='orange', linestyle='--', linewidth=2, label=f'Median: {np.median(ate_errors):.3f}m')
    plt.legend(fontsize=10)
    plt.grid(True, alpha=0.3)

    # Plot 3: Error over trajectory
    plt.subplot(2, 2, 4)
    indices = np.arange(len(ate_errors))
    plt.plot(indices, ate_errors, 'b-', linewidth=1, alpha=0.7)
    plt.fill_between(indices, 0, ate_errors, alpha=0.3)
    plt.xlabel('Trajectory Point Index', fontsize=12)
    plt.ylabel('ATE Error [m]', fontsize=12)
    plt.title('ATE Error Along Trajectory', fontsize=14, fontweight='bold')
    plt.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(f'{output_dir}/trajectory_evaluation.png', dpi=300, bbox_inches='tight')
    print(f"\n✓ Plot saved: {output_dir}/trajectory_evaluation.png")

    plt.close()


def main():
    parser = argparse.ArgumentParser(
        description='Evaluate Visual Odometry accuracy using 3 academic metrics: ATE, RPE, Scale Error'
    )
    parser.add_argument('--groundtruth', required=True, help='Ground truth trajectory (TUM format)')
    parser.add_argument('--estimated', required=True, help='Estimated trajectory (TUM format)')
    parser.add_argument('--max-time-diff', type=float, default=0.02,
                       help='Maximum time difference for association (default: 0.02s)')
    parser.add_argument('--rpe-delta', type=int, default=10,
                       help='Frame interval for RPE computation (default: 10)')
    parser.add_argument('--output-dir', default='evaluation_results',
                       help='Output directory for plots (default: evaluation_results)')
    parser.add_argument('--save-aligned', action='store_true',
                       help='Save aligned trajectory to file')

    args = parser.parse_args()

    print("="*80)
    print("VISUAL ODOMETRY TRAJECTORY EVALUATION")
    print("="*80)
    print(f"Ground Truth: {args.groundtruth}")
    print(f"Estimated:    {args.estimated}")
    print("="*80)

    # Load trajectories
    print("\n[1/6] Loading trajectories...")
    t_gt, P_gt, Q_gt = load_trajectory_tum(args.groundtruth)
    t_est, P_est, Q_est = load_trajectory_tum(args.estimated)
    print(f"  Ground truth: {len(P_gt)} poses")
    print(f"  Estimated:    {len(P_est)} poses")

    # Associate trajectories
    print(f"\n[2/6] Associating trajectories (max time diff: {args.max_time_diff}s)...")
    matches = associate_trajectories(t_gt, t_est, args.max_time_diff)
    print(f"  Matched {len(matches)} / {len(P_est)} poses ({100*len(matches)/len(P_est):.1f}%)")

    if len(matches) < 10:
        print("\n✗ ERROR: Too few matched poses for evaluation!")
        print("  Try increasing --max-time-diff or check timestamp alignment")
        return 1

    # Extract matched poses
    gt_indices = [m[0] for m in matches]
    est_indices = [m[1] for m in matches]
    P_gt_matched = P_gt[gt_indices]
    P_est_matched = P_est[est_indices]
    P_est_unaligned = P_est_matched.copy()

    # Align trajectories using Sim(3)
    print("\n[3/6] Aligning trajectories using Sim(3) (7-DOF)...")
    P_aligned, scale, R, t = align_trajectories_sim3(P_gt_matched, P_est_matched)
    print(f"  Estimated scale: {scale:.4f}")
    print(f"  Translation: [{t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f}] m")

    # Compute Metric 1: ATE (Absolute Trajectory Error)
    print("\n[4/6] Computing ATE (Absolute Trajectory Error)...")
    ate_rmse, ate_mean, ate_median, ate_std, ate_min, ate_max, ate_errors = compute_ate(P_gt_matched, P_aligned)

    # Compute Metric 2: RPE (Relative Pose Error)
    print(f"\n[5/6] Computing RPE (Relative Pose Error, delta={args.rpe_delta})...")
    # Use aligned trajectory for RPE
    rpe_rmse, rpe_mean, rpe_median, rpe_std, rpe_errors = compute_rpe(P_gt_matched, P_aligned, args.rpe_delta)

    # Compute Metric 3: Scale Error
    print("\n[6/6] Computing Scale Error...")
    scale_ratio, scale_drift = compute_scale_error(P_gt_matched, P_est_unaligned)

    # Print results
    print("\n" + "="*80)
    print("EVALUATION RESULTS")
    print("="*80)

    print("\n┌─ METRIC 1: ATE (Absolute Trajectory Error)")
    print("│  Measures overall trajectory accuracy after optimal alignment")
    print("├────────────────────────────────────────")
    print(f"│  RMSE:   {ate_rmse:.4f} m  ← Primary metric")
    print(f"│  Mean:   {ate_mean:.4f} m")
    print(f"│  Median: {ate_median:.4f} m")
    print(f"│  Std:    {ate_std:.4f} m")
    print(f"│  Min:    {ate_min:.4f} m")
    print(f"│  Max:    {ate_max:.4f} m")
    print("└────────────────────────────────────────")

    print("\n┌─ METRIC 2: RPE (Relative Pose Error)")
    print(f"│  Measures local consistency over {args.rpe_delta} frames")
    print("├────────────────────────────────────────")
    print(f"│  Trans RMSE:   {rpe_rmse:.4f} m  ← Primary metric")
    print(f"│  Trans Mean:   {rpe_mean:.4f} m")
    print(f"│  Trans Median: {rpe_median:.4f} m")
    print(f"│  Trans Std:    {rpe_std:.4f} m")
    print("└────────────────────────────────────────")

    print("\n┌─ METRIC 3: Scale Error")
    print("│  Measures scale accuracy (critical for monocular VO)")
    print("├────────────────────────────────────────")
    print(f"│  Scale Ratio:  {scale_ratio:.4f}  ← Primary metric")
    print(f"│  Scale Drift:  {scale_drift:.2f}%")
    print(f"│  (1.0 = perfect scale, >1.0 = overestimation, <1.0 = underestimation)")
    print("└────────────────────────────────────────")

    print("\n" + "="*80)
    print("SUMMARY FOR ACADEMIC REPORTING")
    print("="*80)
    print(f"ATE RMSE:        {ate_rmse:.4f} m")
    print(f"RPE Trans RMSE:  {rpe_rmse:.4f} m")
    print(f"Scale Error:     {abs(1-scale_ratio)*100:.2f}%")
    print("="*80)

    # Generate plots
    print("\nGenerating visualization plots...")
    plot_results(P_gt_matched, P_est_unaligned, P_aligned, ate_errors, args.output_dir)

    # Save aligned trajectory
    if args.save_aligned:
        aligned_file = f'{args.output_dir}/aligned_trajectory.txt'
        t_matched = t_est[est_indices]
        Q_matched = Q_est[est_indices]
        with open(aligned_file, 'w') as f:
            f.write("# Aligned estimated trajectory (TUM format)\n")
            f.write("# timestamp x y z qx qy qz qw\n")
            for i in range(len(P_aligned)):
                f.write(f"{t_matched[i]:.6f} {P_aligned[i,0]:.6f} {P_aligned[i,1]:.6f} {P_aligned[i,2]:.6f} ")
                f.write(f"{Q_matched[i,0]:.6f} {Q_matched[i,1]:.6f} {Q_matched[i,2]:.6f} {Q_matched[i,3]:.6f}\n")
        print(f"✓ Aligned trajectory saved: {aligned_file}")

    # Save numerical results
    results_file = f'{args.output_dir}/evaluation_results.txt'
    with open(results_file, 'w') as f:
        f.write("="*80 + "\n")
        f.write("VISUAL ODOMETRY EVALUATION RESULTS\n")
        f.write("="*80 + "\n\n")
        f.write(f"Ground Truth: {args.groundtruth}\n")
        f.write(f"Estimated:    {args.estimated}\n")
        f.write(f"Matched Poses: {len(matches)} / {len(P_est)}\n\n")

        f.write("METRIC 1: ATE (Absolute Trajectory Error)\n")
        f.write("-" * 40 + "\n")
        f.write(f"RMSE:   {ate_rmse:.6f} m\n")
        f.write(f"Mean:   {ate_mean:.6f} m\n")
        f.write(f"Median: {ate_median:.6f} m\n")
        f.write(f"Std:    {ate_std:.6f} m\n")
        f.write(f"Min:    {ate_min:.6f} m\n")
        f.write(f"Max:    {ate_max:.6f} m\n\n")

        f.write("METRIC 2: RPE (Relative Pose Error)\n")
        f.write("-" * 40 + "\n")
        f.write(f"Trans RMSE:   {rpe_rmse:.6f} m\n")
        f.write(f"Trans Mean:   {rpe_mean:.6f} m\n")
        f.write(f"Trans Median: {rpe_median:.6f} m\n")
        f.write(f"Trans Std:    {rpe_std:.6f} m\n\n")

        f.write("METRIC 3: Scale Error\n")
        f.write("-" * 40 + "\n")
        f.write(f"Scale Ratio: {scale_ratio:.6f}\n")
        f.write(f"Scale Drift: {scale_drift:.2f}%\n\n")

        f.write("="*80 + "\n")
        f.write("SUMMARY (For Academic Reporting)\n")
        f.write("="*80 + "\n")
        f.write(f"ATE RMSE:        {ate_rmse:.4f} m\n")
        f.write(f"RPE Trans RMSE:  {rpe_rmse:.4f} m\n")
        f.write(f"Scale Error:     {abs(1-scale_ratio)*100:.2f}%\n")

    print(f"✓ Results saved: {results_file}")

    print("\n" + "="*80)
    print("✓ EVALUATION COMPLETED SUCCESSFULLY!")
    print("="*80)

    return 0


if __name__ == "__main__":
    sys.exit(main())
