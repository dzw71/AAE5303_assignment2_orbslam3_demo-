# AAE5303 Assignment: Visual Odometry with ORB-SLAM3



**Monocular Visual Odometry Evaluation on UAV Aerial Imagery**

*Hong Kong Island GNSS Dataset - MARS-LVIG*



---

## 📋 Table of Contents

1. [Executive Summary](#-executive-summary)
2. [Introduction](#-introduction)
3. [Methodology](#-methodology)
4. [Dataset Description](#-dataset-description)
5. [Implementation Details](#-implementation-details)
6. [Results and Analysis](#-results-and-analysis)
7. [Visualizations](#-visualizations)
8. [Discussion](#-discussion)
9. [Conclusions](#-conclusions)
10. [References](#-references)
11. [Appendix](#-appendix)

---

## 📊 Executive Summary

This project evaluates **monocular visual odometry (VO)** using **ORB-SLAM3** on the **HKisland_GNSS03** UAV aerial imagery dataset. The estimated trajectory is compared with RTK ground truth in TUM format.

Compared with the example outcome shown in the demo repository, my implementation achieved significantly better trajectory accuracy and scale consistency. I report both:

1. the **optimized full-trajectory result**, and  
2. the **best continuous segment result**, which excludes major discontinuities caused by tracking loss, map switching, or relocalization.


### Key Results

| Metric | Value | Description |
|--------|-------|-------------|
| **ATE RMSE** | **2.3969 m** | Global trajectory accuracy after Sim(3) alignment |
| **RPE Trans RMSE** | **1.2277 m** | Local translation consistency |
| **Scale Error** | **8.25%** | Monocular scale drift |
| **Completeness** | **99.10%** | Matched poses / estimated poses (656 / 662) |
| **Estimated poses** | **662** | Trajectory poses in `KeyFrameTrajectory.txt` |

#### Best Continuous Segment
| Metric | Value | Description |
| --- | --- | --- |
| **ATE RMSE** | **1.4314 m** | Best stable segment global accuracy |
| **RPE Trans RMSE** | **0.7111 m** | Best stable segment local consistency |
| **Scale Error** | **7.36%** | Best stable segment scale drift |
| **Completeness** | **98.19%** | Matched poses / segment poses (325 / 331) |
| **Segment range** | **[331, 662)** | Best continuous segment in the trajectory |


---

## 📖 Introduction

### Background

ORB-SLAM3 is a state-of-the-art visual SLAM system capable of performing:

- Monocular visual odometry
- Stereo visual odometry
- RGB-D SLAM
- Visual-inertial odometry
- Multi-map SLAM with relocalization


This assignment focuses on **Monocular VO mode**, which:

- uses only camera images for pose estimation,
- cannot directly observe absolute scale,
- relies on ORB feature extraction and matching,
- may suffer from long-term drift and tracking resets.


### Objectives

The objectives of this assignment are:

1. Run monocular ORB-SLAM3 on the HKisland_GNSS03 UAV dataset.
2. Extract RTK ground truth trajectory.
3. Evaluate the estimated trajectory using standard metrics.
4. Improve the trajectory quality through parameter tuning and result analysis.


---

## 🔬 Methodology

### ORB-SLAM3 Visual Odometry Overview

ORB-SLAM3 performs visual odometry through the following pipeline:

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│  Input Image    │────▶│   ORB Feature   │────▶│   Feature       │
│  Sequence       │     │   Extraction    │     │   Matching      │
└─────────────────┘     └─────────────────┘     └────────┬────────┘
                                                         │
┌─────────────────┐     ┌─────────────────┐     ┌────────▼────────┐
│   Trajectory    │◀────│   Pose          │◀────│   Motion        │
│   Output        │     │   Estimation    │     │   Model         │
└─────────────────┘     └────────┬────────┘     └─────────────────┘
                                 │
                        ┌────────▼────────┐
                        │   Local Map     │
                        │   Optimization  │
                        └─────────────────┘
```

### Evaluation Metrics

#### 1. ATE (Absolute Trajectory Error)

Measures the RMSE of the aligned trajectory after Sim(3) alignment:

$$ATE_{RMSE} = \sqrt{\frac{1}{N}\sum_{i=1}^{N}\|\mathbf{p}_{est}^i - \mathbf{p}_{gt}^i\|^2}$$
It is used as the main global accuracy metric.

**Reference**: Sturm et al., "A Benchmark for the Evaluation of RGB-D SLAM Systems", IROS 2012

#### 2. RPE (Relative Pose Error) – Drift Rates

Measures local consistency by comparing relative transformations:

$$RPE_{trans} = \|\Delta\mathbf{p}_{est} - \Delta\mathbf{p}_{gt}\|$$

where $\Delta\mathbf{p} = \mathbf{p}(t+\Delta) - \mathbf{p}(t)$

RPE measures local trajectory consistency over a fixed interval.  
In this assignment I report **RPE translation RMSE**, which reflects short-term local motion accuracy.


**Reference**: Geiger et al., "Vision meets Robotics: The KITTI Dataset", IJRR 2013

We report drift as **rates** that are easier to interpret and compare across methods:

- **Translation drift rate** (m/m): \( \text{RPE}_{trans,mean} / \Delta d \)
- **Rotation drift rate** (deg/100m): \( (\text{RPE}_{rot,mean} / \Delta d) \times 100 \)

where \(\Delta d\) is a distance interval in meters (e.g., 10 m).

#### 3. Completeness

Completeness measures how many ground-truth poses can be associated and evaluated:

$$Completeness = \frac{N_{matched}}{N_{gt}} \times 100\%$$

### Why Best Continuous Segment Evaluation?

The full trajectory may contain large discontinuities caused by:

- tracking failure,
- local map failure,
- relocalization,
- multi-map switching.

To better reflect the actual quality of stable tracking, I additionally split the trajectory at large discontinuities and evaluated the **best continuous segment**. This provides a more meaningful measure of real stable VO performance.


### Trajectory Alignment

To evaluate monocular visual odometry fairly, the estimated trajectory is aligned to RTK ground truth using **Sim(3) alignment (7-DOF)**.

This alignment includes:

- **3-DOF Translation**: aligns the trajectory origin
- **3-DOF Rotation**: aligns the trajectory orientation
- **1-DOF Scale**: compensates for monocular scale ambiguity

This is necessary because monocular ORB-SLAM3 cannot directly recover absolute metric scale.

### Evaluation Protocol

This section describes the exact evaluation protocol used in this project.

#### Inputs

- **Ground truth**: `data/rtk_groundtruth.txt`  
  TUM format: `timestamp x y z qx qy qz qw`
- **Estimated trajectory**: `KeyFrameTrajectory.txt`  
  generated by ORB-SLAM3 monocular mode
- **Association threshold**: `max_time_diff = 0.1 s`
  - The extracted image sequence is approximately **10 Hz**
  - The RTK ground truth is approximately **5 Hz**
  - A threshold of **0.1 s** allows most poses to be associated successfully
- **RPE interval**: `delta = 10`
  - In the current evaluation script, RPE is computed over a fixed frame interval of 10 poses

#### Step 1 — Full-Trajectory Evaluation

The full estimated trajectory is evaluated using the project script:

```bash
python3 data/evaluate_vo_accuracy.py \
    --groundtruth data/rtk_groundtruth.txt \
    --estimated KeyFrameTrajectory.txt \
    --output-dir evaluation_results \
    --max-time-diff 0.1 \
    --save-aligned

This script reports:

ATE RMSE
RPE Trans RMSE
Scale Error
matched pose count
Step 2 — Best Continuous Segment Evaluation
Because the full trajectory may contain large discontinuities caused by tracking loss or map switching, I additionally evaluate the best continuous segment:
python3 data/evaluate_best_segment.py \
    --groundtruth data/rtk_groundtruth.txt \
    --estimated /tmp/orbslam3_run_07/KeyFrameTrajectory.txt \
    --output-dir /tmp/orbslam3_run_07/best_segment_eval \
    --max-time-diff 0.1

This script splits the estimated trajectory at large jumps and evaluates each continuous segment separately.
The best segment is selected based on:

low Scale Error
low ATE RMSE
low RPE Trans RMSE
high matched pose count
Step 3 — Completeness
In this project, completeness is computed as:

Completeness (%) = matched_poses / estimated_poses * 100
where matched_poses is the number of estimated poses successfully associated with RTK ground truth under max_time_diff = 0.1 s.

Practical Notes
Trajectory file used:
This project mainly evaluates KeyFrameTrajectory.txt
Since this file contains only keyframes, the reported pose count is lower than frame-level trajectory output
Timestamp format:
Both ground truth and estimated trajectories must use timestamps in seconds
Incorrect timestamps would cause association failure
Association threshold selection:
0.1 s is appropriate for this dataset because image timestamps are near 10 Hz while RTK timestamps are near 5 Hz
A smaller threshold such as 0.02 s would significantly reduce completeness
Why best-segment evaluation is needed:
ORB-SLAM3 monocular VO may create multiple maps or lose tracking during difficult parts of the sequence
Evaluating the best continuous segment better reflects the system’s stable tracking performance

---

## 📁 Dataset Description

### HKisland_GNSS03 Dataset

The dataset is from the **MARS-LVIG** UAV dataset, captured over Hong Kong Island.

| Property | Value |
| --- | --- |
| **Dataset Name** | HKisland_GNSS03 |
| **Source** | MARS-LVIG / UAV aerial imagery |
| **Duration** | 390.78 s |
| **Extracted Images** | 3911 |
| **Image Resolution** | 2448 × 2048 |
| **Frame Rate** | 10 Hz |
| **Ground Truth Type** | RTK trajectory |
| **Ground Truth Poses** | 1955 |


The dataset contains aerial images captured over Hong Kong Island together with RTK positioning data.  
RTK measurements are converted into local ENU coordinates and stored in TUM trajectory format for evaluation.


### Data Sources

| Resource | Link |
|----------|------|
| MARS-LVIG Dataset | https://mars.hku.hk/dataset.html |
| UAVScenes GitHub | https://github.com/sijieaaa/UAVScenes |
| ORB-SLAM3 | https://github.com/UZ-SLAMLab/ORB_SLAM3 |

This assignment uses the **HKisland_GNSS03** sequence from the MARS-LVIG UAV dataset. The image sequence is extracted from the ROS bag file and processed by ORB-SLAM3 in **monocular visual odometry** mode. RTK data is extracted separately and converted into TUM trajectory format for evaluation.

### Ground Truth

Ground truth is generated from RTK (Real-Time Kinematic) GPS measurements. The original RTK latitude, longitude, and altitude data are converted from **WGS84** coordinates into a local **ENU (East-North-Up)** coordinate frame, and then saved in TUM trajectory format for evaluation.

| Property | Value |
|----------|-------|
| **Ground Truth File** | `data/rtk_groundtruth.txt` |
| **RTK Positions** | 1,955 poses |
| **Rate** | 5 Hz |
| **Coordinate System** | WGS84 → Local ENU |
| **Format** | TUM trajectory format |
| **Reference Origin** | First RTK position in the sequence |

The estimated trajectory produced by ORB-SLAM3 is evaluated against this RTK-based ground truth using ATE, RPE, Scale Error, and Completeness.|

---

## ⚙️ Implementation Details

### System Configuration

| Component | Value |
| --- | --- |
| Framework | ORB-SLAM3 |
| Mode | Monocular Visual Odometry |
| Vocabulary | `ORBvoc.txt` |
| Input | Extracted image sequence |
| Ground Truth | RTK trajectory in TUM format |


### Camera Calibration

```yaml
Camera.type: "PinHole"
Camera.fx: 1444.43
Camera.fy: 1444.34
Camera.cx: 1179.50
Camera.cy: 1044.90

Camera.k1: -0.0560
Camera.k2: 0.1180
Camera.p1: 0.00122
Camera.p2: 0.00064
Camera.k3: -0.0627

Camera.width: 2448
Camera.height: 2048
Camera.fps: 10.0
Camera.RGB: 1

```

# AAE5303 Assignment: Visual Odometry with ORB-SLAM3

**Monocular Visual Odometry Evaluation on UAV Aerial Imagery**  
*Hong Kong Island GNSS Dataset - MARS-LVIG*

---

## Table of Contents

- [Executive Summary](#executive-summary)
- [Introduction](#introduction)
- [Methodology](#methodology)
- [Dataset Description](#dataset-description)
- [Implementation Details](#implementation-details)
- [Results and Analysis](#results-and-analysis)
- [Visualizations](#visualizations)
- [Discussion](#discussion)
- [Conclusions](#conclusions)
- [References](#references)
- [Appendix](#appendix)

---

## Executive Summary

This project evaluates **monocular visual odometry (VO)** using **ORB-SLAM3** on the **HKisland_GNSS03** UAV aerial imagery dataset. The estimated trajectory is compared against RTK ground truth in TUM format.

Compared with the demo repository outcome, my implementation achieved significantly better trajectory accuracy and scale consistency. I report both:

1. the **optimized full-trajectory result**
2. the **best continuous segment result**, which excludes major discontinuities caused by tracking loss, map switching, or relocalization

### Key Results

#### Optimized Full Trajectory

| Metric | Value | Description |
| --- | --- | --- |
| **ATE RMSE** | **2.3969 m** | Global trajectory accuracy after Sim(3) alignment |
| **RPE Trans RMSE** | **1.2277 m** | Local translation consistency |
| **Scale Error** | **8.25%** | Monocular scale drift |
| **Completeness** | **99.10%** | Matched poses / estimated poses (656 / 662) |
| **Estimated poses** | **662** | Trajectory poses in `KeyFrameTrajectory.txt` |

#### Best Continuous Segment

| Metric | Value | Description |
| --- | --- | --- |
| **ATE RMSE** | **1.4314 m** | Best stable segment global accuracy |
| **RPE Trans RMSE** | **0.7111 m** | Best stable segment local consistency |
| **Scale Error** | **7.36%** | Best stable segment scale drift |
| **Completeness** | **98.19%** | Matched poses / segment poses (325 / 331) |
| **Segment range** | **[331, 662)** | Best continuous segment in the trajectory |

---

## Introduction

### Background

ORB-SLAM3 is a state-of-the-art visual SLAM framework capable of performing:

- Monocular visual odometry
- Stereo visual odometry
- RGB-D SLAM
- Visual-inertial odometry
- Multi-map SLAM with relocalization

This assignment focuses on **monocular VO mode**, which:

- uses only camera images for pose estimation
- cannot directly observe absolute scale
- relies on ORB feature extraction and matching
- may suffer from long-term drift and tracking resets

### Objectives

The objectives of this assignment are:

1. Run monocular ORB-SLAM3 on the HKisland_GNSS03 UAV dataset
2. Extract RTK ground truth trajectory
3. Evaluate the estimated trajectory using standard metrics
4. Improve the trajectory quality through parameter tuning and result analysis

---

## Methodology

### ORB-SLAM3 Monocular VO Pipeline

The ORB-SLAM3 monocular VO workflow can be summarized as:

1. image input
2. ORB feature extraction
3. feature matching
4. pose estimation
5. local map optimization
6. trajectory output

### Evaluation Metrics

#### Absolute Trajectory Error (ATE)

ATE measures global trajectory accuracy after Sim(3) alignment:

$$
ATE_{RMSE} = \sqrt{\frac{1}{N}\sum_{i=1}^{N}\|p^i_{est} - p^i_{gt}\|^2}
$$

This is the primary global accuracy metric.

#### Relative Pose Error (RPE)

RPE measures local trajectory consistency over a fixed interval.  
In this assignment I report **RPE translation RMSE**, which reflects short-term local motion accuracy.

#### Scale Error

Monocular VO cannot directly estimate absolute metric scale.  
Therefore, scale error is critical for evaluating whether the estimated path length is close to the ground truth path length.

#### Completeness

Completeness measures how much of the trajectory can be successfully associated and evaluated:

$$
Completeness = \frac{N_{matched}}{N_{estimated}} \times 100\%
$$

### Trajectory Alignment

To evaluate monocular visual odometry fairly, the estimated trajectory is aligned to RTK ground truth using **Sim(3) alignment (7-DOF)**.

This alignment includes:

- **3-DOF Translation**: aligns trajectory origin
- **3-DOF Rotation**: aligns trajectory orientation
- **1-DOF Scale**: compensates for monocular scale ambiguity

This is necessary because monocular ORB-SLAM3 cannot directly recover absolute metric scale.

### Evaluation Protocol

The exact evaluation protocol used in this project is:

#### Inputs

- **Ground truth**: `data/rtk_groundtruth.txt`  
  TUM format: `timestamp x y z qx qy qz qw`
- **Estimated trajectory**: `KeyFrameTrajectory.txt`
- **Association threshold**: `max_time_diff = 0.1 s`
  - extracted images are approximately **10 Hz**
  - RTK ground truth is approximately **5 Hz**
  - `0.1 s` allows most poses to be associated successfully
- **RPE interval**: `delta = 10`
  - in the current evaluation script, RPE is computed over a fixed frame interval of 10 poses

#### Full-Trajectory Evaluation

```bash
python3 data/evaluate_vo_accuracy.py \
    --groundtruth data/rtk_groundtruth.txt \
    --estimated KeyFrameTrajectory.txt \
    --output-dir evaluation_results \
    --max-time-diff 0.1 \
    --save-aligned

This script reports:

ATE RMSE
RPE Trans RMSE
Scale Error
matched pose count
Best Continuous Segment Evaluation
Because the full trajectory may contain large discontinuities caused by tracking loss or map switching, I additionally evaluate the best continuous segment:
python3 data/evaluate_best_segment.py \
    --groundtruth data/rtk_groundtruth.txt \
    --estimated /tmp/orbslam3_run_07/KeyFrameTrajectory.txt \
    --output-dir /tmp/orbslam3_run_07/best_segment_eval \
    --max-time-diff 0.1

This script splits the estimated trajectory at large jumps and evaluates each continuous segment separately. The best segment is selected based on:

low Scale Error
low ATE RMSE
low RPE Trans RMSE
high matched pose count
Practical Notes
KeyFrameTrajectory.txt contains only keyframes, so pose count is lower than dense frame-level output
both ground truth and estimated trajectories must use timestamps in seconds
a smaller threshold such as 0.02 s significantly reduces completeness
best-segment evaluation better reflects stable tracking performance when the full trajectory contains resets or discontinuities
Dataset Description
HKisland_GNSS03 Dataset
Property	Value
Dataset Name	HKisland_GNSS03
Source	MARS-LVIG / UAV aerial imagery
Duration	390.78 s
Extracted Images	3911
Image Resolution	2448 × 2048
Frame Rate	10 Hz
Ground Truth Type	RTK trajectory
Ground Truth Poses	1955
The dataset contains aerial images captured over Hong Kong Island together with RTK positioning data. RTK measurements are converted into local ENU coordinates and stored in TUM trajectory format for evaluation.

Data Sources
Resource	Link
MARS-LVIG Dataset	https://mars.hku.hk/dataset.html
UAVScenes GitHub	https://github.com/sijieaaa/UAVScenes
ORB-SLAM3	https://github.com/UZ-SLAMLab/ORB_SLAM3
This assignment uses the HKisland_GNSS03 sequence from the MARS-LVIG UAV dataset. The image sequence is extracted from the ROS bag file and processed by ORB-SLAM3 in monocular visual odometry mode. RTK data is extracted separately and converted into TUM trajectory format for evaluation.

Ground Truth
Ground truth is generated from RTK GPS measurements. The original RTK latitude, longitude, and altitude data are converted from WGS84 coordinates into a local ENU (East-North-Up) coordinate frame, and then saved in TUM trajectory format for evaluation.

Property	Value
Ground Truth File	data/rtk_groundtruth.txt
RTK Positions	1,955 poses
Rate	5 Hz
Coordinate System	WGS84 → Local ENU
Format	TUM trajectory format
Reference Origin	First RTK position in the sequence
The estimated trajectory produced by ORB-SLAM3 is evaluated against this RTK-based ground truth using ATE, RPE, Scale Error, and Completeness.

Implementation Details
System Configuration
Component	Value
Framework	ORB-SLAM3
Mode	Monocular Visual Odometry
Vocabulary	ORBvoc.txt
Input	Extracted image sequence
Ground Truth	RTK trajectory in TUM format
Camera Calibration

Camera.type: "PinHole"
Camera.fx: 1444.43
Camera.fy: 1444.34
Camera.cx: 1179.50
Camera.cy: 1044.90

Camera.k1: -0.0560
Camera.k2: 0.1180
Camera.p1: 0.00122
Camera.p2: 0.00064
Camera.k3: -0.0627

Camera.width: 2448
Camera.height: 2048
Camera.fps: 10.0
Camera.RGB: 1

ORB-SLAM3 Settings
This project uses the ORB-SLAM3 monocular YAML configuration format under Examples/Monocular/DJI_Camera.yaml.
The final tuned configuration was directly edited in this YAML file and used for all optimized runs.

Final ORB Feature Extraction Parameters
Parameter	Value	Description
nFeatures	2600	Features extracted per frame
scaleFactor	1.15	Pyramid scale factor
nLevels	10	Pyramid levels
iniThFAST	18	Initial FAST threshold
minThFAST	7	Minimum FAST threshold
Main Running Command
./Examples/Monocular/mono_tum \
    Vocabulary/ORBvoc.txt \
    Examples/Monocular/DJI_Camera.yaml \
    data/extracted_data

Results and Analysis
Baseline Result
Before optimization, the initial result was:

Metric	Value
ATE RMSE	2.5335 m
RPE Trans RMSE	1.7309 m
Scale Error	8.38%
This baseline already produced reasonable scale consistency, but local tracking accuracy could be improved.

Optimized Full-Trajectory Result
After parameter tuning, the optimized full-trajectory result became:

Metric	Value
ATE RMSE	2.3969 m
RPE Trans RMSE	1.2277 m
Scale Error	8.25%
Completeness	99.10%
Matched / Estimated	656 / 662
This shows that optimization improved both global and local trajectory quality while preserving good scale consistency.

Best Continuous Segment Result
Because the full trajectory still contains some discontinuities, I further evaluated the best continuous segment:

Metric	Value
ATE RMSE	1.4314 m
RPE Trans RMSE	0.7111 m
Scale Error	7.36%
Completeness	98.19%
Matched / Segment Poses	325 / 331
Segment Range	[331, 662)
This result demonstrates that the stable tracking phase of ORB-SLAM3 is significantly better than the whole-trajectory average.

Full-Trajectory Evaluation Output
================================================================================
VISUAL ODOMETRY EVALUATION RESULTS
================================================================================

Ground Truth: data/rtk_groundtruth.txt
Estimated:    KeyFrameTrajectory.txt
Matched Poses: 656 / 662

METRIC 1: ATE (Absolute Trajectory Error)
----------------------------------------
RMSE:   2.3969 m

METRIC 2: RPE (Relative Pose Error)
----------------------------------------
Trans RMSE:   1.2277 m

METRIC 3: Scale Error
----------------------------------------
Scale Ratio: 0.9175
Scale Drift: 8.25%

================================================================================
SUMMARY
================================================================================
ATE RMSE:        2.3969 m
RPE Trans RMSE:  1.2277 m
Scale Error:     8.25%
================================================================================

Best Continuous Segment Output
BEST CONTINUOUS SEGMENT EVALUATION
================================================================================

BEST SEGMENT
--------------------------------------------------------------------------------
Index range:    [331, 662)
Segment poses:  331
Matched poses:  325 / 331
ATE RMSE:       1.4314 m
RPE Trans RMSE: 0.7111 m
Scale Ratio:    0.9264
Scale Error:    7.36%
Completeness:   98.19%

Trajectory Alignment Statistics
Optimized Full Trajectory
Parameter	Value
Association threshold	t_max_diff = 0.1 s
Matched / Estimated	656 / 662
Completeness	99.10%
Scale Ratio	0.9175
Scale Error	8.25%
Best Continuous Segment
Parameter	Value
Segment range	[331, 662)
Matched / Segment Poses	325 / 331
Completeness	98.19%
Scale Ratio	0.9264
Scale Error	7.36%
Performance Analysis
Optimized Full Trajectory
Metric	Value	Interpretation
ATE RMSE	2.3969 m	Good global trajectory accuracy
RPE Trans RMSE	1.2277 m	Good local consistency
Scale Error	8.25%	Good monocular scale stability
Completeness	99.10%	Very high evaluation coverage
Best Continuous Segment
Metric	Value	Interpretation
ATE RMSE	1.4314 m	Very good global accuracy in stable tracking
RPE Trans RMSE	0.7111 m	Strong local consistency
Scale Error	7.36%	Stable scale estimation
Completeness	98.19%	High usable coverage in the best segment
Compared with the demo repository result, my implementation is substantially better in both trajectory accuracy and scale consistency. See zwding7-cell/AAE5303_assignment2_orbslam3_demo-.

Visualizations
Trajectory Comparison
Trajectory Evaluation
<img width="3571" height="2970" alt="image" src="https://github.com/user-attachments/assets/7beb84b4-e062-4827-bcae-ae2659f907cf" />

This figure is generated from the optimized trajectory evaluation and includes:

Top-Left: 2D trajectory before alignment
Top-Right: 2D trajectory after Sim(3) alignment
Bottom-Left: Distribution of ATE errors
Bottom-Right: ATE error along the trajectory
The figure shows that the optimized trajectory generally follows the RTK ground-truth trend after alignment, although some discontinuities remain in the full sequence. This is why the best continuous segment was also evaluated separately.

Discussion
Strengths
High completeness: the optimized full trajectory achieved 99.10% completeness
Good scale consistency: the optimized result maintained a low scale drift of 8.25%
Stable best segment: the best continuous segment achieved ATE = 1.4314 m and RPE = 0.7111 m
Strong improvement over demo outcome: my result is significantly better than the example metrics shown in the demo repository
Limitations
Trajectory discontinuities remain: the full trajectory still contains jumps caused by tracking instability or map switching
Monocular drift still exists: although scale is controlled well, long-term monocular VO still accumulates error
Keyframe-only trajectory: evaluation is based on KeyFrameTrajectory.txt, not dense frame-level output
Error Sources
Fast UAV motion can introduce motion blur and large inter-frame displacement
Aerial scenes often have repeated or weak texture, which makes matching harder
Monocular VO has inherent scale ambiguity, even after tuning
Tracking resets / multi-map behavior can create discontinuities in the full trajectory
Conclusions
This assignment demonstrates monocular visual odometry using ORB-SLAM3 on UAV aerial imagery from the HKisland_GNSS03 dataset.

Key findings are:

ORB-SLAM3 successfully processed the aerial image sequence and generated a valid monocular VO trajectory
The optimized full trajectory achieved:
ATE RMSE = 2.3969 m
RPE Trans RMSE = 1.2277 m
Scale Error = 8.25%
Completeness = 99.10%
The best continuous segment achieved:
ATE RMSE = 1.4314 m
RPE Trans RMSE = 0.7111 m
Scale Error = 7.36%
Completeness = 98.19%
The final optimized result is substantially better than the poor example outcome shown in the demo repository
These results indicate that the tuned ORB-SLAM3 system performs reliably on stable sections of the UAV trajectory and achieves good full-trajectory performance overall.

References
Campos, C., Elvira, R., Rodríguez, J. J. G., Montiel, J. M. M., & Tardós, J. D. (2021). ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM. IEEE Transactions on Robotics, 37(6), 1874-1890.
Sturm, J., Engelhard, N., Endres, F., Burgard, W., & Cremers, D. (2012). A Benchmark for the Evaluation of RGB-D SLAM Systems. IROS 2012.
Geiger, A., Lenz, P., & Urtasun, R. (2012). Vision meets Robotics: The KITTI Dataset. CVPR 2012.
MARS-LVIG Dataset: https://mars.hku.hk/dataset.html
ORB-SLAM3 GitHub: https://github.com/UZ-SLAMLab/ORB_SLAM3
Demo reference repository: zwding7-cell/AAE5303_assignment2_orbslam3_demo-
Appendix
Repository Structure
ORB_SLAM3/
├── Examples/
├── Vocabulary/
├── data/
│   ├── extracted_data/
│   ├── rtk_groundtruth.txt
│   ├── evaluate_vo_accuracy.py
│   └── evaluate_best_segment.py
├── evaluation_results/
└── KeyFrameTrajectory.txt

Running Commands
# 1. Run ORB-SLAM3 monocular VO
./Examples/Monocular/mono_tum \
    Vocabulary/ORBvoc.txt \
    Examples/Monocular/DJI_Camera.yaml \
    data/extracted_data

# 2. Evaluate full trajectory
python3 data/evaluate_vo_accuracy.py \
    --groundtruth data/rtk_groundtruth.txt \
    --estimated KeyFrameTrajectory.txt \
    --output-dir evaluation_results \
    --max-time-diff 0.1 \
    --save-aligned

# 3. Evaluate best continuous segment
python3 data/evaluate_best_segment.py \
    --groundtruth data/rtk_groundtruth.txt \
    --estimated /tmp/orbslam3_run_07/KeyFrameTrajectory.txt \
    --output-dir /tmp/orbslam3_run_07/best_segment_eval \
    --max-time-diff 0.1

Output Trajectory Format (TUM)
# timestamp x y z qx qy qz qw
1698132964.499888 0.0000000 0.0000000 0.0000000 -0.0000000 -0.0000000 -0.0000000 1.0000000
1698132964.599976 -0.0198950 0.0163751 -0.0965251 -0.0048082 0.0122335 0.0013237 0.9999127
...



**AAE5303 - Robust Control Technology in Low-Altitude Aerial Vehicle**

*Department of Aeronautical and Aviation Engineering*

*The Hong Kong Polytechnic University*

Jan 2026

</div>

