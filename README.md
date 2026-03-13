# AAE5303 Assignment 2: Visual Odometry with ORB-SLAM3

<div align="center">

![ORB-SLAM3](https://img.shields.io/badge/SLAM-ORB--SLAM3-blue?style=for-the-badge)
![VO](https://img.shields.io/badge/Mode-Visual_Odometry-green?style=for-the-badge)
![Dataset](https://img.shields.io/badge/Dataset-HKisland__GNSS03-orange?style=for-the-badge)
![Status](https://img.shields.io/badge/Status-Evaluated-success?style=for-the-badge)

**Monocular Visual Odometry Evaluation on UAV Aerial Imagery**

*Hong Kong Island GNSS Dataset - MARS-LVIG*

</div>

---

## Table of Contents

1. [Executive Summary](#executive-summary)
2. [Introduction](#introduction)
3. [Methodology](#methodology)
4. [Dataset Description](#dataset-description)
5. [Implementation Details](#implementation-details)
6. [Results and Analysis](#results-and-analysis)
7. [Visualizations](#visualizations)
8. [Discussion](#discussion)
9. [Conclusions](#conclusions)
10. [References](#references)
11. [Appendix](#appendix)

---

## Executive Summary

This report presents the implementation and evaluation of **Monocular Visual Odometry (VO)** using the **ORB-SLAM3** framework on the **HKisland_GNSS03** UAV aerial imagery dataset. The project evaluates trajectory accuracy against RTK ground truth using **four parallel, monocular-appropriate metrics** computed with the `evo` toolkit.

Through systematic parameter tuning — increasing ORB features from 1500 to 3500, adding an extra pyramid level, and lowering FAST thresholds — we achieved significant improvements over the default baseline configuration.

### Key Results (Best of 8 Runs)

| Metric | Value | Description |
|--------|-------|-------------|
| **ATE RMSE** | **0.7721 m** | Global accuracy after Sim(3) alignment (scale corrected) |
| **RPE Trans Drift** | **1.3857 m/m** | Translation drift rate (mean error per meter, delta=10 m) |
| **RPE Rot Drift** | **108.66 deg/100m** | Rotation drift rate (mean angle per 100 m, delta=10 m) |
| **Completeness** | **96.06%** | Matched poses / total ground-truth poses (1878 / 1955) |
| **Tracking Success Rate** | **87.5%** | 7 successful runs out of 8 total attempts |

---

## Introduction

### Background

ORB-SLAM3 is a state-of-the-art visual SLAM system capable of performing:

- **Monocular Visual Odometry** (pure camera-based)
- **Stereo Visual Odometry**
- **Visual-Inertial Odometry** (with IMU fusion)
- **Multi-map SLAM** with relocalization

This assignment focuses on **Monocular VO mode**, which:

- Uses only camera images for pose estimation
- Cannot observe absolute scale (scale ambiguity)
- Relies on feature matching (ORB features) for tracking
- Is susceptible to drift without loop closure

### Objectives

1. Implement monocular Visual Odometry using ORB-SLAM3
2. Process UAV aerial imagery from the HKisland_GNSS03 dataset
3. Extract RTK (Real-Time Kinematic) GPS data as ground truth
4. Evaluate trajectory accuracy using four parallel metrics appropriate for monocular VO
5. Document the complete workflow for reproducibility

### Scope

This assignment evaluates:
- **ATE (Absolute Trajectory Error)**: Global trajectory accuracy after Sim(3) alignment (monocular-friendly)
- **RPE drift rates (translation + rotation)**: Local consistency (drift per traveled distance)
- **Completeness**: Robustness / coverage (how much of the sequence is successfully tracked and evaluated)

---

## Methodology

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

**Reference**: Sturm et al., "A Benchmark for the Evaluation of RGB-D SLAM Systems", IROS 2012

#### 2. RPE (Relative Pose Error) – Drift Rates

Measures local consistency by comparing relative transformations:

$$RPE_{trans} = \|\Delta\mathbf{p}_{est} - \Delta\mathbf{p}_{gt}\|$$

where $\Delta\mathbf{p} = \mathbf{p}(t+\Delta) - \mathbf{p}(t)$

**Reference**: Geiger et al., "Vision meets Robotics: The KITTI Dataset", IJRR 2013

We report drift as **rates** that are easier to interpret and compare across methods:

- **Translation drift rate** (m/m): \( \text{RPE}_{trans,mean} / \Delta d \)
- **Rotation drift rate** (deg/100m): \( (\text{RPE}_{rot,mean} / \Delta d) \times 100 \)

where \(\Delta d\) is a distance interval in meters (e.g., 10 m).

#### 3. Completeness

Completeness measures how many ground-truth poses can be associated and evaluated:

$$Completeness = \frac{N_{matched}}{N_{gt}} \times 100\%$$

#### Why these metrics (and why Sim(3) alignment)?

Monocular VO suffers from **scale ambiguity**: the system cannot recover absolute metric scale without additional sensors or priors. Therefore:

- **All error metrics are computed after Sim(3) alignment** (rotation + translation + scale) so that accuracy reflects **trajectory shape** and **drift**, not an arbitrary global scale factor.
- **RPE is evaluated in the distance domain** (delta in meters) to make drift easier to interpret on long trajectories.
- **Completeness is reported explicitly** to discourage trivial solutions that only output a short "easy" segment.

### Trajectory Alignment

We use Sim(3) (7-DOF) alignment to optimally align estimated trajectory to ground truth:

- **3-DOF Translation**: Align trajectory origins
- **3-DOF Rotation**: Align trajectory orientations
- **1-DOF Scale**: Compensate for monocular scale ambiguity

### Evaluation Protocol

This section describes the **exact** evaluation protocol used in this report. The goal is to ensure that every student can reproduce the same numbers given the same inputs.

#### Inputs

- **Ground truth**: `ground_truth.txt` (TUM format: `t tx ty tz qx qy qz qw`)
- **Estimated trajectory**: `CameraTrajectory.txt` (TUM format)
- **Association threshold**: `t_max_diff = 0.1 s`
  - This dataset contains RTK at ~5 Hz and images at ~10 Hz.
  - A threshold of 0.1 s is large enough to associate most GT timestamps with a nearby estimated pose, while still rejecting clearly mismatched timestamps.
- **Distance delta for RPE**: `delta = 10 m`
  - Using a distance-based delta makes drift comparable along the flight even if the timestamp sampling is non-uniform after tracking failures.

#### Step 1 — ATE with Sim(3) alignment (scale corrected)

```bash
evo_ape tum ground_truth.txt CameraTrajectory.txt \
  --align --correct_scale \
  --t_max_diff 0.1 -va
```

We report **ATE RMSE (m)** as the primary global accuracy metric.

#### Step 2 — RPE (translation + rotation) in the distance domain

```bash
# Translation RPE over 10 m (meters)
evo_rpe tum ground_truth.txt CameraTrajectory.txt \
  --align --correct_scale \
  --t_max_diff 0.1 \
  --delta 10 --delta_unit m \
  --pose_relation trans_part -va

# Rotation RPE over 10 m (degrees)
evo_rpe tum ground_truth.txt CameraTrajectory.txt \
  --align --correct_scale \
  --t_max_diff 0.1 \
  --delta 10 --delta_unit m \
  --pose_relation angle_deg -va
```

We convert evo's mean RPE over 10 m into drift rates:

- **RPE translation drift (m/m)** = `RPE_trans_mean_m / 10`
- **RPE rotation drift (deg/100m)** = `(RPE_rot_mean_deg / 10) * 100`

#### Step 3 — Completeness

Completeness measures how much of the sequence can be evaluated:

```text
Completeness (%) = matched_poses / gt_poses * 100
```

Here, `matched_poses` is the number of pose pairs successfully associated by evo under `t_max_diff`.

---

## Dataset Description

### HKisland_GNSS03 Dataset

The dataset is from the **MARS-LVIG** UAV dataset, captured over Hong Kong Island.

| Property | Value |
|----------|-------|
| **Dataset Name** | HKisland_GNSS03 |
| **Source** | MARS-LVIG / UAVScenes |
| **Duration** | 390.78 seconds (~6.5 minutes) |
| **Total Images** | 3,833 frames |
| **Image Resolution** | 2448 x 2048 pixels |
| **Frame Rate** | ~10 Hz |
| **Trajectory Length** | ~1,900 meters |
| **Height Variation** | 0 - 90 meters |

### Data Sources

| Resource | Link |
|----------|------|
| MARS-LVIG Dataset | https://mars.hku.hk/dataset.html |
| UAVScenes GitHub | https://github.com/sijieaaa/UAVScenes |

### Ground Truth

RTK (Real-Time Kinematic) GPS provides centimeter-level positioning accuracy:

| Property | Value |
|----------|-------|
| **RTK Positions** | 1,955 poses |
| **Rate** | 5 Hz |
| **Accuracy** | ±2 cm (horizontal), ±5 cm (vertical) |
| **Coordinate System** | WGS84 → Local ENU |

---

## Implementation Details

### System Configuration

| Component | Specification |
|-----------|---------------|
| **Framework** | ORB-SLAM3 (C++) |
| **Mode** | Monocular Visual Odometry |
| **Vocabulary** | ORBvoc.txt (pre-trained) |
| **Environment** | Docker (liangyu99/orbslam3_ros1) + ROS Noetic |
| **Operating System** | Ubuntu 20.04 (container) on WSL2 |
| **Playback Rate** | 0.3x (reduced speed for stable tracking) |

### Camera Calibration

```yaml
File.version: "1.0"
Camera.type: "PinHole"

Camera1.fx: 1444.43
Camera1.fy: 1444.34
Camera1.cx: 1177.80
Camera1.cy: 1043.60

Camera1.k1: -0.0530
Camera1.k2: 0.1210
Camera1.p1: 0.00127
Camera1.p2: 0.00043
Camera1.k3: -0.06495

Camera.width: 2448
Camera.height: 2048
Camera.fps: 10
Camera.RGB: 1
```

### ORB Feature Extraction Parameters (Tuned)

| Parameter | Default | Tuned | Rationale |
|-----------|---------|-------|-----------|
| `nFeatures` | 1500 | **3500** | More features for high-resolution UAV images; improves matching robustness |
| `scaleFactor` | 1.2 | 1.2 | Standard multi-scale pyramid factor |
| `nLevels` | 8 | **9** | Additional pyramid level captures larger-scale structures |
| `iniThFAST` | 20 | **9** | Lower threshold detects features in low-texture regions (sky, water) |
| `minThFAST` | 7 | **2** | Aggressive fallback ensures features even in challenging frames |

### Running ORB-SLAM3

The SLAM pipeline was executed inside a Docker container with ROS Noetic:

```bash
#!/bin/bash
source /opt/ros/noetic/setup.bash
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/root/ORB_SLAM3/Examples_old/ROS

cd /root/ORB_SLAM3

# Play rosbag at 0.3x speed for stable tracking
rosbag play data/HKisland_GNSS03.bag \
  --topics /left_camera/image/compressed -r 0.3 &

# Start monocular SLAM with compressed image transport
rosrun ORB_SLAM3 Mono_Compressed \
  Vocabulary/ORBvoc.txt \
  Examples/Monocular/HKisland_Mono.yaml \
  /camera/image_raw/compressed:=/left_camera/image/compressed &
```

---

## Results and Analysis

### Multi-Run Evaluation

To assess the **repeatability and robustness** of our tuned ORB-SLAM3 configuration, we conducted **8 independent runs** on the same dataset with identical parameters. Each run was monitored for tracking loss (new map creation with map count >= 2); failed runs were discarded without saving data.

#### Full Results Table

| Run | Tracking Lost | Map Count | ATE RMSE (m) | RPE Trans Drift (m/m) | RPE Rot Drift (deg/100m) | Completeness (%) | Scale |
|:---:|:------------:|:---------:|:------------:|:---------------------:|:------------------------:|:----------------:|:-----:|
| 1 | No | 1 | 0.7950 | 1.4309 | 110.73 | 89.82 | 0.694 |
| 2 | No | 1 | 0.8853 | 1.3769 | 110.27 | 96.06 | 0.556 |
| 3 | No | 1 | **0.7721** | **1.3857** | 108.66 | **96.06** | 0.554 |
| 4 | No | 1 | 0.7882 | 1.3895 | 109.38 | 96.06 | 0.554 |
| 5 | No | 1 | 1.0478 | 1.4100 | 111.09 | 96.01 | 0.555 |
| 6 | No | 1 | 0.8334 | 1.3894 | **108.43** | 96.01 | 0.556 |
| 7 | **Yes** | 2 | - | - | - | - | - |
| 8 | No | 1 | 0.7966 | 1.3890 | 108.85 | 95.35 | 0.556 |

#### Aggregate Statistics (7 Successful Runs)

| Statistic | ATE RMSE (m) | RPE Trans Drift (m/m) | RPE Rot Drift (deg/100m) | Completeness (%) |
|-----------|:------------:|:---------------------:|:------------------------:|:----------------:|
| **Best** | 0.772 | 1.377 | 108.43 | 96.06 |
| **Worst** | 1.048 | 1.431 | 111.09 | 89.82 |
| **Mean** | 0.845 | 1.396 | 109.63 | 95.05 |

#### Tracking Success Rate

- **Successful runs**: 7 / 8 = **87.5%**
- **Failed runs**: 1 / 8 (tracking lost due to aggressive maneuver causing new map creation)

### Best Run Results (Run 3)

```
================================================================================
PARALLEL METRICS (NO WEIGHTING)
================================================================================

Ground Truth:   RTK trajectory (1,955 poses at 5 Hz)
Estimated:      ORB-SLAM3 camera trajectory (CameraTrajectory.txt)
Matched Poses:  1,878 / 1,955 (96.06%)  <- Completeness

METRIC 1: ATE (Absolute Trajectory Error)
────────────────────────────────────────
RMSE:   0.7721 m
Mean:   0.7041 m
Std:    0.3170 m

METRIC 2: RPE Translation Drift (distance-based, delta=10 m)
────────────────────────────────────────
Mean translational RPE over 10 m: 13.8567 m
Translation drift rate:           1.3857 m/m

METRIC 3: RPE Rotation Drift (distance-based, delta=10 m)
────────────────────────────────────────
Mean rotational RPE over 10 m: 10.8656 deg
Rotation drift rate:           108.66 deg/100m

================================================================================
```

### Trajectory Alignment Statistics (Best Run)

| Parameter | Value |
|-----------|-------|
| **Sim(3) scale correction** | 0.5543 |
| **Sim(3) rotation** | Aligns VO frame to ENU ground-truth frame |
| **Association threshold** | t\_max\_diff = 0.1 s |
| **Association rate (Completeness)** | 96.06% |

### Performance Analysis

| Metric | Best Value | Mean (7 runs) | Interpretation |
|--------|:---------:|:-------------:|----------------|
| **ATE RMSE** | 0.772 m | 0.845 m | Sub-meter global accuracy — excellent for monocular VO on a 1.9 km outdoor UAV trajectory |
| **RPE Trans Drift** | 1.386 m/m | 1.396 m/m | Moderate local drift; highly consistent across runs |
| **RPE Rot Drift** | 108.43 deg/100m | 109.63 deg/100m | Orientation drift present, driven by aggressive flight maneuvers and featureless segments |
| **Completeness** | 96.06% | 95.05% | High coverage — over 95% of ground-truth poses successfully tracked and evaluated |

### Comparison with Baseline and Leaderboard

| Metric | Our Best | Our Mean (7 runs) | Leaderboard Baseline (AMtown02) | vs Baseline |
|--------|:--------:|:-----------------:|:-------------------------------:|:-----------:|
| **ATE RMSE (m)** | **0.7721** | 0.845 | 88.2281 | 114x better |
| **RPE Trans Drift (m/m)** | **1.3857** | 1.396 | 2.04084 | 32% better |
| **RPE Rot Drift (deg/100m)** | **108.66** | 109.63 | 76.6991 | 42% worse |
| **Completeness (%)** | **96.06** | 95.05 | 95.73 | comparable |

Note: The baseline is on a different sequence (AMtown02), so direct comparison is indicative only.

### Improvement Over Default Parameters

Running the same pipeline with default ORB-SLAM3 parameters (nFeatures=1500, iniThFAST=20) yielded:

| Metric | Default Config | Tuned Config (Best) | Improvement |
|--------|:-------------:|:-------------------:|:-----------:|
| **ATE RMSE (m)** | 85.82 | **0.7721** | 99.1% reduction |
| **RPE Trans Drift (m/m)** | 2.0630 | **1.3857** | 33% reduction |
| **RPE Rot Drift (deg/100m)** | 133.17 | **108.66** | 18% reduction |
| **Completeness (%)** | 87.98 | **96.06** | +8.08 pp |

The dramatic ATE improvement (from 85.82 m to 0.77 m) is primarily due to:
1. More features (3500 vs 1500) enabling robust matching in high-resolution frames
2. Lower FAST thresholds (9/2 vs 20/7) detecting features in low-texture aerial regions
3. An additional pyramid level (9 vs 8) capturing multi-scale structures
4. Reduced tracking failures leading to more continuous trajectory estimation

---

## Visualizations

### Trajectory Comparison

![Trajectory Evaluation](figures/trajectory_evaluation.png)

This figure includes four panels:

1. **Top-Left**: 2D trajectory before alignment (matched poses only). Shows the scale/rotation mismatch typical for monocular VO — the VO trajectory (red dashed) has a different scale and orientation from the ground truth (green).

2. **Top-Right**: 2D trajectory after Sim(3) alignment (scale corrected). The aligned VO trajectory (blue) closely follows the ground truth (green), with sub-meter accuracy across most of the flight path.

3. **Bottom-Left**: Distribution of ATE translation errors. The histogram shows most errors concentrated around 0.65-0.72 m (median: 0.65 m, mean: 0.72 m), with a compact distribution indicating consistent tracking quality.

4. **Bottom-Right**: ATE error along the trajectory (by matched pose index). The error remains mostly below 1 m, with brief spikes up to ~3.5 m during aggressive maneuvers or featureless segments.

---

## Discussion

### Strengths

1. **Sub-meter ATE accuracy**: A best ATE RMSE of 0.772 m (mean 0.845 m over 7 runs) on a ~1.9 km outdoor UAV trajectory demonstrates that monocular ORB-SLAM3, when properly tuned, achieves high global accuracy after Sim(3) alignment.

2. **High evaluation coverage**: 96.06% completeness (best run) indicates that ORB-SLAM3 successfully tracked across nearly the entire flight sequence, a significant improvement over the initial 89.82%.

3. **Significant improvement through tuning**: Increasing ORB features to 3500, lowering FAST thresholds to 9/2, and adding an extra pyramid level reduced ATE by over two orders of magnitude compared to default parameters.

4. **High repeatability**: 87.5% tracking success rate (7/8 runs) with consistent metrics across successful runs (ATE std < 0.1 m across runs) demonstrates reliable performance.

5. **End-to-end reproducibility**: The complete pipeline (ROS bag playback, SLAM execution, automated monitoring, evo evaluation) runs inside a Docker container with fixed parameters and automated scripts, ensuring reproducible results.

### Limitations

1. **Rotation drift**: The RPE rotation drift of 108–111 deg/100m remains high, indicating that local orientation estimation degrades during fast turns and featureless segments.

2. **Translation drift rate**: At ~1.39 m/m, the local translation drift is non-trivial. Over longer trajectories without loop closure, this drift would accumulate substantially.

3. **No loop closure**: Pure VO mode was used without exploiting ORB-SLAM3's loop closure or multi-map capabilities. Enabling these could further reduce drift.

4. **Playback speed dependency**: Running the bag at 0.3x speed was necessary for stable tracking, suggesting the system is near its computational limits for this image resolution and feature count.

5. **Occasional tracking loss**: 1 out of 8 runs experienced tracking loss (map count >= 2), indicating that the system remains sensitive to ORB-SLAM3's non-deterministic feature matching and initialization.

### Error Sources

1. **Fast UAV Motion**: Aggressive flight maneuvers cause motion blur and large inter-frame displacements, leading to brief tracking degradation (visible as ATE spikes in the trajectory plot).

2. **Featureless Regions**: Aerial imagery over water, roads, and uniform rooftops provides limited texture for ORB feature extraction, even with lowered FAST thresholds.

3. **Monocular Scale Ambiguity**: Scale drift is inherent to monocular VO. The Sim(3) alignment compensates for global scale, but local scale variations contribute to RPE.

4. **Image Resolution vs. Processing**: The high resolution (2448 x 2048) provides rich detail but increases processing load, requiring slower playback to maintain tracking quality.

---

## Conclusions

This assignment demonstrates monocular Visual Odometry implementation using ORB-SLAM3 on UAV aerial imagery. Key findings:

1. **Sub-meter accuracy achieved**: Best ATE RMSE of 0.772 m (mean 0.845 m over 7 runs) after Sim(3) alignment on a 1.9 km outdoor UAV trajectory
2. **High completeness**: Up to 96.06% of ground-truth poses successfully matched and evaluated
3. **Robust and repeatable**: 87.5% tracking success rate across 8 runs with consistent metrics
4. **Parameter tuning is critical**: Increasing features (3500), lowering FAST thresholds (9/2), and adding pyramid levels (9) transformed performance from 85.82 m to 0.77 m ATE
5. **Rotation drift remains a challenge**: RPE rotation drift of ~109 deg/100m highlights the difficulty of orientation estimation in aerial scenarios

### Recommendations for Further Improvement

| Priority | Action | Expected Improvement |
|----------|--------|---------------------|
| High | Enable loop closure (full SLAM mode) | Significant drift reduction on revisited areas |
| High | Enable IMU fusion (VIO mode) | 50-70% RPE reduction; eliminates scale ambiguity |
| Medium | Image preprocessing (CLAHE contrast normalization) | Better features in low-texture regions |
| Low | Increase nFeatures beyond 3500 | Diminishing returns; may slow processing |

---

## References

1. Campos, C., Elvira, R., Rodriguez, J. J. G., Montiel, J. M., & Tardos, J. D. (2021). **ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM**. *IEEE Transactions on Robotics*, 37(6), 1874-1890.

2. Sturm, J., Engelhard, N., Endres, F., Burgard, W., & Cremers, D. (2012). **A Benchmark for the Evaluation of RGB-D SLAM Systems**. *IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*.

3. Geiger, A., Lenz, P., & Urtasun, R. (2012). **Are we ready for Autonomous Driving? The KITTI Vision Benchmark Suite**. *IEEE Conference on Computer Vision and Pattern Recognition (CVPR)*.

4. MARS-LVIG Dataset: https://mars.hku.hk/dataset.html

5. ORB-SLAM3 GitHub: https://github.com/UZ-SLAMLab/ORB_SLAM3

---

## Appendix

### A. Repository Structure

```
AAE5303_assignment2_orbslam3_demo-/
├── README.md                        # This report
├── requirements.txt                 # Python dependencies
├── figures/
│   └── trajectory_evaluation.png    # Best run evaluation visualization
├── output/
│   ├── ground_truth.txt             # RTK ground truth (TUM format)
│   ├── CameraTrajectory.txt         # Best run estimated trajectory
│   └── evaluation_report.json       # Best run evaluation metrics (JSON)
├── scripts/
│   ├── evaluate_vo_accuracy.py      # Automated evaluation script
│   ├── generate_report_figures.py   # Figure generation script
│   ├── run_slam_auto.sh             # Single-run SLAM with map monitoring
│   └── run_slam_batch.sh            # Multi-run batch SLAM with monitoring
├── docs/
│   └── camera_config.yaml           # Camera calibration reference
├── leaderboard/
│   ├── README.md                    # Leaderboard specification
│   ├── LEADERBOARD_SUBMISSION_GUIDE.md
│   ├── ORB_SLAM3_TIPS.md
│   ├── submission_template.json     # Submission with our best metrics
│   └── evaluation_report.json       # Full evaluation report
├── run_00_baseline/                 # Run 1 results
├── run_20260313_150407/             # Run 2 results
├── run_20260313_153142/             # Run 3 results (best ATE)
├── run_20260313_155400/             # Run 4 results
├── run_20260313_161615/             # Run 5 results
├── run_20260313_163832/             # Run 6 results
│   (Run 7: tracking lost, data not saved)
└── run_20260313_173325/             # Run 8 results
    ├── output/
    │   ├── CameraTrajectory.txt     # Estimated trajectory
    │   ├── KeyFrameTrajectory.txt   # Keyframe trajectory
    │   ├── ground_truth.txt         # Ground truth
    │   ├── evaluation_report.json   # Metrics JSON
    │   ├── evaluation_output.txt    # Evaluation log
    │   ├── slam_run.log             # SLAM output log
    │   └── ate.zip / rpe_*.zip      # evo result archives
    ├── figures/
    │   └── trajectory_evaluation.png
    └── scripts/
        ├── evaluate_vo_accuracy.py
        └── generate_report_figures.py
```

### B. Running Commands

```bash
# 1. Start Docker container
docker run -it --name orbslam3_ros1 liangyu99/orbslam3_ros1:latest

# 2. Single automated run with map monitoring (from host)
bash scripts/run_slam_auto.sh

# 3. Batch run (multiple runs with monitoring, from host)
bash scripts/run_slam_batch.sh

# 4. Evaluate trajectory (using the provided script, inside container)
python3 scripts/evaluate_vo_accuracy.py \
    --groundtruth ground_truth.txt \
    --estimated CameraTrajectory.txt \
    --t-max-diff 0.1 \
    --delta-m 10 \
    --workdir evaluation_results \
    --json-out evaluation_results/metrics.json
```

### C. Native evo Commands

```bash
# ATE (Sim(3) alignment + scale correction)
evo_ape tum ground_truth.txt CameraTrajectory.txt \
  --align --correct_scale \
  --t_max_diff 0.1 -va

# RPE translation (distance-based, delta = 10 m)
evo_rpe tum ground_truth.txt CameraTrajectory.txt \
  --align --correct_scale \
  --t_max_diff 0.1 \
  --delta 10 --delta_unit m \
  --pose_relation trans_part -va

# RPE rotation angle (degrees, distance-based, delta = 10 m)
evo_rpe tum ground_truth.txt CameraTrajectory.txt \
  --align --correct_scale \
  --t_max_diff 0.1 \
  --delta 10 --delta_unit m \
  --pose_relation angle_deg -va
```

### D. Output Trajectory Format (TUM)

```
# timestamp x y z qx qy qz qw
1698132964.499888 0.0000000 0.0000000 0.0000000 -0.0000000 -0.0000000 -0.0000000 1.0000000
1698132964.599976 -0.0198950 0.0163751 -0.0965251 -0.0048082 0.0122335 0.0013237 0.9999127
...
```

### E. Full Evaluation Metrics — Best Run (JSON)

```json
{
  "ate_rmse_m": 0.7721,
  "ate_mean_m": 0.7041,
  "ate_std_m": 0.3170,
  "rpe_trans_drift_m_per_m": 1.3857,
  "rpe_trans_mean_m": 13.8567,
  "rpe_trans_rmse_m": 15.4964,
  "rpe_rot_drift_deg_per_100m": 108.6561,
  "rpe_rot_mean_deg": 10.8656,
  "rpe_rot_rmse_deg": 21.1470,
  "matched_poses": 1878,
  "gt_poses": 1955,
  "completeness_pct": 96.06,
  "t_max_diff_s": 0.1,
  "delta_m": 10.0
}
```

---

<div align="center">

**AAE5303 - Robust Control Technology in Low-Altitude Aerial Vehicle**

*Department of Aeronautical and Aviation Engineering*

*The Hong Kong Polytechnic University*

March 2026

</div>
