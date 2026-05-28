# Q-CIP Sensor Geometry Optimization

Python/MATLAB research project for stereo 3D reconstruction, sensor-geometry optimization, Bayesian optimization, point-cloud evaluation, and SAR-relevant minimum-look sensing logic.

This project began as an SGBM + WLS stereo reconstruction pipeline and evolved into a controlled geometry-and-decision testbed for studying how viewing geometry affects 3D dimensional estimation of a known target.

---

## Project Summary

Q-CIP uses camera-based stereo reconstruction to study a broader sensing question:

**Which view geometry produces the most reliable mission-relevant measurement?**

The project uses stereo image generation, SGBM disparity estimation, WLS-style refinement, point-cloud reconstruction, ground-truth evaluation, Bayesian optimization, and synthetic tanker geometry studies to compare one-view, two-view, and multi-view sensing strategies.

The project does **not** claim that optical stereo and SAR use the same image-formation model. Instead, it uses optical stereo as a controlled experimental framework for studying geometry selection, objective design, uncertainty thresholding, and decision logic relevant to future SAR-style sensing studies.

---

## Why It Matters

Many 3D reconstruction systems are judged by visual quality or generic reconstruction metrics. In real sensing missions, the important question is often narrower:

* Is the target height reliable?
* Is the length or width estimate stable?
* Is the current look sufficient?
* Would a second look improve the weak dimension?
* Does adding more views improve the mission metric, or does it create fusion artifacts?

Q-CIP shows that reconstruction quality, coverage, dimensional accuracy, and uncertainty thresholds can disagree. The best geometry for mesh accuracy is not necessarily the best geometry for target dimensions.

This makes sensor geometry a mission-design variable, not just a visualization choice.

---

## Methods and Pipeline

The Q-CIP workflow includes:

1. **Synthetic target generation**
   Generate a known oil-tanker-style 3D target for controlled ground-truth comparison.

2. **Stereo rendering**
   Render left/right stereo image pairs under configurable camera geometry.

3. **SGBM disparity estimation**
   Use OpenCV-style Semi-Global Block Matching logic to estimate disparity from rectified stereo views.

4. **WLS-style refinement**
   Apply edge-aware disparity refinement to reduce noise while preserving depth discontinuities.

5. **Point-cloud reconstruction**
   Convert disparity/depth output into 3D point clouds.

6. **Mesh and dimensional evaluation**
   Evaluate reconstructed geometry using mesh RMSE, centroid RMSE, GT coverage, height error, length error, width error, and mean dimensional error.

7. **Bayesian optimization**
   Search large geometry and parameter spaces using repeated-seed Bayesian optimization.

8. **Uncertainty-aware thresholding**
   Use bootstrap-style height resampling to estimate conservative clearance thresholds.

9. **Corrective second-view analysis**
   Test whether a second view improves a known weakness from a first-view reconstruction.

10. **Multi-view saturation study**
    Evaluate whether adding more views improves dimensional reliability or causes over-fusion.

---

## Major Experiments

### Phase A — Two-View Mesh Accuracy

Phase A studied opposite-side two-view geometries and showed that reconstruction quality depends strongly on azimuth, elevation, and baseline.

Key finding:

**Geometry has structure.** Favorable view families emerged, but the best surface accuracy and best coverage were not always the same.

---

### Phase B — One-View Dimensional Accuracy

Phase B shifted the objective from mesh quality to direct dimensional estimation.

The study compared height, length, width, and balanced mean-dimensional error.

Key finding:

**Dimension-specific optima diverge.**
The best view for height was not the best view for length, width, or balanced mean error.

---

### Phase C — Corrective Second View

Phase C treated the second view as a targeted corrective action rather than an automatic improvement.

Key finding:

**A second view helps when it corrects a specific weakness.**
The best second view depends on the first-view error profile and the mission metric.

---

### Four-View Saturation Study

The four-view saturation study tested whether adding more views automatically improves dimensional reliability.

Key finding:

**More views can improve coverage while damaging dimensional accuracy.**
The four-view fused case increased surface coverage but produced major width inflation, showing that completeness does not guarantee reliable measurement.

---

## Key Results

* Built a camera-based stereo 3D reconstruction testbed using SGBM + WLS-style disparity refinement.
* Developed synthetic tanker geometry studies with controllable azimuth, elevation, baseline, HFOV, and range.
* Applied Bayesian optimization to large geometry and reconstruction search spaces.
* Demonstrated that mesh quality, coverage, dimensional accuracy, and uncertainty thresholds can favor different designs.
* Showed that one-view dimensional optima differ for height, length, width, and balanced mean error.
* Demonstrated that a corrective second look can improve the weak dimension when selected strategically.
* Found that four-view fusion increased coverage but caused dimensional collapse through boundary thickening and inter-view disagreement.
* Reframed the project toward SAR-relevant minimum-look decision logic and mission-specific sensing metrics.

---

## Repository Structure

```text
QCIP-Sensor-Geometry-Optimization/
│
├── README.md
├── src/
│   ├── README.md
│   └── MATLAB / Python source files
│
├── figures/
│   ├── README.md
│   └── representative Q-CIP figures
│
├── docs/
│   └── optional project notes
│
└── reports/
    └── optional sanitized report materials
```

---

## Source Code

The `src/` folder contains the core source code for the Q-CIP study.

Representative scripts include workflows for:

* synthetic tanker generation
* stereo rendering
* point-cloud reconstruction
* ground-truth metric computation
* Bayesian optimization
* one-view dimensional sweeps
* corrective second-view studies
* voxel-fusion experiments
* four-view saturation testing
* metrics reporting

This codebase is research-oriented and organized around experimental studies rather than a single production software package.

---

## Representative Figures

The `figures/` folder contains public-facing visual summaries, including:

* Q-CIP SGBM + WLS pipeline diagram
* Phase B one-view dimensional heat map
* Phase C corrective second-view heat maps
* voxel-fusion local grid result
* metric divergence diagram

These figures summarize the central lesson of the project:

**The best sensing geometry depends on the mission metric.**

---

## How to Run

This repository contains research scripts rather than a single unified application.

Typical MATLAB usage:

```matlab
run_phaseC1_one_view_dimensional_sweep
run_phaseC1_one_view_dimensional_bayesopt_v2
run_phaseC2_support_view_bayesopt_anchor_10_8_balanced
run_four_view_capstone
```

Some scripts may require local path adjustments depending on where the target model, image outputs, point clouds, or result folders are stored.

Large generated outputs, full seed logs, and heavy point-cloud files may be excluded from the public repository to keep the project lightweight and shareable.

---

## Tools and Skills Demonstrated

* MATLAB
* Python
* OpenCV / SGBM stereo reconstruction
* WLS-style disparity refinement
* 3D point-cloud reconstruction
* Mesh and dimensional evaluation
* Ground-truth benchmarking
* Bayesian optimization
* Multi-objective metric design
* Sensor-geometry analysis
* Bootstrap-style uncertainty estimation
* SAR-relevant sensing logic
* Technical reporting and research communication

---

## SAR-Relevant Framing

Q-CIP is not a SAR image-formation model. The camera-based stereo pipeline is used as a controlled testbed.

The transferable contribution is the decision framework:

1. Select a sensing geometry.
2. Evaluate the mission-specific metric.
3. Quantify uncertainty.
4. Decide whether another look is worth the sensing cost.

Future SAR-oriented extensions would replace camera-specific metrics with radar-appropriate measures such as image sharpness, signal-to-clutter ratio, coherent phase quality, speckle behavior, target-feature persistence, aspect-angle robustness, and uncertainty in mission-specific estimates.

---

## Public Sharing Note

This repository contains a sanitized public-facing version of the Q-CIP sensor-geometry optimization work.

It is intended to demonstrate the mathematical modeling, reconstruction workflow, optimization framework, and sensing-decision logic behind the project.

Protected, proprietary, restricted, NDA-covered, or non-public materials are excluded.

---

## Author

Dang H. Ngo
M.S., Computational Applied Mathematics
California State University, Fullerton
