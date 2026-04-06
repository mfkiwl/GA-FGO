# GA-FGO: Geometry-Adaptive NLOS Mitigation for Urban GNSS Positioning via Sliding-Window Factor Graph Optimization

This repository provides a minimal public MATLAB release of the core algorithmic components of our geometry-adaptive sliding-window factor graph optimization framework for urban GNSS positioning in challenging environments.

The method combines fisheye-image-based satellite visibility analysis, geometry-adaptive NLOS handling, and joint optimization of pseudorange and time-differenced carrier-phase (TDCP) factors within a sliding-window factor graph. It is designed to improve positioning robustness and accuracy in dense urban scenarios where conventional fixed exclusion or fixed reweighting strategies are often insufficient.

## Overview

Urban GNSS positioning is severely affected by non-line-of-sight (NLOS) reception caused by dense buildings, elevated roads, and other obstructions. In such conditions, simply excluding all detected NLOS observations may weaken satellite geometry, whereas retaining them with fixed down-weighting may allow residual NLOS biases to propagate into the final solution.

To address this issue, the proposed framework consists of three tightly coupled components:

1. **Fisheye-image-based LOS/NLOS classification**  
   Sky-view fisheye images are segmented to obtain a binary sky mask, and satellite directions are projected onto the image plane for satellite-wise visibility classification.

2. **Geometry-adaptive NLOS handling**  
   Based on the number of LOS satellites and the corresponding LOS-only geometry, the algorithm adaptively switches between:
   - **NLOS removal**, when reliable LOS-only geometry is sufficient; and
   - **NLOS down-weighting**, when direct exclusion would degrade solution availability or geometric strength.

3. **Sliding-window factor graph optimization with pseudorange and TDCP factors**  
   Pseudorange observations and inter-epoch TDCP constraints are jointly incorporated into a fixed-lag factor graph and solved iteratively using a Levenberg–Marquardt optimizer.

## Repository Scope

This public release focuses on the **core MATLAB implementation** of the main algorithmic pipeline. The repository is intentionally lightweight and currently includes only the principal modules required to demonstrate the method flow.

At the current stage, the repository provides:
- the main MATLAB demo scripts,
- the core configuration file,
- the fisheye sky-mask generation module,
- the satellite projection module,
- the geometry-adaptive NLOS handling module, and
- the LM-based GA-FGO solver.

This repository does **not** aim to provide a full experimental reproduction package at this stage.

## Current Repository Structure

```text
GA-FGO/
├── README.md
├── run_demo_ga_fgo.m
├── run_demo_fisheye_los_nlos.m
├── config/
│   └── config_default.m
├── preprocess/
│   └── build_sky_mask_otsu.m
├── projection/
│   └── project_satellite_to_image.m
├── handling/
│   └── switch_nlos_strategy.m
└── fgo/
    └── solve_ga_fgo_lm.m