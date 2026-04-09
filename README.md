# GA-FGO: Geometry-Adaptive NLOS Mitigation for Urban GNSS Positioning via Sliding-Window Factor Graph Optimization

This repository provides the core MATLAB implementation of the proposed geometry-adaptive sliding-window factor graph optimization framework for urban GNSS positioning in challenging environments.

The method combines fisheye-image-based satellite visibility analysis, geometry-adaptive NLOS handling, and joint optimization of pseudorange and time-differenced carrier-phase (TDCP) factors within a sliding-window factor graph. It is designed to improve positioning robustness and accuracy in dense urban scenarios where conventional fixed exclusion or fixed reweighting strategies are often insufficient.

## Overview

Urban GNSS positioning is severely affected by non-line-of-sight (NLOS) reception caused by dense buildings, elevated roads, and other obstructions. In such conditions, simply excluding all detected NLOS observations may weaken satellite geometry, whereas retaining them with fixed down-weighting may allow residual NLOS biases to propagate into the final solution.

To address this issue, the proposed framework consists of three tightly coupled components:

1. **Fisheye-image-based LOS/NLOS classification**  
   Sky-view fisheye images are segmented to obtain a binary sky mask, and satellite directions are projected onto the image plane for satellite-wise visibility classification.

2. **Geometry-adaptive NLOS handling**  
   Based on the number of LOS satellites and the corresponding LOS-only geometry, the algorithm adaptively switches between NLOS removal and NLOS down-weighting.

3. **Sliding-window factor graph optimization with pseudorange and TDCP factors**  
   Pseudorange observations and inter-epoch TDCP constraints are jointly incorporated into a fixed-lag factor graph and solved iteratively using a Levenberg–Marquardt optimizer.

## Repository Scope

This repository focuses on the core MATLAB implementation of the main algorithmic pipeline. The current release includes the principal modules required to demonstrate the proposed method flow, including fisheye sky-mask generation, satellite projection, geometry-adaptive NLOS handling, and sliding-window optimization.

## Tested Environment

The code in this repository was developed and tested in the following environment:

- MATLAB R2023a
- GTSAM 4.0.2

## Main Files and Their Roles

- `config/config_default.m`  
  Defines the default parameters of the GA-FGO framework, including sliding-window settings, geometry-adaptive thresholds, and optimization-related options.

- `preprocess/build_sky_mask_otsu.m`  
  Generates a binary sky mask from the fisheye image using grayscale conversion, Otsu thresholding, and basic image post-processing.

- `projection/project_satellite_to_image.m`  
  Projects satellite azimuth and elevation angles onto the fisheye image plane for satellite-wise LOS/NLOS visibility classification.

- `handling/switch_nlos_strategy.m`  
  Implements the geometry-adaptive NLOS handling strategy by switching between NLOS exclusion and NLOS down-weighting according to LOS redundancy and GDOP.

- `fgo/solve_ga_fgo_lm.m`  
  Constructs and solves the sliding-window factor graph with pseudorange and TDCP factors using the Levenberg–Marquardt optimizer.

- `run_demo_fisheye_los_nlos.m`  
  Demonstrates the fisheye-image-based LOS/NLOS classification process.

- `run_demo_ga_fgo.m`  
  Demonstrates the overall GA-FGO pipeline, including observation handling and sliding-window factor graph optimization.

## Current Repository Structure

```text
GA-FGO/
├── config/
│   └── config_default.m
├── fgo/
│   └── solve_ga_fgo_lm.m
├── handling/
│   └── switch_nlos_strategy.m
├── preprocess/
│   └── build_sky_mask_otsu.m
├── projection/
│   └── project_satellite_to_image.m
├── README.md
├── run_demo_fisheye_los_nlos.m
└── run_demo_ga_fgo.m
