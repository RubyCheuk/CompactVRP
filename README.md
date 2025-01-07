# Compact VRP Algorithm

> Link to the paper: [A New Class of Compact Formulations for Vehicle Routing Problems](https://arxiv.org/pdf/2403.00262)

---

## Table of Contents

- [Overview](#overview)
- [Installation](#installation)
- [Code Structure](#code-structure)
- [Results](#results)

---

## Overview

This repository implements the above paper using capacity/time windown discretization and Local Area arcs. The performance of the algorithm is compared with a two-index MILP formulation. Gurobi is used to solve the model.

---

## Installation

Follow these steps to set up the project on your local machine:

1. Clone the repository
2. cd to your repo directory
3. Run the setup script to create and activate a virtual environment, install dependencies, and run the application:

`chmod +x setup_and_run.sh`

`./setup_and_run.sh`

---

## Code Structure

Root folder:  
├── src/  
│   ├── main.py: Main file to run the baseline and new VRP algorithm in the paper.  
│   ├── VRPTWSolver.py: Implementation of the two-index baseline MILP in Section 3.3.  
│   ├── LADiscretizationAlgo.py: Implementation of the new VRP Algorithm 1 in the paper.  
│   ├── VRPTWSolverAlgo.py: Implementation of the new compact VRP model in Section 4.3. Used in LADiscretizationAlgo.  
│   ├── FlowGraphCapacity.py: Capacity/Time window discretization in Section 4.2 and 5.1.   
│   ├── LocalAreaArcs.py: Local area neighborhood implementation in Section 4.1, 4.5.2, and 5.2.   
│   ├── dataLoader.py: Load Solomon benchmark data set.  
│   ├── VRPTWInstance.py: Store dataset loaded by dataLoader as an instance and to be used by algorithms.  
│   ├── Util.py: Helper functions.  
│   └── dataset/: Store some Solomon benchmark dataset from http://www.vrp-rep.org/datasets.html.  
├── README.md  
├── requirements.txt  
└── setup_and_run.sh: Shell file to install packages and run.  

---

## Example Results

Let's mainly focus on the ILP objective, ILP time, and total LP time. The LP Obj and MIP Dual Bound may need further check.  
The ILP Obj are very close to the appendix of the paper. Tiny difference might be from rounding errors.  
Next step improvement: Gurobi model creation time; faster convergence method.

| File Num   | Approach       | LP Obj   | MIP Dual Bound | ILP Obj   | ILP Time   | Total LP Time | Total Run Time  |
|------------|----------------|----------|----------------|-----------|------------|---------------|-----------------|
| RC205_025  | OUR APPROACH   | 260.907  | 338.929        | 338.929   | 87.5607    | 44.8769       | 2859.72         |
| RC205_025  | BASELINE MILP  | 419.913  | 338.929        | 338.929   | 0.287437   | 0             | 0.312919        |
| R103_025   | OUR APPROACH   | 452.752  | 455.698        | 455.698   | 4.7041     | 18.0105       | 169.858         |
| R103_025   | BASELINE MILP  | 553.43   | 455.698        | 455.698   | 4.11145    | 0             | 4.1446          |
| R101_025   | OUR APPROACH   | 618.33   | 618.33         | 618.33    | 1.37088    | 0.735905      | 27.7363         |
| R101_025   | BASELINE MILP  |          | 618.33         | 618.33    | 0.00351286 | 0             | 0.018548        |
| R101_050   | OUR APPROACH   | 1046.7   | 1046.7         | 1046.7    | 7.45831    | 5.63699       | 249.866         |
| R101_050   | BASELINE MILP  | 1051.98  | 1046.7         | 1046.7    | 0.0165319  | 0             | 0.057215        |
| R110_025   | OUR APPROACH   | 420.277  | 445.177        | 445.177   | 15.8692    | 12.344        | 100.203         |
| R110_025   | BASELINE MILP  | 545.89   | 445.177        | 445.177   | 22.0112    | 0             | 22.0381         |
| R110_050   | OUR APPROACH   | 677.897  | 690.016        | 735.441   | 1000.14    | 281.94        | 3099.46         |
| R110_050   | BASELINE MILP  | 919.805  | 615.744        | -         | 1000.03    | 0             | 1000.11         |
| R105_025   | OUR APPROACH   | 531.539  | 531.539        | 531.539   | 2.64606    | 3.11974       | 48.9599         |
| R105_025   | BASELINE MILP  | 593.274  | 531.539        | 531.539   | 0.0753539  | 0             | 0.101321        |



