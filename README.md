# Compact VRP Algorithm

> Link to the paper: [A New Class of Compact Formulations for Vehicle Routing Problems](https://arxiv.org/pdf/2403.00262)

---

## Table of Contents

- [Overview](#overview)
- [Installation](#installation)
- [Code Structure](#code-structure)
- [Example Results](#example-results)

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

Let's mainly focus on the ILP objective, ILP time, and total LP time.  
The ILP Obj are very close to the appendix of the paper. Tiny difference might be from rounding errors.

**Next step improvement:** Gurobi model creation time; faster convergence method.

| file num   | approach      |   lp obj |   mip dual bound |   ILP obj |   ilp time |   total lp time |   total run time |
|------------|---------------|----------|------------------|-----------|------------|-----------------|------------------|
| R101_025   | OUR APPROACH  |   618.33 |           618.33 |    618.33 |     1.3271 |          0.5232 |           8.5267 |
| R101_025   | BASELINE MILP |   618.33 |           618.33 |    618.33 |     0.0027 |          0      |           0.0151 |
| R103_025   | OUR APPROACH  |  452.752 |          455.698 |   455.698 |     3.7312 |         16.6692 |          47.334  |
| R103_025   | BASELINE MILP |  329.855 |          455.698 |   455.698 |     4.02   |          0      |           4.0435 |
| R105_025   | OUR APPROACH  |  531.539 |          531.539 |   531.539 |     2.6129 |          2.6502 |          15.1319 |
| R105_025   | BASELINE MILP |  457.574 |          531.539 |   531.539 |     0.0709 |          0      |           0.0856 |
| R110_025   | OUR APPROACH  |  420.277 |          445.177 |   445.177 |    17.1648 |         32.8874 |          68.2745 |
| R110_025   | BASELINE MILP |  313.938 |          445.177 |   445.177 |    22.6708 |          0      |          22.6948 |


| file num   | approach      |   lp obj |   mip dual bound |   ILP obj |   ilp time |   total lp time |   total run time |
|------------|---------------|----------|------------------|-----------|------------|-----------------|------------------|
| R101_050   | OUR APPROACH  |  1046.7  |           1046.7 |    1046.7 |     7.9002 |          5.0251 |          44.0025 |
| R101_050   | BASELINE MILP |  1032.45 |           1046.7 |    1046.7 |     0.016  |          0      |           0.0525 |
| R102_050   | OUR APPROACH  |  905.244 |          911.443 |   911.443 |    11.3866 |         37.0904 |          89.3405 |
| R102_050   | BASELINE MILP |  601.458 |          911.443 |   911.443 |    23.8083 |          0      |          23.87   |
| R103_050   | OUR APPROACH  | 748.3437 | 775.6517         | 775.6517  | 444.0305   | 324.7485        | 897.583          |
| R103_050   | BASELINE MILP | 502.689  | 745.588          | -         | 1000.02    | 0               | 1000.1           |

| File Num   | Approach       | LP Obj   | MIP Dual Bound | ILP Obj   | ILP Time   | Total LP Time | Total Run Time  |
|------------|----------------|----------|----------------|-----------|------------|---------------|-----------------|
| R110_025   | OUR APPROACH   | 420.277  | 445.177        | 445.177   | 15.8692    | 12.344        | 100.203         |
| R110_025   | BASELINE MILP  | 313.938  | 445.177        | 445.177   | 22.0112    | 0             | 22.0381         |
| RC205_025  | OUR APPROACH   | 260.907  | 338.929        | 338.929   | 87.5607    | 44.8769       | 2859.72         |
| RC205_025  | BASELINE MILP  | 194.288  | 338.929        | 338.929   | 0.287437   | 0             | 0.312919        |
| R104_050   | OUR APPROACH   | 603.613  | 609.176        | 659.94    | 1000.08    | 384.136       | 3334.18         |
| R104_050   | BASELINE MILP  | 466.892  | 548.818        | -         | 1000.04    | 0             | 1000.13         |
| R105_050   | OUR APPROACH   | 899.013  | 914.311        | 914.311   | 59.913     | 32.9914       | 555.451         |
| R105_050   | BASELINE MILP  | 795.369  | 914.311        | 914.311   | 0.507538   | 0             | 0.552254        |
| R110_050   | OUR APPROACH   | 677.897  | 690.016        | 735.441   | 1000.14    | 281.94        | 3099.46         |
| R110_050   | BASELINE MILP  | 483.222  | 615.744        | -         | 1000.03    | 0             | 1000.11         |

**Comment:** Column "lp obj" of the baseline MILP is showing the LP relaxation objective. I have tested some methods:

Method 1. use lp_relaxation_callback:  
    `self.model.optimize(lambda m, where: self.lp_relaxation_callback(m, where))`  
    `self.lp_relaxation_obj = getattr(self.model, "_lp_relax_obj", None)`

Method 2. relax the model and solve again.  
    `self.model = self.model.relax()`  
    `self.model.optimize()`

Method 3. Retrive Root relaxation: objective from Gurobi log.  
    Not implemented for now.

These values have a small gap because Method 1 and 3 are getting the root relaxation objective of the MILP, while this relaxation after presolve would be slightly different from the LP relaxation,  see https://support.gurobi.com/hc/en-us/community/posts/360077249052-Root-relaxation-objective-value-is-different-from-the-objective-value-from-continuous-model.  
For now I use Method 2 to make sure getting the accurate LP relaxation objective. We can switch to Method 1 instead as well.

