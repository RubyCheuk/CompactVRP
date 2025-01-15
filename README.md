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

| File Num   | Approach       | LP Obj    | MIP Dual Bound   | ILP Obj    | ILP Time   | Total LP Time   | Total Run Time   |
|------------|----------------|-----------|------------------|------------|------------|-----------------|------------------|
| R101_025   | OUR APPROACH   | 618.3299  | 618.3299         | 618.3299   | 0.0078     | 0.0498          | 1.8611           |
| R101_025   | BASELINE MILP  | 618.33    | 618.33           | 618.33     | 0.0027     | 0               | 0.0151           |
| R103_025   | OUR APPROACH   | 452.6045  | 455.6982         | 455.6982   | 0.1324     | 1.265           | 11.5795          |
| R103_025   | BASELINE MILP  | 329.855   | 455.698          | 455.698    | 4.02       | 0               | 4.0435           |
| R105_025   | OUR APPROACH   | 531.5386  | 531.5386         | 531.5386   | 0.0273     | 0.3963          | 2.7292           |
| R105_025   | BASELINE MILP  | 457.574   | 531.539          | 531.539    | 0.0709     | 0               | 0.0856           |
| R110_025   | OUR APPROACH   | 420.118   | 445.1768         | 445.1768   | 1.2249     | 1.9727          | 9.1842           |
| R110_025   | BASELINE MILP  | 313.938   | 445.177          | 445.177    | 22.6708    | 0               | 22.6948          |
| RC205_025  | OUR APPROACH   | 260.2709  | 338.9294         | 338.9294   | 0.705      | 8.646           | 38.0963          |
| RC205_025  | BASELINE MILP  | 194.288   | 338.929          | 338.929    | 0.287437   | 0               | 0.312919         |

| file num   | approach      | lp obj   | mip dual bound   | ILP obj   | ilp time   | total lp time   | total run time   |
|------------|---------------|----------|------------------|-----------|------------|-----------------|------------------|
| R101_025   | OUR APPROACH  |    617.1 |            617.1 |     617.1 |        0.0 |             0.1 |              1.8 |
| R101_025   | BASELINE MILP |    617.1 |            617.1 |     617.1 |        0.0 |             0.0 |              0.0 |
| R103_025   | OUR APPROACH  |    451.6 |            454.6 |     454.6 |        0.1 |             1.0 |             10.0 |
| R103_025   | BASELINE MILP |    328.6 |            454.6 |     454.6 |        2.9 |             0.0 |              2.9 |
| R105_025   | OUR APPROACH  |    530.5 |            530.5 |     530.5 |        0.0 |             0.4 |              2.8 |
| R105_025   | BASELINE MILP |    456.5 |            530.5 |     530.5 |        0.1 |             0.0 |              0.1 |
| R110_025   | OUR APPROACH  |    418.3 |            444.1 |     444.1 |        1.3 |             1.9 |              9.1 |
| R110_025   | BASELINE MILP |    312.8 |            444.1 |     444.1 |       21.3 |             0.0 |             21.3 |
| RC205_025  | OUR APPROACH  |    248.2 |            338.0 |     338.0 |        1.2 |             5.9 |             26.3 |
| RC205_025  | BASELINE MILP |    193.7 |            338.0 |     338.0 |        0.4 |             0.0 |              0.4 |

| file num   | approach      |   lp obj |   mip dual bound |   ILP obj |   ilp time |   total lp time |   total run time |
|------------|---------------|----------|------------------|-----------|------------|-----------------|------------------|
| R101_050   | OUR APPROACH  |   1043.4 |           1044.0 |    1044.0 |        0.0 |             0.4 |              9.1 |
| R101_050   | BASELINE MILP |   1029.8 |           1044.0 |    1044.0 |        0.0 |             0.0 |              0.1 |
| R102_050   | OUR APPROACH  |    902.8 |            909.0 |     909.0 |        0.2 |             2.6 |             17.5 |
| R102_050   | BASELINE MILP |    599.1 |            909.0 |     909.0 |       41.1 |             0.0 |             41.1 |
| R103_050   | OUR APPROACH  |    734.0 |            772.9 |     772.9 |        6.3 |            10.7 |             44.0 |
| R103_050   | BASELINE MILP |    499.9 |            729.2 |     -     |     1000.0 |             0.0 |           1000.1 |
| R104_050   | OUR APPROACH  |    598.6 |            625.4 |     625.4 |      154.9 |            79.0 |            327.6 |
| R104_050   | BASELINE MILP |    464.2 |            547.2 |     -     |     1000.0 |             0.0 |           1000.1 |
| R105_050   | OUR APPROACH  |    890.2 |            899.3 |     899.3 |        0.4 |             3.5 |             15.7 |
| R105_050   | BASELINE MILP |    789.5 |            899.3 |     899.3 |        0.9 |             0.0 |              0.9 |
| R110_050   | OUR APPROACH  |    675.1 |            697.0 |     697.0 |       11.4 |            47.0 |            108.1 |
| R110_050   | BASELINE MILP |    481.0 |            610.0 |     -     |     1000.0 |             0.0 |           1000.1 |

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

