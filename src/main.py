"""
Filename: main.py
Author: Yihe Zhuo
Last Modified: 2025-01-06
Description: 
    Main file. Load dataset, run baseline and compact VRP algorithm, and compare their performance.
"""

from dataLoader import SolomonDatasetLoader
from VRPTWSolver import VRPTWSolver
from LADiscretizationAlgo import LADiscretization
from Util import print_comparison_table
import time

def main_baseline(instance):
    start_time = time.time()  # Start timer
    # Solve the VRPTW problem
    solver = VRPTWSolver(instance)
    solver.solve()
    end_time = time.time()  # End timer
    
    # Extract and print the solution
    solution = solver.extract_solution()
    """ 
    if solution:
        print("Optimal Routes:", solution["routes"])
        #print("Selected Edges:", solution["selected_edges"])
        print("Objective Value (Total Distance):", solution["objective_value"])
    else:
        print("No feasible solution found!")
    """
    return {
        "lp_obj": solver.lp_relaxation_obj,
        "mip_dual_bound": solver.mip_dual_bound,
        "ilp_obj": solution["objective_value"] if solution else None,
        "ilp_time": solver.solve_time,
        "total_lp_time": 0,
        "run_time": end_time - start_time,
    }

def main_compactVRP(instance):
    start_time = time.time()  # Start timer

    # Solve the VRPTW problem
    la_discretization = LADiscretization(instance)
    final_obj, ilp_time, lp_relaxation_obj, mip_dual_bound = la_discretization.run()
    end_time = time.time()  # End timer

    return {
        "lp_obj": lp_relaxation_obj,
        "mip_dual_bound": mip_dual_bound,
        "ilp_obj": final_obj if final_obj else None,
        "ilp_time": ilp_time,
        "total_lp_time": la_discretization.lp_time,
        "run_time": end_time - start_time,
    }
    

if __name__ == "__main__":

    '''Load the Solomon dataset'''
    # TODO: add a wrapper to automate all data instances
    dataset_file_path = "src/dataset/solomon-1987-r1/R105_050.xml"  # Replace with other dataset file path
    loader = SolomonDatasetLoader(dataset_file_path)
    instance = loader.load_instance()

    baseline_results = main_baseline(instance)  # Baseline MILP
    compact_results = main_compactVRP(instance) # VRPTW algorithm in the paper

    # TODO: save output as files
    print_comparison_table(dataset_file_path, baseline_results, compact_results)