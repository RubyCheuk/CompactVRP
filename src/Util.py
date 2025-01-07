"""
Filename: Util.py
Author: Yihe Zhuo
Last Modified: 2025-01-06
Description: 
    Helper functions used by other scripts.
"""

from tabulate import tabulate

def calculate_neighbors(instance, k_neighbors, u):
    """
    Compute the neighbors (N^k_u) for a specific customer u based on distance and time window conditions.

    Args:
        u: The specific customer for which neighbors are computed.

    Returns:
        list: A list of neighbors satisfying the conditions.
    """
    customers = instance.customers
    time_windows = instance.time_windows

    # Compute distances to all other customers
    distances = [
        (v, instance.compute_distance(customers[u - 1], customers[v - 1]))
        for v in range(1, instance.num_customers + 1) if v != u
    ]
    distances.sort(key=lambda x: x[1])  # Sort by distance

    # Filter neighbors based on the time window condition
    neighbors = [
        (v, dist) for v, dist in distances
        if time_windows[v - 1][1] >= time_windows[u - 1][0]  # Ensure neighbor v's time window is valid
    ]

    # Select the top-k neighbors
    return [v for v, _ in neighbors[:k_neighbors]]

def print_comparison_table(file_path, baseline_results, compact_results):
    # Extract the file number from the dataset file path
    file_num = file_path.split("/")[-1].split(".")[0]

    # Prepare data for the table
    data = [
        [file_num, "OUR APPROACH", compact_results["lp_obj"], compact_results["mip_dual_bound"], compact_results["ilp_obj"], compact_results["ilp_time"],compact_results["total_lp_time"],compact_results["run_time"]],
        [file_num, "BASELINE MILP", baseline_results["lp_obj"], baseline_results["mip_dual_bound"], baseline_results["ilp_obj"], baseline_results["ilp_time"],baseline_results["total_lp_time"],baseline_results["run_time"]]
    ]

    # Define table headers
    headers = ["file num", "approach", "lp obj", "mip dual bound",  "ILP obj", "ilp time", "total lp time", "total run time"]

    # Print the table
    print(tabulate(data, headers=headers, tablefmt="grid"))