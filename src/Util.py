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

def safe_round(value, precision=4):
    if isinstance(value, (int, float)):
        return round(value, precision)
    return value  # Return as-is if not numeric

def print_comparison_table(file_path, baseline_results, compact_results):
    # Extract the file number from the dataset file path
    file_num = file_path.split("/")[-1].split(".")[0]

    # Define default values for empty baseline_results
    default_values = {
        "lp_obj": "N/A",
        "mip_dual_bound": "N/A",
        "ilp_obj": "N/A",
        "ilp_time": "N/A",
        "total_lp_time": "N/A",
        "run_time": "N/A",
    }
    if not baseline_results:
        baseline_results = default_values

    # Prepare data for the table
    data = [
        [
            file_num, "OUR APPROACH",
            safe_round(compact_results.get("lp_obj", "N/A")),
            safe_round(compact_results.get("mip_dual_bound", "N/A")),
            safe_round(compact_results.get("ilp_obj", "N/A")),
            safe_round(compact_results.get("ilp_time", "N/A")),
            safe_round(compact_results.get("total_lp_time", "N/A")),
            safe_round(compact_results.get("run_time", "N/A")),
        ],
        [
            file_num, "BASELINE MILP",
            safe_round(baseline_results.get("lp_obj", "N/A")),
            safe_round(baseline_results.get("mip_dual_bound", "N/A")),
            safe_round(baseline_results.get("ilp_obj", "N/A")),
            safe_round(baseline_results.get("ilp_time", "N/A")),
            safe_round(baseline_results.get("total_lp_time", "N/A")),
            safe_round(baseline_results.get("run_time", "N/A")),
        ]
    ]

    # Define table headers
    headers = ["file num", "approach", "lp obj", "mip dual bound",  "ILP obj", "ilp time", "total lp time", "total run time"]

    # Print the table
    print(tabulate(data, headers=headers, tablefmt="github"))