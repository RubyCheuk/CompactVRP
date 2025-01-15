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

def safe_round(value, precision=1):
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
    print(tabulate(data, headers=headers, tablefmt="github", floatfmt=".1f"))


def compare_dicts(dict1, dict2):
    """
    Compare two dictionaries and return True if they are exactly the same, including nested dictionaries.

    Args:
        dict1 (dict): The first dictionary.
        dict2 (dict): The second dictionary.

    Returns:
        bool: True if the dictionaries are exactly the same, False otherwise.
    """

    # Check if both are dictionaries
    if not isinstance(dict1, dict) or not isinstance(dict2, dict):
        return False

    # Compare keys
    if set(dict1.keys()) != set(dict2.keys()):
        return False

    # Compare values
    for key in dict1:
        value1 = dict1[key]
        value2 = dict2[key]

        if isinstance(value1, dict) and isinstance(value2, dict):
            # Recursively compare nested dictionaries
            if not compare_dicts(value1, value2):
                return False
        elif value1 != value2:
            # Compare other types of values
            return False

    return True

def customers_need_update(old_dict, new_dict):
    """
    Compare two dictionaries where values are lists, and return a list of keys
    whose list values are different.

    Args:
        old_dict (dict): The first dictionary to compare.
        new_dict (dict): The second dictionary to compare.

    Returns:
        list: A list of keys (`u`) whose list values differ between the dictionaries.
    """
    differing_keys = []

    # Get the union of keys from both dictionaries
    all_keys = set(old_dict.keys()).union(set(new_dict.keys()))

    for key in all_keys:
        old_value = old_dict.get(key, None)
        new_value = new_dict.get(key, None)

        # Check if one of the values is None or the lists are different
        if old_value != new_value:
            differing_keys.append(key)

    return differing_keys
