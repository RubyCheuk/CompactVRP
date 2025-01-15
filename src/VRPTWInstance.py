"""
Filename: VRPTWInstance.py
Author: Yihe Zhuo
Last Modified: 2025-01-06
Description: 
    After data loading, construct a data instance to be used in algorithms.
"""

import math

class VRPTWInstance:
    """Class to represent a Vehicle Routing Problem with Time Windows (VRPTW) instance."""
    def __init__(self, depot, customers, demands, time_windows, service_times, vehicle_capacity=100):
        self.depot = depot  # Use the same node 0 for starting and ending depot for now.
        self.customers = customers  # Cust No. and their X, Y coordinates
        self.demands = demands
        self.time_windows = time_windows    # Ready time, due date
        self.service_times = service_times  # t*_u
        self.vehicle_capacity = vehicle_capacity
        self.num_customers = len(customers)
    
    def compute_distance(self, node1, node2):
        """Calculate the Euclidean distance between two nodes."""
        # TODO: move to Util.py
        distance = math.sqrt((node1['x'] - node2['x'])**2 + (node1['y'] - node2['y'])**2)    
        return math.floor(distance * 10) / 10

    def __str__(self):
        """Provide a readable string representation of the VRPTWInstance object."""
        """ Not using it for now. """
        result = []
        result.append("Depot:")
        result.append(f"  Location: ({self.depot['x']}, {self.depot['y']})")
        result.append(f"  Ready Time: {self.depot['ready_time']}, Due Time: {self.depot['due_time']}")
        result.append(f"  Vehicle Capacity: {self.vehicle_capacity}")
        
        result.append("\nCustomers:")
        for i, customer in enumerate(self.customers):
            result.append(f"  Customer {i + 1}:")
            result.append(f"    Location: ({customer['x']}, {customer['y']})")
            result.append(f"    Demand: {self.demands[i]}")
            result.append(f"    Time Window: {self.time_windows[i]}")
            result.append(f"    Service Time: {self.service_times[i]}")
        
        return "\n".join(result)