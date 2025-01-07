"""
Filename: VRPTWSolver.py
Author: Yihe Zhuo
Last Modified: 2025-01-06
Description: 
    Implementation of the Two-Index Compact Formulation in Section 3.3. 
    This MILP serves as the baseline model for comparison.
"""

import gurobipy as gp
from gurobipy import GRB
import math

class VRPTWSolver:
    """Class to solve a VRPTW instance using Gurobi."""
    def __init__(self, instance):
        self.instance = instance
        self.model = gp.Model("VRPTW")
        self.x = None
        self.tau = None
        self.delta = None
        self.E_star = None
        self.lp_relaxation_obj = None

    def is_feasible(self, u, v):
        """
        Check whether the route [0, u, v, -1] is feasible based on time and capacity.
        """
        customers = self.instance.customers
        demands = self.instance.demands
        time_windows = self.instance.time_windows
        service_times = self.instance.service_times
        vehicle_capacity = self.instance.vehicle_capacity

        if u > 0 and v > 0:
            # skip u whose ealiest start time >= v's latest start time
            if time_windows[u - 1][0] > time_windows[v - 1][1]:
                return False

            # Time feasibility
            if time_windows[u - 1][0] + service_times[u - 1] + self.instance.compute_distance(customers[u - 1], customers[v - 1]) > time_windows[v - 1][1]:
                return False
            
            # Capacity feasibility
            if demands[u - 1] + demands[v - 1] > vehicle_capacity:
                return False

        return True

    def generate_E_star(self):
        """
        Generate the set E* containing all valid routes (u, v).
        """
        num_customers = self.instance.num_customers
        self.E_star = [
            (u, v) for u in range(num_customers + 1) for v in [x for x in range(-1, num_customers + 1) if x != 0]
            if u != v
            and self.is_feasible(u, v)
        ]

    def setup_variables(self):
        """Define decision variables for the VRPTW model."""
        num_customers = self.instance.num_customers
        vehicle_capacity = self.instance.vehicle_capacity
        self.x = self.model.addVars(
            self.E_star,  # Only include edges in E*
            vtype=GRB.BINARY, name="x"
        )
        self.tau = self.model.addVars(
            range(-1, num_customers + 1), lb=0, ub = self.instance.depot["due_time"], vtype=GRB.CONTINUOUS, name="tau"
        )
        self.delta = self.model.addVars(
            range(-1, num_customers + 1), lb = 0, ub=vehicle_capacity, vtype=GRB.CONTINUOUS, name="delta"
        )

    def setup_objective(self):
        """Define the objective function to minimize travel distance."""
        depot = self.instance.depot
        customers = self.instance.customers

        self.model.setObjective(
            gp.quicksum(
                (self.instance.compute_distance(
                    customers[i - 1] if i > 0 else depot,
                    customers[j - 1] if j > 0 else depot
                )
                ) * self.x[i, j]
                for i, j in self.E_star
            ),
            GRB.MINIMIZE
        )

    def setup_constraints(self):
        """Add constraints to the VRPTW model."""
        depot = self.instance.depot
        customers = self.instance.customers
        demands = self.instance.demands
        time_windows = self.instance.time_windows
        service_times = self.instance.service_times
        vehicle_capacity = self.instance.vehicle_capacity
        num_customers = self.instance.num_customers

        # Constraint 1b: Each customer is visited exactly once
        self.model.addConstrs(
            (gp.quicksum(self.x[u, v] for u, v in self.E_star if u == i) == 1
             for i in range(1, num_customers + 1)),
            name="leave_once"
        )

        # Constraint 1c: a vehicle must leave each customer once
        self.model.addConstrs(
            (gp.quicksum(self.x[v, u] for v, u in self.E_star if u == i) == 1
             for i in range(1, num_customers + 1)),
            name="visit_once"
        )

        # Constraint 1f: Flow conservation for depot
        self.model.addConstr(
            gp.quicksum(self.x[0, j] for j in range(1, num_customers + 1)) >= math.ceil(sum(demands) / vehicle_capacity),
            name="depot_outflow"
        )

        # Constraint 1d: Capacity constraints
        self.model.addConstrs(
            (self.delta[v] - demands[v - 1] >= self.delta[u] - (demands[v - 1] + vehicle_capacity) * (1 - self.x[v, u])
            for v, u in self.E_star if u > 0),
            name="capacity"
        )

        # bounds for delta under "min"
        self.model.addConstrs(
            (demands[j - 1] <= self.delta[j]
             for j in range(1, num_customers + 1)),
            name="capacity_feasibility"
        )

        # Constraint 1e: Time constraints
        self.model.addConstrs(
            (
                self.tau[v] >= self.tau[u] + service_times[v - 1] +
                self.instance.compute_distance(customers[v - 1], customers[u - 1]) -
                (depot["due_time"] - time_windows[u-1][0] + self.instance.compute_distance(customers[v - 1], customers[u - 1]) + service_times[v - 1]) 
                * (1 - self.x[v, u])
                for v, u in self.E_star if u > 0 and v > 0
            ),
            name="time_window"
        )

        self.model.addConstrs(
            (
                self.tau[v] >= self.tau[u] + self.instance.compute_distance(depot, customers[u - 1]) -
                (depot["due_time"] - time_windows[u-1][0] + self.instance.compute_distance(depot, customers[u - 1])) 
                * (1 - self.x[v, u])
                for v, u in self.E_star if u > 0 and v == 0
            ),
            name="time_window_from_depot"
        ) 

        """ self.model.addConstrs(
            (
                self.tau[v] >= self.instance.service_times[v - 1] +
                self.instance.compute_distance(self.instance.customers[v - 1], depot) -
                (depot["due_time"] - time_windows[v - 1][0]) * (1 - self.x[v, u])
                for v, u in self.E_star if u == 0 and v != 0
            ),
            name="time_window_to_depot"
        ) """

        self.model.addConstrs(
            (depot["due_time"] - time_windows[j - 1][0] >= self.tau[j]
             for j in range(1, num_customers + 1)),
            name="time_feasibility_ub"
        )
        self.model.addConstrs(
            (self.tau[j] >= depot["due_time"] - time_windows[j - 1][1]
             for j in range(1, num_customers + 1)),
            name="time_feasibility_lb"
        )
        self.model.addConstr(
            (self.tau[0] == depot["due_time"]),
            name="time_feasibility_depot"
        )

    # Callback function to capture LP relaxation objective
    def lp_relaxation_callback(self, model, where):
        if where == GRB.Callback.MIPNODE:  # MIP node callback
            if model.cbGet(GRB.Callback.MIPNODE_STATUS) == GRB.OPTIMAL:
                # Get the LP relaxation objective at the root node
                lp_obj = model.cbGet(GRB.Callback.MIPNODE_OBJBST)
                # Store it if it's the root node
                if not hasattr(model, "_lp_relax_obj"):
                    model._lp_relax_obj = lp_obj  # Save to the model

    def solve(self):
        # Generate E* before solving
        self.generate_E_star()
        self.setup_variables()
        self.setup_objective()
        self.setup_constraints()

        # Write the model to an LP file
        self.model.write("vrptw_baseline_model.lp")

        self.model.setParam('TimeLimit', 1000)  # Set time limit
        self.model.setParam('OutputFlag', 0)    # Disable gurobi output

        """Solve the VRPTW model."""
        self.model.optimize(lambda m, where: self.lp_relaxation_callback(m, where))
        self.solve_status()
        # Get the solve time
        self.solve_time = self.model.Runtime
        self.mip_dual_bound = self.model.ObjBound
        # Retrieve the LP relaxation objective
        self.lp_relaxation_obj = getattr(self.model, "_lp_relax_obj", None)

    def solve_status(self):
        if self.model.status == GRB.OPTIMAL:
            print("Optimal solution found.")
        if self.model.status == GRB.INFEASIBLE:
            print("Model is infeasible. Computing IIS...")
            self.model.computeIIS()
            self.model.write("model.ilp")
            print("IIS written to model.ilp")
        else:
            print(f"Solver ended with status {self.model.status}")

    def extract_solution(self):
        """Extract the solution from the optimized model."""
        if self.model.status == GRB.OPTIMAL:
            # Extract the selected edges
            selected_edges = [(u, v) for u, v in self.E_star if self.x[u, v].X > 0.5]
            
            # Build routes starting and ending at the depot (node 0)
            routes = []
            visited = set()
            
            while True:
                # Find an edge starting from the depot that hasn't been visited
                start_edges = [(u, v) for u, v in selected_edges if u == 0 and v not in visited]
                if not start_edges:  # No more routes
                    break
                
                route = []
                current = start_edges[0][1]  # First node after the depot
                
                while current != 0:  # Follow the route until returning to the depot
                    route.append(current)
                    visited.add(current)
                    # Find the next node connected to the current one
                    next_node = [v for u, v in selected_edges if u == current and v not in visited]
                    if next_node:
                        current = next_node[0]
                    else:
                        break
                
                if route:
                    routes.append(route)
            
            # Print the routes
            for i, route in enumerate(routes):
                print(f"Route {i + 1}:", " ".join(map(str, route)))
            
            # Return the solution as a dictionary
            solution = {
                "routes": routes,
                "objective_value": self.model.ObjVal
            }
            return solution
        else:
            print("No feasible solution found.")
            return None

