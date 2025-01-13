"""
Filename: LADiscretizationAlgo.py
Author: Yihe Zhuo
Last Modified: 2025-01-06
Description: 
    Implementation of Algorithm 1, LA-Discretization Algorithm, in Section 5.3 of the paper
"""

from Util import calculate_neighbors
from VRPTWSolverAlgo import VRPTWSolverAlgo
import time

class LADiscretization:
    def __init__(self, instance, ns=6, ds=5, ts=50, min_inc=1, iter_max=10, zeta=9):
        """
        Initialize the LA-Discretization parameters. All default values are set the same as Section 7 Experiments.
        Args:
            instance: VRPTWInstance object containing problem data.
            ns: Number of local area neighbors.
            ds: Initial capacity bucket size.
            ts: Initial time bucket size.
            min_inc: Minimum LP improvement threshold.
            iter_max: Maximum iterations without improvement before termination.
            zeta: Iterations since last reset before enforcing sufficiency.
        """
        self.instance = instance
        self.ns = ns
        self.ds = ds
        self.ts = ts
        self.min_inc = min_inc
        self.iter_max = iter_max
        self.zeta = zeta
        self.variable_basis = None
        self.constraint_basis = None
        # Initialize parameterizations

        # Local area neighbors
        self.N_u = {u: set() for u in range(1, self.instance.num_customers + 1)}
        for u in range(1, self.instance.num_customers + 1):
            self.N_u[u] = calculate_neighbors(self.instance, self.ns, u)

        # Capacity buckets
        self.WD_u = {u: self.initialize_buckets(self.instance.demands[u - 1], self.instance.vehicle_capacity, ds)
            for u in range(1, self.instance.num_customers + 1)}
        
        # Remaining time buckets. Remaining time when leaving customer u = overall route due_time - time window of service starts - servive time at u
        self.WT_u = {u: self.initialize_buckets(self.instance.depot["due_time"] - self.instance.time_windows[u - 1][1] - self.instance.service_times[u-1], self.instance.depot["due_time"] - self.instance.time_windows[u - 1][0] - self.instance.service_times[u-1], ts)
            for u in range(1, self.instance.num_customers + 1)}
        
        # Tracking LP iterations
        self.iter_since_reset = 0
        self.last_lp_val = -float("inf")

    @staticmethod
    def initialize_buckets(lower_bound, upper_bound, bucket_size):
        """Initialize capacity/time thresholds WD or WT."""
        thresholds = [lower_bound]
        while lower_bound < upper_bound:
            lower_bound += bucket_size
            thresholds.append(min(upper_bound, lower_bound))    # The last bucket length might be < bucket_size
        return sorted(thresholds)

    def solve_lp_relaxation(self, relaxation = True):
        """ Use a bool flag, relaxation, to indicate whether a LP relaxation or the MILP will be solved.
        Line 8: Solve the current LP relaxation and update the parameterization. relaxation = True.
        Line 21: Solve the current LP relaxation and update the parameterization. relaxation = False.
        """
        solver = VRPTWSolverAlgo(self.instance, self.N_u, self.WD_u, self.WT_u, self.variable_basis, self.constraint_basis, relaxation)
        start_time = time.time()
        solver.create_model()
        end_time = time.time()  # End time
        print(f"-----create_model-----Runtime: {end_time - start_time:.4f} seconds")
        solver.solve()
        start_time = time.time()
        print(f"-----solve_model-----Runtime: {-end_time + start_time:.4f} seconds")
        lp_objective = solver.model.ObjVal
        mip_dual_bound = solver.model.ObjBound   # To get the MIP dual bound of the MILP? TODO: need to further check if the usage is correct.

        # Get the solve time
        solve_time = solver.model.Runtime  
        return lp_objective, solve_time, solver, mip_dual_bound, getattr(solver, "variable_basis", None), getattr(solver, "constraint_basis", None)

    def contract_parameters(self, solver):
        """
        Contract parameters (LA-neighbors, time/capacity buckets) for parsimony.
        Used in Lines 10-12 in Algorithm 1.
        """
        start_time = time.time()
        # Set N_u based on k_u based on (10) in Section 5.2.
        for u in range(1, self.instance.num_customers + 1):
            if u in solver.LA_pi:
                # Get the largest k for the current u
                largest_k = max(solver.LA_pi[u].keys())
                # Perform any operation with largest_k if needed
                print(f"u={u}, largest_k={largest_k}")
                self.N_u[u] = self.N_u[u][:largest_k]
            else:
                # If u does not exist in LA_pi, set self.N_u[u] as an empty list
                self.N_u[u] = []
        end_time = time.time()  # End time
        print(f"------contract_parameters_LAarcs-----Runtime: {end_time - start_time:.4f} seconds")

        # Apply contraction logic for WD_u, WT_u in Section 5.1.
        solver.post_process_duals(solver.capacity_pi)
        start_time = time.time()  # End time
        print(f"------contract_parameters_cap-----Runtime: {-end_time + start_time:.4f} seconds")
        solver.post_process_duals(solver.time_pi, 'time')
        end_time = time.time()  # End time
        print(f"------contract_parameters_time-----Runtime: {end_time - start_time:.4f} seconds")

    def expand_parameters(self, lp_solver):
        """Expand parameters (time/capacity buckets) for sufficiency."""
        # Line 17 in Algorithm 1
        self.expand_parameters_capacity_time(lp_solver, "capacity")
        # Line 16 in Algorithm 1
        self.expand_parameters_capacity_time(lp_solver, "time")

    def expand_parameters_capacity_time(self, lp_solver, type="capacity"):
        """Expand parameters (time/capacity buckets) for sufficiency. Use the same function for capacity and time. """
        # Apply expansion logic for WD_u or WT_u based on the type
        target_dict = self.WD_u if type == "capacity" else self.WT_u
        
        # Get the positive zD and zT values from the lp relaxation.
        new_W_u = lp_solver.get_positive_z_capacity_time(type)

        for u in new_W_u:
            if u in target_dict:  # Ensure `u` exists in the target dictionary
                for value in new_W_u[u]:
                    if value not in target_dict[u]:  # Check if value is not in the target dictionary
                        target_dict[u].append(value)  # Add the value if it doesn't exist
                        self.is_parameterized_unchanged = False # parameterized has been changed
                target_dict[u].sort()  # Sort the list in ascending order
            else:
                # If `u` is not in the target dictionary, initialize it with the sorted values from `new_W_u[u]`
                target_dict[u] = sorted(new_W_u[u])
                self.is_parameterized_unchanged = False # parameterized has been changed

    def run(self):
        """Execute Algorithm 1: LA-Discretization algorithm."""

        self.lp_time = 0
        self.total_iters = 0
        while True:
            self.is_parameterized_unchanged = True

            # Line 5-7
            if self.iter_since_reset >= self.zeta:
                for u in range(1, self.instance.num_customers + 1):
                    self.N_u[u] = calculate_neighbors(self.instance, self.ns, u)
                self.is_parameterized_unchanged = False

            # Line 8: given N_u, T_u, D_u for all u \in N, solve the LP relaxation and get [z, y, x, pi, lp_objective]
            lp_objective, lp_time, lp_solver, _, self.variable_basis, self.constraint_basis = self.solve_lp_relaxation()
            self.lp_time += lp_time
            self.total_iters += 1

            print ('---------')
            print ('-----iteration-------', self.iter_since_reset, self.total_iters)
            print ('---lp_objective > self.last_lp_val + self.min_inc: ', lp_objective, self.last_lp_val, self.min_inc)
            print ('---------')
            if lp_objective > self.last_lp_val + self.min_inc:
                # Lines 10-12: Contract parameters for parsimony
                self.contract_parameters(lp_solver)
                self.last_lp_val = lp_objective
                self.iter_since_reset = 0
                self.is_parameterized_unchanged = False

            # Lines 16-17: Expand parameters for sufficiency
            self.expand_parameters(lp_solver)
            
            self.iter_since_reset += 1

            # Line 19: Termination conditions 
            print ('-----is_parameterized_unchanged-------', self.is_parameterized_unchanged)
            if self.iter_since_reset > self.iter_max or self.is_parameterized_unchanged:
                break

        # Line 20:
        self.contract_parameters(lp_solver)

        # Line 21: Solve final MILP using the parameterized LP
        print ('-----solve_final_milp-------')
        final_obj, ilp_time, mip_dual_bound = self.solve_final_milp()
        return final_obj, ilp_time, lp_objective, mip_dual_bound

    def solve_final_milp(self):
        """Solve the final MILP with the generated parameterization."""
        lp_objective, ilp_time, _, mip_dual_bound, _, _ = self.solve_lp_relaxation(False)
        return lp_objective, ilp_time, mip_dual_bound

