"""
Filename: LocalAreaArcs.py
Author: Yihe Zhuo
Last Modified: 2025-01-06
Description: 
    Implementation of Section 6, Local Area Arc Computation, and Section 4.1/4.5.2 LA Neighborhood
"""

import gurobipy as gp
from gurobipy import GRB
import itertools

class LocalAreaArcs:
    """
    Class to compute and enforce Local Area Movement constraints (6g), (8a), and (8b) for each u as an attribute.
    """
    def __init__(self, model, instance, E_star, N_u, x):
        """
        Initialize the LocalAreaMovement class.

        Args:
            model: Gurobi model where constraints will be added.
            instance: VRPTWInstance containing problem data.
            x: Binary decision variables for routes (from VRPTWSolver).
            y: Decision variables for LA arcs.
        """
        self.model = model
        self.instance = instance
        self.E_star = E_star
        self.N_u = N_u
        self.x = x  # Binary variables for (u, v)
        self.y = None  # Continuous variables for orderings
        self.tiny_val = 0.0001  # Epsilon in (8a) and (8b)
        self.R_u_plus = {}  # R+_u, super set of  the efficient frontier
        self.R_u = {}  # R_u, efficient frontier computed by Section 6
        self.E_star_neighbor = {}   # Defined in Section 4.1
        self.E_star_neighbor_uw = {}    # Defined in Section 4.1

        # Used in Constraints (2) and (3)
        self.a_wvr = {}  # For immediate precedence
        self.a_w_star_r = {}  # For final customer in the ordering

        # Used in Constraints (8) and (9)
        self.LA_neighbor_size = {}
        self.ak_wvr = {}  # For immediate precedence
        self.ak_wr = {}
        self.ak_w_star_r = {}  # For final customer in the ordering


    def generate_orderings(self, u):
        """Generate all feasible orderings to construct R+_u."""
        return [[u]] + [
            [u] + list(ordering)  # Convert tuple to list
            for subset_length in range(1, len(self.N_u[u]) + 1)
            for subset in itertools.combinations(self.N_u[u], subset_length)
            for ordering in itertools.permutations(subset)
            if self.is_feasible_ordering(u, ordering)
        ]
 
    def compute_local_area_arcs(self):
        """
        Compute Local Area arcs (LA_arcs) and feasible orderings (R_u) for a specific customer u.

        Args:
            u: The specific customer for which local area arcs and feasible routes are computed.
        """
        for u in range(1, self.instance.num_customers + 1):
            self.R_u_plus[u] = self.generate_orderings(u)

    def generate_efficient_frontier(self):
        """ Implementation of Algorithm 2 in Section 6.4: Computing the Efficient Frontier for all p ∈ P """

        time_windows = self.instance.time_windows
        due_time = self.instance.depot["due_time"]

        # Generate P+ and initialize R_p using the base case (i.e., r = [u_p, v_p]).
        self.p_list, R = self.generate_p()
        
        # Initialize the dictionary
        total_dist_r = {}   # c_r
        earliest_r = {} # phi_r
        latest_r = {}   # phi_hat_r

        for p in self.p_list:
            N_p = p[1:-1]
            for w in N_p:
                # Line 3
                p_hat = [w] + [node for node in N_p if node != w] + [p[-1]]
                for r_minus in R.get(tuple(p_hat), []):
                    # Line 4: r ← [up,r−]
                    r = [p[0]] + r_minus

                    # Line 5: Compute cr, and φr, φˆr via (13)
                    earliest_r[tuple(r)], latest_r[tuple(r)] = self.calculate_earlist_latest_leaving_time(r)
                    total_dist_r[tuple(r)] = self.calculate_total_distance(r)

                    # Lines 6-8
                    if latest_r[tuple(r)] <= due_time - time_windows[p[0]-1][0]:
                        R.setdefault(tuple(p), []).append(r)

            # if r ∈ Rp then r− must lie in Rpˆ
            if tuple(p) not in R:
                continue

            for r in R[tuple(p)]:
                dominated = False
                for r_hat in [ordering for ordering in R[tuple(p)] if ordering != r]:
                    # Line 11
                    if (total_dist_r[tuple(r)] >= total_dist_r[tuple(r_hat)] 
                                and latest_r[tuple(r)] >= latest_r[tuple(r_hat)] 
                                and earliest_r[tuple(r)] - total_dist_r[tuple(r)] <= earliest_r[tuple(r_hat)] - total_dist_r[tuple(r_hat)]):
                        # Only when at least one inequality is strict that we remove r from efficient frontier.
                        if not (total_dist_r[tuple(r)] == total_dist_r[tuple(r_hat)] 
                                and latest_r[tuple(r)] == latest_r[tuple(r_hat)] 
                                and earliest_r[tuple(r)] - total_dist_r[tuple(r)] == earliest_r[tuple(r_hat)] - total_dist_r[tuple(r_hat)]):
                            dominated = True
                            break
                if dominated:
                    R[tuple(p)].remove(r)

        # To construct Ru we then take the union of all terms in R for all v ∈ N⊙→, Nˆ ⊆ N⊙ then finally clip v from the end.
        self.R_u = self.clip_R_u(R)
        for u in self.R_u:
            self.R_u[u].append([u])

    def clip_R_u(self, R):
        """ Clip v from the end. """
        R_u = {}

        for p, routes in R.items():
            key = p[0]  # First element of p
            if key not in R_u:
                R_u[key] = set()  # Use a set to avoid duplicates
            for route in routes:
                if len(route) > 2:
                    R_u[key].add(tuple(route[:-1]))  # Add route without last element as a tuple

        # Convert sets to lists. R_u must be a subset of R_u_plus.
        R_u = {key: [list(value) for value in values if list(value) in self.R_u_plus[key]] for key, values in R_u.items()}

        return R_u

    def calculate_total_distance(self, ordering):
        """ Calcualte c_r. """

        customers = self.instance.customers
        depot = self.instance.depot
        dist = 0
        for i in range(1, len(ordering)):
            u = ordering[i-1]
            v = ordering[i]
            dist += self.instance.compute_distance(customers[u-1] if u > 0 else depot, customers[v-1] if v > 0 else depot)
        return dist
                

    def calculate_earlist_latest_leaving_time(self, ordering):
        """
        Calculate ϕr and ϕ̂r for a given arc sequence recursively. Implementation of Constraints (13) in Section 6.2.

        Parameters:
            ordering (list): An ordered list of customers defining the arc sequence (u1, u2, ..., vr).

        Returns:
            ϕr  -> earliest time that a vehicle could leave ur1 without waiting at any customer considering no waiting.
            ϕ̂r -> latest time that a vehicle could leave ur1 if t+ terms were ignored.
            TODO: need further clarification here. Do we need to consider service time at u as well?
        """
        time_windows = self.instance.time_windows
        customers = self.instance.customers
        due_time = self.instance.depot["due_time"]
        depot = self.instance.depot

        if len(ordering) == 2:
            # Base case: direct travel from u to v
            u, v = ordering
            t_minus_u, t_plus_u = due_time - time_windows[u-1][1], due_time - time_windows[u-1][0]
            t_minus_v, t_plus_v = due_time - time_windows[v-1][1], due_time - time_windows[v-1][0]
            t_uv = self.instance.compute_distance(customers[u-1] if u > 0 else depot, customers[v-1] if v > 0 else depot)
            earliest_r = min(t_plus_u, t_plus_v + t_uv)
            latest_r = max(t_minus_u, t_minus_v + t_uv)
        else:
            # Recursive case
            u = ordering[0]
            next_ordering = ordering[1:]
            
            t_minus_u, t_plus_u = due_time - time_windows[u-1][1], due_time - time_windows[u-1][0]
            next_earliest_r, next_latest_r = self.calculate_earlist_latest_leaving_time(next_ordering)
            t_uw = self.instance.compute_distance(customers[u-1] if u > 0 else depot, customers[next_ordering[0]-1] if next_ordering[0] > 0 else depot)
            earliest_r = min(t_plus_u, next_earliest_r + t_uw)
            latest_r = max(t_minus_u, next_earliest_r + t_uw)

        return earliest_r, latest_r

    def generate_p(self):
        """ P+ be a super-set. """

        p_list = []
        R = {}

        for u, n_list in self.R_u_plus.items():
            for n in n_list:
                if len(n) <= 1:
                    neighbors = self.E_star_neighbor_uw[u].get(u, [])
                    if neighbors:
                        R.update({neighbor: [list(neighbor)] for neighbor in neighbors})
                    continue
                last_node = n[-1]  # Get the last element in n

                neighbors = self.E_star_neighbor_uw[u].get(last_node, [])
                
                if not neighbors:
                    continue
                neighbors = [v for _, v in neighbors]

                # Add tuples for all neighbors of last_node
                p_list += [n + [v] for v in neighbors]
        p_list = sorted(p_list, key=len)
        return p_list, R

    def is_feasible_ordering(self, u, ordering):
        """
        Check if a given ordering (u -> neighbors in ordering) is feasible based on time and capacity constraints.

        Args:
            u: The starting customer.
            ordering: A tuple representing the complete ordering to check.

        Returns:
            bool: True if the ordering is feasible, False otherwise.
        """
        if not self.is_capacity_feasible(u, ordering):
            return False
        if not self.is_time_feasible(u, ordering):
            return False
        return True

    def is_capacity_feasible(self, u, ordering):
        """
        Check if the total demand of an ordering (u -> neighbors in ordering) does not exceed vehicle capacity.

        Args:
            u: The starting customer.
            ordering: A tuple representing the complete ordering to check.

        Returns:
            bool: True if the ordering is capacity feasible, False otherwise.
        """
        demands = self.instance.demands
        vehicle_capacity = self.instance.vehicle_capacity

        total_demand = demands[u - 1] + sum(demands[v - 1] for v in ordering)
        return total_demand <= vehicle_capacity

    def is_time_feasible(self, u, ordering):
        """
        Check if the time constraints are satisfied for an ordering (u -> neighbors in ordering).

        Args:
            u: The starting customer.
            ordering: A tuple representing the complete ordering to check.

        Returns:
            bool: True if the ordering is time feasible, False otherwise.
        """
        depot = self.instance.depot
        customers = self.instance.customers
        time_windows = self.instance.time_windows
        service_times = self.instance.service_times

        current_time = max(time_windows[u - 1][0], self.instance.compute_distance(depot, customers[u - 1]))

        for i, v in enumerate(ordering):
            previous = u if i == 0 else ordering[i - 1]
            travel_time = self.instance.compute_distance(customers[previous - 1], customers[v - 1])
            # Check if the arrival time at v is within the time window
            current_time = max(time_windows[v - 1][0], current_time + service_times[previous - 1] + travel_time)
            if current_time > time_windows[v - 1][1]:
                return False
            
        return True

    def generate_ordering_indicators(self):
        """ Generate the indicators a_wvr and a_w*r for the given orderings R_u."""
        for u in range(1, self.instance.num_customers + 1):
            self.generate_ordering_indicators_original(u)   # a_wvr and a_w*r in (2a)-(2c)
            self.generate_ordering_indicators_by_k(u)   # ak_wvr and ak_w*r in (8a)-(8b). This function needs a_wvr and a_w*r from above.

    def generate_ordering_indicators_original(self, u):
        """
        Generate the indicators a_wvr and a_w*r for the given orderings R_u.

        Args:
            u: customer index.

        Returns:
            dict: A dictionary with keys:
                - "a_wvr": A dictionary where a_wvr[(w, v, r)] = 1 if w immediately precedes v in ordering r, 0 otherwise.
                - "a_w*r": A dictionary where a_w*r[(w, r)] = 1 if w is the final customer in ordering r, 0 otherwise.
        """
        # Initialize a_w_star_r
        for w in self.E_star_neighbor_uw[u]:
            for r in self.R_u[u]:
                r_name = "_".join(map(str, r))
                self.a_w_star_r.setdefault(u, {})[(w, r_name)] = 0
                

        for r_idx, r in enumerate(self.R_u[u]):
            # Iterate over the ordering to populate a_wvr
            r_name = "_".join(map(str, r))

            # Initialize self.a_wvr to 0
            for w, v in self.E_star_neighbor[u]:
                self.a_wvr.setdefault(u, {})[(w, v, r_name)] = 0

            for i in range(len(r)):
                w = r[i]  # Current customer
                if i < len(r) - 1:
                    v = r[i + 1]  # Next customer in the ordering
                    self.a_wvr[u][(w, v, r_name)] = 1  # w precedes v in ordering r

                # Handle the final customer case
                if i == len(r) - 1:
                    self.a_w_star_r[u][(w, r_name)] = 1  # w is the final customer
                else:
                    self.a_w_star_r[u][(w, r_name)] = 0  # w is not the final customer

    def get_k_for_R_u(self, u):
        """
        Implementation of Equation (9). Compute the corresponding k for each list in R_u based on N_u.
        
        Args:
        - u: customer
        
        Returns:
        - dict: A dictionary mapping each list in R_u (as a tuple) to its k value.
        """
        k_values = {}
        for r in self.R_u[u]:
            k_indices = [self.N_u[u].index(elem) + 1 for elem in r if elem in self.N_u[u]]
            k = max(k_indices) if k_indices else 0
            k_values["_".join(map(str, r))] = k  # Store the result with r as a tuple key
        return k_values
    
    def generate_ordering_indicators_by_k(self, u):
        """
        Generate the indicators ak_wr, ak_wvr and ak_w*r for the given orderings R_u.

        Args:
            u: for customer u.

        Returns:
            dict: A dictionary with keys:
                - "ak_wvr": A dictionary where ak_wvr[(w, v, r)] = 1 if w immediately precedes v in ordering r, 0 otherwise.
                - "ak_w*r": A dictionary where ak_w*r[(w, r)] = 1 if w is the final customer in ordering r, 0 otherwise.
        """
        # |N_u|
        self.LA_neighbor_size[u] = len(self.N_u[u]) #max(len(sublist) for sublist in self.R_u[u])

        # Initialize ak_wr
        self.ak_wr[u] = {
            k: {(w, "_".join(map(str, r))): 0 for w in self.N_u[u] + [u] for r in self.R_u[u]}
            for k in range(1, self.LA_neighbor_size[u]+1)
        }

        # Initialize ak_wvr
        self.ak_wvr[u] = {
            k: {(w, v, "_".join(map(str, r))): 0 for w,v in self.E_star_neighbor[u] for r in self.R_u[u]}
            for k in range(1, self.LA_neighbor_size[u]+1)
        }

        # Initialize ak_w_star_r
        self.ak_w_star_r[u] = {k: {} for k in range(1, self.LA_neighbor_size[u]+1)}

        # Set ak_wr, ak_wvr
        k_values = self.get_k_for_R_u(u)
        for r in self.R_u[u]:
            if len(r) == 1:
                continue

            r_name = "_".join(map(str, r))  # Compute r_name once
            k = k_values[r_name]

            # Update ak_wr
            self.ak_wr[u][k].update({(r[i], r_name): 1 for i in range(len(r))})

            # Update ak_wvr
            for w, v in self.E_star_neighbor[u]:
                self.ak_wvr[u][k][(w, v, r_name)] = self.a_wvr.get((w, v, r_name), 0)

        # Update ak_w_star_r
        for k, ak_wr_items in self.ak_wr[u].items():
            for (w, r_name), val in ak_wr_items.items():
                # Check and set ak_w_star_r[k][(w, r_name)] if the condition is met
                if val == 1:
                    if self.a_w_star_r[u].get((w, r_name)) == 1:
                        self.ak_w_star_r[u][k][(w, r_name)] = 1
                        continue  # Skip further checks for this (w, r_name)

                    # Check for v in N_u[k+1:] and compute 'a'
                    valid_v = (v for v in self.N_u[u][k + 1:] if v not in self.N_u[u][:k] or v != w)
                    if sum(self.a_wvr[u].get((w, v, r_name), 0) for v in valid_v) == 1:
                        self.ak_w_star_r[u][k][(w, r_name)] = 1
        
    def compute_E_star_neighbor(self):
        for u in range(1, self.instance.num_customers + 1):
            self.E_star_neighbor[u], self.E_star_neighbor_uw[u] = self.compute_E_star_neighbor_u(u)
        return self.E_star_neighbor, self.E_star_neighbor_uw

    def compute_E_star_neighbor_u(self, u):
        """
        Compute E_star_neighbor[u] for a given customer u based on the orderings in R_u[u].

        Args:
            u (int): The specific customer for which E_star_neighbor[u] is computed.

        Returns:
            None: self.E_star_neighbor[u].
        """
        E_star_neighbor = set()  # Initialize as an empty set
        E_star_neighbor_uw = {}

        for r in self.R_u_plus[u]:
            # if r = [u], no a_wvr will be created
            if len(r) == 1:
                continue
            # Iterate through the ordering to generate (w, v) pairs for a_wvr
            for i in range(len(r)-1):
                w = r[i]  # w is either u (for the first position) or the previous node
                v = r[i + 1]  # v is the current node in the ordering

                # Check if (w, v) meets the criteria
                if (
                    w != v  # w and v are distinct
                ) and (
                    (w, v) in self.E_star  # (w, v) is part of E_star
                ):
                    E_star_neighbor.add((w, v))  # Add the valid pair to the set

        # Collect all final customers w from R_u[u]
        final_customers = {r[-1] for r in self.R_u_plus[u]}
        # Filter edges in E_star
        for w in final_customers:
            E_star_neighbor_uw[w] = []  # Initialize as an empty list
            for (w_edge, v) in self.E_star:  # Use a different variable name for the inner loop
                if w_edge == w and (v not in self.N_u[u] and v != 0 and v != u):  # Ensure w_edge matches the outer loop w
                    E_star_neighbor_uw[w].append((w_edge, v))  # Append the pair (w_edge, v)

        return list(E_star_neighbor), E_star_neighbor_uw

    def add_variables(self):
        self.y = {}  # Initialize a dictionary to store variables for all customers

        for u in range(1, self.instance.num_customers + 1):
            r_names = ["_".join(map(str, r)) for r in self.R_u[u]]

            # Add all variables at once
            self.y[u] = self.model.addVars(
                r_names, lb=0, vtype=GRB.BINARY, name=f"y_u{u}"
            )

    """ functions add_constraints() and add_constraints() are replaced by Constraints (8) and not in use. """
    """ 
    def add_constraints(self):
        # Add constraints (6g), (6h), and (6i) for a specific customer u. 
        for u in range(1, self.instance.num_customers + 1):
            self.add_constraints_u(u)

    def add_constraints_u(self, u):
        # Constraint (6g): LA-arc movement consistency for u
        self.model.addConstr(
            gp.quicksum(self.y[r] for r in self.y) == 1,
            name=f"LA_arc_one_ordering_u{u}"
        )

        # Constraint (6h): Linking x_wv with y_r for u
        self.model.addConstrs(
            (
                self.x[w, v] >= gp.quicksum(
                    self.a_wvr[u][(w, v, r)] * self.y[r] for r in self.y
                )
                for w, v in self.E_star_neighbor[u]
            ),
            name=f"linking_xwv_inner_y_u{u}"
        )

        # Constraint (6i): Linking x_wv with ordering variables for u
        self.model.addConstrs(
            (
                gp.quicksum(
                    self.x[w, v] #for v in range(1, self.instance.num_customers + 1) if v not in self.N_u[u]
                    for w,v in self.E_star_neighbor_uw[u][outer_w]
                ) >= gp.quicksum(
                    self.a_w_star_r[u][(outer_w, r)] * self.y[r] for r in self.y
                )
                for outer_w in self.E_star_neighbor_uw[u]
            ),
            name=f"linking_xwv_outer_y_u{u}"
        )
        """

    """ Constraints 8 in use in the algorithm. """
    def add_constraints_parsimony(self):
        """
        Add constraints (6g), (8a), and (8b) for a specific customer u.
        """
        for u in range(1, self.instance.num_customers + 1):
            # Constraint (6g): LA-arc movement consistency for u
            self.model.addConstr(
                gp.quicksum(self.y[u][r] for r in self.y[u]) == 1,
                name=f"LA_arc_one_ordering_u{u}"
            )

            self.add_constraints_parsimony_u(u, range(1, self.LA_neighbor_size[u]+1))

    def add_constraints_parsimony_u(self, u, k_range):
        """
        Add constraints (8a), and (8b) for a specific customer u.

        Args:
            u: The specific customer for which constraints are added.
        """
        # Constraint (8a): Linking x_wv with y_r for u
        self.model.addConstrs(
            (
                self.tiny_val * k + self.x[w, v] >= gp.quicksum(
                    self.ak_wvr[u][k][(w, v, r)] * self.y[u][r] for r in self.y[u] if (w, v, r) in self.ak_wvr[u][k]
                )
                for k in k_range
                for w, v in self.E_star_neighbor[u] if (v in self.N_u[u][:k] and w in self.N_u[u][:k] + [u])
            ),
            name=f"linking_xwv_inner_y_u{u}"
        )

        # Constraint (8b): Linking x_wv with ordering variables for u
        self.model.addConstrs(
            (
                self.tiny_val * k + gp.quicksum(
                    self.x[w, v] #for v in range(1, self.instance.num_customers + 1) if v not in self.N_u[u]
                    for w,v in self.E_star_neighbor_uw[u][outer_w] if (v not in self.N_u[u][:k])
                ) >= gp.quicksum(
                    self.ak_w_star_r[u][k].get((outer_w, r), 0) * self.y[u][r] for r in self.y[u]
                )
                for k in k_range
                for outer_w in self.E_star_neighbor_uw[u] if outer_w in self.N_u[u][:k] 
            ),
            name=f"linking_xwv_outer_y_u{u}"    # TODO: k = 0? i.e., w = u?
        )

    def get_affected_customers(self, N_u):
        """
        Identify customers whose `LA_neighbor_size` does not match the length of `N_u`.
        Updates `self.LA_neighbor_size` to reflect the new lengths of `N_u`.

        Returns:
            dict: A dictionary of affected customers where the key is the customer index `u`
                and the value is a tuple `(len(N_u[u]), self.LA_neighbor_size[u])`.
        """
        affected_customers = {}

        for u in range(1, self.instance.num_customers + 1):
            current_size = len(N_u[u])
            if current_size != self.LA_neighbor_size[u]:
                # Add the difference to the affected customers
                affected_customers[u] = (self.LA_neighbor_size[u], current_size)

                # Update self.LA_neighbor_size[u] to match the current size
                self.LA_neighbor_size[u] = current_size

        return affected_customers

    def update_constraints_parsimony(self, affected_customers):
        """
        Dynamically update constraints (6g), (8a), and (8b) for customers `u`
        in `affected_customers` based on the difference between `old_k` and `new_k`.

        Args:
            affected_customers (dict): A dictionary where the key is the customer `u` and
                                    the value is a tuple `(old_k, new_k)`, indicating the
                                    difference between the old and new `k` values.
        """
        def add_constraints(u, k_range):
            """
            Add constraints (8a) and (8b) for a given customer `u` and range of `k` values.

            Args:
                u (int): The customer index.
                k_range (iterable): The range of `k` values to consider.
            """
            self.add_constraints_parsimony_u(u, k_range)

        def remove_constraints(u, k_range, prefix):
            for k in k_range:
                # Iterate through all constraints in the model
                for constr in self.model.getConstrs():
                    constr_name = constr.ConstrName
                    if constr_name.startswith(f"{prefix}{u}[{k}"):
                        self.model.remove(constr)
                        #print(f"Removed constraint: {constr_name}")

                        
        # Process each affected customer
        for u, (old_k, new_k) in affected_customers.items():
            if old_k < new_k:
                # Add constraints for k = old_k + 1 to new_k
                add_constraints(u, range(old_k + 1, new_k + 1))
            elif old_k > new_k:
                # Remove constraints for k = new_k + 1 to old_k
                remove_constraints(u, range(new_k + 1, old_k + 1), prefix="linking_xwv_inner_y_u")
                # Example usage for linking_xwv_outer constraints
                remove_constraints(u, range(new_k + 1, old_k + 1), prefix="linking_xwv_outer_y_u")

        # Update the model to reflect changes
        self.model.update()
        #print(f"LA Constraints updated for affected customers: {affected_customers}")


