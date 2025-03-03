"""
Filename: VRPTWSolverAlgo.py
Author: Yihe Zhuo
Last Modified: 2025-01-06
Description: 
    Implementation of the MILP in Section 4.3 Complete Linear Programming Relaxation. 
    Its LP relaxation is solved to determined the parameterization, and it's solved as MILP in the final step of the algorithm.
"""

from VRPTWSolver import VRPTWSolver  # Assuming the class is saved in a file named vrptw_solver.py
from FlowGraphCapacity import FlowGraphCapacityTime
from LocalAreaArcs import LocalAreaArcs
import ast, time
from gurobipy import GRB
from Util import customers_need_update

class VRPTWSolverAlgo(VRPTWSolver):
    """Solver for the MILP model in Section 4.3."""
    def __init__(self, instance, N_u, WD_u, WT_u, var_basis, con_basis):
        super().__init__(instance)  # Inherit initialization from VRPTWSolver
        self.N_u = N_u
        self.WD_u = WD_u
        self.WT_u = WT_u
        self.local_area_movement = None
        self.E_star_neighbor_u = {}
        self.E_star_neighbor_uw = {}
        self.capacity_E_graph = None
        self.time_E_graph = None
        self.capacity_pi = {}
        self.time_pi = {}
        self.LA_pi = {}
        self.var_basis = var_basis
        self.con_basis = con_basis

    def setup_variables(self):
        """Override to add constraints specific to Section 4.3."""
        super().setup_variables()  # Retain base class variables

    def setup_constraints(self):
        """Override to add constraints specific to Section 4.3."""
        super().setup_constraints()  # Retain base class constraints
        self.add_capacity_time_discretization_constraints()  
        self.lam = self.add_LA_arc_constraints(update=False)
        """ 
        self.model.addConstr(
            #(self.x[2,6] + self.x[6,7] + self.x[7,8] + self.x[8,5] + self.x[5,3] + self.x[3,1] + self.x[1,4] == 7),
            (self.x[12,14] + self.x[14,16] + self.x[16,15] + self.x[15,11] + self.x[11,9] + self.x[9,10] + self.x[10,13] + self.x[13,17] == 8),
            name="enforce"
        ) """

    def update(self, N_u, WD_u, WT_u):
        """
        Update parameters and make necessary changes to the model.
        """
        #update_z_list = set(customers_need_update(self.WD_u, WD_u) + customers_need_update(self.WT_u, WT_u))
        
        self.N_u = N_u
        self.WD_u = WD_u
        self.WT_u = WT_u
        affected_customers_LA = self.lam.get_affected_customers(self.N_u)
        self.lam.update_constraints_parsimony(affected_customers_LA)
        #print ('----update-cleanup_z-add_capacity_time_discretization_constraints')
        
        self.cleanup_z()
        self.add_capacity_time_discretization_constraints()

    def cleanup_z(self):
        """
        Remove all declared variables and constraints based on their names.
        Variables and constraints are identified using their naming conventions.
        """
        # Remove variables based on their prefix
        variable_prefixes = ["z_D", "z_T"]  # Variable prefixes for self.z
        for var in self.model.getVars():
            if any(var.varName.startswith(prefix) for prefix in variable_prefixes):
                self.model.remove(var)
                #print(f"Removed variable: {var.varName}")

        # Remove constraints based on their naming conventions
        constraint_prefixes = [
            "capacity_flow_conservation_",
            "time_flow_conservation_",
            "xz_capacity_consistency_",
            "xz_time_consistency_"
        ]

        for constr in self.model.getConstrs():
            if any(constr.ConstrName.startswith(prefix) for prefix in constraint_prefixes):
                self.model.remove(constr)
                #print(f"Removed constraint: {constr.ConstrName}")

        # Update the model to reflect changes
        self.model.update()
        print("All declared variables and constraints have been removed.")

    def add_LA_arc_constraints(self, update=True):
        # Add Local Area Movement constraints for each u
        lam = LocalAreaArcs(self.model, self.instance, self.E_star, self.N_u, self.x)
        lam.compute_local_area_arcs()  # Compute LA arcs for u

        self.E_star_neighbor_u, self.E_star_neighbor_uw = lam.compute_E_star_neighbor()
        #lam.generate_p()
        R = lam.generate_efficient_frontier()
        """ 
        print ('------')
        print ('R_plus length: ', {u: len(lam.R_u_plus[u]) for u in lam.R_u_plus})
        print ('------')
        print ('R length: ', {u: len(lam.R_u[u]) for u in lam.R_u})
        """
        # Generate the indicators
        lam.generate_ordering_indicators()
        lam.add_variables()   
        lam.add_constraints_parsimony()       

        return lam
        
    def setup_objective(self):
        """Override the objective function if needed."""
        super().setup_objective()  # Retain base objective or modify here
        # Example: Minimize costs with additional penalties
        # TODO: review the obj

    def add_capacity_time_discretization_constraints(self):
        
        # Call the secondary class FlowGraphCapacity to add constraints (6j) and (6k)
        capacity_model = FlowGraphCapacityTime(self.model, self.x, self.instance, self.WD_u, 'capacity')
        capacity_model.run_flow_graph()
        self.capacity_E_graph = capacity_model.E_graph
        
        # Call the secondary class FlowGraphCapacity to add constraints (6j) and (6k)
        
        time_model = FlowGraphCapacityTime(self.model, self.x, self.instance, self.WT_u, 'time')
        time_model.run_flow_graph()
        self.time_E_graph = time_model.E_graph
    
    def fetch_duals_capacity_time(self, type='capacity'):
        """
        Group flow‐conservation constraints by node[0], and within each node[0],
        group those whose .Pi are identical (up to some chosen rounding).
        
        This version parses the node from the constraint name instead of relying on
        (constr, node) pairs in flow_constraints.
        """
        dual_groups = {}

        # Look at *all* constraints in the model
        for c in self.model.getConstrs():
            # Only process those named like "capacity_flow_conservation_(..., ..., ...)"
            if c.ConstrName.startswith(f'{type}_flow_conservation'):

                # Extract the substring after 'capacity_flow_conservation_'
                # e.g. "(98, 66.0, 70.0)"
                node_str = c.ConstrName[len(f'{type}_flow_conservation_'):]
                # Convert that substring to a Python tuple
                node = ast.literal_eval(node_str)  # node = (98, 66.0, 70.0), for example

                # Get the .Pi (dual) value
                pi_val_raw = c.Pi
                pi_val = round(pi_val_raw, 6)

                # node_0 is the first element of the tuple
                node_0 = node[0]

                # Create a sub-dict if we haven't seen this node_0 before
                if node_0 not in dual_groups:
                    dual_groups[node_0] = {}
                
                # Group by the (rounded) Pi value
                if pi_val not in dual_groups[node_0]:
                    dual_groups[node_0][pi_val] = []
                
                dual_groups[node_0][pi_val].append(node)
                
        return dual_groups

    def store_positive_duals_for_ku(self):
        """
        Compute and store the sum of positive dual variables for each u and k.
        """
        dual_sums = {}  # Dictionary to store the results: {u: {k: sum_of_positives}}

        # Look at all constraints in the model
        for c in self.model.getConstrs():

            # Process linking_xwv_inner_y_u constraints
            if c.ConstrName.startswith('linking_xwv_inner_y_u'):
                # Only consider positive dual variables
                if c.Pi > 0:
                    print ('----positive c.Pi: ', c.ConstrName, c.Pi)
                    
                    # Parse constraint name
                    name_parts = c.ConstrName.split('linking_xwv_inner_y_u')[1]
                    u_str, kwv_str = name_parts.split('[')
                    u = int(u_str)
                    k, w, v = map(int, kwv_str.rstrip(']').split(','))
                    
                    if u not in dual_sums:
                        dual_sums[u] = {}
                    if k not in dual_sums[u]:
                        dual_sums[u][k] = 0
                    dual_sums[u][k] += c.Pi

            # Process linking_xwv_outer_y_u constraints
            elif c.ConstrName.startswith('linking_xwv_outer_y_u'):
                # Only consider positive dual variables
                if c.Pi > 0:
                    print ('----positive c.Pi: ', c.ConstrName, c.Pi)

                    # Parse constraint name
                    name_parts = c.ConstrName.split('linking_xwv_outer_y_u')[1]
                    u_str, kw_str = name_parts.split('[')
                    u = int(u_str)
                    k, w = map(int, kw_str.rstrip(']').split(','))

                    if u not in dual_sums:
                        dual_sums[u] = {}
                    if k not in dual_sums[u]:
                        dual_sums[u][k] = 0
                    dual_sums[u][k] += c.Pi

        return dual_sums

    def post_process_duals(self, dual_groups, type='capacity'):
        """
        For each customer u in dual_groups:
        - For each pi_val in dual_groups[u], sort the corresponding list of nodes
            by node[1] (d_min) in ascending order.
        - For each consecutive pair (node_i, node_j) in the sorted list, if
            node_i[2] + self.step_size == node_j[1], remove node_i[2] from self.WD_u[u].
        """
        step_size = 1 if type=='capacity' else 0.0001
        data_dict = self.WD_u if type == 'capacity' else self.WT_u
        edge_set = set(self.capacity_E_graph if type == 'capacity' else self.time_E_graph)
        
        for u, pi_dict in dual_groups.items():
            customer_data = data_dict.get(u, set())  # Cache customer-specific data

            for pi_val, node_list in pi_dict.items():
                # Sort by node[1], i.e., d_min
                node_list.sort(key=lambda x: x[1])

                # Loop over consecutive pairs
                for node_current, node_next in zip(node_list, node_list[1:]):
                    # Extract d_max and d_min
                    d_max_current = node_current[2]
                    d_min_next = node_next[1]

                    # Ensure the pair exists in the graph
                    #if (node_current, node_next) not in edge_set:
                        #continue
                    if d_max_current + step_size == d_min_next:
                        # Remove d_max_i from self.WD_u[u] if present
                        if d_max_current in customer_data:
                            customer_data.remove(d_max_current)
            # Update the original data_dict for this customer
            data_dict[u] = customer_data

    def get_positive_z_capacity_time(self, type='capacity'):
        """
        Get positive variables based on the specified type ('capacity' or other).
        
        Args:
        - type (str): Determines the type of variables to process. 
                    'capacity' uses 'z_D' and self.instance.demands, 
                    otherwise uses 'z_T' and self.instance.compute_distance.
        
        Returns:
        - dict: A dictionary with u_j as keys and computed values as values.
        """
        customers = self.instance.customers
        service_times = self.instance.service_times
        demands = self.instance.demands
        depot = self.instance.depot

        # Determine variable prefix and computation logic based on type
        if type == 'capacity':
            variable_prefix = 'z_D'
            compute_value = lambda u_i, u_j, dmax_i, dmax_j: dmax_i - demands[u_i - 1]
        else:
            variable_prefix = 'z_T'
            # remaining time = depot["due_time"] - start_time. Remaining time - transportation time between ui, uj => ui max start time + transportation time between ui, uj
            compute_value = lambda u_i, u_j, dmax_i, dmax_j: round(min(dmax_i - self.instance.compute_distance(customers[u_i-1], customers[u_j-1]) - service_times[u_j - 1], dmax_j), 1)

        # Get all variables with a value > 0 and matching the variable type
        positive_z = [
            var.VarName for var in self.model.getVars()
            if var.VarName.startswith(variable_prefix) and var.X > 0
            and (lambda u_i, u_j: u_j != -1 and u_i != u_j)(
                int(var.VarName.split('[(')[1].split('),(')[0].split(',')[0].strip()),  # Extract u_i
                int(var.VarName.split('[(')[1].split('),(')[1].split(',')[0].strip())   # Extract u_j
            )
        ]

        # Initialize the dictionary to store the results
        new_W_u = {}

        # Iterate over the filtered variables
        for var in positive_z:
            # Extract the tuple from the variable name
            data = var.split(f"{variable_prefix}[")[1].rstrip("]").split("),(")
            u_i_data = tuple(map(float, data[0].strip("()").split(", ")))
            u_j_data = tuple(map(float, data[1].strip("()").split(", ")))

            u_i, dmin_i, dmax_i = int(u_i_data[0]), u_i_data[1], u_i_data[2]
            u_j, dmin_j, dmax_j = int(u_j_data[0]), u_j_data[1], u_j_data[2]
            #print('u_i, dmin_i, dmax_i, u_j, dmin_j, dmax_j:', u_i, dmin_i, dmax_i, u_j, dmin_j, dmax_j)
            
            # Compute the value based on the selected type
            value = compute_value(u_i, u_j, dmax_i, dmax_j)
            
            # Update the dictionary
            if u_j not in new_W_u:
                new_W_u[u_j] = set()
            new_W_u[u_j].add(value)
        """ 
        # Print or return the resulting dictionary
        print ('----after expanding------')
        print('new_W_u:', new_W_u) """
        return new_W_u



    def create_model(self):
        # Generate E* before solving
        self.generate_E_star()

        self.setup_variables()
        self.setup_objective()
        self.setup_constraints()
        
        # Write the model to an LP file
        self.model.write("vrptw_lp_relaxation.lp")
        print ("Model has been created.")

    def solve(self, relaxation=True):    
        """Solve the VRPTW model."""
        self.model.setParam('TimeLimit', 1000)  # Set the time limit to 300 seconds (5 minutes)
        if relaxation:
            self.model = self.model.relax()
            self.model.setParam('OutputFlag', 0)    # Disable gurobi output
        else:
            # Assume var_name is the name of the variable to be enforced as binary
            # Iterate through all variables in the model
            self.model.setParam('OutputFlag', 1)
            for variable in self.model.getVars():
                # Check if the variable name starts with 'x' or 'y'
                if variable.varName.startswith('x') or variable.varName.startswith('y'):
                    variable.vType = GRB.BINARY  # Set the variable type to binary

            # Update the model to apply the changes
            self.model.update()

            '''
            if False:    # Use the basis from the previous iteration to accelerate the solving process
                start_time = time.time()
                if self.var_basis and self.con_basis:
                    # Set the basis status for variables
                    var_dict = {var.varName: var for var in self.model.getVars()}
                    for var_name, basis in self.var_basis.items():
                        if var_name in var_dict:
                            var_dict[var_name].vBasis = basis

                    # Set the basis status for constraints
                    constr_dict = {con.ConstrName: con for con in self.model.getConstrs()}
                    for constr_name, basis in self.con_basis.items():
                        if constr_name in constr_dict:
                            constr_dict[constr_name].cBasis = basis
                end_time = time.time()  # End time
                print(f"-----start_with_basis-----Runtime: {end_time - start_time:.4f} seconds")'''

        self.model.optimize()
        self.solve_status()

        if relaxation:
            """ Get the dual variables for parameter expansion. """
            self.capacity_pi = self.fetch_duals_capacity_time()
            self.time_pi = self.fetch_duals_capacity_time('time')
            self.LA_pi = self.store_positive_duals_for_ku()
            '''
            if False:    # Save variable and constraint basis from the previous model
                start_time = time.time()
                self.variable_basis = {var.varName: var.vBasis for var in self.model.getVars()}
                self.constraint_basis = {constr.ConstrName: constr.cBasis for constr in self.model.getConstrs()}
                end_time = time.time()  # End time
                print(f"-----save_basis-----Runtime: {end_time - start_time:.4f} seconds")'''