"""
Filename: FlowGraphCapacity.py
Author: Yihe Zhuo
Last Modified: 2025-01-06
Description: 
    Implementation of capacity/time discretization in Section 4.2 and 4.5.1.
"""

import gurobipy as gp
from gurobipy import GRB
import time
from collections import defaultdict

class FlowGraphCapacityTime():
    """ Use the same class for capacity and time. Use "type" to denote which constraint to add. """
    def __init__(self, model, x, instance, W_u, type = 'capacity'):
        self.model = model
        self.x = x  # Decision variable x[u, v] from VRPTWSolver
        self.instance = instance    # VRPTW data instance
        self.W_u = W_u  # Depends on the type, := WD_u if type == 'capacity' and := WT_u if type == 'time'
        self.type = type
        self.E_graph = []   # Store the edge set of self.G_graph
        self.G_graph = []   # Store the directed unweighted graph

        if self.type == 'capacity':
            self.ub = self.instance.vehicle_capacity
            self.step_size = 1  # d+_k + step_size = d-_{k+1}
        else:   # type == 'time'
            self.ub = self.instance.depot["due_time"] #max(max(values) for values in self.W_u.values() if values)
            self.step_size = 0.0001 # t+_k + step_size = t-_{k+1}. TODO: Need to improve the bucket creation.


    def createGraphNodes(self):
        """
        Create graph nodes from self.W_u.

        Returns:
            list: A list of tuples (u_i, d-_i, d+_i) or (u_i, t-_i, t+_i) representing the graph nodes.
        """
        # Starting depot (0, self.ub, self.ub): (0, d_0, d_0) for capacity and (0, t_0, t_0) for time
        # Ending depot (-1, 0, self.ub): (-1, 0, d_0) for capacity and (0, 0, t_0) for time
        graph_nodes = [(0, self.ub, self.ub), (-1, 0, self.ub)]

        # Iterate over each customer u to create discretized buckets
        for u, buckets in self.W_u.items():
            # Create nodes for consecutive buckets
            graph_nodes.append((u, buckets[0], buckets[1]))
            for i in range(1, len(buckets) - 1):
                graph_nodes.append((u, buckets[i] + self.step_size, buckets[i + 1]))    # TODO: remove the usage of step_size

        return graph_nodes           

    def createGraphEdges(self):
        """
        Create directed edges for the graph G based on the graph nodes and the rules specified in Section 4.2 of the paper.

        Returns:
            list: A list of directed edges for the graph in the format (i, j), where i and j are nodes in the graph.
        """
        demands = self.instance.demands
        customers = self.instance.customers
        service_times = self.instance.service_times

        edges = []

        # Precompute nodes grouped by customer ID for efficient lookups
        nodes_by_customer = defaultdict(list)
        for node in self.G_graph:
            if node[0] > 0:  # Skip depot nodes
                nodes_by_customer[node[0]].append(node)

        # Add edges from (alpha, 1) to (u, |D_u|) and from (u, 1) to (alpha_bar, 1)
        for node in self.G_graph:
            if node[0] <= 0:  # Skip the starting and ending depot
                continue

            u, d_min, d_max = node
            d_u = self.W_u[u][0]

            # Start node (u, 1)
            if d_min == d_u:  
                # Edge from (u, 1) to (alpha_bar, 1), a route ends after having u as its final customer
                edges.append((node, (-1, 0, self.ub)))  

            # End node (u, |D_u|)
            if d_max == self.W_u[u][-1]:  
                # Edge from (alpha, 1) to (u, |D_u|), a route starts with u as its first customer
                edges.append(((0, self.ub, self.ub), node))  

        # Add edges for (u, k) to (u, k-1)
        for u, nodes in nodes_by_customer.items():
            nodes.sort(key=lambda n: n[1])  # Sort nodes by d_min for efficient pairing
            for i, node in enumerate(nodes):
                for prev_node in nodes[:i]:  # Only look at previous nodes
                    if node[1] == prev_node[2] + self.step_size:
                        edges.append((node, prev_node))


        # Add edges for (u, k) to (v, m) based on capacity/time window conditions
        for u, nodes_u in nodes_by_customer.items():
            for v, nodes_v in nodes_by_customer.items():
                if u == v:
                    continue  # Skip self-loops for different customers

                for i in nodes_u:
                    d_min_u, d_max_u = i[1], i[2]

                    for j in nodes_v:
                        d_min_v, d_max_v = j[1], j[2]

                        if self.type == 'capacity':
                            d_u = demands[u - 1]
                        else:  # Time condition
                            """ 
                            A vehicle leaves ui with time remaining in between t−_i and t+_i and leaves at uj with time remaining between t−_j and t+_j.
                            TODO: need clarification here: discrepancy of discription of t- and t+.
                            1) Section 3.3 says t-, t+ are the amount of time remaining when service starts at customer u. 
                            2) Section 4.2 uses them as the amount of time remaining when leaving customer u.
                            My implementation here follows:
                            Based on t+_i - d_u >= t-_j, the largest remaining time when leaving ui - earliest remaining time when leaving uj >= travel time + service time at uj.
                            """
                            d_u = round(self.instance.compute_distance(customers[u - 1], customers[v - 1]) + service_times[v - 1], 1)

                        if d_max_u - d_u >= d_min_v:
                            if d_min_u > self.W_u[u][0] and d_min_v > d_min_u - self.step_size - d_u:
                                edges.append((i, j))
                            elif d_min_u == self.W_u[u][0]:
                                edges.append((i, j))

        return edges

    def setup_variables(self):
        prefix = "z_D" if self.type == "capacity" else "z_T"

        self.z = self.model.addVars(
            self.E_graph, lb=0, vtype=GRB.CONTINUOUS, name=prefix
        )
        self.model.update()
        
    def setup_constraints(self):
        """
        Add constraints (4a) and (4b).
        """
        # Precompute incoming and outgoing edges for each node
        incoming_edges = defaultdict(list)
        outgoing_edges = defaultdict(list)
        uv_to_edges = defaultdict(list)

        for i, j in self.E_graph:
            incoming_edges[j].append((i, j))
            outgoing_edges[i].append((i, j))
            uv_to_edges[(i[0], j[0])].append((i, j))
       
        # Constraint (6j)(6l): Flow conservation
        for node in self.G_graph:
            if node[0] > 0:
                incoming_flow = gp.quicksum(self.z[edge] for edge in incoming_edges[node])
                outgoing_flow = gp.quicksum(self.z[edge] for edge in outgoing_edges[node])
                self.model.addConstr(incoming_flow == outgoing_flow, name=f"{self.type}_flow_conservation_{node}")

        # Constraint (6k)(6m): Consistency between z^D and x
        for edge in self.x.keys():  # Use x from mainClass
            u, v = edge
            z_edges = gp.quicksum(self.z[i, j] for i, j in uv_to_edges[(u, v)])
            self.model.addConstr(self.model.getVarByName(self.x[u,v].varName) == z_edges, name=f"xz_{self.type}_consistency_{u}_{v}")
            #self.model.addConstr(self.x[u,v] == z_edges, name=f"xz_{self.type}_consistency_{u}_{v}")
        
        self.model.update()

    def run_flow_graph(self):
        """ Create the graph and edges based on the discretization buckets. Then create variables zD, zT and add constraints. """
        self.G_graph = self.createGraphNodes()
        self.E_graph = self.createGraphEdges()
        self.setup_variables()
        self.setup_constraints()
