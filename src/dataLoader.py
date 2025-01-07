"""
Filename: dataLoader.py
Author: Yihe Zhuo
Last Modified: 2025-01-06
Description: 
    Data loader of Solomon benchmark dataset.
"""

from VRPTWInstance import VRPTWInstance
import xml.etree.ElementTree as ET

class SolomonDatasetLoader:
    """Class to load and parse Solomon VRPTW dataset in XML format."""
    
    def __init__(self, file_path):
        """
        Initialize the loader with the dataset file path.
        
        Args:
            file_path (str): Path to the Solomon XML dataset file.
        """
        self.file_path = file_path
    
    def load_instance(self):
        """
        Parses a Solomon XML dataset file into VRPTW instance components.
        
        Returns:
            VRPTWInstance: Parsed VRPTW instance.
        """
        # Parse the XML file
        tree = ET.parse(self.file_path)
        root = tree.getroot()
        
        # Extract depot information
        nodes = root.find("network").find("nodes")
        # Hardcode depot = node 0 for now. TODO: read the depot id from the data file.
        depot_node = nodes.find("node[@id='0']")
        depot = {
            "x": float(depot_node.find("cx").text),
            "y": float(depot_node.find("cy").text),
            "ready_time": 0,
            "due_time": float(root.find("fleet").find("vehicle_profile").find("max_travel_time").text)
        }
        
        # Extract customer data
        customers = []
        demands = []
        time_windows = []
        service_times = []
        
        # Read all other nodes (customers)
        for node in nodes.findall("node"):
            if node.attrib["id"] == "0":  # Skip depot
                continue
            node_id = int(node.attrib["id"])
            cx = float(node.find("cx").text)
            cy = float(node.find("cy").text)
            customers.append({"x": cx, "y": cy})
        
        # Extract request data
        requests = root.find("requests")
        for request in requests.findall("request"):
            quantity = float(request.find("quantity").text)
            start = float(request.find("tw").find("start").text)
            end = float(request.find("tw").find("end").text)
            service_time = float(request.find("service_time").text)
            demands.append(quantity)
            time_windows.append((start, end))
            service_times.append(service_time)
        
        # Create and return the VRPTWInstance
        return VRPTWInstance(
            depot=depot,
            customers=customers,
            demands=demands,
            time_windows=time_windows,
            service_times=service_times,
            vehicle_capacity=float(root.find("fleet").find("vehicle_profile").find("capacity").text)
        )
