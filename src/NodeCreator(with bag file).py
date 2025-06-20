import tkinter as tk
from tkinter import filedialog
import rosbag
import uuid
import json
import datetime
import numpy as np
import pyproj
import matplotlib.pyplot as plt
import networkx as nx

def calculate_utm_zone(lon):
    """
    Calculate the UTM Zone for a given longitude.

    Args:
        lon (float): Longitude in degrees.

    Returns:
        int: UTM Zone number.
    """
    return int((lon + 180) // 6) + 1

def extract_gps_from_bag(bag_file, gps_topic):
    """
    Extract GPS data from a rosbag file.

    Args:
        bag_file (str): Path to the rosbag file.
        gps_topic (str): Topic name of the GPS data.

    Returns:
        list: A list of GPS coordinates (latitude, longitude, altitude).
    """
    gps_data = []
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[gps_topic]):
            lat = msg.latitude
            lon = msg.longitude
            alt = msg.altitude if hasattr(msg, 'altitude') else None
            gps_data.append((lat, lon, alt))
    return gps_data

def create_json_graph(gps_data):
    """
    Create a JSON graph from GPS data with a minimum distance constraint.

    Args:
        gps_data (list): List of GPS coordinates (latitude, longitude, altitude).

    Returns:
        dict: JSON graph structure with Nodes and Links.
    """
    current_date = datetime.datetime.now().strftime("%Y%m%d")
    graph = {"Node": [], "Link": []}
    min_distance = 0.000045  # Approx. 5 meters in latitude/longitude degrees
    #min_distance = 0.00000898  # Approx. 1 meters in latitude/longitude degrees
    #min_distance = 0.00000449  # Approx. 50 centimeters in latitude/longitude degrees

    prev_lat, prev_lon, prev_alt = None, None, None  # Initialize previous point
    node_index = 0  # Node index for unique IDs

    # Create Nodes and Links
    for i, (lat, lon, alt) in enumerate(gps_data):
        if prev_lat is not None:
            distance = np.sqrt((lat - prev_lat)**2 + (lon - prev_lon)**2)
            if distance < min_distance:
                continue  # Skip this point if it's too close to the previous one

        # Calculate UTM Zone and convert GPS to UTM
        utm_zone = calculate_utm_zone(lon)
        is_south = lat < 0  # Check if the location is in the southern hemisphere
        utm_converter = pyproj.Proj(proj='utm', zone=utm_zone, ellps='WGS84', south=is_south)
        easting, northing = utm_converter(lon, lat)

        node_id = f"N{node_index:04d}"  # Unique Node ID
        graph["Node"].append({
            "ID": node_id,
            "AdminCode": "",
            "NodeType": 1,
            "ITSNodeID": "",
            "Maker": "한국도로공사",
            "UpdateDate": current_date,
            "Version": "2021",
            "Remark": f"Node {node_index}",
            "HistType": "02A",
            "HistRemark": "진출입 도로 변경",
            "GpsInfo": {
                "Lat": lat,
                "Long": lon,
                "Alt": alt
            },
            "UtmInfo": {
                "Easting": round(easting, 2),
                "Northing": round(northing, 2),
                "Zone": f"{utm_zone}{'S' if is_south else 'N'}"
            }
        })

        # Create Link to the previous node
        if prev_lat is not None:
            prev_node_id = f"N{node_index - 1:04d}"
            link_id = f"L{node_index - 1:04d}{node_index:04d}"
            graph["Link"].append({
                "ID": link_id,
                "AdminCode": "110",
                "RoadRank": 1,
                "RoadType": 1,
                "RoadNo": "20",
                "LinkType": 3,
                "LaneNo": 2,
                "R_LinkID": f"R_{node_index - 1:04d}",
                "L_LinkID": f"L_{node_index:04d}",
                "FromNodeID": prev_node_id,
                "ToNodeID": node_id,
                "SectionID": "A3_DRIVEWAYSECTION",
                "Length": round(distance * 1000, 2),  # Convert to meters
                "ITSLinkID": f"ITS_{uuid.uuid4().hex[:8]}",
                "Maker": "한국도로공사",
                "UpdateDate": current_date,
                "Version": "2021",
                "Remark": "특이사항 없음",
                "HistType": "02A",
                "HistRemark": "진출입 도로 변경"
            })

        # Update previous point and increment node index
        prev_lat, prev_lon, prev_alt = lat, lon, alt
        node_index += 1

    return graph

def save_graph_to_json(graph, output_file):
    """
    Save the graph to a JSON file.

    Args:
        graph (dict): JSON graph structure.
        output_file (str): Path to the output JSON file.
    """
    with open(output_file, 'w') as f:
        json.dump(graph, f, indent=4, ensure_ascii=False)

def select_bag_file():
    """Open a file dialog to select the bag file."""
    return filedialog.askopenfilename(filetypes=[("ROS Bag Files", "*.bag")])

def save_json_file():
    """Open a file dialog to select the save location for the JSON file."""
    return filedialog.asksaveasfilename(defaultextension=".json", filetypes=[("JSON Files", "*.json")])

def main():
    root = tk.Tk()
    root.withdraw()  # Hide the root window

    print("Select a ROS bag file...")
    bag_file = select_bag_file()
    if not bag_file:
        print("No bag file selected. Exiting.")
        return

    print("Select the output JSON file...")
    output_json = save_json_file()
    if not output_json:
        print("No output file selected. Exiting.")
        return

    gps_topic = "/ublox_f9k/fix"  # You can change this as needed

    print("Extracting GPS data from rosbag...")
    gps_data = extract_gps_from_bag(bag_file, gps_topic)

    print("Creating JSON graph...")
    json_graph = create_json_graph(gps_data)

    print("Saving graph to JSON file...")
    save_graph_to_json(json_graph, output_json)

    print(f"Graph saved to {output_json}")

if __name__ == "__main__":
    main()
