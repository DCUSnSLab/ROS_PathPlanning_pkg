import rosbag
import uuid
import json
import datetime
import numpy as np
import matplotlib.pyplot as plt
import networkx as nx

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
    Create a JSON graph from GPS data.

    Args:
        gps_data (list): List of GPS coordinates (latitude, longitude, altitude).

    Returns:
        dict: JSON graph structure with Nodes and Links.
    """
    current_date = datetime.datetime.now().strftime("%Y%m%d")
    graph = {"Node": [], "Link": []}

    # Create Nodes and Links
    for i, (lat, lon, alt) in enumerate(gps_data):
        node_id = f"N{i:04d}"  # Unique Node ID
        graph["Node"].append({
            "ID": node_id,
            "AdminCode": None,
            "NodeType": 1,
            "ITSNodeID": None,
            "Maker": "한국도로공사",
            "UpdateDate": current_date,
            "Version": "2021",
            "Remark": f"Node {i}",
            "HistType": "02A",
            "HistRemark": "진출입 도로 변경",
            "GpsInfo": {
                "Lat": lat,
                "Long": lon,
                "Alt": alt
            }
        })

        # Create Link to the previous node
        if i > 0:
            prev_node_id = f"N{i-1:04d}"
            prev_lat, prev_lon, _ = gps_data[i - 1]
            distance = np.sqrt((lat - prev_lat)**2 + (lon - prev_lon)**2)
            link_id = f"L{i-1:04d}{i:04d}"
            graph["Link"].append({
                "ID": link_id,
                "AdminCode": "110",
                "RoadRank": 1,
                "RoadType": 1,
                "RoadNo": "20",
                "LinkType": 3,
                "LaneNo": 2,
                "R_LinkID": f"R_{i-1:04d}",
                "L_LinkID": f"L_{i:04d}",
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

def visualize_graph(json_graph):
    """
    Visualize the graph using Matplotlib and NetworkX.

    Args:
        json_graph (dict): JSON graph structure with Nodes and Links.
    """
    # Create a NetworkX graph
    G = nx.Graph()

    # Add nodes with positions
    pos = {}
    for node in json_graph["Node"]:
        node_id = node["ID"]
        lat = node["GpsInfo"]["Lat"]
        lon = node["GpsInfo"]["Long"]
        pos[node_id] = (lon, lat)  # Use longitude as x and latitude as y
        G.add_node(node_id)

    # Add edges with weights
    for link in json_graph["Link"]:
        from_node = link["FromNodeID"]
        to_node = link["ToNodeID"]
        length = link["Length"]
        G.add_edge(from_node, to_node, weight=length)

    # Draw the graph
    plt.figure(figsize=(10, 8))
    nx.draw(
        G,
        pos,
        with_labels=True,
        node_size=250,
        font_size=5,
        font_color='white',
        node_color='blue',
        # edge_color='gray'
    )
    edge_labels = nx.get_edge_attributes(G, 'weight')
    #nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels)
    #plt.title("Waypoint Graph Visualization")
    #plt.xlabel("Longitude")
    #plt.ylabel("Latitude")
    plt.grid(False)
    plt.show()

if __name__ == "__main__":
    # Example usage
    bag_file = "../data/2025-01-03-18-20-30.bag"
    gps_topic = "/gps"
    output_json = "20250103_waypoint_graph.json"

    print("Extracting GPS data from rosbag...")
    gps_data = extract_gps_from_bag(bag_file, gps_topic)

    print("Creating JSON graph...")
    json_graph = create_json_graph(gps_data)

    print("Saving graph to JSON file...")
    save_graph_to_json(json_graph, output_json)

    print("Visualizing graph...")
    visualize_graph(json_graph)