import json
import math
import heapq


def load_graph(file_path):
    with open(file_path, 'r', encoding='utf-8') as f:
        data = json.load(f)

    graph = {}
    nodes = {node['ID']: node for node in data['Node']}

    for link in data['Link']:
        from_node = link['FromNodeID']
        to_node = link['ToNodeID']
        length = link['Length']

        if from_node not in graph:
            graph[from_node] = []
        if to_node not in graph:
            graph[to_node] = []

        graph[from_node].append((to_node, length))
        graph[to_node].append((from_node, length))  # Assuming undirected graph

    return graph, nodes


def haversine(lat1, lon1, lat2, lon2):
    R = 6371  # Earth's radius in km
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    a = math.sin(delta_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def find_nearest_node(lat, lon, nodes):
    min_distance = float('inf')
    nearest_node_id = None

    for node_id, node in nodes.items():
        node_lat = node['GpsInfo']['Lat']
        node_lon = node['GpsInfo']['Long']
        distance = haversine(lat, lon, node_lat, node_lon)

        if distance < min_distance:
            min_distance = distance
            nearest_node_id = node_id

    return nearest_node_id, min_distance


def add_virtual_node(lat, lon, graph, nodes):
    virtual_node_id = "Virtual_Node"
    nearest_node_id, distance = find_nearest_node(lat, lon, nodes)

    nodes[virtual_node_id] = {
        "ID": virtual_node_id,
        "GpsInfo": {"Lat": lat, "Long": lon},
    }

    graph[virtual_node_id] = [(nearest_node_id, distance)]
    graph[nearest_node_id].append((virtual_node_id, distance))

    return virtual_node_id


def a_star(graph, nodes, start, goal):
    def heuristic(node_id):
        lat1, lon1 = nodes[node_id]['GpsInfo']['Lat'], nodes[node_id]['GpsInfo']['Long']
        lat2, lon2 = nodes[goal]['GpsInfo']['Lat'], nodes[goal]['GpsInfo']['Long']
        return haversine(lat1, lon1, lat2, lon2)

    open_set = []
    heapq.heappush(open_set, (0, start))

    came_from = {}
    g_score = {node: float('inf') for node in graph}
    g_score[start] = 0
    f_score = {node: float('inf') for node in graph}
    f_score[start] = heuristic(start)

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]

        for neighbor, weight in graph[current]:
            tentative_g_score = g_score[current] + weight
            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return None  # Path not found


if __name__ == "__main__":
    file_path = "20250107_waypoint_graph_2.json"
    graph, nodes = load_graph(file_path)

    # Example: Input GPS coordinates
    input_lat = 37.2395
    input_lon = 126.7735

    virtual_node_id = add_virtual_node(input_lat, input_lon, graph, nodes)

    start_node = virtual_node_id  # Start from the virtual node
    goal_node = "N0010"  # Example target node

    path = a_star(graph, nodes, start_node, goal_node)
    if path:
        print(f"Path found: {' -> '.join(path)}")
    else:
        print("No path found.")
