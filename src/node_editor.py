import json
import networkx as nx
import tkinter as tk
from tkinter import Canvas, filedialog


# Load the JSON file
def load_json(file_path):
    with open(file_path, 'r') as f:
        return json.load(f)


# Save the JSON file
def save_json(data, file_path):
    with open(file_path, 'w') as f:
        json.dump(data, f, indent=4, ensure_ascii=False)


class NodeEditor:
    def __init__(self, master, json_data, output_file):
        self.master = master
        self.json_data = json_data
        self.output_file = output_file

        # Extract GPS coordinates and normalize
        lats = [node["GpsInfo"]["Lat"] for node in json_data["Node"]]
        longs = [node["GpsInfo"]["Long"] for node in json_data["Node"]]

        self.lat_min, self.lat_max = min(lats), max(lats)
        self.long_min, self.long_max = min(longs), max(longs)

        self.nodes = {
            node["ID"]: self.normalize_coords(
                node["GpsInfo"]["Long"], node["GpsInfo"]["Lat"]
            )
            for node in json_data["Node"]
        }

        self.links = [(link["FromNodeID"], link["ToNodeID"]) for link in json_data["Link"]]

        self.canvas = Canvas(master, width=800, height=600, bg="white")
        self.canvas.pack(fill=tk.BOTH, expand=True)

        self.node_items = {}
        self.selected_node = None

        self.draw_graph()

        self.canvas.bind("<ButtonPress-1>", self.on_click)
        self.canvas.bind("<B1-Motion>", self.on_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_release)

    def normalize_coords(self, longitude, latitude):
        """Normalize GPS coordinates to fit canvas dimensions."""
        x = (longitude - self.long_min) / (self.long_max - self.long_min) * 800
        y = (latitude - self.lat_min) / (self.lat_max - self.lat_min) * 600
        return x, y

    def denormalize_coords(self, x, y):
        """Convert canvas coordinates back to GPS values."""
        longitude = x / 800 * (self.long_max - self.long_min) + self.long_min
        latitude = y / 600 * (self.lat_max - self.lat_min) + self.lat_min
        return longitude, latitude

    def draw_graph(self):
        self.canvas.delete("all")

        # Draw links
        for from_node, to_node in self.links:
            if from_node in self.nodes and to_node in self.nodes:
                x1, y1 = self.nodes[from_node]
                x2, y2 = self.nodes[to_node]
                self.canvas.create_line(x1, y1, x2, y2, fill="gray")

        # Draw nodes
        for node_id, (x, y) in self.nodes.items():
            item = self.canvas.create_oval(
                x - 5, y - 5, x + 5, y + 5, fill="blue", outline="black"
            )
            self.node_items[item] = node_id

    def on_click(self, event):
        for item, node_id in self.node_items.items():
            coords = self.canvas.coords(item)
            if len(coords) == 4:  # Ensure we have valid coordinates
                x1, y1, x2, y2 = coords
                if x1 <= event.x <= x2 and y1 <= event.y <= y2:
                    self.selected_node = node_id
                    break

    def on_drag(self, event):
        if self.selected_node:
            self.nodes[self.selected_node] = (event.x, event.y)
            self.draw_graph()

    def on_release(self, event):
        if self.selected_node:
            for node in self.json_data["Node"]:
                if node["ID"] == self.selected_node:
                    longitude, latitude = self.denormalize_coords(event.x, event.y)
                    node["GpsInfo"]["Long"] = longitude
                    node["GpsInfo"]["Lat"] = latitude
                    break
            save_json(self.json_data, self.output_file)
            print(f"Node {self.selected_node} updated")
        self.selected_node = None


if __name__ == "__main__":
    input_file = "waypoint_graph.json"
    output_file = "waypoint_graph_output.json"

    print("Loading JSON data...")
    data = load_json(input_file)

    print("Launching editor...")
    root = tk.Tk()
    root.title("Node Editor")
    editor = NodeEditor(root, data, output_file)
    root.mainloop()

    print(f"Updated data saved to {output_file}")
