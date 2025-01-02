import tkinter as tk
import networkx as nx
import json


class GraphEditor:
    def __init__(self, master, json_file):
        self.master = master
        self.master.title("Graph Editor")
        self.canvas = tk.Canvas(master, width=800, height=600, bg="white")
        self.canvas.pack(fill=tk.BOTH, expand=True)

        self.graph = nx.Graph()
        self.node_positions = {}
        self.selected_node = None
        self.temp_link_start = None
        self.json_file = json_file

        # Zoom settings
        self.scale_factor = 1.0  # Initial scale

        # Load graph from JSON
        self.load_graph_from_json(json_file)

        self.canvas.bind("<Button-1>", self.on_click)
        self.canvas.bind("<B1-Motion>", self.on_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_release)

        # Key bindings for zoom
        self.master.bind("<Key-plus>", self.zoom_in)
        self.master.bind("<Key-minus>", self.zoom_out)

        self.draw_graph()

    def load_graph_from_json(self, json_file):
        """
        Load the graph from a JSON file.
        """
        with open(json_file, 'r') as f:
            data = json.load(f)

        for node in data["Node"]:
            node_id = node["ITSNodeID"]
            lat = node["GpsInfo"]["Lat"]
            lon = node["GpsInfo"]["Long"]
            if lat is not None and lon is not None:
                x, y = lon * 10, -lat * 10  # Initial scaling
                self.graph.add_node(node_id)
                self.node_positions[node_id] = (x, y)

        for link in data["Link"]:
            self.graph.add_edge(link["FromNodeID"], link["ToNodeID"])

    def zoom_in(self, event=None):
        """
        Zoom in the canvas by increasing the scale factor.
        """
        self.scale_factor *= 1.2
        self.update_node_positions()
        self.draw_graph()

    def zoom_out(self, event=None):
        """
        Zoom out the canvas by decreasing the scale factor.
        """
        self.scale_factor /= 1.2
        self.update_node_positions()
        self.draw_graph()

    def update_node_positions(self):
        """
        Update node positions based on the current scale factor.
        """
        for node_id, (x, y) in self.node_positions.items():
            self.node_positions[node_id] = (x * self.scale_factor, y * self.scale_factor)

    def draw_graph(self):
        """
        Redraw the graph on the canvas.
        """
        self.canvas.delete("all")
        for edge in self.graph.edges:
            x1, y1 = self.node_positions[edge[0]]
            x2, y2 = self.node_positions[edge[1]]
            self.canvas.create_line(x1, y1, x2, y2, fill="gray", width=2)

        for node, (x, y) in self.node_positions.items():
            self.canvas.create_oval(x - 15, y - 15, x + 15, y + 15, fill="blue", outline="black", width=2)
            self.canvas.create_text(x, y, text=str(node), fill="white")

    def on_click(self, event):
        """
        Handle mouse click events on the canvas.
        """
        x, y = event.x, event.y
        clicked_node = self.get_node_at_position(x, y)
        if clicked_node:
            self.selected_node = clicked_node
            if self.temp_link_start is None:
                self.temp_link_start = clicked_node
            else:
                self.graph.add_edge(self.temp_link_start, clicked_node)
                self.temp_link_start = None
        else:
            new_node_id = f"N{len(self.graph.nodes):04d}"
            self.graph.add_node(new_node_id)
            self.node_positions[new_node_id] = (x / self.scale_factor, y / self.scale_factor)
        self.draw_graph()

    def on_drag(self, event):
        """
        Handle mouse drag events to move nodes.
        """
        if self.selected_node is not None:
            self.node_positions[self.selected_node] = (event.x / self.scale_factor, event.y / self.scale_factor)
            self.draw_graph()

    def on_release(self, event):
        """
        Handle mouse release events to finalize node dragging.
        """
        self.selected_node = None

    def get_node_at_position(self, x, y):
        """
        Check if a node exists at the given position.
        """
        for node, (nx, ny) in self.node_positions.items():
            if (x - nx * self.scale_factor) ** 2 + (y - ny * self.scale_factor) ** 2 <= 15 ** 2:
                return node
        return None

    def save_graph_to_json(self, graph, output_file):
        """
        Save the graph to a JSON file.

        Args:
            graph (dict): JSON graph structure.
            output_file (str): Path to the output JSON file.
        """
        with open(output_file, 'w') as f:
            json.dump(graph, f, indent=4)

if __name__ == "__main__":
    json_file = "waypoint_graph.json"

    root = tk.Tk()
    app = GraphEditor(root, json_file)
    root.protocol("WM_DELETE_WINDOW", lambda: (app.save_graph_to_json(), root.destroy()))
    root.mainloop()
