import json
import networkx as nx
import tkinter as tk
from tkinter import Canvas, filedialog, simpledialog


# Load the JSON file
def load_json(file_path):
    with open(file_path, 'r') as f:
        return json.load(f)


# Save the JSON file
def save_json(data, file_path):
    with open(file_path, 'w') as f:
        json.dump(data, f, indent=4, ensure_ascii=False)


class NodeEditor:
    def __init__(self, master):
        self.master = master

        self.json_data = None
        self.output_file = None
        self.nodes = {}
        self.links = []
        self.scale_factor = 1.0

        self.canvas = Canvas(master, width=800, height=600, bg="white")
        self.canvas.pack(fill=tk.BOTH, expand=True)

        self.node_items = {}
        self.selected_node = None

        self.canvas.bind("<ButtonPress-1>", self.on_click)
        self.canvas.bind("<B1-Motion>", self.on_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_release)
        self.canvas.bind("<Button-4>", self.zoom_in)
        self.canvas.bind("<Button-5>", self.zoom_out)

        self.menu = tk.Menu(self.master)
        self.master.config(menu=self.menu)

        file_menu = tk.Menu(self.menu, tearoff=0)
        file_menu.add_command(label="Open", command=self.open_file)
        file_menu.add_command(label="Save As", command=self.save_as)
        self.menu.add_cascade(label="File", menu=file_menu)

    def open_file(self):
        input_file = filedialog.askopenfilename(filetypes=[("JSON files", "*.json")])
        if not input_file:
            return

        self.json_data = load_json(input_file)
        self.output_file = input_file

        lats = [node["GpsInfo"]["Lat"] for node in self.json_data["Node"]]
        longs = [node["GpsInfo"]["Long"] for node in self.json_data["Node"]]

        self.lat_min, self.lat_max = min(lats), max(lats)
        self.long_min, self.long_max = min(longs), max(longs)

        self.nodes = {
            node["ID"]: self.normalize_coords(
                node["GpsInfo"]["Long"], node["GpsInfo"]["Lat"]
            )
            for node in self.json_data["Node"]
        }

        self.links = [(link["FromNodeID"], link["ToNodeID"]) for link in self.json_data["Link"]]
        self.draw_graph()

    def save_as(self):
        output_file = filedialog.asksaveasfilename(defaultextension=".json", filetypes=[("JSON files", "*.json")])
        if not output_file:
            return

        self.output_file = output_file
        save_json(self.json_data, self.output_file)
        print(f"Data saved to {self.output_file}")

    def normalize_coords(self, longitude, latitude):
        """Normalize GPS coordinates to fit canvas dimensions."""
        x = (longitude - self.long_min) / (self.long_max - self.long_min) * 800 * self.scale_factor
        y = (latitude - self.lat_min) / (self.lat_max - self.lat_min) * 600 * self.scale_factor
        return x, y

    def denormalize_coords(self, x, y):
        """Convert canvas coordinates back to GPS values."""
        longitude = x / (800 * self.scale_factor) * (self.long_max - self.long_min) + self.long_min
        latitude = y / (600 * self.scale_factor) * (self.lat_max - self.lat_min) + self.lat_min
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

    def zoom_in(self, event):
        self.scale_factor *= 1.1
        self.update_canvas()

    def zoom_out(self, event):
        self.scale_factor /= 1.1
        self.update_canvas()

    def update_canvas(self):
        self.nodes = {
            node_id: self.normalize_coords(*self.denormalize_coords(x, y))
            for node_id, (x, y) in self.nodes.items()
        }
        self.draw_graph()


if __name__ == "__main__":
    print("Launching editor...")
    root = tk.Tk()
    root.title("Node Editor")
    editor = NodeEditor(root)
    root.mainloop()
