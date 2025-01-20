#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from tkinter import Tk, filedialog, messagebox
from path_planning.srv import CreateGraph

def call_service(file_path):
    """Call the ROS service with the selected JSON file path."""
    rospy.wait_for_service('map_server')
    try:
        # Create a proxy for the service
        service_proxy = rospy.ServiceProxy('map_server', CreateGraph)
        # Call the service with the file path
        response = service_proxy(file_path)
        rospy.loginfo("Service call successful!")
        rospy.loginfo("Graph JSON: %s", response.graph_json)
        rospy.loginfo("Node JSON: %s", response.node_json)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        messagebox.showerror("Service Error", f"Service call failed: {e}")

def select_file_and_call_service():
    """Open a file dialog to select a JSON file and call the service."""
    root = Tk()
    root.withdraw()  # Hide the root window
    root.title("Select a JSON File")

    # Open file dialog
    file_path = filedialog.askopenfilename(
        title="Select a JSON File",
        filetypes=[("JSON Files", "*.json"), ("All Files", "*.*")]
    )

    if file_path:
        rospy.loginfo("Selected file: %s", file_path)
        call_service(file_path)
    else:
        rospy.logwarn("No file selected.")

if __name__ == "__main__":
    # Initialize ROS node
    rospy.init_node('json_file_selector', anonymous=True)

    # Show file selector and call service
    try:
        select_file_and_call_service()
    except rospy.ROSInterruptException:
        rospy.logerr("Node interrupted.")
    except Exception as e:
        rospy.logerr("Error occurred: %s", e)
