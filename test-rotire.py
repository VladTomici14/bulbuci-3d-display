import open3d as o3d
import numpy as np

# Load the 3D model
mesh = o3d.io.read_triangle_mesh("corp.obj")  # Replace with your 3D model file
mesh.compute_vertex_normals()

# Apply a rotation around the y-axis (90 degrees right rotation)
R = mesh.get_rotation_matrix_from_xyz([np.pi/2, 0, 0])  # Rotate 90 degrees around y-axis
mesh.rotate(R, center=mesh.get_center())

# Open3D visualizer setup
vis = o3d.visualization.Visualizer()
vis.create_window(width=1200, height=1000, window_name="3D Model")
vis.add_geometry(mesh)

# Get the view control object
view_control = vis.get_view_control()

# Set the camera starting position and orientation
view_control.set_front([1.0, 0.0, 0.0])   # Direction of the camera view
view_control.set_up([0.0, 1.0, 0.0])      # Which direction is up - corrected to standard up
view_control.set_zoom(0.8)                # Zoom level

while True:
    # Render the 3D model
    vis.poll_events()
    vis.update_renderer()

    # Press 'q' to exit
    if vis.poll_events() == 0:  # Change this line if you need a specific keybinding
        break

# Cleanup
vis.destroy_window()

