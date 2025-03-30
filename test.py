import cv2
import numpy as np
import open3d as o3d
from picamera2 import Picamera2

# Load the 3D model
mesh = o3d.io.read_triangle_mesh("DEFAULT.obj")  # Replace with your 3D model file
mesh.compute_vertex_normals()

# Initialize the camera
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)}))
picam2.start()

# Open3D visualizer setup
vis = o3d.visualization.Visualizer()
vis.create_window(width=300, height=300, window_name="3D Model")
vis.add_geometry(mesh)

angle = 0  # Rotation angle

while True:
    # Capture frame from camera
    frame = picam2.capture_array()
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    # Rotate the 3D model
    R = mesh.get_rotation_matrix_from_xyz((0, angle, 0))
    mesh.rotate(R, center=(0, 0, 0))
    vis.update_geometry(mesh)

    # Render 3D model
    vis.poll_events()
    vis.update_renderer()

    # Convert Open3D window to an image
    image = np.asarray(vis.capture_screen_float_buffer(do_render=True)) * 255
    image = image.astype(np.uint8)

    # Resize and overlay 3D model onto camera feed
    overlay = cv2.resize(image, (200, 200))
    frame[50:250, 50:250] = overlay  # Place the 3D model in the top-left corner

    # Show final output
    cv2.imshow("Camera Feed with 3D Model", frame)

    # Increase angle for rotation
    angle += 0.05

    # Press 'q' to exit
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# Cleanup
cv2.destroyAllWindows()
vis.destroy_window()

