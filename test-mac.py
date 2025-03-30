import cv2
import numpy as np
import open3d as o3d

# Load the 3D model
mesh = o3d.io.read_triangle_mesh("DEFAULT.ply")  # Replace with your 3D model file
mesh.compute_vertex_normals()

# OpenCV camera setup (0 refers to the default webcam)
camera = cv2.VideoCapture(0)

# Open3D visualizer setup
vis = o3d.visualization.Visualizer()
vis.create_window(width=500, height=500, window_name="3D Model")
vis.add_geometry(mesh)

angle = 0  # Rotation angle

while True:
    # Capture frame from the default webcam
    ret, frame = camera.read()

    if not ret:
        print("Failed to grab frame from camera")
        break

    # Convert to RGB (OpenCV uses BGR by default, Open3D expects RGB)
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Rotate the 3D model
    R = mesh.get_rotation_matrix_from_xyz((0, angle, 90))
    mesh.rotate(R, center=(0, 0, 0))
    # vis.update_geometry(mesh)

    # Render 3D model
    vis.poll_events()
    vis.update_renderer()

    # Convert Open3D window to an image (as a numpy array)
    image = np.asarray(vis.capture_screen_float_buffer(do_render=True)) * 255
    image = image.astype(np.uint8)

    # Resize and overlay 3D model onto camera feed
    overlay = cv2.resize(image, (200, 200))  # Resize for overlay size
    frame[50:250, 50:250] = overlay  # Place the 3D model in the top-left corner

    # Show the final output
    cv2.imshow("Camera Feed with 3D Model", frame)

    # Increase the angle to rotate the model
    angle += 0.05

    # Press 'q' to exit
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# Cleanup
camera.release()
cv2.destroyAllWindows()
vis.destroy_window()

