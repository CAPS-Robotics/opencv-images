# HYPOTHETICAL RaspPI camera code

import cv2
import numpy as np
import apriltag
import math

def draw_pose(frame, pose, center, tag_size):
    # Extract rotation and translation vectors
    (R, t) = (pose[:3, :3], pose[:3, 3])

    # Convert to axis-angle representation
    axis_angle = cv2.Rodrigues(R)[0]

    # Define the axes
    axis_length = tag_size / 2
    axes = np.float32([[axis_length, 0, 0], [0, axis_length, 0], [0, 0, axis_length]]).reshape(-1, 3)

    # Project 3D points to image plane
    axesIm = cv2.projectPoints(axes, axis_angle, t, camera_matrix, dist_coeffs)[0].reshape(-1, 2)

    # Draw the axes
    center = tuple(map(int, center))
    cv2.line(frame, center, tuple(map(int, axesIm[0])), (0, 0, 255), 3)  # X-axis (Red)
    cv2.line(frame, center, tuple(map(int, axesIm[1])), (0, 255, 0), 3)  # Y-axis (Green)
    cv2.line(frame, center, tuple(map(int, axesIm[2])), (255, 0, 0), 3)  # Z-axis (Blue)

def rotation_matrix_to_euler_angles(R):
    sy = np.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    singular = sy < 1e-6
    if not singular:
        x = np.arctan2(R[2,1] , R[2,2])
        y = np.arctan2(-R[2,0], sy)
        z = np.arctan2(R[1,0], R[0,0])
    else:
        x = np.arctan2(-R[1,2], R[1,1])
        y = np.arctan2(-R[2,0], sy)
        z = 0
    return np.array([x, y, z])

# Initialize camera
cap = cv2.VideoCapture(0)  # Use 0 for the default camera, adjust if needed

# Set camera resolution
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

# Create AprilTag detector
options = apriltag.DetectorOptions(families="tag36h11")
detector = apriltag.Detector(options)

# Camera parameters (replace these with your actual camera parameters)
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fx, fy = 800, 800  # focal length
cx, cy = width / 2, height / 2  # optical center
camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)
dist_coeffs = np.zeros((4, 1))  # Assuming no lens distortion

# Tag size in meters (replace with your tag's actual size)
tag_size = 0.08  # 8 cm

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect AprilTags
    results = detector.detect(gray)

    for r in results:
        # Estimate 3D pose
        pose, e0, e1 = detector.detection_pose(r, (fx, fy, cx, cy), tag_size)

        # Draw the pose
        draw_pose(frame, pose, r.center, tag_size)

        # Calculate Euler angles
        R = pose[:3, :3]
        euler_angles = rotation_matrix_to_euler_angles(R)
        euler_degrees = np.degrees(euler_angles)

        # Draw tag ID and pose information
        cv2.putText(frame, f"ID: {r.tag_id}", (int(r.center[0]), int(r.center[1]) - 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.putText(frame, f"Roll: {euler_degrees[0]:.2f}, Pitch: {euler_degrees[1]:.2f}, Yaw: {euler_degrees[2]:.2f}",
                    (int(r.center[0]), int(r.center[1]) + 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Display the resulting frame
    cv2.imshow('AprilTag 3D Pose Estimation', frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture and close windows
cap.release()
cv2.destroyAllWindows()
