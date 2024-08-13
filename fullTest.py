import cv2
import numpy as np
import apriltag
import math
import json
import keyboard
from networktables import NetworkTables as nt
import logging
import threading

# Network tables setup
logging.basicConfig(level=logging.DEBUG)
nt.initialize(server="roboRIO-2410-FRC.local")
sd = nt.getTable("SmartDashboard")

# Camera setup
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

if not cap.isOpened():
    raise IOError("Cannot open camera")

# AprilTag detector setup
options = apriltag.DetectorOptions(families="tag36h11")
detector = apriltag.Detector(options)

# Camera parameters
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fx, fy = 800, 800  # focal length
cx, cy = width / 2, height / 2  # optical center
camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)
dist_coeffs = np.zeros((4, 1))  # Assuming no lens distortion

# Constants
CENTER_COLOR = (203, 192, 255)
CORNER_COLOR = (50, 205, 50)
LINE_LENGTH = 5
TAG_SIZE = 0.08  # 8 cm

# Helper functions
def draw_pose(frame, pose, center, tag_size):
    (R, t) = (pose[:3, :3], pose[:3, 3])
    axis_angle = cv2.Rodrigues(R)[0]
    axis_length = tag_size / 2
    axes = np.float32([[axis_length, 0, 0], [0, axis_length, 0], [0, 0, axis_length]]).reshape(-1, 3)
    axesIm = cv2.projectPoints(axes, axis_angle, t, camera_matrix, dist_coeffs)[0].reshape(-1, 2)
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

def plotPoint(image, center, color):
    center = (int(center[0]), int(center[1]))
    image = cv2.line(image, (center[0] - LINE_LENGTH, center[1]), (center[0] + LINE_LENGTH, center[1]), color, 3)
    image = cv2.line(image, (center[0], center[1] - LINE_LENGTH), (center[0], center[1] + LINE_LENGTH), color, 3)
    return image

def plotText(image, center, color, text):
    center = (int(center[0]) + 4, int(center[1]) - 4)
    return cv2.putText(image, str(text), center, cv2.FONT_HERSHEY_SIMPLEX, 1, color, 3)

# Main loop
while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    results = detector.detect(gray)

    if not results:
        cv2.line(frame, [int(width/2), 0], [int(width/2), height], (0, 255, 0), 3)
        info = {"id": "none", "angle": "N/A", "dist": "N/A"}
        print(json.dumps(info))
        sd.putNumber("id", -1)
        sd.putNumber("angle", 1000)
        sd.putNumber("dist", -1)
    else:
        for r in results:
            # Draw detection
            frame = plotPoint(frame, r.center, CENTER_COLOR)
            cv2.line(frame, [int(width/2), 0], [int(width/2), height], (0, 255, 0), 3)
            for corner in r.corners:
                frame = plotPoint(frame, corner, CORNER_COLOR)
            frame = plotText(frame, r.center, CENTER_COLOR, str(r.tag_id))

            # Calculate distance and angle
            corners = r.corners
            s1 = math.dist(corners[0], corners[1])
            s2 = math.dist(corners[1], corners[2])
            s3 = math.dist(corners[2], corners[3])
            s4 = math.dist(corners[3], corners[0])

            val = (s1 + s4) / (s2 + s3)
            ang = -1964 * val + 2034
            ang = round(float("%.1f" % (ang/10)) * 2) / 2 * 10

            if (s1 + s4) < (s2 + s3):
                ang = 180 - ang

            # Calculate distance
            A = ((corners[0][0]*corners[1][1] - corners[1][0]*corners[0][1]) +
                 (corners[1][0]*corners[2][1] - corners[2][0]*corners[1][1]) +
                 (corners[2][0]*corners[3][1] - corners[3][0]*corners[2][1]) +
                 (corners[3][0]*corners[0][1] - corners[0][0]*corners[3][1])) / 2

            f = 60  # focal length
            f_x, f_y = 706.6876743, 713.88098602
            pxmm = ((f_x + f_y) / 2) / f
            dist = 161.5 * f / (A / pxmm)
            dist = 1.063 * dist + 52.06

            # Estimate 3D pose
            pose, e0, e1 = detector.detection_pose(r, (fx, fy, cx, cy), TAG_SIZE)
            draw_pose(frame, pose, r.center, TAG_SIZE)

            # Calculate Euler angles
            R = pose[:3, :3]
            euler_angles = rotation_matrix_to_euler_angles(R)
            euler_degrees = np.degrees(euler_angles)

            # Draw pose information
            cv2.putText(frame, f"Roll: {euler_degrees[0]:.2f}, Pitch: {euler_degrees[1]:.2f}, Yaw: {euler_degrees[2]:.2f}",
                        (int(r.center[0]), int(r.center[1]) + 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Update NetworkTables
            info = {"id": r.tag_id, "angle": ang, "dist": dist}
            print(json.dumps(info))
            sd.putNumber("id", r.tag_id)
            sd.putNumber("angle", ang)
            sd.putNumber("dist", dist)

    cv2.imshow('AprilTag Detection and Pose Estimation', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
