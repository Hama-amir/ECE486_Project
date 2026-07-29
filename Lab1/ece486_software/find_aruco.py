import os
import cv2
import numpy as np


def transform_camera_to_world(X_c_new, R, T):
    return R @ X_c_new + T


script_dir = os.path.dirname(os.path.abspath(__file__))

# Load transformation matrices from the same folder as this script
R = np.load(os.path.join(script_dir, "R.npy"))
T = np.load(os.path.join(script_dir, "T.npy"))

# Load the predefined dictionary
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

# Camera parameters (assuming some default values, you should calibrate your camera)
camera_matrix = np.array([[685.83035286,   0,         288.20303825],
                          [  0,         686.94865624, 227.25786837],
                          [  0,           0,           1        ]], dtype=np.float32)
                          
dist_coeffs = np.array([[ 0.20789158, -1.06720034, -0.01013679, -0.0125471,   2.85072425]], dtype=np.float32)
                          
# Open webcam
cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    
    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Detect markers
    corners, ids, _ = detector.detectMarkers(gray)
    
    if ids is not None:
        # Draw detected markers
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        
        # Estimate pose of each marker
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, dist_coeffs)
        
        for i in range(len(ids)):
            # Transform the camera pose (tvecs) to the world frame using the transform function
            X_c_new = tvecs[i].flatten()  # Extract translation vector
            X_w_new = transform_camera_to_world(X_c_new, R, T)
            
            # Draw frame axes (this will still be in camera coordinates, no transformation needed)
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.05)

            # Display transformed world coordinates on screen
            pos_text = f"ID: {ids[i][0]} World Pos: {X_w_new}"
            cv2.putText(frame, pos_text, tuple(corners[i][0][0].astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    # Display the resulting frame
    cv2.imshow('Aruco Marker Detection', frame)
    
    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
