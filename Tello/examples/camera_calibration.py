import cv2
import numpy as np
import yaml
import glob

# Chessboard size (number of inner corners per row and column)
CHECKERBOARD = (9, 6)
SQUARE_SIZE = 1.0  # Adjust if you know the real square size

# Criteria for corner sub-pixel accuracy
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points, e.g., (0,0,0), (1,0,0), (2,0,0), ... (8,5,0)
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2) * SQUARE_SIZE

objpoints = []  # 3D world points
imgpoints = []  # 2D image points

# Load all checkerboard images (modify the path as needed)
images = glob.glob('checkerboard_images/*.jpg')  # Put calibration images in this folder

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    if ret:
        objpoints.append(objp)
        refined_corners = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        imgpoints.append(refined_corners)

        # Draw and show corners
        img = cv2.drawChessboardCorners(img, CHECKERBOARD, refined_corners, ret)
        cv2.imshow('Checkerboard', img)
        cv2.waitKey(500)

cv2.destroyAllWindows()

# Camera calibration
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# Save the calibration data
calibration_data = {
    'camera_matrix': camera_matrix.tolist(),
    'dist_coeff': dist_coeffs.tolist()
}

with open('calibration_matrix.yaml', 'w') as f:
    yaml.dump(calibration_data, f)

print("Calibration complete! Data saved in 'calibration_matrix.yaml'")
