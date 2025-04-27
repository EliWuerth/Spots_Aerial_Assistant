import cv2
import numpy as np

# Get the predefined ArUco dictionary
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)

# Create an ArUco marker
marker_id = 0  # Marker ID
marker_size = 500  # Size in pixels

# Create an empty image
marker_image = np.zeros((marker_size, marker_size), dtype=np.uint8)

# Draw the marker
cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size, marker_image, 1)

# Save and show the marker
cv2.imwrite(f"aruco_marker_{marker_id}.png", marker_image)
cv2.imshow("ArUco Marker", marker_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
