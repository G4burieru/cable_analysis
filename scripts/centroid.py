import cv2
import numpy as np

# Read the image
image = cv2.imread('cable_analysis/imgs/2.png')

# Convert the image to grayscale
gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Apply Gaussian blur to reduce noise and improve edge detection
blurred_image = cv2.GaussianBlur(gray_image, (5, 5), 0)

# Apply Canny edge detection
edges = cv2.Canny(blurred_image, threshold1=30, threshold2=150)

# Find contours in the edge image
contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Create a mask to isolate the grey object
mask = np.zeros_like(edges)
cv2.drawContours(mask, contours, -1, 255, thickness=cv2.FILLED)

# Calculate the moments of the contours
moments = cv2.moments(mask)

# Calculate the centroid of the cables
if moments['m00'] != 0:
    cx = int(moments['m10'] / moments['m00'])
    cy = int(moments['m01'] / moments['m00'])
    centroid = (cx, cy)
    cv2.circle(image, centroid, 5, (0, 255, 0), -1)  # Draw a circle at the centroid

# Display the results
cv2.imshow('Original Image', image)
cv2.waitKey(0)
cv2.destroyAllWindows()
