import cv2
import numpy as np

# Read the image
image = cv2.imread('cable_analysis/imgs/4.png')

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

# Apply the mask to the original image
isolated_object = cv2.bitwise_and(image, image, mask=mask)

# Display the results
cv2.imshow('Original Image', image)
cv2.imshow('Isolated Object', isolated_object)
cv2.waitKey(0)
cv2.destroyAllWindows()
