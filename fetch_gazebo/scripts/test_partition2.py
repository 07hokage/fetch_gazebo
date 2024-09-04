import cv2
import numpy as np
from sklearn.cluster import KMeans

# Load the occupancy map from a PNG file
occupancy_map = cv2.imread('map.png', cv2.IMREAD_GRAYSCALE)

# Thresholding to ensure the image is binary
_, binary_image = cv2.threshold(occupancy_map, 127, 255, cv2.THRESH_BINARY)

# Detect edges using Canny
edges = cv2.Canny(binary_image, 50, 150, apertureSize=3)

# Detect lines using Hough Transform
lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=100, minLineLength=100, maxLineGap=50)

# Create a blank image to draw lines on
line_image = np.zeros_like(binary_image)

if lines is not None:
    for line in lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(line_image, (x1, y1), (x2, y2), 255, 3)  # Thicker lines for stronger influence

# Combine free-space coordinates with line features
free_space_coords = np.column_stack(np.where(binary_image == 255))
line_coords = np.column_stack(np.where(line_image == 255))
features = np.vstack((free_space_coords, line_coords))

# Number of regions (clusters) we want to partition the free space into
k = 15  # You can adjust this value based on how many regions you want

# Apply K-Means clustering to combined features
kmeans = KMeans(n_clusters=k, random_state=0).fit(features)
labels = kmeans.labels_

# Create a new image where each region is assigned a different value
segmented_map_color = np.zeros((binary_image.shape[0], binary_image.shape[1], 3), dtype=np.uint8)
colors = np.random.randint(0, 255, size=(k, 3), dtype=np.uint8)

for i in range(k):
    cluster_coords = features[labels == i]
    for (x, y) in cluster_coords:
        segmented_map_color[x, y] = colors[i]

# Post-processing: Merge segments that belong to the same straight corridor
# Here you can add a region merging algorithm based on the direction and alignment

# Save the color-segmented map
cv2.imwrite('segmented_occupancy_map_color_improved.png', segmented_map_color)

# Display the original and color-segmented maps
cv2.imshow('Original Occupancy Map', occupancy_map)
cv2.imshow('Segmented Occupancy Map', segmented_map_color)
cv2.waitKey(0)
cv2.destroyAllWindows()
