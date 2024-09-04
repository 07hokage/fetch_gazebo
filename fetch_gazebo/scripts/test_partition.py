import cv2
import numpy as np
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt

# Load the occupancy map from a PNG file
occupancy_map = cv2.imread('map.png', cv2.IMREAD_GRAYSCALE)
# Thresholding to ensure the image is binary (optional if the image is already binary)
_, binary_image = cv2.threshold(occupancy_map, 127, 255, cv2.THRESH_BINARY)

# Define a structuring element (kernel) for morphological operations
# The size of the kernel affects the size of the segments that will be removed
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

# Apply morphological opening (erosion followed by dilation)
# This operation removes small white noise
opened_image = cv2.morphologyEx(binary_image, cv2.MORPH_OPEN, kernel)

# Optional: Apply morphological closing to close small holes within white regions
closed_image = cv2.morphologyEx(opened_image, cv2.MORPH_CLOSE, kernel)
# Verify the shape is as expected (4000x4000)
# assert occupancy_map.shape == (4000, 4000)

# Extract the free space (where pixel value == 255)
free_space_coords = np.column_stack(np.where(closed_image == 255))

# Number of regions (clusters) we want to partition the free space into
k = 18  # You can change this value based on how many regions you want

# Apply K-Means clustering to free space
kmeans = KMeans(n_clusters=k, random_state=0).fit(free_space_coords)

# Assign different labels (0, 1, ..., k-1) to each cluster
labels = kmeans.labels_

# Create a new image where each region is assigned a different color
segmented_map_color = np.zeros((closed_image.shape[0], closed_image.shape[1], 3), dtype=np.uint8)

# Assign a unique color to each cluster
colors = np.random.randint(0, 255, size=(k, 3), dtype=np.uint8)

# Assign a unique intensity to each cluster (different gray levels)
for i in range(k):
    cluster_coords = free_space_coords[labels == i]
    for (x, y) in cluster_coords:
        segmented_map_color[x, y] = colors[i]  # Assign the cluster's color to the pixels

# Display the original occupancy map and the segmented map
# plt.figure(figsize=(12, 6))
# plt.subplot(1, 2, 1)
# plt.title("Original Occupancy Map")
# plt.imshow(occupancy_map, cmap='gray')

# plt.subplot(1, 2, 2)
# plt.title(f"Segmented Map with {k} Regions")
# plt.imshow(segmented_map, cmap='gray')

# plt.show()

# Optionally, save the segmented map
cv2.imwrite('segmented_occupancy_map.png', segmented_map_color)
