import time
import random
from shapely.geometry import Point, Polygon

# Define a polygon (triangle for simplicity)
polygon_points = [(0, 0), (4, 0), (2, 4)]
polygon = Polygon(polygon_points)

# Define the number of points to generate
num_points = 300

# Generate random points within a bounding box around the polygon
# To make sure the points cover the area around the polygon, 
# we use the bounding box limits of the polygon.
min_x, min_y, max_x, max_y = polygon.bounds
points = [(random.uniform(min_x - 2, max_x + 2), random.uniform(min_y - 2, max_y + 2)) for _ in range(num_points)]

# Start timing the loop
start_time = time.time()

# Check whether each point is inside the polygon
for point in points:
    p = Point(point)
    is_inside = polygon.contains(p)
    # print(f"Point {point} is inside the polygon: {is_inside}")

# End timing and print the loop time
end_time = time.time()
loop_time = end_time - start_time
print(f"Loop execution time: {loop_time:.6f} seconds")
