#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import PolygonStamped, Point32
from std_msgs.msg import Header

def compute_fov_in_map_frame(intrinsics, RT_camera, RT_base, max_depth=20):
    """
    Computes the FOV in the map frame based on the camera intrinsics and transformation matrices.
    """
    fx = intrinsics[0][0]
    fy = intrinsics[1][1]
    px = intrinsics[0][2]
    py = intrinsics[1][2]
    
    # Compute horizontal and vertical FOV based on the camera intrinsics
    image_width = 2 * px  # Assuming image center px = image width / 2
    image_height = 2 * py  # Assuming image center py = image height / 2

    hfov = 2 * np.arctan(image_width / (2 * fx))  # Horizontal FOV in radians
    vfov = 2 * np.arctan(image_height / (2 * fy))  # Vertical FOV in radians

    # Define frustum corner points in the camera frame
    frustum_points_camera_frame = np.array([
        [max_depth * np.tan(hfov / 2), max_depth * np.tan(vfov / 2), max_depth, 1],  # Top-right
        [-max_depth * np.tan(hfov / 2), max_depth * np.tan(vfov / 2), max_depth, 1],  # Top-left
        [max_depth * np.tan(hfov / 2), -max_depth * np.tan(vfov / 2), max_depth, 1],  # Bottom-right
        [-max_depth * np.tan(hfov / 2), -max_depth * np.tan(vfov / 2), max_depth, 1],  # Bottom-left
    ])

    # Transform the frustum points from the camera frame to the map frame
    frustum_points_base_frame = RT_camera @ frustum_points_camera_frame.T
    frustum_points_map_frame = RT_base @ frustum_points_base_frame.T

    # Convert the points into a usable 2D polygon in the map frame (x, y)
    fov_polygon_map = frustum_points_map_frame[:2, :].T  # Only x, y components
    return fov_polygon_map

def publish_fov_polygon(fov_polygon_map):
    """
    Publishes the FOV as a PolygonStamped message to visualize in RViz.
    """
    # Create the polygon message
    polygon_msg = PolygonStamped()
    polygon_msg.header = Header()
    polygon_msg.header.stamp = rospy.Time.now()
    polygon_msg.header.frame_id = "map"  # Make sure the FOV is in the map frame

    # Add points to the polygon
    for point in fov_polygon_map:
        pt = Point32()
        pt.x, pt.y = point[0], point[1]
        pt.z = 0  # FOV is 2D in the map frame
        polygon_msg.polygon.points.append(pt)

    # Publish the polygon message
    pub.publish(polygon_msg)
    rospy.loginfo("Published FOV polygon in the map frame")

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('fov_publisher_node')

    # Create a publisher for the FOV polygon
    pub = rospy.Publisher('/fov_polygon', PolygonStamped, queue_size=10)

    # Camera intrinsics (example values, adjust based on your setup)
    intrinsics = [
        [574.0527954101562, 0.0, 319.5],
        [0.0, 574.0527954101562, 239.5],
        [0.0, 0.0, 1.0]
    ]
    
    # Example transformation matrices (RT_camera and RT_base)
    # Replace these with the actual transformation matrices from your system
    RT_camera = np.eye(4)  # Identity matrix as a placeholder (camera relative to base)
    RT_base = np.eye(4)    # Identity matrix as a placeholder (base relative to map)

    # Define maximum depth (adjust as needed)
    max_depth = 10

    rate = rospy.Rate(1)  # Publish at 1 Hz
    while not rospy.is_shutdown():
        # Compute the FOV in the map frame
        fov_polygon_map = compute_fov_in_map_frame(intrinsics, RT_camera, RT_base, max_depth)

        # Publish the FOV polygon
        publish_fov_polygon(fov_polygon_map)

        rate.sleep()
