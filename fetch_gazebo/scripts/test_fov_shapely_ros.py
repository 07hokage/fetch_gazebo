#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PolygonStamped, Point32
import numpy as np
from listener import ImageListener
import time
from utils import get_fov_points_in_baselink, denormalize_depth_image

def transform_points(points, RT_base):
    """
    Transforms points from base frame to map frame using the RT_base matrix.
    """
    # Convert points to numpy array for matrix operations
    points = np.array(points, dtype=float)  # Ensure points is a float array

    if not isinstance(RT_base, np.ndarray):
        raise TypeError("RT_base should be a numpy array")

    # Ensure RT_base is a 4x4 matrix
    if RT_base.shape != (4, 4):
        raise ValueError("RT_base should be a 4x4 matrix")
    
    # Apply rotation and translation
    transformed_points = np.dot(RT_base[:3, :3], points.T).T + RT_base[:3, 3]
    
    # Convert back to list of tuples
    return transformed_points.tolist()


def publish_polygon():
    # Initialize the ROS node
    rospy.init_node('polygon_publisher', anonymous=True)
    
    listener = ImageListener()
    
    # Give some time for the ImageListener to initialize if needed
    time.sleep(3)
    
    # Create a publisher object
    pub = rospy.Publisher('polygon_topic', PolygonStamped, queue_size=10)
    
    # Define the loop rate
    rate = rospy.Rate(10)  # 1 Hz

    while not rospy.is_shutdown():
        # Define the polygon points (triangle in this case)
        # polygon_points = [(0, 0, 0), (4, 0,0 ), (2, 4,0 )]
        depth = denormalize_depth_image(listener.depth, 20)
        polygon_points = get_fov_points_in_baselink(depth, listener.RT_camera)
        print(polygon_points)
        
        # Retrieve the RT_base matrix from the listener
        RT_base = listener.RT_base
        
        # Transform polygon points
        transformed_points = transform_points(polygon_points, RT_base)
        
        # Create a PolygonStamped message
        polygon_msg = PolygonStamped()
        polygon_msg.header.frame_id = "map"  # Set the frame_id according to your application
        polygon_msg.polygon.points = [Point32(x, y, 0) for x, y, z in transformed_points]
        
        # Publish the PolygonStamped message
        pub.publish(polygon_msg)
        
        # Log information
        rospy.loginfo("Published polygon with points: %s", polygon_msg.polygon.points)
        
        # Sleep for the specified rate
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_polygon()
    except rospy.ROSInterruptException:
        pass
