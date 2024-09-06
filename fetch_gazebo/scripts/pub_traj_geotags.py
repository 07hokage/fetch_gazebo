from utils import read_graph_json
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import numpy as np

class PosePublisher:
    def __init__(self):
        self.publisher = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
        rospy.init_node('pose_publisher', anonymous=True)
    
    def publish_poses(self):
        with np.load("surveillance_traj.npz") as traj_file:
            surveillance_traj = traj_file["traj"]
        marker_array = MarkerArray()

        for sno, data in enumerate(surveillance_traj):
            color = [1, 0, 0]  # Red for location marker
            
  

            # Create a cylinder for the pin (the "tail" of the marker)
            cylinder_marker = Marker()
            cylinder_marker.header.frame_id = "map"
            cylinder_marker.header.stamp = rospy.Time.now()
            cylinder_marker.ns = "geo_tags"
            cylinder_marker.id = sno + 1000  # Ensure unique IDs
            cylinder_marker.type = Marker.MESH_RESOURCE  # Cylinder for the tail of the location marker
            cylinder_marker.mesh_resource = "package://fetch_gazebo/marker/files/location_marker.stl"
            cylinder_marker.action = Marker.ADD
            cylinder_marker.pose.position.x = data[0]
            cylinder_marker.pose.position.y = data[1]
            cylinder_marker.pose.position.z = 0  # Start the cylinder at the base
            cylinder_marker.scale.x = 0.1 # Thickness of the pin
            cylinder_marker.scale.y = 0.1
            cylinder_marker.scale.z = 0.1  # Length of the pin
            cylinder_marker.color.a = 1.0
            cylinder_marker.color.r = color[0]
            cylinder_marker.color.g = color[1]
            cylinder_marker.color.b = color[2]
            marker_array.markers.append(cylinder_marker)

        self.publisher.publish(marker_array)
        print("Markers published")

    def run(self):
        print("Publishing location markers")
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            self.publish_poses()
            rate.sleep()

if __name__ == '__main__':
    try:
        pose_publisher = PosePublisher()
        pose_publisher.run()
    except rospy.ROSInterruptException:
        pass
