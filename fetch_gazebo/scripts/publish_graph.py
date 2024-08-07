from utils import read_graph_json
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

class PosePublisher:
    def __init__(self):
        self.publisher = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
        rospy.init_node('pose_publisher', anonymous=True)
    
    
    def publish_poses(self):
        graph = read_graph_json()
        color_palette = [[1,0,0],[0,1,0],[0,0,1]]
        categories=["table", "chair", "door"]
        marker_array = MarkerArray()
        for sno,  data in enumerate(graph.nodes(data=True)):
            node, data = data
            pose = data["pose"]
            color = color_palette[categories.index(data["category"])]
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "poses"
            marker.id = sno
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = pose[0]
            marker.pose.position.y = -pose[1]
            marker.pose.position.z = pose[2]
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            marker.color.a = 1.0
            marker.color.b = color[0]
            marker.color.g = color[1]
            marker.color.r = color[2]
            marker_array.markers.append(marker)

        self.publisher.publish(marker_array)
        print("publish")

  

    def run(self):
        print("publish")
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            self.publish_poses()
            rate.sleep()

if __name__ == '__main__':
    try:
        print("publish")
        pose_publisher = PosePublisher()
        print("publish")
        pose_publisher.run()
    except rospy.ROSInterruptException:
        pass
