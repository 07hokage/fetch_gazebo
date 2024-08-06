import json
import rospy
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

class PosePublisher:
    def __init__(self):
        self.publisher = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
        rospy.init_node('pose_publisher', anonymous=True)
    
    def load_tree_json(self, filepath=None):
        assert filepath is not None
        with open(filepath, "r") as jsonfile:
            tree = json.load(jsonfile)
        tree = json.loads(tree)
        return tree

    def extract_poses(self, tree):
        poses = []
        self._extract_poses_recursive(tree, poses)
        return poses

    def _extract_poses_recursive(self, tree, poses):
        print(tree["pose"])
        poses.append(tree["pose"])
        try:
            self._extract_poses_recursive(tree["children"][0], poses)
        except:
            pass
        return poses
        # for child in node.get("children", []):
        #     self._extract_poses_recursive(child, poses)

    def publish_poses(self, poses):
        marker_array = MarkerArray()
        for i, pose in enumerate(poses):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "poses"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = pose[0]
            marker.pose.position.y = -pose[1]
            marker.pose.position.z = pose[2]
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)

        self.publisher.publish(marker_array)

    def print_names(self, tree):
        print(tree.keys())
        print(tree["pose"])
        try:
            self.print_names(tree['children'][0])
        except KeyError:
            pass

    def run(self, filepath):
        tree = self.load_tree_json(filepath)
        self.print_names(tree)
        poses = self.extract_poses(tree)
        print(poses)
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            self.publish_poses(poses)
            rate.sleep()

if __name__ == '__main__':
    try:
        pose_publisher = PosePublisher()
        pose_publisher.run('tree.json')
    except rospy.ROSInterruptException:
        pass
