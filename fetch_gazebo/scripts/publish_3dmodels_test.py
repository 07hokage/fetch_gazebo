from utils import read_graph_json
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

class PosePublisher:
    def __init__(self):
        self.publisher = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
        rospy.init_node('pose_publisher', anonymous=True)
    
    def create_table_marker(self, pose, marker_id, color):
        """Creates a table marker with a single central leg."""
        marker_array = MarkerArray()

        # Table dimensions (top and center leg)
        table_top_size = [0.8, 0.8, 0.1]  # Tabletop width, depth, thickness
        leg_size = [0.15, 0.15, 0.7]  # Single central leg dimensions
        
        # Create tabletop marker
        table_top_marker = Marker()
        table_top_marker.header.frame_id = "map"
        table_top_marker.header.stamp = rospy.Time.now()
        table_top_marker.ns = "table"
        table_top_marker.id = marker_id
        table_top_marker.type = Marker.CUBE
        table_top_marker.action = Marker.ADD
        table_top_marker.pose.position.x = pose[0]
        table_top_marker.pose.position.y = pose[1]
        table_top_marker.pose.position.z =  0.75  # Height of the tabletop
        table_top_marker.scale.x = table_top_size[0]
        table_top_marker.scale.y = table_top_size[1]
        table_top_marker.scale.z = table_top_size[2]
        table_top_marker.color.r, table_top_marker.color.g, table_top_marker.color.b = color
        table_top_marker.color.a = 1.0  # Fully opaque
        marker_array.markers.append(table_top_marker)

        # Create central leg marker
        leg_marker = Marker()
        leg_marker.header.frame_id = "map"
        leg_marker.header.stamp = rospy.Time.now()
        leg_marker.ns = "table_leg"
        leg_marker.id = marker_id + 1
        leg_marker.type = Marker.CUBE
        leg_marker.action = Marker.ADD
        leg_marker.pose.position.x = pose[0]
        leg_marker.pose.position.y = pose[1]
        leg_marker.pose.position.z =  0.35  # Position the base of the leg
        leg_marker.scale.x = leg_size[0]
        leg_marker.scale.y = leg_size[1]
        leg_marker.scale.z = leg_size[2]
        leg_marker.color.r, leg_marker.color.g, leg_marker.color.b = color
        leg_marker.color.a = 1.0  # Fully opaque
        marker_array.markers.append(leg_marker)

        return marker_array

    def create_chair_marker(self, pose, marker_id, color):
        """Creates a simple chair marker."""
        marker_array = MarkerArray()

        # Chair dimensions
        seat_size = [0.5, 0.5, 0.1]  # Seat width, depth, thickness
        leg_size = [0.05, 0.05, 0.4]  # Chair leg dimensions
        backrest_size = [0.5, 0.05, 0.5]  # Backrest dimensions

        # Create seat marker
        seat_marker = Marker()
        seat_marker.header.frame_id = "map"
        seat_marker.header.stamp = rospy.Time.now()
        seat_marker.ns = "chair"
        seat_marker.id = marker_id
        seat_marker.type = Marker.CUBE
        seat_marker.action = Marker.ADD
        seat_marker.pose.position.x = pose[0]
        seat_marker.pose.position.y = pose[1]
        seat_marker.pose.position.z = 0.4  # Height of the seat
        seat_marker.scale.x = seat_size[0]
        seat_marker.scale.y = seat_size[1]
        seat_marker.scale.z = seat_size[2]
        seat_marker.color.r, seat_marker.color.g, seat_marker.color.b = color
        seat_marker.color.a = 1.0  # Fully opaque
        marker_array.markers.append(seat_marker)

        # Create chair legs (4 legs)
        leg_positions = [
            [0.2, 0.2], [-0.2, 0.2], [0.2, -0.2], [-0.2, -0.2]
        ]
        for i, leg_pos in enumerate(leg_positions):
            leg_marker = Marker()
            leg_marker.header.frame_id = "map"
            leg_marker.header.stamp = rospy.Time.now()
            leg_marker.ns = "chair_legs"
            leg_marker.id = marker_id + i + 1
            leg_marker.type = Marker.CUBE
            leg_marker.action = Marker.ADD
            leg_marker.pose.position.x = pose[0] + leg_pos[0]
            leg_marker.pose.position.y = pose[1] + leg_pos[1]
            leg_marker.pose.position.z = 0.2  # Height for the leg base
            leg_marker.scale.x = leg_size[0]
            leg_marker.scale.y = leg_size[1]
            leg_marker.scale.z = leg_size[2]
            leg_marker.color.r, leg_marker.color.g, leg_marker.color.b = color
            leg_marker.color.a = 1.0  # Fully opaque
            marker_array.markers.append(leg_marker)

        # Create backrest marker
        backrest_marker = Marker()
        backrest_marker.header.frame_id = "map"
        backrest_marker.header.stamp = rospy.Time.now()
        backrest_marker.ns = "chair_backrest"
        backrest_marker.id = marker_id + 5
        backrest_marker.type = Marker.CUBE
        backrest_marker.action = Marker.ADD
        backrest_marker.pose.position.x = pose[0]
        backrest_marker.pose.position.y = pose[1] - 0.25  # Position backrest behind the seat
        backrest_marker.pose.position.z = 0.7  # Height of the backrest
        backrest_marker.scale.x = backrest_size[0]
        backrest_marker.scale.y = backrest_size[1]
        backrest_marker.scale.z = backrest_size[2]
        backrest_marker.color.r, backrest_marker.color.g, backrest_marker.color.b = color
        backrest_marker.color.a = 1.0  # Fully opaque
        marker_array.markers.append(backrest_marker)

        return marker_array

    def publish_poses(self):
        graph = read_graph_json()
        color_palette = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]  # Red, Green, Blue for table, chair, door
        categories = ["table", "chair", "door"]
        marker_array = MarkerArray()

        for sno, data in enumerate(graph.nodes(data=True)):
            node, data = data
            pose = data["pose"]
            category = data["category"]
            color = color_palette[categories.index(category)]

            if category == "table":
                marker_array.markers.extend(self.create_table_marker(pose, sno * 10, color).markers)
            elif category == "chair":
                marker_array.markers.extend(self.create_chair_marker(pose, sno * 10, color).markers)

        self.publisher.publish(marker_array)
        rospy.loginfo("Published objects")

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.publish_poses()
            rate.sleep()

if __name__ == '__main__':
    try:
        pose_publisher = PosePublisher()
        pose_publisher.run()
    except rospy.ROSInterruptException:
        pass
