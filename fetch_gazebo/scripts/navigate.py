import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class Navigate:
    def __init__(self) -> None:
        self.move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base_client.wait_for_server()


    def navigate(self, pose, set_orientation = False, orientation_qt = None, move_head = False):
        pass

    def track_trajectory(self, waypoints = []):
        pass

    def navigate_to_object_class(self, graph, _class = None):
        pass

