import json
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class Navigator:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.init_node('navigator', anonymous=True)
        self.client.wait_for_server()
    
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

    def navigate_to_pose(self, pose):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = pose[0]
        goal.target_pose.pose.position.y = -pose[1]
        goal.target_pose.pose.position.z = 0
        goal.target_pose.pose.orientation.w = 1.0

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()

        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()

    def run(self, filepath):
        tree = self.load_tree_json(filepath)
        poses = self.extract_poses(tree)
        for pose in poses:
            result = self.navigate_to_pose(pose)
            if result:
                rospy.loginfo("Successfully reached the goal!")
            else:
                rospy.logerr("Failed to reach the goal!")

if __name__ == '__main__':
    try:
        navigator = Navigator()
        navigator.run('tree.json')
    except rospy.ROSInterruptException:
        pass
