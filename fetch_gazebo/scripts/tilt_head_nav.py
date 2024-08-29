# Point the head using controller
from control_msgs.msg import PointHeadAction, PointHeadGoal
import rospy
import actionlib
from std_msgs.msg import String

class PointHeadClient(object):
    def __init__(self):
        self.client = actionlib.SimpleActionClient(
            "head_controller/point_head", PointHeadAction
        )
        rospy.loginfo("Waiting for head_controller...")
        self.isnav = "yes"
        self.success = self.client.wait_for_server(timeout = rospy.Duration(3.0))
        self.isnav_sub = rospy.Subscriber("isnav", String, self.isnav_callback,queue_size=10)
        self.look_once = False
        if (self.success is False):
            rospy.loginfo("no point head controller available")
        else:
            rospy.loginfo("Use head_controller/point_head")


    def isnav_callback(self,msg):
        self.isnav = msg.data
        print(self.isnav)

    def look_at(self, x, y, z, frame, duration=1.0):
        """
        Turning head to look at x,y,z
        :param x: x location
        :param y: y location
        :param z: z location
        :param frame: the frame of reference
        :param duration: given time for operation to calcualte the motion plan
        :return:
        """
        goal = PointHeadGoal()
        goal.target.header.stamp = rospy.Time.now()
        goal.target.header.frame_id = frame
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z
        goal.min_duration = rospy.Duration(duration)
        self.client.send_goal(goal)
        self.client.wait_for_result()
    
    def loop(self):
        #subscribe to nav topic
        #when not navigating, dont rotate
        #when nmavigating,  rotate head
        while not rospy.is_shutdown():
            if self.isnav=="yes":
                self.look_once=False
                head_action.look_at(0.6, 0.2, 0.7, "base_link",3)
                head_action.look_at(0.6, -0.2, 0.7, "base_link", 3)
            elif self.look_once == False:
                self.look_once=True
                head_action.look_at(0.6, 0, 0.7, "base_link", 3)

                print(f"not navigating\n")
        

if __name__=="__main__":
    rospy.init_node("set_head")
    head_action = PointHeadClient()
    head_action.loop()
    rospy.spin()
    

        