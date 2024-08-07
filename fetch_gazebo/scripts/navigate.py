#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from utils import read_graph_json, plot_point_on_map

def movebase_client():

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    target_class =input("choose to navigate to\n 1. table\n2. door\n3. chair\n")
    graph = read_graph_json()
    target_position = []
    for node, data in graph.nodes(data=True):
            if target_class in data["category"]:
                print(f"{target_class} found in graph\n")
                print(f"node details {data}\n")
                target_position = data["pose"]
                plot_point_on_map(target_position)
                to_nav = input("do you want to navigate to this point y/n \n")
                if to_nav =="y":
                    goal = MoveBaseGoal()
                    goal.target_pose.header.frame_id = "map"
                    goal.target_pose.header.stamp = rospy.Time.now()
                    goal.target_pose.pose.position.x = target_position[0]
                    goal.target_pose.pose.orientation.y = target_position[1]
                    client.send_goal(goal)
                    wait = client.wait_for_result()
                    if not wait:
                        rospy.logerr("Action server not available!")
                        rospy.signal_shutdown("Action server not available!")
                    else:
                        return client.get_result()
                next = input("do you want to reach next instance? y/n \n")
                if next =="y":
                    pass
                else:
                    break

    
    

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')
        result = movebase_client()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
