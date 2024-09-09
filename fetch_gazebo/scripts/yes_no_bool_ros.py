#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

def yes_no_publisher():
    # Initialize the ROS node
    rospy.init_node('yes_no_publisher', anonymous=True)

    # Create a publisher on the /yes_no topic with Bool message type
     # Create a publisher on the /yes_no topic with Int32 message type
    pub = rospy.Publisher('/yes_no', Int32, queue_size=10)

    # Set the rate at which the message is published (e.g., 1 Hz)
    rate = rospy.Rate(1)  # 1 Hz

    # Initialize the message to True
    yes_or_no = 0

    # Keep publishing the message until the node is shutdown
    while not rospy.is_shutdown():
        # Publish the message
        rospy.loginfo(f"Publishing: {yes_or_no}")
        
        i=0
        # Toggle between True and False (yes/no)
        yes_or_no = input("yes or no")
        while i<20: 
            pub.publish(int(yes_or_no))
            rospy.sleep(0.1)
            i+=1
        # Sleep for the remainder of the loop cycle
        rate.sleep()

if __name__ == '__main__':
    try:
        yes_no_publisher()
    except rospy.ROSInterruptException:
        pass
