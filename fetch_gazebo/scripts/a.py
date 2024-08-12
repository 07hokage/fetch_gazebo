#!/usr/bin/env python

import rospy
import tf
import json
import signal
import sys
import numpy as np
from geometry_msgs.msg import PoseStamped

class PoseSaver:
    def __init__(self):
        rospy.init_node('pose_saver', anonymous=True)
        
        self.tf_listener = tf.TransformListener()
        self.rate = rospy.Rate(10)  # 10 Hz

        self.poses = []
        self.corrected_poses = []
        self.map_to_odom = None

        rospy.loginfo("Waiting for transform between map, odom, and base_link...")
        self.tf_listener.waitForTransform("map", "odom", rospy.Time(0), rospy.Duration(4.0))
        self.tf_listener.waitForTransform("odom", "base_link", rospy.Time(0), rospy.Duration(4.0))
        rospy.sleep(5)
        rospy.loginfo("Transforms are ready.")

        # Register signal handler to save data on Ctrl+C
        signal.signal(signal.SIGINT, self.signal_handler)

    def get_transform(self, from_frame, to_frame):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(from_frame, to_frame, rospy.Time(0))
            return trans, rot
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("TF Exception: %s", str(e))
            return None, None

    def run(self):
        while not rospy.is_shutdown():
            trans_odom_base, rot_odom_base = self.get_transform("odom", "base_link")
            trans_map_base, rot_map_base = self.get_transform("map", "base_link")

            if trans_odom_base and rot_odom_base:
                self.poses.append({"trans": trans_odom_base, "rot": rot_odom_base})

            if trans_map_base and rot_map_base:
                self.corrected_poses.append({"trans": trans_map_base, "rot": rot_map_base})

            self.rate.sleep()

    def signal_handler(self, sig, frame):
        rospy.loginfo("Saving data and exiting...")

        # Get the final map to odom transformation
        trans_map_odom, rot_map_odom = self.get_transform("map", "odom")
        self.map_to_odom = {"trans": trans_map_odom, "rot": rot_map_odom}

        # Save the data to a JSON file
        data = {
            "poses": self.poses,
            "corrected_poses": self.corrected_poses,
            "map_to_odom": self.map_to_odom
        }

        with open('pose_data.json', 'w') as outfile:
            json.dump(data, outfile, indent=4)

        rospy.loginfo("Data saved to pose_data.json")
        sys.exit(0)

if __name__ == '__main__':
    try:
        pose_saver = PoseSaver()
        pose_saver.run()
    except rospy.ROSInterruptException:
        pass
