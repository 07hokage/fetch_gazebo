#!/usr/bin/env python

import rospy
import tf
import numpy as np
from geometry_msgs.msg import PoseStamped

class PoseCorrection:
    def __init__(self):
        rospy.init_node('pose_correction_listener', anonymous=True)
        
        self.tf_listener = tf.TransformListener()
        self.rate = rospy.Rate(10)  # 10 Hz
        
        self.map_frame = "map"
        self.odom_frame = "odom"
        self.base_link_frame = "base_link"
        
        rospy.loginfo("Waiting for transform between map, odom, and base_link...")
        self.tf_listener.waitForTransform(self.map_frame, self.odom_frame, rospy.Time(0), rospy.Duration(4.0))
        self.tf_listener.waitForTransform(self.odom_frame, self.base_link_frame, rospy.Time(0), rospy.Duration(4.0))
        rospy.loginfo("Transforms are ready.")

    def get_transform(self, from_frame, to_frame):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(from_frame, to_frame, rospy.Time(0))
            return trans, rot
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("TF Exception: %s", str(e))
            return None, None

    def transform_to_matrix(self, trans, rot):
        translation_matrix = np.array([
            [1, 0, 0, trans[0]],
            [0, 1, 0, trans[1]],
            [0, 0, 1, trans[2]],
            [0, 0, 0, 1]
        ])

        rotation_matrix = tf.transformations.quaternion_matrix(rot)

        transform_matrix = np.dot(translation_matrix, rotation_matrix)
        return transform_matrix

    def compute_corrected_pose(self):
        trans_map_odom, rot_map_odom = self.get_transform(self.map_frame, self.odom_frame)
        trans_odom_base, rot_odom_base = self.get_transform(self.odom_frame, self.base_link_frame)

        if trans_map_odom and rot_map_odom and trans_odom_base and rot_odom_base:
            # Transform from map to odom
            map_to_odom_matrix = self.transform_to_matrix(trans_map_odom, rot_map_odom)

            # Transform from odom to base_link (uncorrected pose)
            odom_to_base_matrix = self.transform_to_matrix(trans_odom_base, rot_odom_base)

            # Compute corrected pose (map to base_link)
            corrected_pose_matrix = np.dot(map_to_odom_matrix, odom_to_base_matrix)

            return odom_to_base_matrix, corrected_pose_matrix
        return None, None

    def display_poses(self):
        while not rospy.is_shutdown():
            uncorrected_pose, corrected_pose = self.compute_corrected_pose()

            if uncorrected_pose is not None and corrected_pose is not None:
                rospy.loginfo("Uncorrected Pose (Odom -> Base_Link):\n%s", uncorrected_pose)
                rospy.loginfo("Corrected Pose (Map -> Base_Link):\n%s", corrected_pose)

            self.rate.sleep()

if __name__ == '__main__':
    try:
        pose_correction = PoseCorrection()
        pose_correction.display_poses()
    except rospy.ROSInterruptException:
        pass
