#!/usr/bin/env python

import rospy
import json
import numpy as np
import tf
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class PoseCorrector:
    def __init__(self):
        rospy.init_node('pose_corrector', anonymous=True)

        self.marker_pub = rospy.Publisher('corrected_poses', Marker, queue_size=10)
        self.saved_marker_pub = rospy.Publisher('saved_corrected_poses', Marker, queue_size=10)

        rospy.loginfo("Loading data from JSON file...")
        self.data = self.load_data()
        rospy.loginfo("Data loaded successfully.")

        self.rate = rospy.Rate(1)  # 1 Hz

    def load_data(self):
        try:
            with open('pose_data.json', 'r') as infile:
                data = json.load(infile)
            return data
        except IOError:
            rospy.logerr("Error: Could not read pose_data.json file.")
            rospy.signal_shutdown("Could not read JSON file.")
            return None

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

    def apply_correction(self, pose, correction_matrix):
        pose_matrix = self.transform_to_matrix(pose['trans'], pose['rot'])
        corrected_matrix = np.dot(correction_matrix, pose_matrix)
        return corrected_matrix

    def publish_marker(self, points, ns, color, marker_pub):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05  # Line width
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0
        marker.ns = ns
        marker.pose.orientation.w = 1.0

        for point in points:
            marker.points.append(self.create_point(point))

        marker_pub.publish(marker)

    def create_point(self, array):
        point = Point()
        point.x = array[0]
        point.y = array[1]
        point.z = array[2]
        return point

    def run(self):
        rospy.loginfo("Transforming saved poses using the final correction...")
        map_to_odom_matrix = self.transform_to_matrix(self.data['map_to_odom']['trans'], self.data['map_to_odom']['rot'])
        corrected_points = []
        saved_corrected_points = []

        # Apply correction to saved uncorrected poses
        for pose in self.data['poses']:
            corrected_matrix = self.apply_correction(pose, map_to_odom_matrix)
            corrected_points.append(corrected_matrix[:3, 3])

        rospy.loginfo("Transformed saved uncorrected poses to corrected poses.")

        # Extract saved corrected poses directly from JSON
        for saved_pose in self.data['corrected_poses']:
            saved_corrected_points.append(saved_pose['trans'])

        rospy.loginfo("Publishing corrected and saved corrected poses as markers...")
        while not rospy.is_shutdown():
            self.publish_marker(corrected_points, "corrected_poses", [1.0, 0.0, 0.0], self.marker_pub)
            self.publish_marker(saved_corrected_points, "saved_corrected_poses", [0.0, 1.0, 0.0], self.saved_marker_pub)
            rospy.loginfo("Markers published.")
            self.rate.sleep()

if __name__ == '__main__':
    try:
        pose_corrector = PoseCorrector()
        pose_corrector.run()
    except rospy.ROSInterruptException:
        pass
