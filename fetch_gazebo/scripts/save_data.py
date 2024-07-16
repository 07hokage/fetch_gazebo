from listener import ImageListener
import rospy
import numpy as np
import cv2
import os
import datetime
import time
import sys

class SaveData:
    def __init__(self, time_interval):
        rospy.init_node("img_listen_n_infer")
        self.listener = ImageListener("Fetch")
        time.sleep(5)
        self.time_delay = time_interval
        self.create_directory()

    def create_directory(self):
        # Create a directory named as the current date and time in the current working directory
        current_time = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.main_dir_name = os.path.join(os.getcwd(), current_time)
        self.color_dir_name = os.path.join(self.main_dir_name, "color")
        self.depth_dir_name = os.path.join(self.main_dir_name, "depth")
        self.pose_dir_name = os.path.join(self.main_dir_name, "pose")
        self.map_dir_name = os.path.join(self.main_dir_name, "map")
        os.makedirs(self.main_dir_name)
        os.makedirs(self.color_dir_name)
        os.makedirs(self.depth_dir_name)
        os.makedirs(self.pose_dir_name)
        os.makedirs(self.map_dir_name)

    def save_data(self):
        data_count = 0
        while not rospy.is_shutdown():
            rgb, depth, RT_camera, RT_laser, RT_robot, robot_velocity, RT_goal, map_data = self.listener.get_data_to_save()
            np.savez("{}_pose.npz".format(os.path.join(self.pose_dir_name, "{:06d}".format(data_count))), RT_camera=RT_camera, RT_robot=RT_robot, robot_velocity=robot_velocity, RT_goal=RT_goal)
            cv2.imwrite("{}_color.png".format(os.path.join(self.color_dir_name, "{:06d}".format(data_count))), rgb)
            cv2.imwrite("{}_depth.png".format(os.path.join(self.depth_dir_name, "{:06d}".format(data_count))), depth)
            cv2.imwrite("{}_map.png".format(os.path.join(self.map_dir_name, "{:06d}".format(data_count))), map_data.astype(np.uint8))
            rospy.sleep(self.time_delay)
            data_count += 1

if __name__ == "__main__":
    saver = SaveData(sys.argv[1])
    saver.save_data()