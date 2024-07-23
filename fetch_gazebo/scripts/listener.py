#!/usr/bin/env python
"""ROS image listener"""

import threading
import numpy as np
# from scipy.io import savemat

import rospy
import tf
import tf2_ros
import message_filters
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cv_bridge import CvBridge

from ros_utils import ros_qt_to_rt

from nav_msgs.msg import  Odometry, OccupancyGrid
import time
import yaml

lock = threading.Lock()



class ImageListener:

    def __init__(self, camera='Fetch'):

        self.cv_bridge = CvBridge()

        self.im = None
        self.depth = None
        self.rgb_frame_id = None
        self.rgb_frame_stamp = None

        # initialize a node
        self.tf_listener = tf.TransformListener()      

        self.base_frame = 'base_link'
        rgb_sub = message_filters.Subscriber('/head_camera/rgb/image_raw', Image, queue_size=10)
        depth_sub = message_filters.Subscriber('/head_camera/depth_registered/image_raw', Image, queue_size=10)
        msg = rospy.wait_for_message('/head_camera/rgb/camera_info', CameraInfo)
        self.camera_frame = 'head_camera_rgb_optical_frame'
        self.target_frame = self.base_frame
        self.lidar_pub = rospy.Publisher("/lidar_pc", PointCloud2, queue_size=10)
        rospy.Subscriber("/odom", Odometry, self.odometry_callback)
        self.robot_pose = None
        self.robot_velocity = None
        self.current_goal = np.zeros((4,4))
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.map_info_saved=0


        # update camera intrinsics
        intrinsics = np.array(msg.K).reshape(3, 3)
        self.intrinsics = intrinsics
        self.fx = intrinsics[0, 0]
        self.fy = intrinsics[1, 1]
        self.px = intrinsics[0, 2]
        self.py = intrinsics[1, 2]
        print(intrinsics)

        queue_size = 1
        slop_seconds = 0.1
        ts = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size, slop_seconds)
        ts.registerCallback(self.callback_rgbd)

    def odometry_callback(self, odometry):
        self.robot_pose = ros_qt_to_rt( [odometry.pose.pose.orientation.x,
                                            odometry.pose.pose.orientation.y,
                                            odometry.pose.pose.orientation.z,
                                            odometry.pose.pose.orientation.w],
                                            [odometry.pose.pose.position.x,
                                        odometry.pose.pose.position.y,
                                        odometry.pose.pose.position.z])
        self.robot_velocity = np.array([odometry.twist.twist.linear.x,
                                         odometry.twist.twist.linear.y,
                                         odometry.twist.twist.linear.z, 
                                          odometry.twist.twist.angular.x,
                                          odometry.twist.twist.angular.y,
                                          odometry.twist.twist.angular.z ])
    
    def callback_rgbd(self, rgb, depth):
    
        # get camera pose in base
        try:
             #print("callback")
             trans, rot = self.tf_listener.lookupTransform(self.base_frame, self.camera_frame, rospy.Time(0))
             RT_camera = ros_qt_to_rt(rot, trans)
             self.trans_l, self.rot_l = self.tf_listener.lookupTransform(self.base_frame, 'laser_link', rospy.Time(0))
             RT_laser = ros_qt_to_rt(self.rot_l, self.trans_l)
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Update failed... " + str(e))
            RT_camera = None
            RT_laser = None             

        if depth.encoding == '32FC1':
            depth_cv = self.cv_bridge.imgmsg_to_cv2(depth)
        elif depth.encoding == '16UC1':
            depth_cv = self.cv_bridge.imgmsg_to_cv2(depth).copy().astype(np.float32)
            depth_cv = depth_cv.copy().astype(np.float32)
            depth_cv /= 1000.0
        else:
            rospy.logerr_throttle(
                1, 'Unsupported depth type. Expected 16UC1 or 32FC1, got {}'.format(
                    depth.encoding))
            return

        im = self.cv_bridge.imgmsg_to_cv2(rgb, 'bgr8')
        with lock:
            self.im = im.copy()
            self.depth = depth_cv.copy()
            self.rgb_frame_id = rgb.header.frame_id
            self.rgb_frame_stamp = rgb.header.stamp
            self.height = depth_cv.shape[0]
            self.width = depth_cv.shape[1]
            self.RT_camera = RT_camera
            self.RT_laser = RT_laser

    def goal_callback(self, goal):
        '''
        -----------goal-format-----------
        target_pose:
            position: 
                x: 1.3249285221099854
                y: 0.3744138479232788
                z: 0.0
            orientation: 
                x: 0.0
                y: 0.0
                z: -0.0888939371846976
                w: 0.9960410975114445

        This call back becomes active only at the instant a goal is published.
        so at the moment user gives a goal, this function is called. To maintain continuity, 
        Until a new goal occurs, self.goal represents currently active goal
        '''
        self.current_goal = ros_qt_to_rt([goal.goal.target_pose.pose.orientation.x,
                                    goal.goal.target_pose.pose.orientation.y,
                                    goal.goal.target_pose.pose.orientation.z,
                                    goal.goal.target_pose.pose.orientation.w],
                                    [goal.goal.target_pose.pose.position.x,
                                 goal.goal.target_pose.pose.position.y,
                                 goal.goal.target_pose.pose.position.z])

    def map_callback(self, map_data):
        # Save map image
        rospy.loginfo("Received map data")
        map_array = np.array(map_data.data, dtype=np.int8).reshape((map_data.info.height, map_data.info.width))
        map_image = 255- map_array
        if self.map_info_saved==0:
            # Save map metadata
            self.map_origin = [map_data.info.origin.position.x, map_data.info.origin.position.y,]
            self.map_resolution = map_data.info.resolution
            self.resize_in_meter = 20
            self.resize_factor = ( 2 * abs(self.map_origin[0]) )/ self.resize_in_meter
            map_metadata = {
                'image': 'initial_map.png',
                'resolution': self.map_resolution,
                'origin': self.map_origin,
                'occupied_thresh': 0.65,
                'free_thresh': 0.196,
                'negate': 0,
                'resizeFactor': self.resize_factor
            }
            print("resize factor",self.resize_factor)
            with open('initial_map.yaml', 'w') as yaml_file:
                yaml.dump(map_metadata, yaml_file, default_flow_style=False)
            self.map_info_saved=1
<<<<<<< HEAD
=======
        #self.map_img = map_image[1200:2800,1200:2800]  ## savinf full map takes only 28KB
>>>>>>> d3acc88e1f8f539526585dbc4d02500e26101b7e
        self.map_img = map_image

    def get_data_to_save(self):

        with lock:
            if self.im is None:
                return None, None, None, None, None, self.intrinsics
            im_color = self.im.copy()
            depth_image = self.depth.copy()
            RT_camera = self.RT_camera.copy()
            RT_laser = self.RT_laser.copy()
            RT_robot = self.robot_pose.copy()
            RT_goal = self.current_goal.copy()
            robot_velocity = self.robot_velocity.copy()
            map_data = self.map_img.copy()
        return im_color, depth_image, RT_camera, RT_laser, RT_robot, robot_velocity, RT_goal, map_data






if __name__ == '__main__':
    # test_basic_img()
    rospy.init_node("image_listener")    
    listener = ImageListener()
    time.sleep(3)
