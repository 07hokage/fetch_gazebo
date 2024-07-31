import os
import cv2
import sys
import numpy as np
from utils import compute_xyz
import rospy
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, Image
from std_msgs.msg import Header
from listener import ImageListener
import time
import ros_numpy
from ros_utils import *
import tf
depth_ = None

def depth_callback(depth):
    global depth_
    print(type(depth))
    depth_ = ros_numpy.numpify(depth)
    print(type(depth_))

def main(data_folder):
    rospy.init_node("depth_to_pc")
    tf_listener = tf.TransformListener()
    time.sleep(3)
    trans, rot = tf_listener.lookupTransform("base_link", "head_camera_rgb_optical_frame", rospy.Time(0))
    RT_camera = ros_qt_to_rt(rot, trans)
    # listener_ = ImageListener()
    # time.sleep(3)
    depth_folder = os.path.join(data_folder, "depth")
    pose_folder = os.path.join(data_folder, "pose")
    print(f"depth folder {depth_folder}\n")
    depth_files = sorted([f for f in os.listdir(depth_folder) if f.endswith(".png")])
    print(depth_files)
    depth_sub = rospy.Subscriber('/head_camera/depth_registered/image_raw', Image, depth_callback)
    intrinsics = [[574.0527954101562, 0.0, 319.5],[0.0, 574.0527954101562, 239.5], [0.0, 0.0, 1.0]]
    fx = intrinsics[0][0]
    fy = intrinsics[1][1]
    px = intrinsics[0][2]
    py = intrinsics[1][2]

    time.sleep(3)
    pc_header = Header()
    pc_pub = rospy.Publisher("depth_to_pc_",PointCloud2, queue_size=10)
    default = "000064_depth.png"
    print(f"default  {default}")
    for depth_path in depth_files:
        trans, rot = tf_listener.lookupTransform("base_link", "head_camera_depth_optical_frame", rospy.Time(0))
        RT_camera = ros_qt_to_rt(rot, trans)
        print(RT_camera)
        # time.sleep(10)
        trans_l, rot_l = tf_listener.lookupTransform("map", "base_link", rospy.Time(0))
        RT_base = ros_qt_to_rt(rot_l, trans_l)
        print(RT_base)
        # # depth_image = cv2.imread(os.path.join(depth_folder,default),0)
        # # _, depth_image, _, _, _, _, _, _ = listener_.get_data_to_save()
        # time.sleep(20)
        depth_image = depth_
        # depth_image[depth_image is np.nan] = 0.0
        print(f"depth_iameg shape {depth_image.shape}")
        xyz_array = compute_xyz(depth_image, fx,fy,px,py,480,640)
        print(f"xya array sahpe {xyz_array.shape}")
        xyz_array = xyz_array.reshape((-1,3))
        # xyz_array = np.nan_to_num(xyz_array, nan=0)
        mask = ~(np.all(xyz_array == [0.0, 0.0, 0.0], axis=1))
        print("mask", mask)
        xyz_array = xyz_array[mask]
        print(xyz_array.shape)
        # pose_file = depth_path.split("_depth.png")[0] + "_pose.npz"
        # with np.load(os.path.join(pose_folder,pose_file)) as data:
        #     RT_camera = data["RT_camera"]
        
        xyz_base = np.dot(RT_camera[:3,:3],xyz_array.T).T
        xyz_base +=RT_camera[:3,3]

        # xyz_world = np.dot(RT_base[:3,:3],xyz_base.T).T
        # xyz_world +=RT_base[:3,3]
        
        


        pc_header.stamp = rospy.Time.now()
        pc_header.frame_id = "base_link"
        pc_message = pc2.create_cloud_xyz32(pc_header, xyz_base)
        pc_pub.publish(pc_message)
        print("publishged")
        rospy.sleep(0.5)

if __name__ == "__main__":
    print(f"depth to pc convertion")
    main(sys.argv[1])






