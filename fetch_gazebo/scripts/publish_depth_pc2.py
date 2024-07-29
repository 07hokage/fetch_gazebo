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


def main(data_folder):
    rospy.init_node("depth_to_pc")

    depth_folder = os.path.join(data_folder, "depth")
    pose_folder = os.path.join(data_folder, "pose")
    color_folder = os.path.join(data_folder, "color")
    print(f"depth folder {depth_folder}\n")
    depth_files = sorted([f for f in os.listdir(depth_folder) if f.endswith(".png")])
    print(depth_files)
    intrinsics = [[574.0527954101562, 0.0, 319.5],[0.0, 574.0527954101562, 239.5], [0.0, 0.0, 1.0]]
    # intrinsics = [[554.254691191187, 0.0, 320.5],[0.0, 554.254691191187, 240.5], [0.0, 0.0, 1.0]]
    fx = intrinsics[0][0]
    fy = intrinsics[1][1]
    px = intrinsics[0][2]
    py = intrinsics[1][2]
    time.sleep(3)
    pc_header = Header()
    pc_pub = rospy.Publisher("depth_to_pc_",PointCloud2, queue_size=10)
    # default = "000064_depth.png"
    # print(f"default  {default}")
    for depth_path in depth_files:
        print(f"depth image {depth_path}")
        pose_file = depth_path.split("_depth.png")[0] + "_pose.npz"
        depth_image = cv2.imread(os.path.join(depth_folder,depth_path),0)
        depth_image = 6-(6*(depth_image/255))
        with np.load(os.path.join(pose_folder,pose_file)) as data:
            RT_camera = data["RT_camera"]
            RT_robot = data["RT_base"]
            # depth_image = data["depth"]
        print(depth_image)
        # depth_image /= 100
        color_file =  depth_path.split("_depth.png")[0] + "_color.png"
        color_image = cv2.imread(os.path.join(color_folder, color_file))
        print(f"depth_iameg shape {depth_image.shape}")
        
        max_= depth_image.max()
        min_=depth_image.min()
        print(f"------ {max_}, {min_} ----- \n")
        # depth_image_normalized_255 = 255*(2-depth_image )/2
        # print(depth_image.dtype)
        # cv2.imshow("depthimage", depth_image_normalized_255)
        # cv2.waitKey(0)
        xyz_array = compute_xyz(depth_image, fx,fy,px,py,480,640)
        print(f"xya array sahpe {xyz_array.shape}")
        xyz_array = xyz_array.reshape((-1,3))
        
        

        print(RT_camera)
        print(RT_robot)
        
        xyz_base = np.dot(RT_camera[:3,:3],xyz_array.T).T
        xyz_base +=RT_camera[:3,3]

        # xyz_base = np.dot(RT_camera[:3, :3], xyz_array.T).T + RT_camera[:3, 3]


        xyz_world = np.dot(RT_robot[:3,:3],xyz_base.T).T
        xyz_world +=RT_robot[:3,3]

        pc_header.stamp = rospy.Time.now()
        pc_header.frame_id = "map"
        pc_message = pc2.create_cloud_xyz32(pc_header, xyz_world)
        pc_pub.publish(pc_message)
        print("publishged")
        cv2.imshow("rgb image", color_image)
        cv2.waitKey(3)
        # time.sleep(1)
        # rospy.sleep(0.5)

if __name__ == "__main__":
    print(f"depth to pc convertion")
    main(sys.argv[1])






