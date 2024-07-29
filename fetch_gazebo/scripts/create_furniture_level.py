import os
import cv2
import sys
import numpy as np
from utils import (
    pose_to_map_pixel,
    read_map_image,
    read_map_metadata,
    display_map_image,
    is_nearby,
    denormalize_depth_image,
    pose_in_map_frame,
)

from anytree import Node, RenderTree
from anytree.exporter import UniqueDotExporter


class CreateLevelOne:
    def __init__(self, root_dir) -> None:
        self.root_dir = root_dir
        assert os.path.isdir(root_dir)
        self.segments_dir = os.path.join(self.root_dir, "segments")
        self.color_dir = os.path.join(self.root_dir, "color")
        self.pose_dir = os.path.join(self.root_dir, "pose")
        self.depth_dir = os.path.join(self.root_dir, "depth")
        self.create_level()

    def get_pose(self, data_id):
        with np.load(os.path.join(self.pose_dir, f"{data_id}_pose.npz")) as data:
            robot_pose = data["RT_base"]
            cam_pose = data["RT_camera"]
        depth_image = cv2.imread(
            os.path.join(self.depth_dir, f"{data_id}_depth.png"), cv2.IMREAD_UNCHANGED
        ).astype(np.float32)
        depth_array = denormalize_depth_image(depth_image, max_depth=6)
        return pose_in_map_frame(cam_pose, robot_pose, depth_array, segment=None)

    def create_level(self):
        self.segment_folders = os.listdir(self.segments_dir)
        self.segment_folders.sort()
        print(self.segment_folders)
        for dir in self.segment_folders:
            if self.segment_folders.index(dir) == 0:
                self.root = Node(dir, pose=self.get_pose(dir))
                prev_node = self.root
            else:
                pose = self.get_pose(dir)
                if is_nearby(prev_node.pose, pose, threshold=0.5):
                    continue
                else:
                    next_node = Node(dir, parent=prev_node, pose=self.get_pose(dir))
                    prev_node = next_node

    def visualize_simple_tree(self, save_img=True):
        print(RenderTree(self.root))
        if save_img:
            UniqueDotExporter(self.root).to_picture("root.png")

    def visualize_tree_on_map(self, map_file_path="", window_size=5):
        map_image = read_map_image(map_file_path)
        print(map_image.shape)
        metadata_file_path = map_file_path.split(".png")[0] + ".yaml"
        map_metadata = read_map_metadata(metadata_file_path)
        for _, _, node in RenderTree(self.root):
            # print("pose", node.pose)
            x, y = pose_to_map_pixel(map_metadata, node.pose)
            map_image[
                y - window_size // 2 : y + window_size // 2,
                x - window_size // 2 : x + window_size // 2,
                :,
            ] = [255, 0, 0]
        display_map_image(map_image)

    def save_tree(self):
        UniqueDotExporter(self.root).to_picture("root.png")


if __name__ == "__main__":
    level = CreateLevelOne(sys.argv[1])
    level.visualize_tree_on_map(map_file_path="/home/ash/Downloads/2024-07-23_00-22-59/map/000110_map.png")
    level.save_tree()
