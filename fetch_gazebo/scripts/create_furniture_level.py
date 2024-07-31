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
        # segment = cv2.imread(
        #     os.path.join(self.segments_dir, f"{data_id}/mask_0.png"),0)
        segment = None
        return robot_pose, pose_in_map_frame(cam_pose, robot_pose, depth_array, segment=segment)

    def create_level(self):
        self.segment_folders = os.listdir(self.segments_dir)
        self.segment_folders.sort()
        for dir in self.segment_folders:
            if self.segment_folders.index(dir) == 0:
                robot_pose, pose = self.get_pose(dir)
                self.root = Node(dir, pose=pose, robot_pose=robot_pose)
                prev_node = self.root
            else:
                robot_pose, pose = self.get_pose(dir)
                if is_nearby(prev_node.pose, pose, threshold=0.5):
                    continue
                else:
                    next_node = Node(dir, parent=prev_node, pose=pose, robot_pose=robot_pose)
                    prev_node = next_node

    def visualize_simple_tree(self, save_img=True):
        # print(RenderTree(self.root))
        if save_img:
            UniqueDotExporter(self.root).to_picture("root.png")

    def visualize_tree_on_map(self, map_file_path="", window_size=5):
        map_image = read_map_image(map_file_path)
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
            x_robot, y_robot = pose_to_map_pixel(map_metadata, [node.robot_pose[0,3], node.robot_pose[1,3]])
            map_image[
                y_robot - window_size // 2 : y_robot + window_size // 2,
                x_robot - window_size // 2 : x_robot + window_size // 2,
                :,
            ] = [0, 255, 0]
            display_map_image(map_image)

    def save_tree(self):
        UniqueDotExporter(self.root).to_picture("root.png")
    # def save_tree_yaml(self, filepath=None):
    #     level_dict = DictExporter().export(self.root)
    #     print(level_dict)
    #     # print(yaml.dump(level_dict, default_flow_style=False))
    #     with open(filepath, 'w') as file:
    #         yaml.dump(level_dict, file)

    def save_tree_json(self, filepath=None):
        exporter = JsonExporter(indent=2)
        tree = exporter.export(self.root)
        with open(filepath, "w") as jsonfile:
            json.dump(tree, jsonfile)

    def load_tree_json(self, filepath=None):
        assert filepath is not None
        with open(filepath, "r") as jsonfile:
            tree = json.load(jsonfile)
        tree = json.loads(tree)
        return tree


    # def load_tree_yaml(self, filepath=None):
    #     with open(filepath, "r") as file:
    #         tree_data = yaml.safe_load(file)
    #     # print(tree_data.keys())
    #     return tree_data
    
    def print_names(self, tree):
        print(tree["name"])
        try:
            self.print_names(tree['children'][0])
        except KeyError:
            pass

        
            

if __name__ == "__main__":
    level = CreateLevelOne(sys.argv[1])
    level.visualize_tree_on_map(map_file_path="/home/ash/irvl/test/installations/fetch_related/fetch_ws/src/fetch_gazebo/fetch_gazebo/scripts/2024-07-29_23-21-38/map/000023_map.png")
    level.save_tree()
