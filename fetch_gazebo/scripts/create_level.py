import os
from os.path import join
import sys

import cv2
import numpy as np

import networkx as nx
from networkx import Graph


from utils import (
    pose_in_map_frame,
    pose_to_map_pixel,
    read_map_image,
    read_map_metadata,
    display_map_image,
    denormalize_depth_image,
    is_nearby_in_map,
    save_graph_json,
    read_graph_json,
    read_and_visualize_graph,
    visualize_graph

)


class CreateLevelGraph:
    def __init__(self, root_dir, add_root) -> None:
        self.root_dir = root_dir
        self.object_classes = ["table", "chair", "door"]
        self.graph = Graph()
        self.prepare_data()
        self.create_level(add_root)

    def prepare_data(self):
        """
        Create paths for color, pose, depth, map, segments directories.
        Also create a handler to access segmented image based on class and segment id
        """
        assert os.path.isdir(self.root_dir)
        self.color_dir = join(self.root_dir, "color")
        self.pose_dir = join(self.root_dir, "pose")
        self.depth_dir = join(self.root_dir, "depth")
        self.map_dir = join(self.root_dir, "map")
        self.segments_dir = join(self.root_dir, "segments")
        self.final_map = join(self.root_dir, "map.png")
        self.final_map_metadatafile = join(self.root_dir,"map.yaml")

    def get_pose(self, node_id, class_name, mask_image_name):
        """
        computes mean pose of the segment pointcloud and return poses
        Args: node_id: segment folder id
        return value: robot pose as [x,y,z], mean pose of segment as [x,y,z] in map frame
        """
        with np.load(join(self.pose_dir, f"{node_id}_pose.npz")) as data:
            robot_pose = data["RT_base"]
            cam_pose = data["RT_camera"]

        depth_image = cv2.imread(
            join(self.depth_dir, f"{node_id}_depth.png"), 0
        )

        depth_array = denormalize_depth_image(depth_image, max_depth=20)
        # print(depth_array.max())

        segment = cv2.imread(
            join(self.segments_dir, f"{node_id}/{class_name}/{mask_image_name}"), 0
        )
        # segment = None
        return robot_pose.tolist(), pose_in_map_frame(
            cam_pose, robot_pose, depth_array, segment=segment
        )
    
    def extract_detection_confidence(self, mask_image_name):
        confidence = float((mask_image_name.split('_')[-1]).split('.png')[0])
        return confidence
    
    def create_level(self, add_root=True):
        """
        creates graph of segmented class instances
        Args: add_root.  if true, classes are added as root nodes. this is for viz only
        graph can be accessed by self.graph
        """
        # read and sort segment folders
        self.segment_folders = os.listdir(self.segments_dir)
        self.segment_folders.sort()
        print(f"{len(self.segment_folders)} segment folders")
        if add_root:
            self.graph.add_node("table")
            self.graph.add_node("chair")
            self.graph.add_node("door")

        self.pose_list = {"table":[], "chair":[], "door":[]}
        self.threshold = {"table": 1.3, "chair":0.5
                          , "door": 1}
        for directory in self.segment_folders:
            for class_directory in os.listdir(join(self.segments_dir, directory)):
                for sub_id, mask_image_name in enumerate(os.listdir(
                    join(self.segments_dir, join(directory, class_directory))
                )):
                    robot_pose, pose = self.get_pose(
                        directory, class_directory, mask_image_name
                    )
                    if pose is None:
                        continue
                    self.pose_list[class_directory], _is_nearby = is_nearby_in_map(
                        self.pose_list[class_directory], pose, threshold=self.threshold[class_directory]
                    )
                    if _is_nearby:
                        continue
                    confidence_value = self.extract_detection_confidence(mask_image_name=mask_image_name)
                    self.graph.add_node(
                        f"{class_directory}_{directory}_{sub_id}",
                        id=f"{class_directory}_{directory}_{sub_id}",
                        pose=pose,
                        robot_pose=robot_pose,
                        category=class_directory,
                        confidence = confidence_value
                    )
                    if add_root:
                        self.graph.add_edge(class_directory,f"{class_directory}_{directory}_{sub_id}" )
                    self.pose_list[class_directory].append(pose)
                    
        # print(self.graph.nodes())



if __name__ == "__main__":
    level = CreateLevelGraph(sys.argv[1], add_root=False)
    save_graph_json(level.graph)
    read_and_visualize_graph(level.final_map, level.final_map_metadatafile,  on_map=True, catgeories=["table","chair", "door"])
