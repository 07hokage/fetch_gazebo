import os
import cv2
import sys
import numpy as np
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from PIL import Image, ImageTk

def process_image(color_file, color_folder, poses_folder, map_folder, map_resolution, map_origin):
    color_path = color_file
    base_name = os.path.basename(color_file)
    pose_file = base_name.split("_color.png")[0] + "_pose.npz"
    pose_path = os.path.join(poses_folder, pose_file)

    map_file = base_name.split("_color.png")[0] + "_map.png"
    map_path = os.path.join(map_folder, map_file)

    window_size = 20
    resize_window = 30
    cv2.namedWindow("Map Image", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Map Image", 4000, 4000)
    if os.path.exists(pose_path):
        # Read the color image
        color_image = cv2.imread(color_path)
        map_image = cv2.imread(map_path)

        print(map_image.shape)
        # Load the corresponding pose file
        with np.load(pose_path) as data:
            robot_pose = data['RT_base']
        robot_position = robot_pose[0:2, 3]

        map_pixel_x = int((robot_position[0] - map_origin[0]) / map_resolution)
        map_pixel_y = int((robot_position[1] - map_origin[1]) / map_resolution)
        print(map_pixel_x, map_pixel_y)
        print(map_image[:, :, 0].shape)

        resized_color_image = cv2.resize(color_image, (resize_window, resize_window))
        map_image[map_pixel_y-window_size//2: map_pixel_y+window_size//2,
                  map_pixel_x-window_size//2: map_pixel_x+window_size//2,
                  2] = 255

        map_image[map_pixel_y-window_size//2: map_pixel_y+window_size//2,
                  map_pixel_x-window_size//2: map_pixel_x+window_size//2,
                  0:2] = 0

        # Comment this if you do not want to see the color image on the map and just a redbox
        map_image[map_pixel_y-resize_window//2: map_pixel_y+resize_window//2,
                  map_pixel_x-resize_window//2: map_pixel_x+resize_window//2,
                :] = resized_color_image
        
        cv2.imshow("Map Image", map_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        print(f"Robot Pose for {color_file}: {robot_position}")
    else:
        messagebox.showerror("Error", f"Pose file {pose_file} not found.")

def create_thumbnail(image_path, size=(100, 100)):
    with Image.open(image_path) as img:
        img.thumbnail(size)
        return ImageTk.PhotoImage(img)

def on_image_select(event, color_folder, poses_folder, map_folder, map_resolution, map_origin):
    selected_image = event.widget.cget("text")
    image_path = os.path.join(color_folder, selected_image)
    process_image(image_path, color_folder, poses_folder, map_folder, map_resolution, map_origin)

def main(data_folder):
    color_folder = os.path.join(data_folder, "color")
    poses_folder = os.path.join(data_folder, "pose")
    map_folder = os.path.join(data_folder, "map")
    # map_resolution = 0.05000000074505806
    # TODO: read resolution from the .yaml file 
    map_resolution = 0.1
    map_origin = (-100.0, -100.0)
    color_files = sorted([f for f in os.listdir(color_folder) if f.endswith(".png")])

    root = tk.Tk()
    root.title("Image Selector")

    label = tk.Label(root, text="Select an image:")
    label.pack()

    frame = tk.Frame(root)
    frame.pack(fill=tk.BOTH, expand=True)

    canvas = tk.Canvas(frame)
    scrollbar = ttk.Scrollbar(frame, orient=tk.VERTICAL, command=canvas.yview)
    scrollable_frame = ttk.Frame(canvas)

    scrollable_frame.bind(
        "<Configure>",
        lambda e: canvas.configure(
            scrollregion=canvas.bbox("all")
        )
    )

    canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
    canvas.configure(yscrollcommand=scrollbar.set)

    thumbnails = {}
    for color_file in color_files:
        thumbnail = create_thumbnail(os.path.join(color_folder, color_file))
        thumbnails[color_file] = thumbnail
        label = tk.Label(scrollable_frame, image=thumbnail, text=color_file, compound="top")
        label.pack(side="top", fill="both", padx=5, pady=5)
        label.bind("<Button-1>", lambda event, color_folder=color_folder, poses_folder=poses_folder, map_folder=map_folder, map_resolution=map_resolution, map_origin=map_origin: on_image_select(event, color_folder, poses_folder, map_folder, map_resolution, map_origin))

    canvas.pack(side="left", fill=tk.BOTH, expand=True)
    scrollbar.pack(side="right", fill=tk.Y)

    root.mainloop()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python script.py <data_folder>")
    else:
        main(sys.argv[1])
