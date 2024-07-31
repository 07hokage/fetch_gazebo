# # Python program to explain cv2.resizeWindow() method 

# # Importing cv2 
# import cv2 

# # Path 
# path = '/home/ash/irvl.pgm'

# # Reading an image in default mode 
# image = cv2.imread(path,-1) 
# print(image.shape)
# # Naming a window 
# cv2.namedWindow("Resized_Window", cv2.WINDOW_NORMAL) 

# # Using resizeWindow() 
# cv2.resizeWindow("Resized_Window", 4000, 4000) 

# # Displaying the image 
# cv2.imshow("Resized_Window", image) 
# cv2.waitKey(0) 

# import cv2
# import numpy as np

# def preprocess_depth_array(depth_array):
#     # Replace np.nan with 0 or any other specific value
#     depth_array = np.nan_to_num(depth_array, nan=0.0, posinf=np.max(depth_array[np.isfinite(depth_array)]), neginf=0.0)
#     print(f"after preprocess")
#     print(depth_array[100:105, 100:105])
#     return depth_array

# def save_depth_array_as_image(depth_array, filename):
#     # Preprocess to handle np.nan and np.inf
#     depth_array = preprocess_depth_array(depth_array)

#     # Normalize the depth array to the range [0, 65535] for 16-bit PNG
#     # normalized_depth = (depth_array - np.min(depth_array)) / (np.max(depth_array) - np.min(depth_array))
#     # print(np.min(depth_array), np.max(depth_array))
#     # normalized_depth = (depth_array - np.min(depth_array)) / (12 - np.min(depth_array))
#     normalized_depth = (20-depth_array)/20
#     print(np.min(depth_array), np.max(depth_array))
#     depth_image = (normalized_depth * 255).astype(np.uint8)

#     # Save the image using OpenCV
#     cv2.imwrite(filename, depth_image)

# def read_depth_array_from_image(filename):
#     # Read the image using OpenCV
#     depth_image = cv2.imread(filename, cv2.IMREAD_UNCHANGED)

#     # Convert the image back to the original depth array
#     depth_array = depth_image.astype(np.float32) / 255.0

#     # Rescale to the original depth range if known
#     original_min = 0
#     original_max = 20
#     # depth_array = depth_array * (original_max - original_min) + original_min
#     depth_array = 20*(1-depth_array)

#     return depth_array

# # Example usage
# depth_array = np.random.rand(480, 640) * 10  # Replace with your depth array
# depth_array[100, 100] = np.nan  # Introduce a NaN value
# depth_array[200, 200] = np.inf  # Introduce an Inf value
# depth_array[300, 300] = -np.inf  # Introduce a -Inf value

# save_depth_array_as_image(depth_array, 'depth_image.png')
# restored_depth_array = read_depth_array_from_image('depth_image.png')

# print("Original depth array (sample):", depth_array[100:105, 100:105])
# print("Restored depth array (sample):", restored_depth_array[100:105, 100:105])

import numpy as np

def compute_xyz(depth_img, fx, fy, px, py, height, width):
    indices = np.indices((height, width), dtype=np.float32).transpose(1, 2, 0)
    z_e = depth_img
    x_e = (indices[..., 1] - px) * z_e / fx
    y_e = (indices[..., 0] - py) * z_e / fy
    xyz_img = np.stack([x_e, y_e, z_e], axis=-1)  # Shape: [H x W x 3]
    return xyz_img

# Example usage with a 4x4 depth image
depth_img = np.array([
    [0.0, 0.1, 1.0, 1.0],
    [1.0, 1.0, 1.0, 1.0],
    [1.0, 1.0, 1.0, 1.0],
    [1.0, 1.0, 1.0, 1.0]
], dtype=np.float32)

# Camera intrinsic parameters
fx = 1.0  # Focal length in x axis
fy = 1.0  # Focal length in y axis
px = 2.0  # Principal point x
py = 2.0  # Principal point y

# Image dimensions
height, width = depth_img.shape

# Compute the XYZ image
xyz_img = compute_xyz(depth_img, fx, fy, px, py, height, width)
xyz_img = xyz_img.reshape((-1,3))

print("Depth Image:")
print(depth_img)
print("\nXYZ Image:")
print(xyz_img)


