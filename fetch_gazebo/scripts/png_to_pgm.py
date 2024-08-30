
from PIL import Image
import numpy as np

# Open the PNG image
img = Image.open("map.png")

# Convert the image to grayscale
img_gray = img.convert("L")
# img_gray = img_gray.transpose(Image.FLIP_TOP_BOTTOM)
img_gray = img_gray.rotate(180, expand=True)
img_gray = img_gray.transpose(Image.FLIP_LEFT_RIGHT)
# Convert the grayscale image to a NumPy array
img_array = np.array(img_gray)
# img_array = img_array.T


# Apply the transformations:
# Black (0) in PNG should become Gray (127) in PGM
# White (255) in PNG should stay White (255) in PGM
# Gray (127) in PNG should become Black (0) in PGM
i1 = np.where(img_array == 0)  # Black to Gray
i2 = np.where(img_array == 155)  # Gray to Black
img_array[i1] = 155
img_array[i2] = 0
# Convert the NumPy array back to a PIL image
pgm_image = Image.fromarray(img_array)
print("save")
# Save the transformed image as a PGM file
pgm_image.save("map2.pgm")