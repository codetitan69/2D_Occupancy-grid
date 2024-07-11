from PIL import Image
import numpy as np
import cv2

# Load the images
image_path_1 = "saved_images/image2.jpg"
image_path_2 = "saved_images/image1.jpg"

image_path_3 = "saved_images/image3.jpg"
image_path_4 = "saved_images/image4.jpg"

def stitch (image_path1,image_path2):

    image1 = Image.open(image_path1)
    image2 = Image.open(image_path2)

    # Convert images to numpy arrays
    image1_array = np.array(image1)
    image2_array = np.array(image2)

    # Calculate the best column to stitch the images
    best_column, overlap_start = find_best_column(image1_array, image2_array)

    # Create a new blank image to hold the stitched result
    stitched_image = Image.new('RGB', (image1_array.shape[1] + image2_array.shape[1] - overlap_start, image1_array.shape[0]))

    # Paste the first image onto the stitched image
    stitched_image.paste(image1, (0, 0))

    # Paste the second image onto the stitched image at the calculated position
    stitched_image.paste(image2, (best_column, 0))

    return stitched_image

# Define the overlap calculation method
def find_best_column(image1_array, image2_array):
    height, width, _ = image1_array.shape
    min_diff = float('inf')
    best_column = 0
    overlap_start = 0
    
    for overlap in range(width // 2, width):
        diff = np.sum(np.abs(image1_array[:, width - overlap:, :] - image2_array[:, :overlap, :]))
        if diff < min_diff:
            min_diff = diff
            best_column = width - overlap
            overlap_start = overlap

    return best_column, overlap_start

def rotate (path):
    image = cv2.imread(path)
    rotated_image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
    cv2.imwrite(path, rotated_image)

def rotate_back (path):
    image = cv2.imread(path)
    rotated_image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
    cv2.imwrite(path, rotated_image)


# Save and display the stitched image
stitched_image_path = "stitched/final.png"

stitched_image1 = stitch(image_path_1,image_path_2)
stitched_image1.save("stitched/bottom.png")

stitched_image2 = stitch(image_path_4,image_path_3)
stitched_image2.save("stitched/top.png")

rotate("stitched/bottom.png")
rotate("stitched/top.png")

final = stitch("stitched/bottom.png","stitched/top.png")
final.save(stitched_image_path)
rotate_back(stitched_image_path)