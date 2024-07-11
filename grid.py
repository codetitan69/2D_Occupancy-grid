import cv2
import numpy as np
import matplotlib.pyplot as plt
import yaml

def image_to_occupancy_grid(image_path, grid_resolution=(512, 512)):
    # Load the color image
    color_image = cv2.imread(image_path)
    
    # Convert to grayscale
    gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
    
    # Resize the grayscale image to the desired occupancy grid resolution
    resized_image = cv2.resize(gray_image, grid_resolution, interpolation=cv2.INTER_AREA)
    
    # Apply edge detection (Canny)
    edges = cv2.Canny(resized_image, 100, 200)
    
    # Apply adaptive thresholding
    adaptive_thresh = cv2.adaptiveThreshold(resized_image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                            cv2.THRESH_BINARY_INV, 11, 2)
    
    # Combine edge detection and adaptive thresholding
    combined = cv2.bitwise_or(edges, adaptive_thresh)
    
    # Convert to binary occupancy grid (1 = occupied, 0 = free)
    occupancy_grid = (combined > 0).astype(np.uint8) * 255  # Multiply by 255 to get 0 and 255 values
    
    return occupancy_grid

def save_to_pgm(occupancy_grid, output_path):
    cv2.imwrite(output_path, occupancy_grid)

def save_to_yaml(yaml_path, grid_resolution):
    data = {
        'resolution': grid_resolution,
        'format': 'pgm'
    }
    with open(yaml_path, 'w') as yaml_file:
        yaml.dump(data, yaml_file, default_flow_style=False)

# Example usage
image_path = 'stitched/final.png'  # Replace with the path to your image
pgm_output_path = 'grid/grid.pgm'  # Output .pgm file path
yaml_output_path = 'grid/grid.yaml'  # Output YAML file path

occupancy_grid = image_to_occupancy_grid(image_path)
save_to_pgm(occupancy_grid, pgm_output_path)
save_to_yaml(yaml_output_path, occupancy_grid.shape)

