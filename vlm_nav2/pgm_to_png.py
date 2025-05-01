import cv2
import yaml
import numpy as np

def load_yaml(yaml_file):
    with open(yaml_file, 'r') as f:
        return yaml.safe_load(f)

def world_to_pixel(x, y, origin_x, origin_y, resolution, img_height):
    """
    Converts world coordinates (in metres) to pixel coordinates.
    
    Args:
      x, y: World coordinates in metres.
      origin_x, origin_y: World coordinates corresponding to pixel (0,0) in the map.
      resolution: Metres per pixel.
      img_height: Height of the image (to account for y-axis inversion).
    
    Returns:
      (pixel_x, pixel_y): The corresponding pixel coordinates.
    """
    pixel_x = int((x - origin_x) / resolution)
    # For pixel_y, image y=0 is at the top, so we invert the y coordinate:
    pixel_y = int(img_height - ((y - origin_y) / resolution))
    return pixel_x, pixel_y

def annotate_map(pgm_filename, yaml_filename, output_png):
    # Load YAML metadata
    metadata = load_yaml(yaml_filename)
    pgm_image_path = metadata['image']
    resolution = metadata['resolution']
    origin = metadata['origin']
    origin_x, origin_y = origin[0], origin[1]  # Extract origin coordinates from the YAML data

    # Load PGM image as grayscale
    img = cv2.imread(pgm_filename, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"Unable to load image {pgm_filename}")

    img_height, img_width = img.shape

    # Convert to a BGR image so we can draw in color
    color_img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    # Compute the pixel coordinates for the origin
    origin_pixel = world_to_pixel(0, 0, origin_x, origin_y, resolution, img_height)
    # Draw a red filled circle at the origin (0,0) in world coordinates
    cv2.circle(color_img, origin_pixel, radius=5, color=(0, 0, 255), thickness=-1)

    # Save as PNG
    cv2.imwrite(output_png, color_img)
    print(f"Annotated PNG saved as {output_png}. Origin pixel: {origin_pixel}")

if __name__ == "__main__":
    # Adjust these filenames as needed
    pgm_filename = "home_map_1.pgm"
    yaml_filename = "home_map_1.yaml"
    output_png = "home_map_1_annotated.png"

    annotate_map(pgm_filename, yaml_filename, output_png)
