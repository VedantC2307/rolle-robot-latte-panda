#!/usr/bin/env python3
"""
visualize_goal.py

Load a map image (PGM) and its YAML, prompt for a goal pose (meters),
convert to pixel coordinates, and display the goal on the map.
"""

import argparse
import yaml
from PIL import Image
import matplotlib.pyplot as plt

def main():
    parser = argparse.ArgumentParser(
        description='Visualize a Nav2 goal on a static map')
    parser.add_argument('--map', required=True,
                        help='Path to map image (PGM)')
    parser.add_argument('--yaml', required=True,
                        help='Path to map YAML file')
    args = parser.parse_args()

    # Load map metadata
    with open(args.yaml, 'r') as f:
        data = yaml.safe_load(f)
    resolution = data['resolution']
    origin = data.get('origin', [0.0, 0.0, 0.0])
    ox, oy = origin[0], origin[1]

    # Load map image
    img = Image.open(args.map)
    W, H = img.size

    # Prompt for goal in meters
    x = float(input('Enter goal x (meters): '))
    y = float(input('Enter goal y (meters): '))

    # Convert to pixel coordinates
    u = (x - ox) / resolution - 0.5
    v = H - ((y - oy) / resolution) - 0.5
    u_px = int(round(u))
    v_px = int(round(v))

    print(f'Pixel coordinates: u={u_px}, v={v_px} (of image size {W}×{H})')

    # Plot the map and overlay the goal
    fig, ax = plt.subplots()
    ax.imshow(img, origin='upper')
    ax.scatter([u_px], [v_px], marker='x', s=100)
    ax.set_title(f'Goal at pixel ({u_px}, {v_px})')
    ax.axis('off')
    plt.show()

if __name__ == '__main__':
    main()
