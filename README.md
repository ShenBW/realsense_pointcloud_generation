
## Installation

pip install open3d

pip install pyrealsense2

## Usage

Combine RGB information and depth data from the depth camera to synthesize a 3D point cloud with color information.

"""
python pointcloud_generator.py
"""

## Methods to Enhance the Quality of Depth Data

1. Set the depth module to manual exposure and increase the exposure time appropriately.

2. Post-process depth images using spatial filter and temporal filter.
