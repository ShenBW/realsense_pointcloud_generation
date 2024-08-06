import numpy as np
import pyrealsense2 as rs
from matplotlib import pyplot as plt
import cv2
import open3d as o3d
from realsense_depth import DepthCamera
from utils import depth2PointCloud
from utils import create_point_cloud_file2

resolution_width, resolution_height = (640, 480)

is_post_process = True

clipping_distance = 1.00 #remove from the depth image all values above a given value (meters).
                         # Disable by giving negative value (default)

def main():

    depth_camera = DepthCamera(resolution_width, resolution_height)

    depth_scale = depth_camera.get_depth_scale()

    try: 
        while True:

            isGet, origin_depth_frame, color_frame = depth_camera.get_frame()
            if not isGet:
                print("Unable to get a frame")
                
            if is_post_process:
                depth_frame = depth_camera.filter_depth_frame(origin_depth_frame)
            else:
                depth_frame = origin_depth_frame
        
            # pointcloud in RGB camera coordinates
            points_xyz_rgb = depth2PointCloud(depth_frame, color_frame, depth_scale, clipping_distance)
            create_point_cloud_file2(points_xyz_rgb,"cloud.ply")
        
            pcd=o3d.geometry.PointCloud()
            pcd.points=o3d.utility.Vector3dVector(points_xyz_rgb[:,0:3])
            pcd.colors=o3d.utility.Vector3dVector(points_xyz_rgb[:,3:6]/255.0)
            o3d.visualization.draw_geometries([pcd])
        
    finally:
        depth_camera.release() # release rs pipeline


if __name__ == '__main__':
    main()
