import numpy as np
import pyrealsense2 as rs
from matplotlib import pyplot as plt
import cv2
import open3d as o3d

def depth2PointCloud(depth_frame, color_frame, depth_scale, clipping_distance):
    """Transform a depth image into a point cloud with one point for each
    pixel in the image, using the camera transform for a camera
    centred at cx, cy with field of view fx, fy.

    depth is a 2-D ndarray with shape (rows, cols) containing
    depths from 1 to 254 inclusive. The result is a 3-D array with
    shape (rows, cols, 3). Pixels with invalid depth in the input have
    NaN for the z-coordinate in the result.

    """
    
    intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
    depth_image = np.asanyarray(depth_frame.get_data()) * depth_scale 
    color_image = np.asanyarray(color_frame.get_data())
    print("The size of depth image: ", depth_image.shape)
    print("The size of color image: ", color_image.shape)
    rows, cols = depth_image.shape

    c, r = np.meshgrid(np.arange(cols), np.arange(rows), sparse=True)
    r = r.astype(float)
    c = c.astype(float)
    
    valid = (depth_image > 0) & (depth_image < clipping_distance) # remove from the depth image all values above a given value (meters).
    valid = np.ravel(valid)
    z = depth_image 
    x =  z * (c - intrinsics.ppx) / intrinsics.fx
    y =  z * (r - intrinsics.ppy) / intrinsics.fy
   
    z = np.ravel(z)[valid]
    x = np.ravel(x)[valid]
    y = np.ravel(y)[valid]
    
    r = np.ravel(color_image[:,:,0])[valid]
    g = np.ravel(color_image[:,:,1])[valid]
    b = np.ravel(color_image[:,:,2])[valid]
    
    pointsxyzrgb = np.dstack((x, y, z, b, g, r))
    pointsxyzrgb = pointsxyzrgb.reshape(-1,6)

    return pointsxyzrgb

# Function to create point cloud file
def create_point_cloud_file2(vertices, filename):
    ply_header = '''ply
		format ascii 1.0
		element vertex %(vert_num)d
		property float x
		property float y
		property float z
		property uchar red
		property uchar green
		property uchar blue
		end_header
		'''
    with open(filename, 'w') as f:
        f.write(ply_header %dict(vert_num=len(vertices)))
        np.savetxt(f,vertices,'%f %f %f %d %d %d')  

def loadPointCloud():

    pcd = o3d.io.read_point_cloud("mame.ply") # Read the point cloud
    # Visualize the point cloud within open3d
    o3d.visualization.draw_geometries([pcd]) 

    # Convert open3d format to numpy array
    # Here, you have the point cloud in numpy format. 
    point_cloud_in_numpy = np.asarray(pcd.points) 
    print(point_cloud_in_numpy)
