import pyrealsense2 as rs
import numpy as np

class DepthCamera:
    def __init__(self, resolution_width, resolution_height):
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        config = rs.config()
        
        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
       
        # Getting the depth sensor's depth scale (see rs-align example for explanation)
        depth_sensor = device.first_depth_sensor()
        # Get depth scale of the device
        self.depth_scale = depth_sensor.get_depth_scale()
        
        # Turn off auto exposure
        depth_sensor.set_option(rs.option.enable_auto_exposure, 0)
        # Set exposure time (microseconds)
        depth_sensor.set_option(rs.option.exposure, 8500)
        
        # Create an align object
        align_to = rs.stream.color
        self.align = rs.align(align_to)
        
        device_product_line = str(device.get_info(rs.camera_info.product_line))
        print("device product line:", device_product_line)
        
        config.enable_stream(rs.stream.depth,  resolution_width,  resolution_height, rs.format.z16, 30)
        config.enable_stream(rs.stream.color,  resolution_width,  resolution_height, rs.format.bgr8, 30)
        
        # Start streaming
        self.pipeline.start(config)
        
    def get_depth_scale(self):
        """
        "scaling factor" refers to the relation between depth map units and meters; 
        it has nothing to do with the focal length of the camera.
        Depth maps are typically stored in 16-bit unsigned integers at millimeter scale, thus to obtain Z value in meters, the depth map pixels need to be divided by 1000.
        """
        return self.depth_scale  

    def get_frame(self):
        for _ in range(100):
            self.pipeline.wait_for_frames()
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        if not depth_frame or not color_frame:
            return False, None, None
        
        return True, depth_frame, color_frame

    def filter_depth_frame(self, depth_frame):
        spatial = rs.spatial_filter()
        spatial.set_option(rs.option.filter_magnitude, 2)
        spatial.set_option(rs.option.filter_smooth_alpha, 0.50)
        spatial.set_option(rs.option.filter_smooth_delta, 20)
        spatial.set_option(rs.option.holes_fill, 0) 
        
        temporal = rs.temporal_filter()
        temporal.set_option(rs.option.filter_smooth_alpha, 0.4)
        temporal.set_option(rs.option.filter_smooth_delta, 20)
        temporal.set_option(rs.option.holes_fill, 3)
        
        filtered_depth_frame = spatial.process(depth_frame)
        filtered_depth_frame = temporal.process(filtered_depth_frame)
        
        return filtered_depth_frame

    def release(self):
        self.pipeline.stop()