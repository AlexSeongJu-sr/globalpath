

## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#####################################################
##                  Export to PLY                  ##
#####################################################

# First import the library
import pyrealsense2 as rs
import numpy as np
import cv2

# Declare pointcloud object, for calculating pointclouds and texture mappings
pc = rs.pointcloud()
# We want the points object to be persistent so we can display the last cloud when a frame drops
points = rs.points()

# Declare RealSense pipeline, encapsulating the actual device and sensors
pipe = rs.pipeline()
config = rs.config()
# Enable depth stream
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
align = rs.align(rs.stream.color)

# Start streaming with chosen configuration
pipe.start(config)

# We'll use the colorizer to generate texture for our PLY
# (alternatively, texture can be obtained from color or infrared stream)
colorizer = rs.colorizer()
try:
    # Wait for the next set of frames from the camera
    for i in range(30):
        pipe.wait_for_frames()

    frames = pipe.wait_for_frames()

    # Get aligned frames
    aligned_depth_frame = frames.get_depth_frame()  # aligned_depth_frame is a 640x480 depth image
    color_frame = frames.get_color_frame()
    color_image = np.array(color_frame.get_data())
    print (np.mean(color_image[:,:,0]))
    print (np.mean(color_image[:,:,1]))
    print (np.mean(color_image[:,:,2]))
    #color_image[:,:,1] = 0.5 * color_image[:,:,1]
    cv2.imwrite("test.png", color_image)
    print(type(color_frame))

    pc.map_to(color_frame)
    # Generate the pointcloud and texture mappings
    points = pc.calculate(aligned_depth_frame)
    print("Saving to 1.ply...")
    points.export_to_ply("1.ply", color_frame)
    print("Done")



finally:
    pipe.stop()