## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#####################################################
##                  Export to PLY                  ##
#####################################################

# First import the library
import pyrealsense2 as rs
import numpy as np

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
#colorizer.set_option(rs.option.color_scheme, 3)
thre = rs.threshold_filter(0.1, 1.8)


try:
    # Wait for the next set of frames from the camera
    for i in range(10):
        pipe.wait_for_frames()

    frames = pipe.wait_for_frames()
    aligned_frames = align.process(frames)
    color = aligned_frames.get_color_frame()
    depth = aligned_frames.get_depth_frame()
    colorized = colorizer.process(aligned_frames)

    # Create save_to_ply object
    ply = rs.save_to_ply("1.ply")

    # Set options to the desired values
    # In this example we'll generate a textual PLY with normals (mesh is already created by default)
    ply.set_option(rs.save_to_ply.option_ply_binary, False)
    ply.set_option(rs.save_to_ply.option_ply_normals, True)
    ply.set_option(rs.save_to_ply.option_ignore_color, False)
    ply.set_option(rs.save_to_ply.option_ply_threshold, True)


    print("Saving to 1.ply...")
    # Apply the processing block to the frameset which contains the depth frame and the texture
    ply.process(colorized)
    print("Done")
finally:
    pipe.stop()