## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2017 Intel Corporation. All Rights Reserved.
#https://qiita.com/idev_jp/items/0f71cf831b604f8adcea. save as pcd with open3d & pyrealsense
#https://github.com/IntelRealSense/librealsense/issues/3894. cropping with transform

#####################################################
##                  Export to PLY                  ##
#####################################################

# First import the library
import pyrealsense2 as rs


# Declare pointcloud object, for calculating pointclouds and texture mappings
pc = rs.pointcloud()
# We want the points object to be persistent so we can display the last cloud when a frame drops
points = rs.points()

# Declare RealSense pipeline, encapsulating the actual device and sensors
pipe = rs.pipeline()
config = rs.config()
# Enable depth stream
config.enable_stream(rs.stream.depth)
config.enable_stream(rs.stream.color)

# Start streaming with chosen configuration
pipe.start(config)

# We'll use the colorizer to generate texture for our PLY
# (alternatively, texture can be obtained from color or infrared stream)
colorizer = rs.colorizer()
align_to = rs.stream.color
align = rs.align(align_to)
thre = rs.threshold_filter(0.1, 2.0)

try:
    # Wait for the next set of frames from the camera
    for i in range(5):
        pipe.wait_for_frames()

    frames = pipe.wait_for_frames()
    #aligned_frames = align.process(frames)
    depth = frames.get_depth_frame()
    filtered = thre.process(depth)
    color = frames.get_color_frame()

    pc.map_to(color)
    points = pc.calculate(filtered)
    points.export_to_ply("1.ply", color)

    # Set options to the desired values
    # In this example we'll generate a textual PLY with normals (mesh is already created by default)
    #ply.set_option(rs.save_to_ply.option_ply_binary, False)
    #ply.set_option(rs.save_to_ply.option_ply_normals, True)


finally:
    pipe.stop()
