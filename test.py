import pyrealsense2

## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2017 Intel Corporation. All Rights Reserved.
# https://qiita.com/idev_jp/items/0f71cf831b604f8adcea. save as pcd with open3d & pyrealsense
# https://github.com/IntelRealSense/librealsense/issues/3894. cropping with transform

#####################################################
##                  Export to PLY                  ##
#####################################################

# First import the library
import pyrealsense2 as rs
import open3d as o3d
from open3d.open3d.geometry import PointCloud
from open3d.open3d.geometry import OrientedBoundingBox
import numpy as np
import time

pc = rs.pointcloud()
pipe = rs.pipeline()
cfg = rs.config()
thre = rs.threshold_filter(0.1, 2)
align = rs.align(rs.stream.color)

profile = pipe.start(cfg)
intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(intr.width, intr.height, intr.fx, intr.fy, intr.ppx, intr.ppy)

for i in range(10):
    pipe.wait_for_frames()

frames = pipe.wait_for_frames()
aligned_frames = align.process(frames)
depth_frame = aligned_frames.get_depth_frame()
filtered = thre.process(depth_frame)

color_frame = aligned_frames.get_color_frame()


depth=o3d.geometry.Image(np.asanyarray(filtered.get_data()))
color=o3d.geometry.Image(np.asanyarray(color_frame.get_data()))

rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth, convert_rgb_to_intensity =False)
pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, pinhole_camera_intrinsic)

center = np.array([[0.0], [0.0], [0.5]])
extent = np.array([[1], [1], [1]])
R = np.identity(3)

box = OrientedBoundingBox(center, R, extent)
pcd = pcd.crop(box)
print(pcd.points)
print(np.asarray(pcd.points))
o3d.io.write_point_cloud('arm2.ply',pcd)
pipe.stop()


