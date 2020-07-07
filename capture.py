import pyrealsense2

## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2017 Intel Corporation. All Rights Reserved.
#https://qiita.com/idev_jp/items/0f71cf831b604f8adcea. save as pcd with open3d & pyrealsense
#https://github.com/IntelRealSense/librealsense/issues/3894. cropping with transform

#####################################################
##                  Export to PLY                  ##
#####################################################

# First import the library
import pyrealsense2 as rs
from open3d.open3d.geometry import PointCloud
from open3d.open3d.geometry import OrientedBoundingBox
from open3d.open3d.utility import Vector3dVector
import open3d as o3d
import numpy as np
import time
import pdb

def capture(cfg, pipe, threshold, box) :
    pcd = PointCloud()
    
    try:
        # Wait for the next set of frames from the camera
        profile = pipe.start(cfg)
        intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(intr.width, intr.height, intr.fx, intr.fy,
                                                                     intr.ppx, intr.ppy)
        align = rs.align(rs.stream.color)

        for i in range(30):
            pipe.wait_for_frames()

        frames = pipe.wait_for_frames()
        aligned_frames = align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        filtered = threshold.process(depth_frame)

        color_frame = aligned_frames.get_color_frame()

        depth = o3d.geometry.Image(np.asanyarray(filtered.get_data()))
        color = o3d.geometry.Image(np.asanyarray(color_frame.get_data()))

        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth, convert_rgb_to_intensity=False)
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, pinhole_camera_intrinsic)
        pcd = pcd.crop(box)

    finally:
        pipe.stop()
        return pcd

