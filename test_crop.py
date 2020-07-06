import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import math, time
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
#action type is NavigateToPose
from nav2_msgs.action import NavigateToPose
import capture
import mytransformation as tr
import open3d as o3d
import numpy as np
import copy
import pyrealsense2 as rs
from open3d.open3d.geometry import OrientedBoundingBox
import pdb

def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh

def prepare_dataset(source, target, voxel_size):
    print(":: Load two point clouds and disturb initial pose.")

    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source_down, target_down, source_fpfh, target_fpfh

def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, distance_threshold,
        o3d.registration.TransformationEstimationPointToPoint(False), 4, [
            o3d.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.registration.RANSACConvergenceCriteria(4000000, 500))
    return result


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])


source = o3d.io.read_point_cloud('view1.ply')
target = o3d.io.read_point_cloud('view2.ply')

R = np.identity(3)
tmp = np.asarray(source.points)
cut = 0.4
xmin, xmax = np.min(tmp[:, 0]), np.max(tmp[:, 0])
ymin, ymax = np.min(tmp[:, 1]), np.max(tmp[:, 1])
zmin, zmax = np.min(tmp[:, 2]), np.max(tmp[:, 2])


source_center = np.array([[(xmin+xmax)/2], [(ymin+ymax)/2], [(zmin+zmax-cut)/2]])
source_extent = np.array([[xmax-xmin], [ymax-ymin], [zmax-zmin-cut]])
source_box = OrientedBoundingBox(source_center, R, source_extent)
source = source.crop(source_box)
o3d.visualization.draw_geometries([source])

R = np.identity(3)
tmp = np.asarray(target.points)
xmin, xmax = np.min(tmp[:, 0]), np.max(tmp[:, 0])
ymin, ymax = np.min(tmp[:, 1]), np.max(tmp[:, 1])
zmin, zmax = np.min(tmp[:, 2]), np.max(tmp[:, 2])

target_center = np.array([[(xmin+xmax)/2], [(ymin+ymax)/2], [(zmin+zmax-cut)/2]])
target_extent = np.array([[xmax-xmin], [ymax-ymin], [zmax-zmin-cut]])
target_box = OrientedBoundingBox(target_center, R, target_extent)
target = target.crop(target_box)
o3d.visualization.draw_geometries([target])


print('before regi')
draw_registration_result(source, target, np.identity(4))
o3d.io.write_point_cloud('before_regi.ply', source + target)

voxel_size = 0.05  # means 5cm for the dataset
source_down, target_down, source_fpfh, target_fpfh = \
    prepare_dataset(source, target, voxel_size)

result_ransac = execute_global_registration(source_down, target_down,
                                                source_fpfh, target_fpfh,
                                                voxel_size)
print("global regi")
draw_registration_result(source, target, result_ransac.transformation)
o3d.io.write_point_cloud('after_ransac.ply', source.transform(result_ransac.transformation)+target)

start = time.time()
print("Point-to-point ICP registration is applied on original point")
print("clouds to refine the alignment. Distance threshold 0.1.")
result_icp = o3d.registration.registration_icp(
    source, target, 0.1, np.identity(4),
    o3d.registration.TransformationEstimationPointToPoint())
end=time.time()
print("icp time :", end-start)
draw_registration_result(source, target, result_icp.transformation)
o3d.io.write_point_cloud('after_icp.ply', source.transform(result_icp.transformation)+target)

