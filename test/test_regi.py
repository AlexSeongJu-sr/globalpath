# Open3D: www.open3d.org
# The MIT License (MIT)
# See license file or visit www.open3d.org for details
# examples/Python/Advanced/global_registration.py
import open3d as o3d
import numpy as np
import copy
import pdb
def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
#    source_temp.paint_uniform_color([1, 0.706, 0])
#    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])
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
def prepare_dataset(voxel_size):
    print(":: Load two point clouds and disturb initial pose.")
    source = o3d.io.read_point_cloud("0.ply")
    target = o3d.io.read_point_cloud("1.ply")
    draw_registration_result(source, target, np.identity(4))
    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source, target, source_down, target_down, source_fpfh, target_fpfh
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

def refine_registration(source, target, voxel_size):
    distance_threshold = voxel_size * 0.4
    print(":: Point-to-plane ICP registration is applied on original point")
    print("   clouds to refine the alignment. This time we use a strict")
    print("   distance threshold %.3f." % distance_threshold)
    result = o3d.registration.registration_icp(
        source, target, distance_threshold, np.identity(4),
        o3d.registration.TransformationEstimationPointToPoint())
    return result

if __name__ == "__main__":
    voxel_size = 0.05  # means 5cm for the dataset
    source0 = o3d.io.read_point_cloud('./0708_res/2/0.ply')
    source1 = o3d.io.read_point_cloud('./0708_res/2/1.ply')
    source2 = o3d.io.read_point_cloud('./0708_res/2/2.ply')

    print("before icp")
    o3d.visualization.draw_geometries([source0, source1, source2])

    result_icp_1 = refine_registration(source1, source0, voxel_size)
    result_icp_2 = refine_registration(source2, source1, voxel_size)

    res = source0 + source1.transform(result_icp_1.transformation) + source2.transform(result_icp_2.transformation).transform(result_icp_1.transformation)
    print("after icp")
    o3d.visualization.draw_geometries([res])