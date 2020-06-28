# Open3D: www.open3d.org
# The MIT License (MIT)
# See license file or visit www.open3d.org for details

# examples/Python/Advanced/colored_pointcloud_registration.py

import numpy as np
import copy
import open3d as o3d
import time

def draw_registration_result_original_color(source, target, transformation):
    source_temp = copy.deepcopy(source)
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target])

if __name__ == "__main__":

    print("1. Load two point clouds and show initial pose")
    source = o3d.io.read_point_cloud("../../TestData/ColoredICP/15.ply")
    target = o3d.io.read_point_cloud("../../TestData/ColoredICP/14.ply")

    # draw initial alignment
    current_transformation = np.identity(4)
    draw_registration_result_original_color(source, target,
                                            current_transformation)

    # point to plane ICP
    current_transformation = np.identity(4)

    trans1 = np.asarray([[0.9887, 0., -0.15, 0.34],
                         [0., 1., 0., 0.],
                         [0.15, 0., 0.9887, 0.],
                         [0., 0., 0., 1.]])
    trans2 = np.asarray([[0.9798, 0., -0.2, 0.34],
                         [0., 1., 0., 0.],
                         [0.2, 0., 0.9798, 0.],
                         [0., 0., 0., 1.]])

    source2=copy.deepcopy(source)
    source3=copy.deepcopy(source)
    start=time.time()
    source2.transform(trans2)
    end=time.time()
    source3.transform(trans2)
    print("before icp regi")
    o3d.visualization.draw_geometries([source2, target])

    start=time.time()
    print("2. Point-to-plane ICP registration is applied on original point")
    print("   clouds to refine the alignment. Distance threshold 0.02.")
    result_icp = o3d.registration.registration_icp(
        source2, target, 0.02, current_transformation,
        o3d.registration.TransformationEstimationPointToPlane())
    end=time.time()
    print("icp time :", end-start)
    draw_registration_result_original_color(source2, target,
                                            result_icp.transformation)

    print("color regi")
    # colored pointcloud registration
    # This is implementation of following paper
    # J. Park, Q.-Y. Zhou, V. Koltun,
    # Colored Point Cloud Registration Revisited, ICCV 2017
    voxel_radius = [0.04, 0.02, 0.01]
    max_iter = [50, 30, 14]
    current_transformation = np.identity(4)
    print("3. Colored point cloud registration")
    start=time.time()
    for scale in range(3):
        iter = max_iter[scale]
        radius = voxel_radius[scale]
        #print([iter, radius, scale])
        #print("3-1. Downsample with a voxel size %.2f" % radius)
        source_down = source3.voxel_down_sample(radius)
        target_down = target.voxel_down_sample(radius)
        #print("3-2. Estimate normal.")
        source_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))
        target_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))

        #print("3-3. Applying colored point cloud registration")
        result_icp = o3d.registration.registration_colored_icp(
            source_down, target_down, radius, current_transformation,
            o3d.registration.ICPConvergenceCriteria(relative_fitness=1e-6,
                                                    relative_rmse=1e-6,
                                                    max_iteration=iter))
        current_transformation = result_icp.transformation
        #print(result_icp)

    end=time.time()
    print("color regi time :", end - start)
    draw_registration_result_original_color(source3, target, result_icp.transformation)