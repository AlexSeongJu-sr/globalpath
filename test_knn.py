import open3d as o3d
import numpy as np

pcd = o3d.io.read_point_cloud("vim.ply")
pcd_tree = o3d.geometry.KDTreeFlann(pcd)
[k, idx, _] = pcd_tree.search_knn_vector_3d(pcd.points, 3)
print("k: ", k)
print("idx: ",idx)
#print(np.asarray(pcd.points)[idx[1:], :])
