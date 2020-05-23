
import open3d as o3d
import numpy as np
from scipy import sparse
from numpy.random import randint

def make_neighborhoods(params, data):
    n = len(data.src)
    data.neighborhoods = [[] for i in range(params.pyramid_levels)]
    data.neighborhood_maps = [[] for i in range(params.pyramid_levels - 1)]

    kdtree = o3d.geometry.KDTreeFlann(data.src)
    [i, dist] = kdtree.search_knn_vector_3d(data.src, params.max_neighbors+1)
    i = np.array(i)
    i = i[:, 2:].reshape(1, -1)
    dist = np.array(dist)
    dist = dist[:, 2:].reshape(1, -1)

    j = np.array(range(1,n+1))
    j = np.tile(j, (1, params.max_neighbors))
    hood = sparse.coo_matrix(dist, (i, j), shape = (n,n))
    hood = hood + hood.transpose().multiply(np.logical_xor(hood.toarray(), hood.toarray().transpose()))
    hood = hood.toarray()
    hood = np.exp(-(np.multiply(hood, hood)/(2*params.neighbor_sigma**2)))
    row_sums = hood.sum(axis = 1)
    row_sums[row_sums<1] = 1
    tmp = np.divide(np.ones((n,n)),row_sums)
    hood = np.matmul(sparse.spdiags(tmp, 0, n, n), hood)
    data.neighborhoods[1] = hood

    n = len(hood)
    for level in range(2, params.pyramid_levels+1):
        new_neighbors = randint(1,n+1, size = n//2)
        data.neighborhood_maps[level-1] = new_neighbors
        h2 = hood[new_neighbors][:]
        h2 = np.matmul(h2, h2.transpose())

        h2 = h2[1:len(h2)+1] = 0 #check
        th = 0.1*max(h2)
        n = len(hood)
        row_sums = hood.sum(axis = 1)
        row_sums[row_sums<1] = 1
        tmp = np.divide(np.ones((n,n)), row_sums)
        hood = np.matmul(sparse.spdiags(tmp, 0, n, n), hood)
        data.neighborhoods[level] = hood






