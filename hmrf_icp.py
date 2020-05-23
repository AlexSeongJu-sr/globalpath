import numpy as np
import icp_data
import open3d as o3d
import make_neighborhoods
from math import log
import math


def E_step(y, z, theta, params, data):
    mean_field = np.matmul(data.neighborhoods[data.pyramid_level], z)

    r_in = params.beta*mean_field - 0.5 *log(2*math.pi) -log(theta.in_std) - (y - theta.in_mean)**2./(2*theta.in_std**2)+params.gamma
    r_in(isnan)

    if params.em_outlier_dist == 'normal':
        r_out = -params.beta*mean_field - 0.5*log(2*math.pi) - log(theta.out_std) - (y - theta.out_mean)**2./(2*theta.out_std**2) - params.gamma
    elif params.em_outlier_dist == 'logistic':
        s = math.sqrt(3)/math.pi*theta.out_std
        r_out = -params.beta*mean_field - log(s) - (y-theta.out_mean)/s - 2*log(1+exp(-(y-theta.out_mean)/s))-params.gamma

def EM(y, z, max_iter, params, data):
    z1 = [z[2:][:]; np.zeros()]
    for iters in range(1, max_iter+1):
        z2 = z1
        z1 = z
        theta = M_step(z,y)


def EM_pyramid(y, z, max_iter, params, data):
    zs = [z]
    ys = [y]

    for p in range(2, params.pyramid_levels+1):
        zs.append(zs[p-1][data.neighborhood_maps[p-1]])
        ys.append(ys[p-1][data.neighborhood_maps[p-1]]) #check

    for p in range(params.pyramid_levels, 0, -1):
        data.pyramid_level = p
        zs[p], theta, iters, data = EM(ys[p], zs[p], max_iter, params, data)



def hmrf_icp(ref, src, params):
    data = icp_data()
    #if params.neighbor_structure == 'unstructured':
    data.src = src
    data.ref = ref
    data.em_iters = []
    data.icp_iters = 0

    n_ref = len(ref)
    n_src = len(src)

    if params.verbose:
        print('building kdtree')
    kdtree = o3d.geometry.KDTreeFlann(ref)

    tf = params.t_init
    src = np.matmul(params.t_init,src.transpose()).transpose()

    if params.neighbor_structure == 'unstructured':
        grid = False
        z = np.ones(n_src)
        if params.verbose:
            print('Making neighborhoods')

        data = make_neighborhoods(params, data)
        if params.verbose:
            print('Done making neighborhoods')
    else:
        print("nenighbor_structure must be grid or unstructured")

    if params.verbose:
        print('starting icp iterations')

    for icp_iter in range(1, params.icp_iter_max):
        idx, y = kdtree.search_knn_vector_3d(src)

        if icp_iter == 1:
            z[y>]

            z, theta, iters, data = EM_pyramid(y, z, params.em_iter_max_start, params, data)
            z, theta, iters, data = EM_pyramid(y, z, params.em_iter_max, params, data)
        else:
            z, theta, iters, data = EM_pyramid(y, z, params.em_iter_max, params, data)



















