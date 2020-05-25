import numpy as np
import icp_data
import open3d as o3d
import make_neighborhoods
from math import log, pi
import math


class Theta():
    def __init__(self):
        self.in_mean
        self.in_std
        self.out_mean
        self.out_std

def M_step(z, y):
    theta = Theta()
    n_1 = np.sum((1+z)/2)
    theta.in_mean = np.sum((1+z)/2*y)/n_1
    theta.in_std = math.sqrt(np.sum((1+z)/2*y*y)/n_1 - theta.in_mean**2)
    nm_1 = np.sum((1-z)/2)
    theta.out_mean = np.sum((1-z)/2*y)/nm_1
    theta.out_std = math.sqrt(np.sum((1-z)/2*y*y)/nm_1 - theta.out_mean**2)
    return theta

def E_step(y, z, theta, params, data):
    mean_field = np.matmul(data.neighborhoods[data.pyramid_level], z)

    r_in = params.beta*mean_field - 0.5 *log(2*pi) -log(theta.in_std) - (y - theta.in_mean)**2/(2*theta.in_std**2)+params.gamma
    r_in[np.isnan(r_in)] = params.beta*mean_field[np.isnan(r_in)]

    if params.em_outlier_dist == 'normal':
        r_out = -params.beta*mean_field - 0.5*log(2*pi) - log(theta.out_std) - (y - theta.out_mean)**2/(2*theta.out_std**2) - params.gamma
    elif params.em_outlier_dist == 'logistic':
        s = math.sqrt(3)/math.pi*theta.out_std
        r_out = -params.beta*mean_field - log(s) - (y-theta.out_mean)/s - 2*log(1+np.exp(-(y-theta.out_mean)/s))-params.gamma

    r_out[np.isnan(r_out)] = params.beta*mean_field[np.isnan(r_out)]
    r_min = min(r_in, r_out)
    z = 2*np.divide(np.exp(r_in-r_min) , np.exp(r_out - r_min) + np.exp(r_in - r_min))
    return z


def EM(y, z, max_iter, params, data):
    z1 = np.random.randint(low = -1, high = 2, size = (len(z), 3))
    check = 0
    for iters in range(1, max_iter+1):
        z2 = z1.copy()
        z1 = z.copy()
        theta = M_step(z,y)
        z = E_step(y, z, theta, params, data)

        if np.all(z*z1) or (iters>1 and np.all(z*z2)):
            check = iters
            break

    data.em_iters.append(check)
            

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



















