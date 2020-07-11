import argparse
import numpy as np

def get_parameters():

    parser = argparse.ArgumentParser(description = '3D scene graph generation', formatter_class = argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument('--dist_from_obstacle', default = 20, help = '1 = 5cm')
    parser.add_argument('--origin', default = (-4.67, -3.79), help = 'origin coordinates in map.yaml')
    parser.add_argument('--slam_map', default = './slam_map/map_0710.pgm', help = 'map directory')
    parser.add_argument('--object_category', default = ['teddy bear', 'bottle', 'backpack', 'chair'], help = 'object category to look for')
    parser.add_argument('--crop_size', default = np.array([[1.5], [2.0], [2.0]]), help = 'crop_box size(x, y, z)' )
    parser.add_argument('--cut_distance', default = 1.7, help = 'object farther than this would be ignored. meter unit')
    parser.add_argument('--save_dir', default = './res/', help = 'result files are saved')
    parser.add_argument('--local_limit', default = 14, help = 'If the distance between far_point & curr_point is under this limit, local path will set curr_point as center. Otherwise, obj_pose will be center.')
    parser.add_argument('--local_radius', default = 10, help = 'local path radius')

    return parser.parse_args()
