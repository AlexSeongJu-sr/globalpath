import argparse
import numpy as np

def get_parameters():

    parser = argparse.ArgumentParser(description = '3D scene graph generation', formatter_class = argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument('--dist_from_obstacle', default = 20, help = '1 = 5cm')
    parser.add_argument('--origin', default = (-4.34, -3.85), help = 'origin coordinates in map.yaml')
    parser.add_argument('--slam_map', default = './slam_map/map_0708_2.pgm', help = 'map directory')
    parser.add_argument('--object_category', default = ['tv', 'teddy bear', 'chair'], help = 'object category to look for')
    parser.add_argument('--crop_size', default = np.array([[1.5], [2.0], [2.0]]), help = 'crop_box size(x, y, z)' )
    parser.add_argument('--cut_distance', default = 1.8, help = 'object farther than this would be ignored. meter unit')
    parser.add_argument('--save_dir', default = './res/', help = 'result files are saved')

    return parser.parse_args()
