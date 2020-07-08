import numpy as np

class icp_params:
    def __init__(self):
        self.do_scale = False
        self. dt_thresh = 0.01
        self.dth_thresh = 0.001
        self.scale_thresh = 0.001
        self.t_init =np.identity(4)
        self.icp_iter_max = 100
        self.em_iter_max_start = 1000
        self.em_iter_max = 100
        self.em_outlier_dist = 'normal'
        self.neighbor_structure = 'grid'
        self.neighbor_sigma = 1e-1
        self.max_neighbors = 10
        self.mode = 'all'
        self.D = 1
        self.beta = 2
        self.gamma = 0
        self.debug = False
        self.verbose = False
        self.w = 640
        self.h = 480
        self.pyramid_levels = 3