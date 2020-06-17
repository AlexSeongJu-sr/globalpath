import numpy as np

def localize(ref, src, do_scale):

    if len(ref[0]) == 4:
        ref = ref[:, :3]/ref[:,3]
        src = src[:, :3]/ref[:,3]

    ref_centroid = np.mean(ref, axis = 0)
    src_centroid = np.mean(src, axis = 0)

    scale = 1
    if do_scale:
       scale =

