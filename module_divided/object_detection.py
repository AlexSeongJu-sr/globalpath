"""
this module includes detect_object function.

"""

import time
import integrated as inte
import numpy as np
import pyrealsense2 as rs

# use detectron2 to detect certain objects and their positions
def detect_object(cfg, cfg2, predictor, object_cate,tf):
    object_info=[]
    start1 = time.time()
    profile = inte.pipe.start(cfg)
    for x in range(5):
        inte.pipe.wait_for_frames()
    frameset = inte.pipe.wait_for_frames()
    color_frame = frameset.get_color_frame()
    depth_frame = frameset.get_depth_frame()
    # Cleanup:
    inte.pipe.stop()
    color = np.asanyarray(color_frame.get_data())

    align = rs.align(rs.stream.color)
    frameset = align.process(frameset)

    # Update color and depth frames:
    aligned_depth_frame = frameset.get_depth_frame()
    depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics

    start2 = time.time()
    outputs = predictor(color)
    end = time.time()
    print("infer time :", end - start2)
    print("entire time :", end - start1)

    # pred classes represents classified object numbers
    # you can see each object number at https://github.com/facebookresearch/detectron2/blob/989f52d67d05445ccd030d8f13d6cc53e297fb91/detectron2/data/datasets/builtin_meta.py
    print("classes : ", outputs["instances"].pred_classes)


    out_class = outputs["instances"].pred_classes
    out_boxes = outputs["instances"].pred_boxes
    out_scores = outputs["instances"].scores # score means probability

    centers = out_boxes.get_centers() # get center coordinates of boxes
    depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()

    idx = 0

    for i in out_class:
        name_t = inte.cate[i]["name"]
        dont_want=True
        for item in object_cate:
            if item == name_t:
                dont_want=False
        if dont_want:
            continue
        score_t = out_scores.cpu().numpy()[idx]
        x = (centers[idx].cpu().numpy()[0])
        y = (centers[idx].cpu().numpy()[1])
        # x, y is pixel coordinates -> use round ftn
        x = round(x)
        y = round(y)
        # get depth using aligned_depth_frame
        depth = aligned_depth_frame.get_distance(int(x), int(y))
        # get camera coordinates with intrinsic
        depth_point = rs.rs2_deproject_pixel_to_point(
            depth_intrin, [x, y], depth)
        idx = idx + 1
        coordi = np.array([depth_point[0], depth_point[1], depth_point[2], 1]).T
        print("camera coordi :", coordi)
        coordi_global = np.matmul(tf, coordi).T
        print("global coordi: ", coordi_global)
        object_info.append({"name" : name_t, "score" : score_t, "coordi" : coordi_global})
        print("name :",name_t)
        print("coordi ;", coordi_global)
    return object_info