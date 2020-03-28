"""
this module integrates all modules and implements the automated robot_capturing system.
It takes a pre-slammed map and generates global path points. When it gets to global points,
it detects objects & go to local path points based on object pose. When it gets to local points,
it captures the object with realsense camera. (not yet)After capturing, it does some registrations and generates scene graph data.
"""

import PIL.Image as pilimg
import math
import cv2
import numpy as np
import transformation as tr
import navigation as nav2
import object_detection as ob
import globalpath as gp
import pyrealsense2 as rs

import detectron2
from detectron2.utils.logger import setup_logger
setup_logger()
from detectron2 import model_zoo
from detectron2.engine import DefaultPredictor
from detectron2.config import get_cfg
from detectron2.utils.visualizer import Visualizer
from detectron2.data import MetadataCatalog
from detectron2.data.datasets import builtin_meta

def get_distance(pos1, pos2):
    x_dist=pos1[0]-pos2[0]
    y_dist=pos1[1]-pos2[1]
    val=math.sqrt(x_dist**2+y_dist**2)
    return val

def get_ori(fro, to):
    x=to[0]-fro[0]
    y=to[1]-fro[1]
    a= np.array([x,y])
    siz= np.linalg.norm(a)
    return (x/siz, y/siz)


def is_in(pos): # is a point in the map?
    if (pos[0]<0 or pos[0]>=size[0]) or (pos[1]<0 or pos[1]>=size[1]):
        return False
    else :
        return True

class Point:
    def __init__(self, coordi, color, ori):
        self.coordi=coordi
        self.color=color
        self.ori=ori


#bring object categories ex)tv, person, mouse etc
cate=builtin_meta.COCO_CATEGORIES

# Read slam_map image
im = pilimg.open('/home/dvision/map1_200324.pgm')

# Fetch image pixel data to numpy array
pix = np.array(im)
print(pix.shape)
size=pix.shape

# divide map into small fragments with size 10 x 10
i_part = pix.shape[0]//10 + 1
j_part = pix.shape[1]//10 + 1
parted_yellow = [[[]for item in range(j_part)] for item in range(i_part)]
parted_num = [[0 for item in range(j_part)] for item in range(i_part)]
parted_sample = [[[]for item in range(j_part)] for item in range(i_part)]

visited = [[0 for item in range(size[1])] for item in range(size[0])]
yellow_group=[[0 for item in range(size[1])] for item in range(size[0])]

color = []
ori = []
for i in range(size[0]):
    color.append(['w' for j in range(size[1])]) # initialize wiht white color
    ori.append([[0,0] for j in range(size[1])])


r=int(input("put distance: ")) # radius r means, set global path 'r * 0.05' meters away from obstacles
yellow_num=0
yellow_points=[]
zero_points=[]
image=np.zeros((size[0],size[1],3), np.uint8)
yellow = [0,255,255]
red = [0, 0, 255]
blue = [255,0,0]
black=[0,0,0]

for i in range(size[0]):
    for j in range(size[1]):
      if pix[i][j]==0:
         zero_points.append([i,j])
         image[i][j]=blue
         for k in range(2*r+2):
            x=i-r-1+k
            for l in range(2*r+2):
               y=j-r-1+l
               if is_in((x,y)) and color[x][y] != 'b':
                   d = get_distance((x, y), (i, j))
                   if d < r :
                       color[x][y] = 'b'
                       image[x][y] = black
                   if d>=r and d<r+1.5 and color[x][y]=='w' and pix[x][y] == 254:
                       color[x][y]='y'
                       ori[x][y]=get_ori((x,y), (i,j))
                       image[x][y]=yellow

group_num=0
for i in range(size[0]):
    for j in range(size[1]):
      if color[i][j] =='y' :
            if (not visited[i][j]):
                group_num+=1 # group num starts from 1.
                gp.dfs(visited,i,j,group_num) # mark each point with group_num.
            parted_i = i//10
            parted_j = j//10
            yellow_points.append(Point((i,j), color[i][j], ori[i][j]))
            parted_yellow[parted_i][parted_j].append(Point((i,j), color[i][j], ori[i][j]))
            yellow_num+=1
            parted_num[parted_i][parted_j]+=1

red_group=[[] for _ in range(group_num+1)]

for i in range(i_part):
    for j in range(j_part):
        # choose random points in each fragments(10 x 10 size). sampling rate is 0.05.
        parted_sample[i][j] = np.random.choice(parted_yellow[i][j], size=round(parted_num[i][j]*0.05), replace=False)

red_num=0
for i in range(i_part):
    for j in range(j_part):
        for item in parted_sample[i][j]:
            (x,y)=item.coordi
            gn=yellow_group[x][y] # gn = group number
            image[x][y]=red
            red_num+=1
            red_group[gn].append(item)

print("yellow num, red num :", yellow_num, red_num)
print("group number :", group_num)

(x,y) = map(float,input("put origin coordinates: ").split())
origin = (x,y)
resolution = 0.05

# type object categories that you want to detect. ex) tv, person, cellphone etc
object_cate=input("put wanted_objects : ").split()

# Setup for detecting object:
pipe = rs.pipeline()
cfg = rs.config() # config for pyrealsense
cfg2 = get_cfg() # config for detectron
cfg2.merge_from_file(model_zoo.get_config_file("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"))
cfg2.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.5  # set threshold for this model
# Find a model from detectron2's model zoo. You can use the https://dl.fbaipublicfiles... url as well
cfg2.MODEL.WEIGHTS = model_zoo.get_checkpoint_url("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml")
predictor = DefaultPredictor(cfg2)

# set configuration for realsense. frame size is set to 848 x 480.
cfg.enable_stream(rs.stream.depth,  848, 480, rs.format.z16, 90)
cfg.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)

# put local radius for local path planning
local_r = int(input("put local_r : "))

# calculate robot_camera extrinsic matrix
z_90=np.array([[0, 1, 0, 0],
              [-1, 0, 0, 0],
              [0, 0, 1, 0],
              [0, 0, 0, 1]])
x_90=np.array([[1, 0, 0, 0],
              [0, 0, 1, 0],
              [0, -1, 0, 0],
              [0, 0, 0, 1]])
camera_trans=np.array([[1, 0, 0, 0.149],
                      [0, 1, 0, 0],
                      [0, 0, 1, 0.99],
                      [0, 0, 0, 1]])
inter = np.matmul(camera_trans, z_90)
extrinsic_robot_camera = np.matmul(inter, x_90) # put extrinsic matrix between robot & camera

for i in range(group_num):
    print("path", i)
    if len(red_group[i+1])==0 or len(red_group[i+1])==1: # when 0 or 1 point, pass
        continue
    path = gp.MST(red_group[i+1]) # get global path for each group
    a= tr.transform_coordi(path, pix, origin, resolution) # transform pixel to world coordinates
    b=[(item.coordi, item.ori) for item in a]
    for global_point in b:
        current_pose = (global_point[0][0], global_point[0][1])
        current_ori = (global_point[1][0], global_point[1][1])
        print("go global_point")
        nav2.nav2(global_point[0][0], global_point[0][1], global_point[1][0], global_point[1][1])
        tf = tr.get_tf(current_pose, current_ori, extrinsic_robot_camera) # current pose should be updated !!!
        object_info = ob.detect_object(cfg, cfg2, predictor, object_cate, tf) # get global coordinates of object
        for item in object_info: #local path planning
            coordi = item["coordi"]
            object_pixel = tr.transform_inverse(coordi, pix, origin, resolution)
            current_pixel = tr.transform_inverse(current_pose, pix, origin, resolution)
            local_path_pixel = nav2.find_local(object_pixel, current_pixel, color, local_r)
            local_path = tr.transform_coordi(local_path_pixel, pix, origin, resolution)
            for local_point in local_path:
                print("local_point :", (local_point.coordi[0], local_point.coordi[1]))
            print("go local_point")
            for local_point in local_path:
                nav2(local_point.coordi[0], local_point.coordi[1], local_point.ori[0], local_point.ori[1])
                # capture & resgistration

#cv2.imwrite('/home/3dvision/path_image/result2_0318.png',image)
