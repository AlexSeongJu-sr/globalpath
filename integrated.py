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
import mytransformation as tr
from mytransformation import Point
import navigation as nav2
import object_detection as ob
import globalpath as gp
import capture
import pyrealsense2 as rs
import open3d as o3d
import copy
import matplotlib.pyplot as plt
import pdb

import detectron2
from open3d.open3d.geometry import OrientedBoundingBox
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
    if (pos[0] < 0 or pos[0] >= im_size[0]) or (pos[1] < 0 or pos[1] >= im_size[1]):
        return False
    else :
        return True

# for dividing yellow points into each group(cluster) using dfs.
def dfs(visited, x, y, group_num):
    visited[x][y]=1
    yellow_group[x][y]=group_num
    for i in direction:
        for j in direction:
            pos = (x+i, y+j)
            if is_in(pos) and visited[x+i][y+j]==0 and color[x+i][y+j]=='y':
                dfs(visited,x+i,y+j, group_num)

# for finding reset point when navigation goes wrong.
# reset point should be 'local_r' radius away from goal pose and the color should be yellow or white.
# yellow is global path. black is near obstacles.
def find_reset(x, y, local_r):
    center = tr.transform_inverse((x,y), im_pix, origin, resolution)
    print("center :",center)
    x=center[0]
    y=center[1]
    for k in range(2*local_r+2) :
        for l in range(2*local_r+2) :
            i = x-local_r-1+k
            j = y-local_r-1+l
            d= get_distance((i,j), center)
            if (color[i][j]=='y' or color[i][j] == 'w') and d>=local_r and d<local_r+1.0:
                reset=tr.transform_coordi2((i,j), im_pix, origin, resolution)
                print("reset :",reset)
                return reset


# find local path with object pose, current pose, radius
def find_local(obj_pose, current_pose, color, local_r):
    local_points = []
    x = current_pose[0]
    y = current_pose[1]
    for k in range(2*local_r+2) :
        for l in range(2*local_r+2) :
            i = x-local_r-1+k
            j = y-local_r-1+l
            d= get_distance((i,j), current_pose)
            if color[i][j]=='y' and d>=local_r and d<local_r+1.0:
                ori=get_ori((i,j), obj_pose)
                dupl=False
                for item in local_points:
                    if get_distance((i,j), item.coordi) <=1.5:
                        dupl=True
                if not dupl:
                    local_points.append(Point((i,j),'r', ori))
    local_points.append(Point(current_pose, 'lo', get_ori(current_pose, obj_pose)))
    return local_points



# bring object categories ex)tv, person, mouse etc
cate=builtin_meta.COCO_CATEGORIES
# direction for dfs
direction = [-1, 0, 1]

# Read slam_map image
im = pilimg.open('/home/dvision/map_200625.pgm') #load 2D slam map

# Fetch image pixel data to numpy array. values are one of 0, 205, 254. 0 for obstacle, 205 for near obstacle, 254 for empty space.
im_pix = np.array(im)
print(im_pix.shape)
im_size=im_pix.shape

# divide map into small fragments of size 10 x 10
i_part = im_pix.shape[0] // 10 + 1
j_part = im_pix.shape[1] // 10 + 1
parted_yellow = [[[]for item in range(j_part)] for item in range(i_part)]
parted_num = [[0 for item in range(j_part)] for item in range(i_part)]
parted_red = [[[] for item in range(j_part)] for item in range(i_part)]

visited = [[0 for item in range(im_size[1])] for item in range(im_size[0])]
yellow_group=[[0 for item in range(im_size[1])] for item in range(im_size[0])]

color = []
ori = [] # orientation
for i in range(im_size[0]):
    color.append(['w' for j in range(im_size[1])]) # initialize wiht white color
    ori.append([[0,0] for j in range(im_size[1])])


r=int(input("put distance: ")) # radius r means, set global path 'r * 0.05' meters away from obstacles. 20 means 1 meter.
yellow_num=0
yellow_points = []
zero_points = []
image_color = np.zeros((im_size[0], im_size[1], 3), np.uint8) #gets color for each pixel
yellow = [255, 255, 0]
red = [255, 0, 0]
blue = [0, 0, 255]
black = [0, 0, 0]

for i in range(im_size[0]):
    for j in range(im_size[1]):
      if im_pix[i][j]==0:
         zero_points.append([i,j])
         image_color[i][j]=blue
         for k in range(2*r+2):
            x=i-r-1+k
            for l in range(2*r+2):
               y=j-r-1+l
               if is_in((x,y)) and color[x][y] != 'b':
                   d = get_distance((x, y), (i, j))
                   if d < r :
                       color[x][y] = 'b' #colorize black
                       image_color[x][y] = black
                   if d>=r and d<r+1.5 and color[x][y]=='w' and im_pix[x][y] == 254:
                       color[x][y]='y'
                       ori[x][y]=get_ori((x,y), (i,j))
                       image_color[x][y]=yellow

group_num=0
for i in range(im_size[0]):
    for j in range(im_size[1]):
      if color[i][j] =='y' :
            if (not visited[i][j]):
                group_num+=1  # group num starts from 1.
                dfs(visited,i,j,group_num) # mark each point with group_num.
            parted_i = i//10
            parted_j = j//10
            ypoint = Point((i,j), color[i][j], ori[i][j])
            yellow_points.append(ypoint)
            parted_yellow[parted_i][parted_j].append(ypoint)
            yellow_num+=1
            parted_num[parted_i][parted_j]+=1

red_group=[[] for _ in range(group_num+1)]

for i in range(i_part):
    for j in range(j_part):
        # choose random points in each fragments(10 x 10 size). sampling rate is 0.05.
        parted_red[i][j] = np.random.choice(parted_yellow[i][j], size=round(parted_num[i][j] * 0.05), replace=False)

red_num=0
for i in range(i_part):
    for j in range(j_part):
        for point_ in parted_red[i][j]:
            (x,y) = point_.coordi
            gn = yellow_group[x][y] # gn = group number
            image_color[x][y] = red
            red_num+=1
            red_group[gn].append(point_)

print("yellow num, red num :", yellow_num, red_num)
print("group number :", group_num)

#visualize
print("show global path & path_point")
plt.imshow(image_color)
plt.show()


(x,y) = map(float,input("put origin coordinates: ").split())
origin = (x,y)
resolution = 0.05

# type object categories that you want to detect. ex) tv, person, cellphone etc
object_cate = input("put wanted_objects(ex. tv) : ").split()

# Setup for detecting object:
pipe = rs.pipeline()
cfg = rs.config() # config for pyrealsense
cfg2 = get_cfg() # config for detectron
cfg2.merge_from_file(model_zoo.get_config_file("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"))
cfg2.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.5  # set threshold for this model
# Find a model from detectron2's model zoo. You can use the https://dl.fbaipublicfiles... url as well
cfg2.MODEL.WEIGHTS = model_zoo.get_checkpoint_url("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml")
predictor = DefaultPredictor(cfg2)

# set configuration for realsense. frame size is set 848 x 480. speed to 30 fps.
cfg.enable_stream(rs.stream.depth,  848, 480, rs.format.z16, 30)
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
thre = rs.threshold_filter(0.1, 1.8)
# Declare pointcloud object, for calculating pointclouds and texture mappings
pc = rs.pointcloud()

center = np.array([[0.0], [0.0], [1.0]])
extent = np.array([[1], [1], [2]])
R = np.identity(3)

box = OrientedBoundingBox(center, R, extent) # for crop when capturing


for i in range(group_num):
    print("path", i)
    if len(red_group[i+1])==0 or len(red_group[i+1])==1: # when 0 or 1 point, pass
        continue
    path = gp.MST(red_group[i+1], get_distance) # get global path for each group
    a= tr.transform_coordi(path, im_pix, origin, resolution) # transform pixel to world coordinates
    b=[(item.coordi, item.ori) for item in a]
    for global_point in b:
        p = nav2.nav2(global_point[0][0], global_point[0][1], global_point[1][0], global_point[1][1], find_reset)
        current_pose_glb = (p.position.x, p.position.y)
        current_ori = (p.orientation.z, p.orientation.w)
        tf = tr.get_tf(current_pose_glb, current_ori, extrinsic_robot_camera)
        object_info = ob.detect_object(cfg, pipe, cate, predictor, object_cate, tf) # get global coordinates of object

        for point_ in object_info: #local path planning
            name = point_["name"]
            coordi = point_["coordi"]
            tag = name + '_'+ str(coordi) + '.ply'
            object_pixel = tr.transform_inverse(coordi, im_pix, origin, resolution)
            current_pixel = tr.transform_inverse(current_pose_glb, im_pix, origin, resolution)
            local_path_pixel = find_local(object_pixel, current_pixel, color, local_r)
            local_path = tr.transform_coordi(local_path_pixel, im_pix, origin, resolution)
            for local_point in local_path:
                print("local_point :", (local_point.coordi[0], local_point.coordi[1]))
            print("go local_point")
            idx = 0
            points = []
            for local_point in local_path:
                current_pose = nav2.nav2(local_point.coordi[0], local_point.coordi[1], local_point.ori[0], local_point.ori[1], find_reset)
                ori = (current_pose.orientation.z, current_pose.orientation.w)
                (cos, sin) = tr.transform_ori_inverse(ori)
                (x, y) = (current_pose.position.x, current_pose.position.y)
                r_trans = np.array([[cos, 0, -sin, 0],
                                   [0, 1, 0, 0],
                                   [sin, 0, cos, 0],
                                   [0, 0, 0, 1]])
                t_trans = np.array([[1, 0, 0, -y],
                                   [0, 1, 0, 0],
                                   [0, 0, 1, x],
                                   [0, 0, 0, 1]])
                print("capture start")
                tmp = capture.capture(cfg, pipe, thre, pc, box)
                print("capture end")
                tmp = tmp.transform(r_trans)
                tmp = tmp.transform(t_trans)
                points.append(tmp)

            current_transformation = np.identity(4)
            matrices = []
            result = copy.deepcopy(points[0])
            points_down = []
            compare = copy.deepcopy(points[0])  #to compare before & after registration
            for n in range(len(local_path)-1):
                compare+=points[n+1]
            o3d.io.write_point_cloud('compare.ply', compare) # save point clouds before registration for comparison

            for point_ in points:
                pcd_down = point_.voxel_down_sample(0.04)
                pcd_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius = 0.08, max_nn=30))
                points_down.append(pcd_down)
            for n in range(len(local_path)):
                o3d.io.write_point_cloud(str(n) + '.ply', points[n])  # save each point cloud
                if n == 0:
                    continue
                # " Point-to-plane ICP registration is applied on original point"
                # "   clouds to refine the alignment. Distance threshold 0.04."
                result_icp = o3d.registration.registration_icp(points_down[n], points_down[n-1], 0.04, current_transformation,
                    o3d.registration.TransformationEstimationPointToPlane())
                matrices.append(result_icp.transformation)
                transform_matrix = np.identity(4)

                for mat in matrices:
                    transform_matrix = np.matmul(transform_matrix, mat)
                bef_transform = copy.deepcopy(points[n])
                result = result + bef_transform.transform(transform_matrix)

            o3d.io.write_point_cloud(tag, result) # save an object as point cloud

                # capture & resgistration

#cv2.imwrite('/home/3dvision/path_image/result2_0318.png',image)
