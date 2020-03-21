import PIL.Image as pilimg
import math
import cv2
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

#action type is NavigateToPose
from nav2_msgs.action import NavigateToPose
import numpy as np                        # fundamental package for scientific computing
import pyrealsense2 as rs                 # Intel RealSense cross-platform open-source API
import detectron2
import time
from detectron2.utils.logger import setup_logger
setup_logger()

import random
from detectron2 import model_zoo
from detectron2.engine import DefaultPredictor
from detectron2.config import get_cfg
from detectron2.utils.visualizer import Visualizer
from detectron2.data import MetadataCatalog
from detectron2.data.datasets import builtin_meta

print("Environment Ready")

cate=builtin_meta.COCO_CATEGORIES

# Read image
im = pilimg.open('/home/dvision/map2_200318.pgm')
# Display image
print(im)

# Fetch image pixel data to numpy array
pix = np.array(im)
print(pix.shape)
size=pix.shape

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

def transform_ori(ori):
    w=math.sqrt((1+ori[0])/2)
    z=ori[1]/(2*w)
    new_ori=(z, w)
    return new_ori

def transform_coordi(points, im, origin,resolution): #origin from yaml file
    xsize=im.shape[1]
    ysize=im.shape[0]
    x_ori=origin[0]
    y_ori=origin[1]
    num=len(points)
    new_points=[]
    for i in range(num):
        coordi = points[i].coordi
        ori = points[i].ori
        x_new = coordi[1]*resolution
        y_new = (ysize-coordi[0]-1)*resolution
        x_new = x_new + x_ori
        y_new = y_new + y_ori
        ori_new = transform_ori((ori[1], -ori[0]))
        new_points.append(Point((x_new,y_new),'r',ori_new))
    return new_points

i_part = pix.shape[0]//10 + 1
j_part = pix.shape[1]//10 + 1
parted_yellow = [[[]for item in range(j_part)] for item in range(i_part)]
parted_num = [[0 for item in range(j_part)] for item in range(i_part)]
parted_sample = [[[]for item in range(j_part)] for item in range(i_part)]

def is_in(pos):
    if (pos[0]<0 or pos[0]>=size[0]) or (pos[1]<0 or pos[1]>=size[1]):
        return False
    else :
        return True

class Point:
    def __init__(self, coordi, color, ori):
        self.coordi=coordi
        self.color=color
        self.ori=ori

class Edge:
    def __init__(self,u,v,w):
        self.u=u
        self.v=v
        self.w=w

class MyClient(Node):

    def __init__(self):
        super().__init__('my_client')
#action type : NavigateToPose, action name : NavigateToPose
        self._action_client = ActionClient(self,NavigateToPose,'NavigateToPose')

    def send_goal(self, xpose, ypose, zpose, wpose):
        goal_msg = NavigateToPose.Goal()
#give values to each of PoseStamped types
        goal_msg.pose.header.stamp.sec = 0
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.pose.position.x = xpose
        goal_msg.pose.pose.position.y = ypose
        goal_msg.pose.pose.orientation.z = zpose
        goal_msg.pose.pose.orientation.w = wpose
        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

#check if goal is accepted by server
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()

        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        rclpy.shutdown()

def nav2(xpose, ypose, zori, wori):
    rclpy.init(args=None)
    action_client = MyClient()
    action_client.send_goal(xpose, ypose, zori, wori)
    rclpy.spin(action_client)

def detect_object(cfg, cfg2, predictor, object_cate,tf):
    object_info=[]
    # Store next frameset for later processing:
    start1 = time.time()
    profile = pipe.start(cfg)
    frameset = pipe.wait_for_frames()
    color_frame = frameset.get_color_frame()
    depth_frame = frameset.get_depth_frame()
    # Cleanup:
    pipe.stop()
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
    out_scores = outputs["instances"].scores

    centers = out_boxes.get_centers()
    depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
    print("depthscale :", depth_scale)

    idx = 0

    for i in out_class:
        name_t = cate[i]["name"]
        dont_want=True
        for item in object_cate:
            if item == name_t:
                dont_want=False
        if dont_want:
            continue
        score_t = out_scores.cpu().numpy()[idx]
        # print(centers[idx])
        x = (centers[idx].cpu().numpy()[0])
        y = (centers[idx].cpu().numpy()[1])
        # x, y is pixel coordinates
        x = round(x)
        y = round(y)
        depth = aligned_depth_frame.get_distance(int(x), int(y))
        # get camera coordinates with intrinsic
        depth_point = rs.rs2_deproject_pixel_to_point(
            depth_intrin, [x, y], depth)
        idx = idx + 1
        coordi = np.array([depth_point[0], depth_point[1], depth_point[2], 1]).T
        coordi_global = np.matmul(tf, coordi).T
        object_info.append({"name" : name_t, "score" : score_t, "coordi" : coordi_global})
    return object_info

def get_tf(robot_pose, robot_ori, extrinsic_robot_camera):
    sin_2 = robot_ori[0]
    cos_2 = robot_ori[1]
    sin = 2*sin_2*cos_2
    cos = cos_2**2 - sin_2**2
    robot_translation = np.array([[1, 0, 0, robot_pose[0]],
                           [0, 1, 0, robot_pose[1]],
                           [0, 0, 1, 0],
                           [0, 0, 0, 0]])
    robot_rotation = np.array([[cos, -sin, 0, 0],
                              [sin, cos, 0, 0],
                              [0, 0, 1, 0],
                              [0, 0, 0, 1]])
    tf=np.matmul(robot_rotation, robot_translation)
    tf=np.matmul(extrinsic_robot_camera, tf)
    return tf

"""
def transform_coordi(points, im, origin,resolution): #origin from yaml file
    xsize=im.shape[1]
    ysize=im.shape[0]
    x_ori=origin[0]
    y_ori=origin[1]
    num=len(points)
    new_points=[]
    for i in range(num):
        coordi = points[i].coordi
        ori = points[i].ori
        x_new = coordi[1]*resolution
        y_new = (ysize-coordi[0]-1)*resolution
        x_new = x_new + x_ori
        y_new = y_new + y_ori
        ori_new = transform_ori((ori[1], -ori[0]))
        new_points.append(Point((x_new,y_new),'r',ori_new))
    return new_points
"""

def transform_inverse(coordi, im, origin, resolution): #transform cartesian to pixel
    xsize=im.shape[1]
    ysize=im.shape[0]
    x_ori=origin[0]
    y_ori=origin[1]
    x_old=coordi[0] - x_ori
    y_old=coordi[1] - y_ori
    x_pix = round(-1 * (y_old/resolution+1-ysize))
    y_pix = round(x_old/resolution)
    return (x_pix, y_pix)

def find(a, uf):
    if (uf[a]<0):
        return a
    else :
        return find(uf[a], uf)

def merge(a,b,uf):
    a=find(a, uf)
    b=find(b, uf)
    if (a==b):
        return False
    else :
        uf[b]=a
        return True

def dfs_point(visit_point, points, ad, result, num):
    if (not visit_point[num]):
        result.append(points[num])
        visit_point[num] = 1
        for item in ad[num]:
            dfs_point(visit_point, points, ad, result, item)

def MST(points):
    n=len(points)
    ad=[[] for _ in range(n)]
    visit_point=[0 for _ in range(n)]
    edgelist=[]
    uf=[]
    for i in range(n):
        uf.append(-1)
    for i in range(n):
        for j in range(i+1,n):
            edgelist.append(Edge(i, j, get_distance(points[i].coordi, points[j].coordi)))
    edge_sorted=sorted(edgelist, key = lambda x: x.w)
    edge_path=[]
    cnt=0
    for edge in edge_sorted:
        if merge(edge.u, edge.v, uf):
            cnt += 1
            ad[edge.u].append(edge.v)
            ad[edge.v].append(edge.u)
            edge_path.append(edge)
        if cnt == (n - 1):
            break
    result=[]
    dfs_point(visit_point, points, ad, result, 0)
    return result

def find_local(obj_pose, current_pose, color, local_r):
    local_points = []
    x = current_pose[0]
    y = current_pose[1]
    for k in range(2*local_r+2) :
        for l in range(2*local_r+2) :
            i = x-local_r-1+k
            j = y-local_r-1+k
            d= get_distance((i,j), current_pose)
            if color[i][j]=='y' and d>=local_r and d<1.5:
                ori=get_ori((i,j), obj_pose)
                local_points.append(Point((i,j),'r', ori))
    local_points.append(Point(current_pose, 'lo', get_ori(current_pose, obj_pose)))
    return local_points


visited = [[0 for item in range(size[1])] for item in range(size[0])]
print("visited :", len(visited))
direction = [-1, 0, 1]
yellow_group=[[0 for item in range(size[1])] for item in range(size[0])]


def dfs(visited, x, y, group_num):
    visited[x][y]=1
    yellow_group[x][y]=group_num
    for i in direction:
        for j in direction:
            pos = (x+i, y+j)
            if is_in(pos) and visited[x+i][y+j]==0 and color[x+i][y+j]=='y':
                dfs(visited,x+i,y+j, group_num)

color = []
ori = []
for i in range(size[0]):
    color.append(['w' for j in range(size[1])])
    ori.append([[0,0] for j in range(size[1])])


r=int(input("put distance: "))
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
                group_num+=1
                dfs(visited,i,j,group_num)
            parted_i = i//10
            parted_j = j//10
            yellow_points.append(Point((i,j), color[i][j], ori[i][j]))
            parted_yellow[parted_i][parted_j].append(Point((i,j), color[i][j], ori[i][j]))
            yellow_num+=1
            parted_num[parted_i][parted_j]+=1

red_group=[[] for _ in range(group_num+1)]

for i in range(i_part):
    for j in range(j_part):
        parted_sample[i][j] = np.random.choice(parted_yellow[i][j], size=round(parted_num[i][j]*0.05), replace=False)

red_num=0
for i in range(i_part):
    for j in range(j_part):
        for item in parted_sample[i][j]:
            (x,y)=item.coordi
            gn=yellow_group[x][y]
            image[x][y]=red
            red_num+=1
            red_group[gn].append(item)

print(yellow_num, red_num)
print("group number :", group_num)

(x,y) = map(float,input("put origin coordinates: ").split())
origin = (x,y)
resolution = 0.05

object_cate=input("put wnated_objects : ").split()

# Setup:
pipe = rs.pipeline()
cfg = rs.config() # config for pyrealsense
cfg2 = get_cfg() # config for detectron
cfg2.merge_from_file(model_zoo.get_config_file("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"))
cfg2.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.5  # set threshold for this model
# Find a model from detectron2's model zoo. You can use the https://dl.fbaipublicfiles... url as well
cfg2.MODEL.WEIGHTS = model_zoo.get_checkpoint_url("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml")
predictor = DefaultPredictor(cfg2)

# set configuration for realsense
cfg.enable_stream(rs.stream.depth,  848, 480, rs.format.z16, 90)
cfg.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)


local_r = int(input("put local_r : "))

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
inter = np.matmul(z_90, camera_trans)
extrinsic_robot_camera = np.matmul(x_90, inter) # put extrinsic matrix between robot & camera


for i in range(group_num):
    print("path", i)
    if len(red_group[i+1])==0 or len(red_group[i+1])==1:
        continue
    path = MST(red_group[i+1])
    a= transform_coordi(path, pix, origin, resolution)

    b=[(item.coordi, item.ori) for item in a]
    for global_point in b:
        current_pose = (global_point[0][0], global_point[0][1])
        current_ori = (global_point[1][0], global_point[1][1])
        nav2(global_point[0][0], global_point[0][1], global_point[1][0], global_point[1][1])
        tf = get_tf(current_pose, current_ori, extrinsic_robot_camera)
        object_info = detect_object(cfg, cfg2, predictor, object_cate, tf) # get global coordinates of object
        for item in object_info: #local path planning
            coordi = item["coordi"]
            object_pixel = transform_inverse(coordi, pix, origin, resolution)
            current_pixel = transform_inverse(current_pose, pix, origin, resolution)
            local_path_pixel = find_local(object_pixel, current_pixel, color, local_r)
            local_path = transform_coordi(local_path_pixel, pix, origin, resolution)
            for local_point in local_path:
                nav2(local_point.coordi[0], local_point.coordi[1], local_point.ori[0], local_point.ori[1])
                # capture & resgistration

#cv2.imwrite('/home/3dvision/path_image/result2_0318.png',image)
