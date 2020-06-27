"""
this module includes functions for transforming between coordinates

"""
import math
import numpy as np

class Point:
    def __init__(self, coordi, color, ori):
        self.coordi = coordi
        self.color = color
        self.ori = ori

# transform cos, sin to z, w quaternions
def transform_ori(ori):
    w=math.sqrt((1+ori[0])/2)
    z=ori[1]/(2*w)
    new_ori=(z, w)
    return new_ori

def transform_ori_inverse(ori):
    z = ori[0]
    w = ori[1]
    cos = w**2 - z**2
    sin = 2*z*w
    return (cos, sin)

#transform pixel coordinates to world coordinates
def transform_coordi2(point, im, origin, resolution):
    xsize=im.shape[1]
    ysize=im.shape[0]
    x_ori=origin[0]
    y_ori=origin[1]
    x_new = point[1]*resolution
    y_new = (ysize-point[0]-1)*resolution
    x_new = x_new + x_ori
    y_new = y_new + y_ori
    return (x_new, y_new)

#transform pixel coordinates to world coordinates
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

# get transformation matrix
def get_tf(robot_pose, robot_ori, extrinsic_robot_camera):
    sin_2 = robot_ori[0]
    cos_2 = robot_ori[1]
    sin = 2*sin_2*cos_2
    cos = cos_2**2 - sin_2**2
    robot_translation = np.array([[1, 0, 0, robot_pose[0]],
                           [0, 1, 0, robot_pose[1]],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]])
    robot_rotation = np.array([[cos, -sin, 0, 0],
                              [sin, cos, 0, 0],
                              [0, 0, 1, 0],
                              [0, 0, 0, 1]])
    tf=np.matmul(robot_translation, robot_rotation)
    tf=np.matmul(tf, extrinsic_robot_camera)
    return tf

#transform world coordinates to pixel
def transform_inverse(coordi, im, origin, resolution):
    xsize=im.shape[1]
    ysize=im.shape[0]
    x_ori=origin[0]
    y_ori=origin[1]
    x_old=coordi[0] - x_ori
    y_old=coordi[1] - y_ori
    x_pix = round(-1 * (y_old/resolution+1-ysize))
    y_pix = round(x_old/resolution)
    return (x_pix, y_pix)
