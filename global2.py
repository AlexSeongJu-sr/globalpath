import PIL.Image as pilimg
import numpy as np
import math
import cv2

# Read image
im = pilimg.open('map951_200214.pgm')

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
        ori_new = (ori[1], -ori[0])
        new_points.append(Point((x_new,y_new),'r',ori_new))
    return new_points

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

def find_local(obj_pose, current_pose, color, r):
    local_points = []
    local_points.append(Point(current_pose, 'r', get_ori(current_pose, obj_pose)))
    x = current_pose[0]
    y = current_pose[1]
    for k in range(2*r+2) :
        for l in range(2*r+2) :
            i = x-r-1+k
            j = y-r-1+k
            d= get_distance((i,j), current_pose)
            if color[i][j]=='r' and d>=r and d<1.5:
                ori=get_ori((i,j), obj_pose)
                local_points.append(Point((i,j),'r', ori))
    return local_points

color = []
ori = []
for i in range(size[0]):
    color.append(['w' for j in range(size[1])])
    ori.append([[0,0] for j in range(size[1])])


r=int(input("put distance: "))
red_num=0
red_points=[]
zero_points=[]
image=np.zeros((size[0],size[1],3), np.uint8)
red = [0,255,255]
unknown = [0, 0, 255]
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
                       color[x][y]='r'
                       ori[x][y]=get_ori((x,y), (i,j))
                       image[x][y]=red


for i in range(size[0]):
    for j in range(size[1]):
      if color[i][j] =='r' :
        red_points.append(Point((i,j), color[i][j], ori[i][j]))
        red_num+=1

print(red_num)
sample_points=np.random.choice(red_points, size=round(red_num*0.05), replace=False)
print(sample_points.shape[0])
a=[(item.coordi, item.ori) for item in sample_points]
print(a)

for i in range(sample_points.shape[0]):
    (x,y)=sample_points[i].coordi
    image[x][y]=unknown
cv2.imwrite('result6.png',image)
(x,y) = map(float,input("put origin coordinates: ").split())
origin = (x,y)
resolution = 0.05
new_red = transform_coordi(sample_points,pix,origin,resolution)
a=[(item.coordi,item.ori) for item in new_red]
print(a)






