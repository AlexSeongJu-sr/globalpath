import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import math, time
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
#action type is NavigateToPose
from nav2_msgs.action import NavigateToPose
import capture
import mytransformation as tr
import open3d as o3d
import numpy as np
import copy
import pyrealsense2 as rs
from open3d.open3d.geometry import OrientedBoundingBox
import pdb

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])

class MyClient(Node):

    def __init__(self):
        super().__init__('test_client')
        # action type : NavigateToPose, action name : NavigateToPose. This is processed in the navigation stack.
        # create action_clieent which sends goal pose.
        # 'amcl_pose' is for comparing between goal pose & current pose
        self.model_pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.poseCallback)
        self.initial_pose_received = False # is 'amcl_pose' received?

    def poseCallback(self, msg):
        self.current_pose = msg.pose.pose # update current pose
        self.initial_pose_received = True # To make sure that robot odometry information is received
        print("callback")

pdb.set_trace()
rclpy.init()
test_client = MyClient()

points = []
box = OrientedBoundingBox()
cfg = rs.config() # config for pyrealsense
cfg.enable_stream(rs.stream.depth,  848, 480, rs.format.z16, 30)
cfg.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)
pipe = rs.pipeline()
thre = rs.threshold_filter(0.1, 3.0)

input("Move turtlebot manually. first pose ready?")
rclpy.spin(test_client)
current_pose = test_client.current_pose
print("current_pose : ",current_pose)
ori = (current_pose.orientation.z, current_pose.orientation.w)
(cos, sin) = tr.transform_ori_inverse(ori)
(x, y) = (current_pose.position.x, current_pose.position.y)
r_trans = np.array([[cos, 0, sin, 0],
                    [0, 1, 0, 0],
                    [-sin, 0, cos, 0],
                    [0, 0, 0, 1]])
t_trans = np.array([[1, 0, 0, -y],
                    [0, 1, 0, 0],
                    [0, 0, 1, x],
                    [0, 0, 0, 1]])
print("capture start")
tmp = capture.capture(cfg, pipe, thre, box)
print("capture end")
tmp.transform(r_trans)
tmp.transform(t_trans)
points.append(tmp)

input("Move turtlebot manually. second pose ready?")
rclpy.spin_once(test_client)
current_pose = test_client.current_pose
print("current_pose : ",current_pose)
ori = (current_pose.orientation.z, current_pose.orientation.w)
ori = (current_pose.orientation.z, current_pose.orientation.w)
(cos, sin) = tr.transform_ori_inverse(ori)
(x, y) = (current_pose.position.x, current_pose.position.y)
r_trans = np.array([[cos, 0, sin, 0],
                    [0, 1, 0, 0],
                    [-sin, 0, cos, 0],
                    [0, 0, 0, 1]])
t_trans = np.array([[1, 0, 0, -y],
                    [0, 1, 0, 0],
                    [0, 0, 1, x],
                    [0, 0, 0, 1]])

print("second capture start")
tmp = capture.capture(cfg, pipe, thre, box)
print("second capture end")
tmp.transform(r_trans)
tmp.transform(t_trans)
points.append(tmp)

print("before icp regi")
o3d.io.write_point_cloud('before_icp.ply', points[0]+points[1])

start=time.time()
print("Point-to-point ICP registration is applied on original point")
print("clouds to refine the alignment. Distance threshold 0.1.")
result_icp = o3d.registration.registration_icp(
    points[0], points[1], 0.1, np.identity(4),
    o3d.registration.TransformationEstimationPointToPoint())
end=time.time()
print("icp time :", end-start)
o3d.io.write_point_cloud('after_icp.ply', points[0].transform(result_icp.transformation)+points[1])
rclpy.shutdown()




