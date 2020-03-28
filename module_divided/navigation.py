"""
this module uses nav2 stack for navigation. The client node sends "NavigateToPose" action message as goal.
If a goal fails, it either resends the goal or get back to reset point until it succeeds the original goal.
"""

import integrated as inte
import transformation as t
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import math, time
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
#action type is NavigateToPose
from nav2_msgs.action import NavigateToPose

"""
define Node for navigation. this node send action messages as goal pose and subscribes 'amcl_pose'
"""

class MyClient(Node):

    def __init__(self):
        super().__init__('my_client')
        # action type : NavigateToPose, action name : NavigateToPose
        # create action_clieent which sends goal pose.
        self._action_client = ActionClient(self,NavigateToPose,'NavigateToPose')
        # 'amcl_pose' is for comparing between goal pose & current pose
        self.model_pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.poseCallback)
        self.initial_pose_received = False # is 'amcl_pose' subscribed ?

    def poseCallback(self, msg):
        self.current_pose = msg.pose.pose # update current pose
        self.initial_pose_received = True

    def distanceFromGoal(self, goal_pose):
        dx = self.current_pose.position.x - goal_pose.position.x
        dy = self.current_pose.position.y - goal_pose.position.y
        distance = math.sqrt(dx * dx + dy * dy)
        return distance

    def yawFromGoal(self, goal_pose):
        theta = 2 * math.asin(self.current_pose.orientation.z) * 180 / math.pi
        theta2 = 2 * math.asin(goal_pose.orientation.z) * 180 / math.pi
        a = theta - theta2
        a = (a + 180) % 360 - 180
        dif = float(abs(a))
        return dif

    def send_goal(self, xpose, ypose, zpose, wpose):
        goal_msg = NavigateToPose.Goal()
        # goal_msg.pose.header.stamp.sec = round(self.get_clock().now().nanoseconds*(10**-9))
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.pose.position.x = xpose
        goal_msg.pose.pose.position.y = ypose
        goal_msg.pose.pose.orientation.z = zpose
        goal_msg.pose.pose.orientation.w = wpose
        self._action_client.wait_for_server()
        count=0
        succeed_count=0
        while True:
            print("count :", count)
            self._send_goal_future = self._action_client.send_goal_async(
            goal_msg)
            # wait until feedback comes
            rclpy.spin_until_future_complete(self,self._send_goal_future)
            goal_handle = self._send_goal_future.result()

            if not goal_handle.accepted:
                self.get_logger().info('Goal rejected :(')
                return

            self.get_logger().info('Goal accepted :)')
            self._get_result_future = goal_handle.get_result_async()
            # wait until result comes
            rclpy.spin_until_future_complete(self, self._get_result_future)
            status = self._get_result_future.result().status
            if (status == GoalStatus.STATUS_SUCCEEDED): #nav2 stack thinks goal succeeded
                self.get_logger().info("Goal Succeeded")
                if not self.initial_pose_received or succeed_count>=3: #for catching bug with nav2 stack
                    print("wait for initial_pose / bbuk nan geo im")
                    rclpy.spin_once(self, timeout_sec = 1)
                    return False
                print("distance :", self.distanceFromGoal(goal_msg.pose.pose))
                print("yaw :", self.yawFromGoal(goal_msg.pose.pose))
                # if difference is too big, send goal again
                if self.distanceFromGoal(goal_msg.pose.pose) > 0.15 or self.yawFromGoal(goal_msg.pose.pose) > 15:
                    succeed_count+=1
                    continue
                break
            else: # if goal fails, send goal again
                count+=1
                continue
        return True

# rclpy order : init -> client generation -> shutdown
def nav2(xpose, ypose, zori, wori):
    rclpy.init(args=None)
    print("nav pose:",xpose, ypose, zori, wori)
    action_client = MyClient()
    passed = action_client.send_goal(xpose, ypose, zori, wori)
    rclpy.shutdown() #shutdown for removing current context
    while not passed: # if fails, go to reset point and try again until succeed
        reset = find_reset(xpose, ypose, 10)
        rclpy.init(args=None)
        _action_client = MyClient()
        _action_client.send_goal(reset[0], reset[1], 0.0, 1.0)
        rclpy.shutdown()
        rclpy.init(args=None)
        _action_client = MyClient()
        passed = _action_client.send_goal(xpose,ypose,zori,wori)
        rclpy.shutdown()
    print("sleep 3sec")
    time.sleep(3)

# for finding reset point when navigation goes wrong.
# reset point should be 'local_r' radius away from goal pose and the color should be yellow or white.
# yellow is global path. black is near obstacles.
def find_reset(x, y, local_r):
    center = t.transform_inverse((x,y), inte.pix, inte.origin, inte.resolution)
    print("center :",center)
    x=center[0]
    y=center[1]
    for k in range(2*local_r+2) :
        for l in range(2*local_r+2) :
            i = x-local_r-1+k
            j = y-local_r-1+l
            d= inte.get_distance((i,j), center)
            if (inte.color[i][j]=='y' or inte.color[i][j] == 'w') and d>=local_r and d<local_r+1.0:
                reset=t.transform_coordi2((i,j), inte.pix, inte.origin, inte.resolution)
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
            d= inte.get_distance((i,j), current_pose)
            if color[i][j]=='y' and d>=local_r and d<local_r+1.0:
                ori=inte.get_ori((i,j), obj_pose)
                dupl=False
                for item in local_points:
                    if inte.get_distance((i,j), item.coordi) <=1.5:
                        dupl=True
                if not dupl:
                    local_points.append(inte.Point((i,j),'r', ori))
    local_points.append(inte.Point(current_pose, 'lo', inte.get_ori(current_pose, obj_pose)))
    return local_points
