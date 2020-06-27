"""
this module uses nav2 stack for navigation. The client node sends "NavigateToPose" action message as goal.
If a goal fails, it either resends the goal or get back to reset point until it succeeds the original goal.
"""

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

            #self.get_logger().info('Goal accepted :)')
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
                if self.distanceFromGoal(goal_msg.pose.pose) > 0.15 or self.yawFromGoal(goal_msg.pose.pose) > 17:
                    succeed_count+=1
                    count+=1
                    continue
                break
            else: # if goal fails, send goal again
                count+=1
                continue
        return True

# rclpy order : init -> client generation -> shutdown
def nav2(xpose, ypose, zori, wori, find_reset):
    rclpy.init(args=None)
    print("nav pose:",xpose, ypose, zori, wori)
    action_client = MyClient()
    passed = action_client.send_goal(xpose, ypose, zori, wori)
    if passed:
        current_pose = action_client.current_pose
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
        if passed:
            current_pose = _action_client.current_pose
        rclpy.shutdown()
    print("sleep 3sec")
    time.sleep(3)
    return current_pose

