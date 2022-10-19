#!/usr/bin/python3

# import libraries
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Empty
from turtlesim_interfaces.srv import SetGoal, GetPosition
from turtlesim_interfaces.action import GoToGoal

class FollowerInterfaceServer(Node):
    """ ROS node that directly consists of /go_to_goal action server.
    
    This class describes the behavior of /go_to_goal action.
    
    ROS Interfaces:
        Publishers : 
        Subscribers : 
        Service Servers : 
        Service Clients : /set_goal, /enable, /get_position
        Action Servers : /go_to_goal
        Action Clients : 
        Parameters :
    Attributes:
        isActive : 
    """
    # Constructor for FollowerInterfaceServer
    def __init__(self):
        # construct node
        super().__init__('action_scheduler_server')
        # create and add ROS action server for /got_to_goal
        self.go_to_goal_action_server = ActionServer(self,GoToGoal,'/go_to_goal',self.execute_callback)
        # create and add ROS service client /set_goal
        self.set_goal_client = self.create_client(SetGoal,'/set_goal')
        # create and add ROS service client /enable
        self.enable_client = self.create_client(Empty,'/enable')
        # create and add ROS service client /get_position
        self.get_position_client = self.create_client(GetPosition,'/get_position')
        # isActive is default to False
        self.isActive = False
    # Callback for action server
    def execute_callback(self,goal_handle):
        self.get_logger().info(f'Executing action...')
        # obtain the start time and initial position when called
        # print(goal_handle.request.goal)
        init_time = self.get_clock().now()
        init_position = self.send_get_position_request()
        # set goal at the controller 
        goal = goal_handle.request.goal
        self.send_set_goal_request(goal)
        while self.future.done():
            self.get_logger().info('Wait for set_goal to finish...')
        # enable the controller once the goal is set
        self.send_enable_request()
        while self.future.done():
            self.get_logger().info('Wait for enable ...')
        # activate action loop, exit when triggered by /notify_arrival
        self.isActive = True
        feedback_msg = GoToGoal.Feedback()
        while self.isActive:
            # compute elapsed time
            dt = self.get_clock().now()-init_time
            feedback_msg.elasped_time = dt.nanoseconds/1e9
            goal_handle.publish_feedback(feedback_msg)
        # get result to succeed
        goal_handle.succeed()
        # return absolute distance as result
        result = GoToGoal.Result()
        position = self.send_get_position_request()
        result.distance = np.linalg.norm(np.array([position.x,position.y])-np.array([init_position.x,init_position.y]))
        return result
    def send_set_goal_request(self,position):
        # send the given goal to /set_goal
        req = SetGoal.Request()
        req.position = position
        self.future = self.set_goal_client.call_async(req)
    def send_enable_request(self):
        # call /enable
        req = Empty.Request()
        self.future = self.enable_client.call_async(req)
    def send_get_position_request(self):
        # call and wait for position response from the controller
        req = GetPosition.Request()
        # print(req)
        self.future = self.get_position_client.call_async(req)
        while not self.future.done():
            self.get_logger().info('Get current position...')
        # print(self.future.result().position)
        return self.future.result().position
        
class FollowerInterface(Node):
    """ ROS node that represents an interface for via-point following controller node 
        that consists of /go_to_goal action server.
    
    This class provides /go_to_goal action interface, which allows the robot to go 
    to a given goal position via ROS action /go_to_goal.
    
    ROS Interfaces:
        Publishers : 
        Subscribers : 
        Service Servers : /notify_arrival
        Service Clients : 
        Action Servers : 
        Action Clients : 
        Parameters :
    Attributes:
        action_server : ROS action server for /go_to_goal
    """
    # Constructor for ActionScheduler
    def __init__(self,action_server):
        # construct node
        super().__init__('action_scheduler')
        # assign the given action server to the node
        self.action_server = action_server
        # create and add ROS service server /notify_arrival
        self.notify_arrival_server = self.create_service(Empty,'/notify_arrival',self.notify_arrival_callback)
    def notify_arrival_callback(self,request,response):
        # exit the action loop when requested
        self.action_server.isActive = False
        return response
        
def main(args=None):
    rclpy.init(args=args)
    try:
        action_server = FollowerInterfaceServer()
        interface = FollowerInterface(action_server=action_server)
        # create executor
        executor = MultiThreadedExecutor(num_threads=4)
        # add nodes to the executor
        executor.add_node(action_server)
        executor.add_node(interface)
        try:
            # spin both nodes in the executor
            executor.spin()
        finally:
            executor.shutdown()
            action_server.destroy_node()
            interface.destroy_node()
    finally:
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()
