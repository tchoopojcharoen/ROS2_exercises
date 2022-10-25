#!/usr/bin/python3
import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point

from std_srvs.srv import Empty
from turtlesim_interfaces.srv import RandGoal, SetGoal

class Scheduler(Node):
    def __init__(self):
        super().__init__('scheduler')
        self.rand_goal_service = self.create_service(RandGoal,'rand_goal',self.rand_goal_callback)
        self.enable_client = self.create_client(Empty,'enable')
        self.set_goal_client = self.create_client(SetGoal,'set_goal')
        self.notify_arrival_service = self.create_service(Empty,'notify_arrival',self.notify_arrival_callback)
        
        self.via_points = np.array([[2.0,5.0,8.0,1.0,9.0,2.0],[1.0,9.0,1.0,6.0,6.0,1.0]])
        self.num_via_points = self.via_points.shape[1]
        self.idx = 0 
        current_goal = Point()
        current_goal.x = self.via_points[0][self.idx]
        current_goal.y = self.via_points[1][self.idx]

        while not self.set_goal_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set_goal service not available, waiting again')
        
        self.send_set_goal_request(current_goal)

        while not self.enable_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('enable service not available, waiting again')
        self.send_enable_request()

    def rand_goal_callback(self,request,response):
        goal = 9*np.random.rand(2)+0.5
        response.position = Point()
        response.position.x = goal[0]
        response.position.y = goal[1]
        self.send_set_goal_request(response.position)
        self.send_enable_request()
        self.get_logger().info(f'New goal => x:{goal[0]},y:{goal[1]}')
        return response
    def send_enable_request(self):
        req = Empty.Request()
        self.future = self.enable_client.call_async(req)
    def send_set_goal_request(self,position):
        req = SetGoal.Request()
        req.position = position
        self.future = self.set_goal_client.call_async(req)
    def notify_arrival_callback(self,request,response):
        self.idx = self.idx + 1
        if self.idx < self.num_via_points:
            current_goal = Point()
            current_goal.x = self.via_points[0][self.idx]
            current_goal.y = self.via_points[1][self.idx]
            self.send_set_goal_request(current_goal)
            self.send_enable_request()
        return response

def main(args=None):
    rclpy.init(args=args)
    node = Scheduler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
