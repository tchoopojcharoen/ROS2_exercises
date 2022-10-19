#!/usr/bin/python3

# import libraries
import numpy as np
import rclpy
from geometry_msgs.msg import Point
from std_srvs.srv import Empty
from turtlesim_control.base_controller import BaseController
from turtlesim_interfaces.srv import SetGoal, GetPosition

class ViaPointFollower(BaseController):
    """ROS node that respresents controller for following a via point. 
    ROS Interfaces:
        Publishers : 
        Subscribers : 
        Service Servers : /set_goal, /enable, /get_position
        Service Clients : /notify_arrival
        Action Servers :
        Action Clients :
        Parameters : 
    Attributes:
            
    """
    def __init__(self):
        # construct base controller
        super().__init__('via_point_follower')
        # create and add ROS service server /set_goal
        self.set_goal_service = self.create_service(SetGoal,'/set_goal',self.set_goal_callback)
        # create and add ROS service server /enable
        self.enable_service = self.create_service(Empty,'/enable',self.enable_callback)
        # create and add ROS service server /get_position
        self.get_position = self.create_service(GetPosition,'/get_position',self.get_position_callback)
        # create and add ROS service client /notify_arrival
        self.notify_arrival_client = self.create_client(Empty,'/notify_arrival')
    def set_goal_callback(self,request,response):
        #  assign goal
        self.goal = np.array([request.position.x,request.position.y])
        return response
    def enable_callback(self,request,response):
        #  enable the controller
        self.isEnable = True
        return response
    def get_position_callback(self,request,response):
        #  respond with the current position
        response.position = Point()
        response.position.x = self.pose.x
        response.position.y = self.pose.y        
        return response
    def send_notify_arrival_request(self):
        # send request to /notify_arrival
        req = Empty.Request()
        self.future = self.notify_arrival_client.call_async(req)
    def arrival_callback(self):
        # when arrived, disable the controller and send notification to /notify_arrival server
        self.isEnable = False
        self.send_notify_arrival_request()
        self.get_logger().info("Caught up with another turtle.")
    def departure_callback(self):
        # when departed, enable the controller
        self.isEnable = True
        self.get_logger().info("Oh no!, the other turtle has moved !")

def main(args=None):
    # initiate ROS Client Library
    rclpy.init(args=args)
    # create ViaPointFollower
    via_point_follower = ViaPointFollower()
    # spin ViaPointFollower
    rclpy.spin(via_point_follower)
    # if ViaPointFollower stops, deconstruct it
    via_point_follower.destroy_node()
    # shut down ROS CLIENT LIBRARY
    rclpy.shutdown()
if __name__=='__main__':
    main()
