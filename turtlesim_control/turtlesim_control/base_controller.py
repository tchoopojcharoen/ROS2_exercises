# import libraries
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class BaseController(Node):
    """ROS node that respresents base go-to-point controller for turtlesim

    This class is an abstract class that provides go-to-point functionality
    for turtlesim in ROS2. 

    ROS Interfaces:
        Publishers : /cmd_vel
        Subscribers : /pose
        Service Servers :
        Service Clients :
        Action Servers :
        Action Clients :
        Parameters : gain
    Attributes:
        pose: pose of turtlesim
        goal: goal location of the controller
        isEnable: a flag indicating controller status 
            True => controller is enabled 
            False => controller is disabled
    """
    # Constructor for BaseController
    def __init__(self,name):
        # construct node
        super().__init__(name)
        # create and add ROS publisher for /cmd_vel
        self.command_publisher = self.create_publisher(Twist,'/cmd_vel',10)
        # create a timer for the ROS publisher 
        timer_period = 0.1
        self.timer = self.create_timer(timer_period,self.timer_callback)
        # create a ROS subscriber for /pose 
        self.pose_subscription = self.create_subscription(Pose,'/pose',self.pose_callback,10)
        # declare 'gain' as ROS parameter
        self.declare_parameters(namespace='',parameters=[('gain',5.0),])
        # assign default values for the attributes
        self.pose = Pose()
        self.goal = None
        self.isEnable = False
    # Callback for timer
    def timer_callback(self):
        # Publish /cmd_vel when enabled
        # Check for departure when disabled
        if self.isEnable:
            # Compute and publish /cmd_vel
            msg = self.control()
            self.command_publisher.publish(msg)
        else:
            current_position = np.array([self.pose.x,self.pose.y])
            # check whether goal has been assigned
            if self.isEnable is None:
                dp = self.goal-current_position
                # if current position is relatively far from the goal
                if np.linalg.norm(dp)>0.1:
                    # call departure behavior
                    self.departure_callback()
    # Callback for /pose subscriber
    def pose_callback(self,msg):
        self.pose = msg
    # Control law computation
    def control(self):
        msg = Twist()
        current_position = np.array([self.pose.x,self.pose.y])
        dp = self.goal-current_position
        e = np.arctan2(dp[1],dp[0])-self.pose.theta
        K = self.get_parameter('gain').get_parameter_value().double_value
        w = K*np.arctan2(np.sin(e),np.cos(e))
        
        # if current position is relatively far from the goal
        if np.linalg.norm(dp)>0.1:
            v = 1.0
        # if current position is close to the goal
        else:
            v = 0.0
            w = 0.0
            # call arrival behavior
            self.arrival_callback()
        msg.linear.x = v
        msg.angular.z = w
        return msg
    # Behavior when robot arrives at the goal
    def arrival_callback(self):
        pass
    # Behavior when the goal is moved
    def departure_callback(self):
        pass
