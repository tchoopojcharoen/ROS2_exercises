#!/usr/bin/python3
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import yaml
class InverseKinematics(Node):
    def __init__(self):
        super().__init__('inverse_kinematics')
        self.command_publisher = self.create_publisher(Float64MultiArray,'/velocity_controllers/commands',10)
        
        self.cmd_vel_subscription = self.create_subscription(Twist,'/cmd_vel',self.cmd_vel_callback,10)
        self.cmd_vel = Twist()
        self.period = 0.1
        self.timer = self.create_timer(self.period,self.timer_callback)
        self.counter = 0
        with open(sys.argv[1]) as f:
            model_parameters = yaml.load(f, Loader=yaml.loader.SafeLoader)
        self.wheel_separation = float(model_parameters['wheel_separation'])
        self.wheel_radius = float(model_parameters['wheel_radius'])
        
        self.get_logger().info(f'WS:{self.wheel_separation}')
        self.get_logger().info(f'WR:{self.wheel_radius}')
    def timer_callback(self):
        hold_time = 0.5
        if self.counter < hold_time:
            self.counter = self.counter + self.period
            if self.counter >=hold_time:
                cmd = Float64MultiArray()
                cmd.data = [0.0,0.0]
                self.command_publisher.publish(cmd)
    def cmd_vel_callback(self,msg:Twist):
        cmd = Float64MultiArray()
        cmd.data = self.compute(msg.linear.x,msg.angular.z)
        self.counter = 0
        self.command_publisher.publish(cmd)
    def compute(self,v,w):
        left_wheel_velocity = v/self.wheel_radius-w*self.wheel_separation/(2*self.wheel_radius)
        right_wheel_velocity = v/self.wheel_radius+w*self.wheel_separation/(2*self.wheel_radius)        
        return [left_wheel_velocity,right_wheel_velocity]

def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
