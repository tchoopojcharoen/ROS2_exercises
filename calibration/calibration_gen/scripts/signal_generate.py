#!/usr/bin/python3

import numpy as np

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import os
from std_msgs.msg import Float64MultiArray
import yaml 

class CalibrationGen(Node):
    def __init__(self):
        super().__init__('signal_generator')
        
        # establish timer
        self.timer_period = 0.1
        self.sensor_publisher = self.create_publisher(Float64MultiArray,'sensor_data',10)
        self.timer = self.create_timer(self.timer_period,self.timer_callback)
        
        calibration_gen_path = get_package_share_directory('calibration_gen')
        path = os.path.join(calibration_gen_path,'config','config.yaml')
        with open(path) as f:
            self.properties = yaml.load(f, Loader=yaml.loader.SafeLoader)
        self.get_logger().info(f'Starting Signal Generator')
    def timer_callback(self):

        ### Add codes here
        
        ###
        msg = Float64MultiArray()
        msg.data = x[0].tolist()
        self.sensor_publisher.publish(msg)
        self.get_logger().info(f'Publish {len(x[0])}-channel signal to the network.')
    
def main(args=None):
    rclpy.init(args=args)
    node = CalibrationGen()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()