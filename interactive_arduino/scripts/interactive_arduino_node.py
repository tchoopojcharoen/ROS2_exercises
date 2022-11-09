#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from interactive_arduino.msg import Imu6DOF
from interactive_arduino.msg import DemoShieldInput
from geometry_msgs.msg import Twist
class Interactive_arduino(Node):

    def __init__(self):
        super().__init__('interactive_arduino')
        self.subscription_imu = self.create_subscription(Imu6DOF,'imu_arduino',self.listener_callback_imu,10)
        self.subscription_imu
        self.subscription_input = self.create_subscription(DemoShieldInput,'input_arduino',self.listener_callback_input,10)
        self.subscription_input

        self.publisher_ = self.create_publisher(Twist,'/cmd_vel',10)
        self.demoshieldinput = DemoShieldInput()

    def listener_callback_imu(self, msg):
        ax=msg.specific_force[0]
        gx=msg.angular_velocity[0]
        msgg=Twist()
    
        if(self.demoshieldinput.button.states[0]==1):
            msgg.linear.x=-float(ax)*0.2  # * gain
        else:
            msgg.linear.x=0.0

        if(self.demoshieldinput.button.states[1]==1):
            msgg.angular.z=-float(gx)
        else:
            msgg.angular.z=0.0
        self.publisher_.publish(msgg)

    def listener_callback_input(self, msg):
        self.demoshieldinput = msg



def main(args=None):
    rclpy.init(args=args)
    interactive_arduino = Interactive_arduino()
    rclpy.spin(interactive_arduino)
    interactive_arduino.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
