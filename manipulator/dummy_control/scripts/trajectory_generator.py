#!/usr/bin/python3

"""
This program is free software: you can redistribute it and/or modify it 
under the terms of the GNU General Public License as published by the Free Software Foundation, 
either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program. 
If not, see <https://www.gnu.org/licenses/>.

created by Thanacha Choopojcharoen at CoXsys Robotics (2022)
"""

from dummy_control.trajectory import Trajectory
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from dummy_kinematics_interfaces.srv import SetConfig
import sys
import numpy as np

class TrajectoryGenerator(Node):
    def __init__(self):
        super().__init__('trajectory_generator')
        self.robot_name = sys.argv[1]
        trajectory_topic = 'joint_trajectory_position_controller/joint_trajectory'
        self.sub_joint_states = self.create_subscription(JointState,'joint_states',self.joint_states_callback,10)
        self.pub_via_points = self.create_publisher(JointTrajectory,trajectory_topic,10)
        self.srv_set_joints = self.create_service(SetConfig,'set_joints',self.set_joints_callback)
        self.declare_parameters(namespace='',parameters=[('joints',['joint_1','joint_2','joint_3'])])
        self.joint_names = self.get_parameter('joints').value
        self.joint_states = JointState()
        self.trajectory = Trajectory()
    def joint_states_callback(self,msg):
        self.joint_states = msg
    def set_joints_callback(self,request:SetConfig.Request,response:SetConfig.Response):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.joint_names
        self.get_logger().info(f'{msg.joint_names}')
        # fixed via-point
        q = np.array([[0.1,-0.1,0.2],[-0.1,0.2,0.1]]).T
        v_max = [1]*3
        a_max = [0.1]*3

        self.trajectory.set_position(q)
        self.trajectory.set_bound(v_max,a_max)
        self.trajectory.time_optimal(np.array([self.joint_states.position]).T)
        time_from_start = 0.0
        for i in range(len(self.trajectory.position)):
            via_point = JointTrajectoryPoint()
            via_point.positions = self.trajectory.position[i]
            via_point.velocities = self.trajectory.velocity[i]
            time_from_start = time_from_start+self.trajectory.duration[i]
            via_point.time_from_start = Duration()
            via_point.time_from_start.sec = np.math.floor(time_from_start)
            via_point.time_from_start.nanosec = int((time_from_start-np.math.floor(time_from_start))*(10**9))
            msg.points.append(via_point)
        self.pub_via_points.publish(msg)
        return response

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
