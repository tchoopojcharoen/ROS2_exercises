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
"""

import sys, os, yaml
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import JointState
from dummy_kinematics_interfaces.srv import SolveIK, SetConfig
from dummy_kinematics.kinematics import inverse_kinematics
from ament_index_python.packages import get_package_share_directory
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class KinematicsServer(Node):
    def __init__(self):
        super().__init__('kinematics_server')
        self.joint_state_publisher = self.create_publisher(JointState,'/joint_states',10)
        self.IK_server = self.create_service(SolveIK,'solve_ik',self.IK_callback)
        self.FK_server = self.create_service(SetConfig,'set_config',self.set_config_callback)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer,self)

        self.get_logger().info(f'Starting kinematics solver with node name {self.get_node_names()[0]}')
        description_pkg_path = get_package_share_directory('dummy_description')
        parameter_path = os.path.join(description_pkg_path,'config','DH_parameters.yaml')
        with open(parameter_path) as f:
            self.param = yaml.load(f,Loader=yaml.SafeLoader)
        self.joint_names = ['joint_1','joint_2','joint_3']
        msg = JointState()
        msg.name = self.joint_names
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = [0.,0.,0.]
        self.joint_states = msg
        self.timer = self.create_timer(0.1,self.timer_callback)
    def timer_callback(self):
        self.joint_states.header.stamp = self.get_clock().now().to_msg()
        self.joint_state_publisher.publish(self.joint_states)
        
    def set_config_callback(self,requst:SetConfig.Request,response:SetConfig.Response):
        self.joint_states.header.stamp = self.get_clock().now().to_msg()
        self.joint_states.position = requst.joint_states.position
        self.joint_state_publisher.publish(self.joint_states)
        trans = self.tf_buffer.lookup_transform('base_link','end_effector',Duration(seconds=0))
        
        response.position.x = trans.transform.translation.x
        response.position.y = trans.transform.translation.y
        response.position.z = trans.transform.translation.z
        
        return response
    def IK_callback(self,request:SolveIK.Request,response:SolveIK.Response):
        p = (request.position.x,request.position.y,request.position.z)
        gamma_list = [1,1]
        idx = 0
        for g in request.arm_config.data :
            if (g != 1) & (g != -1):
                self.get_logger().warning(f'The arm configuration [{idx}] should be an array of 1 or -1.')
                if g>=0:
                    gamma_list[idx] = 1
                    self.get_logger().warning(f'The positive arm configuration [{idx}] is mapped to gamma = 1')
                else:
                    gamma_list[idx] = -1
                    self.get_logger().warning(f'The negative arm configuration [{idx}] is mapped to gamma = -1')
            else:
                gamma_list[idx] = g
            idx = idx+1
        gamma = (gamma_list[0],gamma_list[1])
        l_1 = self.param['DH'][1][1]
        l_2 = self.param['DH'][2][1]
        l_3 = self.param['ee_position'][0]
        h_1 = self.param['DH'][0][3]
        h_3 = self.param['ee_position'][1]
        q,flag = inverse_kinematics(p,gamma,l_1,l_2,l_3,h_1,h_3)
        
        for q_i in q:
            response.joint_states.position.append(q_i)
        response.flag.data = flag
        if flag:
            response.joint_states.name = self.joint_names
            response.joint_states.header.stamp = self.get_clock().now().to_msg()
            self.joint_states = response.joint_states
        return response

def main(args=None):

    rclpy.init(args=args)
    node = KinematicsServer()
    try:
        while rclpy.ok():
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        print('repeater stopped cleanly')
    except BaseException:
        print('exception in repeater:', file=sys.stderr)
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown() 

if __name__=='__main__':
    main()
