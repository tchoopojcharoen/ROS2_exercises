#!/usr/bin/python3

# import module
import numpy as np
import rclpy
import sys
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Point
from turtlesim_interfaces.action import GoToGoal 

class Scheduler(Node):

    def __init__(self):
        super().__init__('scheduler')
        self._action_client = ActionClient(
            self,
            GoToGoal,
            '/go_to_goal')
        # list of via points 
        self.via_points = np.array([[2.0,5.0,8.0,1.0,9.0,2.0],[1.0,9.0,1.0,6.0,6.0,1.0]])
        # self.via_points = np.array([[2.0],[1.0]])
        # compute number of via points
        self.num_via_points = self.via_points.shape[1]
        # initialize an index of list of via points to 0
        self.num = 0
        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('action not available, waiting again...')

    def send_goal(self, goal):
        goal_msg = GoToGoal.Goal()  
        goal_msg.goal = goal
        # print(self._action_client.wait_for_server(timeout_sec=1.0))
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)    
        self._send_goal_future.add_done_callback(self.goal_response_callback) 
    
    
    def goal_response_callback(self, future): 
        # Checking Action Server Result
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected...')
            return
        self.get_logger().info('Goal accepted...')
            
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
   
    def get_result_callback(self, future):
        # Action Result Callback
        result = future.result().result
        self.get_logger().info('Result {0}'.format(result.distance))
        self.get_logger().info('Turtle Arrive at Goal\n')
        if self.task_scheduler_callback():
            position_goal = self.updated_goal() 
            self.send_goal(position_goal)
        
    def feedback_callback(self, feedback_msg):
        # Action Feedback Callback
        feedback = feedback_msg.feedback
        # self.get_logger().info('Feedback {0}'.format(feedback.elasped_time))
        
    def task_scheduler_callback(self):
        # Task_scheduler Callback Function checking viapoint index
        if self.num != self.num_via_points:
            self.get_logger().info('Go to viapoint index: {0}'.format(self.num))
            return True
        else:
            self.get_logger().info("Out of viapoint")
            return False

    def updated_goal(self):
        # Updateing Goal Position return the current viapoint
        current_goal = Point()
        current_goal.x = self.via_points[0][self.num]
        current_goal.y = self.via_points[1][self.num]
        self.get_logger().info('Go to point X: {0} Y: {1}'.format(current_goal.x, current_goal.y))
        self.num += 1
        return current_goal
        

def main(args=None):
    rclpy.init(args=args)
    scheduler = Scheduler()

    # Send the goal  
    if scheduler.task_scheduler_callback():
        position_goal = scheduler.updated_goal() 
        scheduler.send_goal(position_goal)

    # Spin to execute callbacks
    try:
        while rclpy.ok():
            rclpy.spin_once(scheduler)
    except KeyboardInterrupt:
        print('repeater stopped cleanly')
    except BaseException:
        print('exception in repeater:', file=sys.stderr)
        raise
    finally:
        # scheduler.destroy_node()
        rclpy.shutdown() 

    


if __name__ == '__main__':
    main()
