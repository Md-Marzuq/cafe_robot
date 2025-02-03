#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        
        # Action client for Nav2
        self._action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )
        
        # Predefined locations (x, y, theta)
        self.locations = {
            'home': [0.0, 0.0, 0.0],
            'kitchen': [2.0, 0.0, 0.0],
            'table1': [1.0, 2.0, 1.57],
            'table2': [3.0, 3.0, 1.57],
            'table3': [4.0, 1.0, 1.57]
        }

    def go_to_location(self, location_name):
        """Send a navigation goal to Nav2"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._create_pose(self.locations[location_name])
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def _create_pose(self, position):
        """Create PoseStamped message"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]
        pose.pose.orientation.z = position[2]
        return pose

    def goal_response_callback(self, future):
        """Handle goal acceptance/rejection"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted!')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handle final result"""
        result = future.result().result
        self.get_logger().info(f'Navigation finished with status: {result.status}')

def main(args=None):
    rclpy.init(args=args)
    nav_controller = NavigationController()
    
    # Example: Navigate to kitchen
    nav_controller.go_to_location('kitchen')
    
    rclpy.spin(nav_controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()