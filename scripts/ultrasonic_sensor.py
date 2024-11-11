#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

class ObjectAvoidance(Node):

    def __init__(self):
        super().__init__('object_avoid')
        
        # Subscriber for the ultrasonic range data
        self.navigate_action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.ultra_dist_sub = self.create_subscription(
            Range, '/ultrasonic', self.ultrasonic_callback, 10)
        

        # Publisher for the velocity command
        self.twist_publisher = self.create_publisher(Twist, '/bcr_bot/cmd_vel', 10)
        
        # Initialize the Twist message for velocity commands
        self.cmd_vel = Twist()
        
        # Set the threshold distance (in meters) for obstacle avoidance
        self.threshold_dist = 1.0  # 1 meter

    def ultrasonic_callback(self, msg):        
        # Get the distance value from the ultrasonic sensor (in meters)
        distance = msg.range
        self.get_logger().info(f'Received distance: {distance:.2f} m')

        # Object avoidance logic based on the distance from the sensor
        if distance < self.threshold_dist:
            self.get_logger().info('Obstacle detected! Stopping the robot.')
            
            # If the obstacle is too close, stop the robot
            self.cmd_vel.linear.x = 0.0  # Stop moving forward
            self.cmd_vel.angular.z = 0.0  # Stop rotating
            
        else:
            self.get_logger().info('No obstacle detected. Moving forward.')
            
            # If no obstacle is close, continue moving forward
            self.cmd_vel.linear.x = 0.0  # Move forward at 0.5 m/s
            self.cmd_vel.angular.z = 0.0  # No rotation
        
        # Publish the velocity command
        self.twist_publisher.publish(self.cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    
    # Create the object avoidance node
    object_avoidance_node = ObjectAvoidance()
    
    # Run the node until it is interrupted
    rclpy.spin(object_avoidance_node)
    
    # Clean up and shutdown
    object_avoidance_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
