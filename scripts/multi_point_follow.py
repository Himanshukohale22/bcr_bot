#! /usr/bin/env python3

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration

def main():
    rclpy.init()

    navigator = BasicNavigator()

    ## starting position and orientation of robot 

    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0510573
    initial_pose.pose.position.y = 0.02532
    initial_pose.pose.orientation.z = -0.0077
    initial_pose.pose.orientation.w = 0.9997
    
    navigator.setInitialPose(initial_pose)
    
    # Activate navigation, if not autostarted. This should be called after setInitialPose()
    # or this will initialize at the origin of the map and update the costmap with bogus readings.
    # If autostart, you should `waitUntilNav2Active()` instead.
    # navigator.lifecycleStartup()

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()

    # If desired, you can change or load the map as well
    # navigator.changeMap('/path/to/map.yaml')

    # You may use the navigator to clear or obtain costmaps
    # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
    # global_costmap = navigator.getGlobalCostmap()
    # local_costmap = navigator.getLocalCostmap()

    ## extracting this pose from ros2 topic echo /waypoints

    ## Defining a list of goals 
    goal_pose = []

    # first goal 
     
    goal_pose0 = PoseStamped()
    goal_pose0.header.frame_id = 'map'
    goal_pose0.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose0.pose.position.x = 1.0883
    goal_pose0.pose.position.y = -5.60599
    goal_pose0.pose.orientation.w = 0.84183
    goal_pose0.pose.orientation.z = -0.53936
    
    goal_pose.append(goal_pose0)

    # 2nd goal

    goal_pose1 = PoseStamped()
    goal_pose1.header.frame_id = 'map'
    goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose1.pose.position.x = 0.0
    goal_pose1.pose.position.y = 0.0 
    goal_pose1.pose.orientation.z = 0.0 
    goal_pose1.pose.orientation.w = 0.0 

    goal_pose.append(goal_pose1)

    # 3rd goal

    goal_pose2 = PoseStamped()
    goal_pose2.header.frame_id = 'map'
    goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose2.pose.position.x = -4.376722
    goal_pose2.pose.position.y = -3.279634
    goal_pose2.pose.orientation.z = -0.01584
    goal_pose2.pose.orientation.w = 0.9998

    goal_pose.append(goal_pose2)

    # 4rth goal

    goal_pose3 = PoseStamped()
    goal_pose3.header.frame_id = 'map'
    goal_pose3.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose3.pose.position.x = -4.37672
    goal_pose3.pose.position.y = -3.279634
    goal_pose3.pose.orientation.z = -0.01584
    goal_pose3.pose.orientation.w = 0.9998

    goal_pose.append(goal_pose3)

    # navigate to each goal 

    navigator.goToPose(goal_pose)

    for goal in goal_pose:
        navigator.goToPose(goal)

        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
        if feedback:
            print(f"Distance to goal: {feedback.distance_remaining:.2f} meters")

    # Check result
    result = navigator.getResult()
    if result == navigator.NavigationResult.SUCCEEDED:
        print("Reached the goal!")
    else:
        print("Failed to reach the goal!")

    print("All goals have been processed.")

    navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()
    