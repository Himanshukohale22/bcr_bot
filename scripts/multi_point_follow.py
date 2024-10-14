#! /usr/bin/env python3

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration

def main():
    rclpy.init()

    navigator = BasicNavigator()


## starting position and orientation of robot 

## initial pose of robot 

#   pose:
#     position:
#       x: 0.05105732059858015
#       y: 0.025322412398853383
#       z: 0.0
#     orientation:
#       x: 0.0
#       y: 0.0
#       z: -0.007728561266917568
#       w: 0.9999701342243895


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

    #   pose:
    #     position:
    #       x: 1.088379801747891
    #       y: -5.605996773014956
    #       z: 0.0
    #     orientation:
    #       x: 0.0
    #       y: 0.0
    #       z: -0.5397361018485153
    #       w: 0.8418342713155417

    # Go to our demos first goal pose
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 1.0883
    goal_pose.pose.position.y = -5.60599
    goal_pose.pose.orientation.w = 0.84183
    goal_pose.pose.orientation.z = -0.53936


    ## insert a condition for wait till goal is reached 



    # sanity check a valid path exists
    # path = navigator.getPath(initial_pose, goal_pose)

    #       pose:
    #     position:
    #       x: 1.6543943881988525
    #       y: -7.895281791687012
    #       z: 0.2
    #     orientation:
    #       x: 0.0
    #       y: 0.0
    #       z: -0.7161132324937167
    #       w: 0.6979841246385193
    #   scale:


    goal_pose1 = PoseStamped()
    goal_pose1.header.frame_id = 'map'
    goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose1.pose.position.x = 0.0
    goal_pose1.pose.position.y = 0.0 
    goal_pose1.pose.orientation.z = 0.0 
    goal_pose1.pose.orientation.w = 0.0 


    ## insert a condition for wait till goal is reached 

    #   pose:
    # position:
    #   x: -4.376722812652588
    #   y: -3.2796342372894287
    #   z: 0.0
    # orientation:
    #   x: 0.0
    #   y: 0.0
    #   z: -0.015840582932375228
    #   w: 0.9998745300948327


    goal_pose2 = PoseStamped()
    goal_pose2.header.frame_id = 'map'
    goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose2.pose.position.x = -4.376722
    goal_pose2.pose.position.y = -3.279634
    goal_pose2.pose.orientation.z = -0.01584
    goal_pose2.pose.orientation.w = 0.9998


    ## insert a condition for wait till goal is reached 


#   pose:
#     position:
#       x: -4.376722812652588
#       y: -3.2796342372894287
#       z: 0.0
#     orientation:
#       x: 0.0
#       y: 0.0
#       z: -0.015840582932375228
#       w: 0.9998745300948327



    goal_pose3 = PoseStamped()
    goal_pose3.header.frame_id = 'map'
    goal_pose3.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose3.pose.position.x = -4.37672
    goal_pose3.pose.position.y = -3.279634
    goal_pose3.pose.orientation.z = -0.01584
    goal_pose3.pose.orientation.w = 0.9998


    ## insert a condition for wait till goal is reached 

    navigator.goToPose(goal_pose)
    navigator.goToPose(goal_pose1)
    navigator.goToPose(goal_pose2)
    navigator.goToPose(goal_pose3)

    i = 0
    while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(
                'Estimated time of arrival: '
                + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                    / 1e9
                )
                + ' seconds.'
            )

            # Some navigation timeout to demo cancellation
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                navigator.cancelTask()

            # Some navigation request change to demo preemption
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
                goal_pose.pose.position.x = 0.0
                goal_pose.pose.position.y = 0.0
                navigator.goToPose(goal_pose)

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()



# update in this code to do 
# 1. to insert the feeback 
# 2. to set the condition that if goal is reached then only go to other goal 
