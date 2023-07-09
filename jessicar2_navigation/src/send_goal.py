#!/usr/bin/env python

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient
import sys
import select
from geometry_msgs.msg import PoseWithCovarianceStamped

def send_initial_pose(x, y, orientation):
    rospy.init_node('simple_navigation_goals')
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
    rospy.sleep(1)  # Wait for the publisher to initialize

    pose_msg = PoseWithCovarianceStamped()
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.header.frame_id = 'map'
    pose_msg.pose.pose.position.x = x
    pose_msg.pose.pose.position.y = y
    pose_msg.pose.pose.orientation.z = orientation

    pub.publish(pose_msg)
    rospy.loginfo("2D Pose Estimate sent!")
    
def main():
    # Connect to ROS


    # Create a SimpleActionClient for move_base
    client = SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    valid_locations = {
        'a': {
            'name': 'room',
            'position': [0.048842549324035645, -0.013771653175354004],
            'orientation': [0, 0, 0.19176673889160156, 0.9813697387080105]
        },
        'b': {
            'name': 'Bedroom',
            'position': [8.1, 4.3],
            'orientation': [0, 0, 1, 0]
        },
        'c': {
            'name': 'Front Door',
            'position': [10.5, 2.0],
            'orientation': [0, 0, 1, 0]
        }
    }

    while True:
        print("\nWhere do you want the robot to go?")
        for loc_num, loc_info in valid_locations.items():
            print("{} = {}".format(loc_num, loc_info['name']))

        user_choice = raw_input("Enter a goal: ")

        if user_choice not in valid_locations:
            print("Invalid selection. Please try again.")
            continue

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = valid_locations[user_choice]['position'][0]
        goal.target_pose.pose.position.y = valid_locations[user_choice]['position'][1]
        goal.target_pose.pose.orientation.x = valid_locations[user_choice]['orientation'][0]
        goal.target_pose.pose.orientation.y = valid_locations[user_choice]['orientation'][1]
        goal.target_pose.pose.orientation.z = valid_locations[user_choice]['orientation'][2]
        goal.target_pose.pose.orientation.w = valid_locations[user_choice]['orientation'][3]

        print("\nGoal Location: {}".format(valid_locations[user_choice]['name']))

        rospy.loginfo("Sending goal")
        client.send_goal(goal)

        while not rospy.is_shutdown():
            # Check if any input is available
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                # Input received, stop the robot
                client.cancel_goal()
                print("Input received. Robot stopped.")
                break

            if client.get_state() == 3:
                rospy.loginfo("The robot has arrived at the goal location")
                break

            rospy.sleep(0.1)

        choice_to_continue = raw_input("\nWould you like to go to another destination? (Y/N): ")
        if choice_to_continue.lower() != 'y':
            break

if __name__ == '__main__':
    try:
        x = 2.27808523178
        y = 0.161879599094
        orientation =0.000115394592285
        send_initial_pose(x, y, orientation)
        main()
    except rospy.ROSInterruptException:
        pass