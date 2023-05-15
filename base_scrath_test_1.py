#!/usr/bin/env python

# Imports
import math
import time

import rospy
import actionlib
from tf import TransformListener
from geometry_msgs.msg import Quaternion, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler


# File Path
PATH = '/home/divagar/mbf_tuning/final_src_dir/demo.txt'


class RobotMovement:
    """! The robot movement base class.

    This class is used to get the current position of the robot and processing the automatic goals.
    """

    def __init__(self):
        """! The Sensor base class initializer.
        """

        rospy.init_node('robot_controller', anonymous=False)
        self.tf = TransformListener()
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def get_current_position(self):
        """! Get the current position from the map.

        @return positions of x, y and yaw.
        """

        (trans, rot) = self.tf.lookupTransform('/map', '/map', rospy.Time.now())
        return trans[0], trans[1], rot[2]

    def move(self, distance, degree, wait_for_goall):
        """! Process the distance and degree.

        @param distance the distance unit in meters.
        @param degree the degree unit in degrees.
        """
        x, y, current_degree = self.get_current_position()
        theta = math.radians(degree)
        x += distance * math.cos(theta)
        y += distance * math.sin(theta)
        theta += current_degree
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "base_link"
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        q = Quaternion(*quaternion_from_euler(0, 0, theta))
        goal.target_pose.pose.orientation = q
        self.client.send_goal(goal)
        if wait_for_goall == "True":
            self.client.wait_for_result()

        # print("Current degree: {} degrees".format(current_degree))
    def rotate_360(self):
        # Get the current position and orientation
        x, y, current_degree = self.get_current_position()

        # Set the desired angular distance to 360 degrees
        angular_distance = math.radians(360)

        # Set the goal orientation as the current orientation plus the desired angular distance
        goal_orientation = current_degree + angular_distance

        # Create the MoveBaseGoal message
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        q = Quaternion(*quaternion_from_euler(0, 0, goal_orientation))
        goal.target_pose.pose.orientation = q

        # Send the goal to the action server
        self.client.send_goal(goal)

        # Wait for the goal to complete
        self.client.wait_for_result()

        # Log the result
        result = self.client.get_result()
        rospy.loginfo("Result: {}".format(result))

    def main_run(self, file_path):
        if not ('csv' in file_path):
            try:
                distance, degree, wait_for_goal,is_cancel = map(str, file_path.split(','))
                if is_cancel != 'True':
                    self.move(float(distance), float(degree), wait_for_goal)
                else:
                    print('calling cancell all goals')
                    self.client.cancel_all_goals()
                # rospy.sleep(0.5)
                # time.sleep(0.1)
            except (ValueError, KeyboardInterrupt) as error:
                rospy.logerr("Failed to send the goal: {}".format(str(error)))
        # if file_path == "":
        #
        #     while not rospy.is_shutdown():
        #         try:
        #             input_str = input("Enter distance and degree: ")
        #             distance, degree, wait_for_goal = map(str, input_str.split(','))
        #             self.move(float(distance), float(degree), wait_for_goal)
        #         except (ValueError, KeyboardInterrupt) as error:
        #             rospy.logerr("Failed to send the goal: {}".format(str(error)))
        else:
            with open(file_path, 'r') as f:
                for line in f:
                    try:
                        distance, degree, wait_for_goal = map(str, line.strip().split(','))
                        self.move(float(distance), float(degree), wait_for_goal)
                        rospy.sleep(8)
                    except (ValueError, KeyboardInterrupt) as error:
                        rospy.logerr("Failed to send the goal: {}".format(str(error)))
