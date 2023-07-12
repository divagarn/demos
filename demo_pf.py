#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion, PoseStamped
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import math
import numpy as np
from tf import TransformListener
from tf.transformations import quaternion_from_euler


class PersonFollower:
    def __init__(self):
        rospy.init_node('person_follower')
        self.tf = TransformListener()
        self.bridge = CvBridge()
        self.depth_image = None
        self.LocalObj = None
        self.frontStopflag = False
        self.frontSlowflag = False
        self.backStopflag = False
        self.leftStopflag = False
        self.rightStopflag = False
        self.retGoalValuesAng = []
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()

    def run(self):
        """
        Starts the person follower node by subscribing to necessary topics and spinning the ROS node.
        """
        rospy.Subscriber("/person_position", String, self.person_detect)
        # rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.image_depth_callback)
        rospy.spin()

    def person_detect(self, data_):
        """
        Callback function for person position detection. Analyzes the depth image and handles person following behavior.
        Args:
            data_ (String): Person position data in the format "depth,angle,depvel,angvel".
        """
        if self.depth_image is None:
            return

        self.frontSlowflagObs = False
        self.frontStopflagObs = False

        if data_.data == "person_lost":
            self.move_to_last_goal()
            return

        depth_, angle_, depvel_, angvel_ = [float(x) for x in data_.data.split(",")]
        obsflag = self.Obstacledetector(self.depth_image, depth_)

        if (not depth_ is None) and (0 < depth_ <= 0.35):
            depth_ = (0.35 - 0.6)
        else:
            depth_ = (depth_ - 0.6)

        if depth_ < 0.2:
            if -3 < angle_ < 3:
                angle_ = 0
        elif 0.2 <= depth_:
            if -2 < angle_ < 2:
                angle_ = 0

        if not data_.data and self.move_base_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            self.move_to_last_goal()

        if obsflag == 20:
            self.frontStopflagObs = True

        self.retGoalValuesAng = [depth_, angle_, depvel_, angvel_]

        if not self.frontStopflag and not self.backStopflag and not self.leftStopflag and not self.rightStopflag:
            rospy.set_param('/person_follower/movement', "0")

        if self.retGoalValuesAng[0] <= -0.15:
            if not self.backStopflag:
                self.send_move_base_velocity(self.retGoalValuesAng[0], 0)

            if (self.leftStopflag and self.retGoalValuesAng[1] > 0) or (self.rightStopflag and self.retGoalValuesAng[1] < 0):
                self.send_move_base_velocity(0, 0)
            else:
                self.send_move_base_velocity(0, math.radians(-1 * self.retGoalValuesAng[1]))

        elif -0.15 < self.retGoalValuesAng[0] <= 0.05:
            self.send_move_base_velocity(0, math.radians(-1.9 * self.retGoalValuesAng[1]))

        elif 0.05 < self.retGoalValuesAng[0]:
            if not (self.frontStopflag or self.frontStopflagObs):
                self.send_move_base_velocity(self.retGoalValuesAng[0], 0)
            else:
                self.send_move_base_velocity(0, 0)

    def image_depth_callback(self, msg):
        """
        Callback function for depth image. Converts the image to a numpy array for further processing.
        Args:
            msg (Image): Depth image message.
        """
        cv2_img1 = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        self.depth_image = np.array(cv2_img1, dtype=np.float32)

    def Obstacledetector(self, ffulldepthframe, depthoftarget):
        """
        Detects obstacles in the depth frame based on the target depth and returns an obstacle flag.
        Args:
            ffulldepthframe (numpy.ndarray): Full depth frame.
            depthoftarget (float): Target depth value.
        Returns:
            int: Obstacle flag (0 if no obstacle, 20 if obstacle detected).
        """
        ffulldepthframe = ffulldepthframe.astype('uint16')
        fulldepthframe = ffulldepthframe.copy()

        obstacleflag = 0
        limit = depthoftarget * 1000
        if depthoftarget == 0.6:
            limit = 1500

        idxOfnonzero_ = np.where(fulldepthframe > 0)[0]

        ratio = idxOfnonzero_.shape[0] / (fulldepthframe.size + 1)

        depframe = fulldepthframe.copy()
        depframe[depframe == 0] = 10000

        boundarray = np.zeros((480, 640))
        boundarray[440:480, 256:384] = min(1500, limit)  # 1
        boundarray[400:440, 256:384] = min(1680, limit)  # 2
        boundarray[360:400, 256:384] = min(1700, limit)  # 3
        boundarray[320:360, 256:384] = min(1600, limit * 0.7)  # 4
        boundarray[0:320, 256:384] = min(1000, limit * 0.7)  # 5
        boundarray[:400, 192:256] = min(1000, limit * 0.7)  # 6
        boundarray[:400, 384:448] = min(1000, limit * 0.7)  # 7
        boundarray[400:, 192:256] = min(800, limit)  # 12
        boundarray[400:, 384:448] = min(800, limit)  # 13
        boundarray[-120:, 128:192] = min(600, limit)  # 8
        boundarray[-120:, 448:512] = min(600, limit)  # 9
        boundarray[:-120, 128:192] = min(700, limit)  # 10
        boundarray[:-120, 448:512] = min(700, limit)  # 11

        diff = depframe - boundarray

        noofpts = np.where(diff < 0)[0].shape[0]

        if noofpts > 300 or ratio < 0.8:
            obstacleflag = 20

        return obstacleflag

    def lidar_callback(self, msg1):
        """
        Callback function for LiDAR scan data. Updates the LocalObj variable with the ranges from the scan.
        Args:
            msg1 (LaserScan): LiDAR scan message.
        """
        self.LocalObj = np.array(msg1.ranges)
        self.LocalObj[np.isinf(self.LocalObj)] = 10
        self.LocalObj[self.LocalObj < 0.24] = 10
        self.LocalObj = self.LocalObj - 0.3

    def send_move_base_velocity(self, linear_vel, angular_vel):
        """
        Sends velocity commands to the move_base action server.
        Args:
            linear_vel (float): Linear velocity.
            angular_vel (float): Angular velocity.
        """
        x, y, current_degree = self.get_current_position()
        theta = math.radians(angular_vel) + math.radians(current_degree)
        x += linear_vel * math.cos(theta)
        y += linear_vel * math.sin(theta)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "base_link"
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        q = Quaternion(*quaternion_from_euler(0, 0, theta))
        goal.target_pose.pose.orientation = q
        self.move_base_client.send_goal(goal)
        print("Current degree: {} degrees".format(current_degree))

    def get_current_position(self):
        """
        Gets the current position of the robot in the map frame.
        Returns:
            tuple: Current x, y position, and current degree (rotation angle).
        """
        (trans, rot) = self.tf.lookupTransform('/map', '/base_link', rospy.Time(0))
        return trans[0], trans[1], math.degrees(rot[2])

    def move_to_last_goal(self):
        """
        Move the robot to the last goal point.
        """
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"

        # Get the last goal point
        last_goal = self.move_base_client.get_result()

        if last_goal is not None:
            # Set the goal position and orientation
            goal.target_pose.pose = last_goal.pose

            # Send the goal to the action server
            self.move_base_client.send_goal(goal)
            self.move_base_client.wait_for_result()

            # Print the result of the goal
            if self.move_base_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("Robot reached the last goal point.")
            else:
                rospy.logerr("Failed to reach the last goal point.")
        else:
            rospy.logwarn("No previous goal point found.")


if __name__ == '__main__':
    follower = PersonFollower()
    follower.run()
