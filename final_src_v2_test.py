import math
import rospy
import actionlib
from std_msgs.msg import String
from tf import TransformListener
from geometry_msgs.msg import Quaternion, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

class RobotMovement:
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=False)
        self.tf = TransformListener()
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        rospy.Subscriber("/person_position", String, self.person_detect)

    def person_detect(self, data_):
        """
        Callback function for receiving person detection data.

        Args:
            data_: A String message containing person detection data.
        """
        rospy.loginfo("Started subscribing")
        if data_.data == "person_lost":
            self.move_to_last_goal()
            return

        depth, angle = map(float, data_.data.split(","))
        angle = -angle

        if -5 < angle < 5:
            angle = 0

        if depth >= 0.6:
            depth -= 0.6
        elif 0.3 <= depth < 0.6:
            depth = 0
        elif 0 <= depth < 0.3:
            depth = -0.1

        rospy.loginfo("Depth: %f, Angle: %f", depth, angle)
        self.move(depth, angle)

        if not data_.data and self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            self.move_to_last_goal()

    def move_to_last_goal(self):
        """
        Move the robot to the last goal point.
        """
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        last_goal = self.client.get_result()

        if last_goal is not None:
            goal.target_pose.pose = last_goal.pose
            self.client.send_goal(goal)
            self.client.wait_for_result()

            if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("Robot reached the last goal point.")
            else:
                rospy.logerr("Failed to reach the last goal point.")
        else:
            rospy.logwarn("No previous goal point found.")

    def get_current_position(self):
        """
        Get the current position of the robot.

        Returns:
            The current x, y coordinates and orientation angle of the robot.
        """
        (trans, rot) = self.tf.lookupTransform('/map', '/map', rospy.Time(0))
        return trans[0], trans[1], math.degrees(rot[2])

    def move(self, distance, degree):
        """
        Move the robot by a given distance and degree.

        Args:
            distance: The distance to move the robot in meters.
            degree: The angle to rotate the robot in degrees.
        """
        x, y, current_degree = self.get_current_position()
        theta = math.radians(degree) + math.radians(current_degree)
        x += distance * math.cos(theta)
        y += distance * math.sin(theta)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "base_link"
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        q = Quaternion(*quaternion_from_euler(0, 0, theta))
        goal.target_pose.pose.orientation = q
        self.client.send_goal(goal)
        rospy.loginfo("Current degree: %f degrees", current_degree)

    def main_run(self, file_path):
        """
        Run the main loop to control the robot movement.

        Args:
            file_path: The path to a file containing movement commands in CSV format
                       (distance, degree). Alternatively, a single movement command
                       can be passed as a string (distance, degree).
        """
        if not file_path.endswith('.csv'):
            try:
                distance, degree = map(float, file_path.split(','))
                self.move(distance, degree)
            except (ValueError, KeyboardInterrupt) as error:
                rospy.logerr("Failed to send the goal: %s", str(error))
        else:
            with open(file_path, 'r') as f:
                for line in f:
                    try:
                        distance, degree = map(float, line.strip().split(','))
                        self.move(distance, degree)
                        rospy.sleep(8)
                    except (ValueError, KeyboardInterrupt) as error:
                        rospy.logerr("Failed to send the goal: %s", str(error))

    def close_all(self):
        """
        Clean up and close all components.
        """
        self.client.cancel_all_goals()
        rospy.signal_shutdown("All components closed")


if __name__ == '__main__':
    controller = RobotMovement()
    rospy.on_shutdown(controller.close_all)
    rospy.spin()
