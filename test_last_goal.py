#!/usr/bin/env python

import rospy
from std_msgs.msg import String


def callback(data):

        rospy.loginfo("Here's what was subscribed: %s", data.data)

        global_name = rospy.get_param("/person_follower/tracker")

        if global_name == "Tracking Failure":
                pub = rospy.Publisher('/person_position', String, queue_size=10)
                pub.publish(String("person_lost"))

        print('Data from received')


def main():

        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("/person_position", String, callback)
        rospy.spin()

if __name__ == '__main__':

        try:
                main()
        except rospy.ROSInterruptException:
                pass

