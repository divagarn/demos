#!/usr/bin/env python

import rospy
from std_msgs.msg import String


def callback(data):

        # print the actual message in its raw format
        rospy.loginfo("Here's what was subscribed: %s", data.data)
        
        if not (data.data):
        	pub = rospy.Publisher('/person_position', std_msgs.msg.String, queue_size=10)
        	pub.publish(std_msgs.msg.String("person_lost"))


        # otherwise simply print a convenient message on the terminal
        print('Data from received')


def main():

        # initialize a node by the name 'listener'.
        # you may choose to name it however you like,
        # since you don't have to use it ahead
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("/person_position", String, callback)

        # spin() simply keeps python from
        # exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':

        # you could name this function
        try:
                main()
        except rospy.ROSInterruptException:
                pass


#import rospy
#from std_msgs.msg import String

#def param_callback(param_value):
#    if param_value == "tracking_failure":
#        person_lost_pub.publish("Person lost!")

#rospy.init_node('tracking_monitor')

#param_sub = rospy.Subscriber('/your_parameter_topic', String, param_callback)

#person_lost_pub = rospy.Publisher('/person_position', String, queue_size=10)

#rospy.spin()
