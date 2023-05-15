#!/usr/bin/env python

# Imports
import sys
import time

import rospy
from geometry_msgs.msg import PolygonStamped
from nav_msgs.msg import OccupancyGrid
import numpy as np
import cv2
from base_scratch import *
import  signal


class SIGINT_handler():
    def __init__(self):
        self.SIGINT = False

    def signal_handler(self, signal, frame):
        print('You pressed Ctrl+C!')
        self.SIGINT = True


handler = SIGINT_handler()
signal.signal(signal.SIGINT, handler.signal_handler)

class MapExplorer:
    def __init__(self):
        self.ROBOT_X = 0
        self.ROBOT_Y = 0
        self.MAP_ORIGIN_POSITION_X = -1
        self.MAP_ORIGIN_POSITION_Y = -1
        self.MAP_RESOLUTION = -1
        self.TEMP_ORIGINAL_POSITION_X = -1
        self.TEMP_ORIGINAL_POSITION_Y = -1
        self.EXPLORATION = '0,90,False,False'
        self.STOP_ROTATION = False
        self.MONITOR = False

    def footprint_callback(self, data):
        # Extract the x and y coordinates from the PolygonStamped message
        rob_x = [p.x for p in data.polygon.points]
        rob_y = [p.y for p in data.polygon.points]
        robot_center = 0.15  # 25 cm
        robot_center_y = 0
        rob_center_x_coordinates = [x + robot_center for x in rob_x]
        rob_center_y_coordinates = [y + robot_center_y for y in rob_y]
        rob_x_mean = sum(rob_center_x_coordinates) / len(rob_center_x_coordinates)
        rob_y_mean = sum(rob_center_y_coordinates) / len(rob_center_y_coordinates)
        # rob_y_mean = sum(rob_y) / len(rob_y)
        if self.MAP_RESOLUTION != -1:
            # self.ROBOT_X = (rob_x_mean - self.MAP_ORIGIN_POSITION_X) / self.MAP_RESOLUTION
            # self.ROBOT_Y = (rob_y_mean - self.MAP_ORIGIN_POSITION_Y) / self.MAP_RESOLUTION
            self.ROBOT_X = int((rob_x[0] - self.MAP_ORIGIN_POSITION_X) / self.MAP_RESOLUTION)
            self.ROBOT_Y = int((rob_y[0] - self.MAP_ORIGIN_POSITION_Y) / self.MAP_RESOLUTION)
            #print((int(self.ROBOT_X)), (int(self.ROBOT_Y)))
            if self.ROBOT_X == self.TEMP_ORIGINAL_POSITION_X and self.ROBOT_Y == self.TEMP_ORIGINAL_POSITION_Y and self.MONITOR:
                self.STOP_ROTATION = True

    # def exploration(self):
    #     controller = RobotMovement()
    #     self.TEMP_ORIGINAL_POSITION_X = self.ROBOT_X
    #     self.TEMP_ORIGINAL_POSITION_Y = self.ROBOT_Y
    #     print('init',self.TEMP_ORIGINAL_POSITION_X,self.TEMP_ORIGINAL_POSITION_Y)
    #     # To do 360 degree rotation.
    #     lc = 0
    #     while True:
    #         try:
    #             controller.main_run(self.EXPLORATION)
    #             print(self.STOP_ROTATION,self.MONITOR)
    #             if self.STOP_ROTATION:
    #                 print("exit")
    #                 self.EXPLORATION = '0,90,False,True'
    #                 controller.main_run(self.EXPLORATION)
    #                 sys.exit()
    #             time.sleep(3)
    #             self.MONITOR = True
    #             if handler.SIGINT:
    #                 break
    #         except KeyboardInterrupt:
    #             sys.exit()
    def explor(self):
        controller = RobotMovement()
        controller.rotate_360()


    def callback(self, msg):
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg.data)
        data_in = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        self.MAP_ORIGIN_POSITION_X = msg.info.origin.position.x
        self.MAP_RESOLUTION = msg.info.resolution
        self.MAP_ORIGIN_POSITION_Y = msg.info.origin.position.y
        data_in = cv2.merge([data_in, data_in, data_in])
        if self.ROBOT_X != 0 and self.ROBOT_Y != 0:
            data_in = cv2.circle(data_in, (int(self.ROBOT_X), int(self.ROBOT_Y)), radius=2, color=(0, 0, 255),
                                 thickness=-1)
        # data_in = cv2.flip(data_in, 1)
        #cv2.imshow('test', data_in)
        #cv2.waitKey(1)

    def run(self):

        rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, self.callback)
        # rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, self.callback)
        rospy.Subscriber('/move_base/global_costmap/footprint', PolygonStamped, self.footprint_callback)
        # self.exploration()
        self.explor()
        rospy.spin()


if __name__ == '__main__':
    # rospy.init_node('map_explorer', anonymous=True)
    map_explorer = MapExplorer()
    map_explorer.run()
