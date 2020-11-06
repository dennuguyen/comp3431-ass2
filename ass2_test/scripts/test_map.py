#!/usr/bin/env python

import cv2 as cv
import numpy as np
import rospy

# from ass2_test.msg import RoadInfo
from scipy import stats
from nav_msgs.msg import OccupancyGrid


def map_callback(data):
    print(data.info)
    print(type(data.data))


def main():
    rospy.Subscriber('/map', OccupancyGrid, map_callback)

    rospy.init_node('test_map', anonymous=True)
    
    rospy.spin()
    
    
if __name__ == '__main__':
    main()
