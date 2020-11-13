#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from ass2_test.msg import RoadInfo

MAX_SPEED = 0.26
MAX_TURN = 1.0

def callback_road_info(data):
    # TODO: Perform a NOT color mask for the color of the lane markers
    #   before doing visual processing
    # OPTIONAL TODO: Perform a color mask for the color of the road
    #   before doing visual processing
    # OPTIONAL TODO: Experiment with mapping the lane mask to a birds-
    #   eye-view
    # OPTIONAL TODO: Experiment with publishing the lane mask as a map
    #   viewable in RViz
    road_info_data = data.data

    turn = 0.0
    drive = 0.0

    msg = Twist()

    # Turning must scale positively with higher [0] and negatively
    # with higher [1].
    # TODO: Come up with a way to select a "midpoint" between the two
    #   values that isn't necessarily (0, 0)
    turn = (road_info_data[0] / 180.0) - (road_info_data[1] / 240.0)

    # Forward movement speed scales with how much road we have left
    # TODO: Parametrize these scaling factors
    drive = max(0, (road_info_data[2] - 10.0) / 240.0)

    # Slow down in cases where we need to make large turns
    drive *= max(0, 1 - abs(turn))

    # Check if we are close to something in front of us
    if road_info_data[2] < 35:
        # TODO: Experiment with checking how many pixels are part of the
        #   lane and using that as a metric for how much road there is left
        
        # TODO: If these values are equal, we are probably at an intersection
        # TODO: Intersection handler goes here
        #   Keep logic simple. i.e., move forward for x seconds to clear
        #   the line if no stop sign is detected

        # This is the turn value if we are not at an intersection
        # TODO: Keep turning even when both values are equal
        turn = (road_info_data[0] - road_info_data[4]) / 60.0

    msg.linear.x = drive * MAX_SPEED
    msg.angular.z = turn * MAX_TURN

    # TODO: Zero out linear movement if another turtlebot is detected
    
    global cmd_vel
    cmd_vel.publish(msg)

if __name__ == '__main__':
    rospy.Subscriber('/road_info', RoadInfo, callback_road_info)

    global cmd_vel
    cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    rospy.init_node('ass2_cpu', anonymous=True)

    rospy.sleep(10.0)
    rospy.spin()
