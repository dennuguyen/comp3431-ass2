#!/usr/bin/env python

import rospy, time
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from ass2_test.msg import RoadInfo
from std_msgs.msg import Bool
from enum import Enum

MAX_SPEED = 0.26
MAX_TURN = 1.0

TURN_ANGVEL = 0.20
TURN_LINVEL = 0.20
TURN_TOTAL_CYCLES = 130
FORWARD_TOTAL_CYCLES = 30

class Direction(Enum):
    LEFT = 1
    FORWARD = 2
    RIGHT = 3

at_intersection = False
cycles_turned = 0
cycles_forward = 0
msg = None
pub = None
moving = False
blocked = False
direction = Direction.FORWARD
direction_picked = False

def callback_road_info(data):
    global moving
    global blocked
    global at_intersection
    global direction
    global cycles_turned 
    global cycles_forward
    global msg
    global direction_picked

    if not blocked:
        road_info_data = data.data

        turn = 0.0
        drive = 0.0

        # Turning must scale positively with higher [0] and negatively
        # with higher [1].
        turn = (road_info_data[0] / 180.0) - (road_info_data[1] / 240.0)

        # Forward movement speed scales with how much road we have left
        drive = max(0, (road_info_data[2] - 10.0) / 240.0)

        # Slow down in cases where we need to make large turns
        drive *= max(0, 1 - abs(turn))

        # Check if we are close to something in front of us
        if road_info_data[2] < 35:

            # If turn value is very low, we are at an intersection
            if abs(road_info_data[0] - road_info_data[4]) < 5:
                at_intersection = True
            else:
                at_intersection = False

            turn = (road_info_data[0] - road_info_data[4]) / 60.0

            print("ATINT: ", at_intersection)

        # If we've just arrived at an intersection, set direction forward
        # and state that we're moving through an intersection
        if at_intersection and cycles_forward == 0:
            print("CROSSING INTERSECTION")	
            direction = Direction.FORWARD
            moving = True

        # If we've already moved forward for the specified number of cycles
        # and haven't picked a direction to turn yet
        if moving and cycles_forward == FORWARD_TOTAL_CYCLES and not direction_picked:
            # ???
            if sum(road_info_data)/len(road_info_data) > 80:
                direction = Direction.FORWARD
                cycles_turned = TURN_TOTAL_CYCLES - 20
            
            # Default to turning left
            else:
                direction = Direction.LEFT
            print("PICKING DIRECTION: " + str(direction))
            direction_picked = True

        # If we're moving through an intersection
        if moving:

            # If we haven't gone forward for the specified number of cycles yet
            if cycles_forward != FORWARD_TOTAL_CYCLES:

                # Move forward
                msg.linear.x = TURN_LINVEL
                msg.angular.z = 0
                cycles_forward += 1
                print("FORWARD " + str(cycles_forward))

            # If we've already gone forward for the specified number of cycles
            else:

                # Set the turning direction accordingly
                if direction == Direction.FORWARD:
                    msg.linear.x = TURN_LINVEL
                    msg.angular.z = 0

                elif direction == Direction.LEFT:
                    msg.linear.x = TURN_LINVEL
                    msg.angular.z = -TURN_ANGVEL

                elif direction == Direction.RIGHT:
                    msg.linear.x = TURN_LINVEL
                    msg.angular.z = -TURN_ANGVEL

                cycles_turned += 1

                # If we've already turned for the specified number of cycles
                if cycles_turned >= TURN_TOTAL_CYCLES:

                    # Reset all intersection handling state variables
                    moving = False
                    direction_picked = False
                    cycles_turned = 0
                    cycles_forward = 0
                    at_intersection = False

        else:
            # This is the turn value if we are not at an intersection
            msg.linear.x = drive * MAX_SPEED
            msg.angular.z = turn * MAX_TURN

    # Stop if we're in a blocked state
    else:
        msg.linear.x = 0
        msg.angular.z = 0

    # Publish the message
    global pub
    pub.publish(msg)


def callback_stop_detection(data):
    global blocked
    blocked = data.data


def main_ass2():
    rospy.init_node("ass2_cpu", anonymous=True)

    # Sleep to allow simulator to load positions
    r = rospy.Rate(10)
    rospy.sleep(5)

    rospy.Subscriber('/key_points', Bool, callback_stop_detection)
    rospy.Subscriber('/road_info', RoadInfo, callback_road_info)

    global pub
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    msg = Twist()

    try:
            main_ass2()
    except rospy.ROSInterruptException:
            rospy.loginfo("node is shut down.")
