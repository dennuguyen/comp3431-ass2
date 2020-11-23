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

    # if not blocked:
    if True:
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
            # if abs(road_info_data[0] - road_info_data[4]) < 5:
            #     at_intersection = True
            # else:
            #     at_intersection = False
            turn = (road_info_data[0] - road_info_data[4]) / 60.0
            print("ATINT: ", at_intersection)

        if at_intersection and cycles_forward == 0:
            print("CROSSING INTERSECTION")	
            direction = Direction.FORWARD
            moving = True

        if moving and cycles_forward == FORWARD_TOTAL_CYCLES and not direction_picked:
            #Not sure about the values
            #Not sure how to pick where to turn
            #Forward on long gap, left otherwise will manually complete the map
            if sum(road_info_data)/len(road_info_data) > 80:
                direction = Direction.FORWARD
                cycles_turned = TURN_TOTAL_CYCLES - 20
            else:
                direction = Direction.LEFT
            print("PICKING DIRECTION: " + str(direction))
            direction_picked = True

        if moving:
            # TODO: Intersection handler goes here
            #   Keep logic simple. i.e., move forward for x seconds to clear
            #   the line if no stop sign is detected
            #print("MOVING")
            if cycles_forward != FORWARD_TOTAL_CYCLES:
                msg.linear.x = TURN_LINVEL
                msg.angular.z = 0
                cycles_forward += 1
                print("FORWARD " + str(cycles_forward))
            else:
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
                if cycles_turned >= TURN_TOTAL_CYCLES:
                    moving = False
                    direction_picked = False
                    cycles_turned = 0
                    cycles_forward = 0
                    at_intersection = False
        else:
            #print("Definitely in here!")
            # This is the turn value if we are not at an intersection
            # TODO: Keep turning even when both values are equal
            msg.linear.x = drive * MAX_SPEED
            msg.angular.z = turn * MAX_TURN
    else:
        msg.linear.x = 0
        msg.angular.z = 0

    global pub
    pub.publish(msg)

    #r = rospy.Rate(1)
    #r.sleep()

    #msg.linear.x = 0
    #msg.angular.z = 0
    #pub.publish(msg)

def callback_stop_detection(data):
    # Five number array
    global blocked
    blocked = data.data

# def callback_intersection_vel(data):
#     # Five number array
#     road_info_data = data.data
#     global msg
#     global intersection_vel
#     global direction
#     global moving
#     global blocked
#     global cyclesTurned
#
#     newMsg = Twist()
#

def main_ass2():
    rospy.init_node("ass2_cpu", anonymous=True)
    r = rospy.Rate(10)
    rospy.Subscriber('/key_points', Bool, callback_stop_detection)
    # if (stop_flag == 1):
    #     # We've definitely been told to stop due to something.
    #     msg.linear.x = 0
    #     msg.angular.z = 0
    # else:
    #     # Otherwise, check whether we are at an intersection or not?
    rospy.Subscriber('/road_info', RoadInfo, callback_road_info)
    # if (intersection_flag == 1):
    #     pub_intersection_detected = rospy.Publisher('intersection_detected', Twist, queue_size=1)
    #     pub_intersection_detected.publish(msg)
    #     time.sleep(30)
    #     rospy.Subscriber('/road_info', RoadInfo, callback_road_info)

    global pub
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.spin()

    # global msg
    # while not rospy.is_shutdown():
    #     if msg is not None:
    #         pub.publish(msg)
    #         r.sleep()

if __name__ == '__main__':
    # global msg
    msg = Twist()
    # global r

    try:
            main_ass2()
    except rospy.ROSInterruptException:
            rospy.loginfo("node is shut down.")
