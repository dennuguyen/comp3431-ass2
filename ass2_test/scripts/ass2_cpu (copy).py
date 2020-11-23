#!/usr/bin/env python

import rospy, time
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from ass2_test.msg import RoadInfo
from std_msgs.msg import Bool

MAX_SPEED = 0.26
MAX_TURN = 1.0

curr_state = None

# r = rospy.Rate(10)

'''
if normal and slow
    state = at_intersection
if at_intersection and blocked
    state = at_intersection
if at_intersection and not blocked
    state = crossing
if crossing and time > 2 secs
    state = normal
'''

def callback_road_info(data):
    global curr_state 

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
        # if abs(road_info_data[0] - road_info_data[4]) < 5:
        #     at_intersection = True
        # else:
        #     at_intersection = False
        turn = (road_info_data[0] - road_info_data[4]) / 80.0
        # print("ATINT: ", at_intersection)

    # if at_intersection and cycles_forward == 0:
    #     print("CROSSING INTERSECTION")	
    #     direction = Direction.FORWARD
    #     moving = True

    # if moving and cycles_forward == FORWARD_TOTAL_CYCLES and not direction_picked:
    #     #Not sure about the values
    #     #Not sure how to pick where to turn
    #     #Forward on long gap, left otherwise will manually complete the map
    #     if sum(road_info_data)/len(road_info_data) > 80:
    #         direction = Direction.FORWARD
    #         cycles_turned = TURN_TOTAL_CYCLES - 20
    #     else:
    #         direction = Direction.LEFT
    #     print("PICKING DIRECTION: " + str(direction))
    #     direction_picked = True

    # if moving:
    #     #   Keep logic simple. i.e., move forward for x seconds to clear
    #     #   the line if no stop sign is detected
    #     #print("MOVING")
    #     if cycles_forward != FORWARD_TOTAL_CYCLES:
    #         msg.linear.x = TURN_LINVEL
    #         msg.angular.z = 0
    #         cycles_forward += 1
    #         print("FORWARD " + str(cycles_forward))
    #     else:
    #         if direction == Direction.FORWARD:
    #             msg.linear.x = TURN_LINVEL
    #             msg.angular.z = 0
    #         elif direction == Direction.LEFT:
    #             msg.linear.x = TURN_LINVEL
    #             msg.angular.z = -TURN_ANGVEL
    #         elif direction == Direction.RIGHT:
    #             msg.linear.x = TURN_LINVEL
    #             msg.angular.z = -TURN_ANGVEL

    #         cycles_turned += 1
    #         if cycles_turned >= TURN_TOTAL_CYCLES:
    #             moving = False
    #             direction_picked = False
    #             cycles_turned = 0
    #             cycles_forward = 0
    #             at_intersection = False
    # else:
    #     #print("Definitely in here!")
    #     # This is the turn value if we are not at an intersection
    #     # TODO: Keep turning even when both values are equal
        # msg.linear.x = drive * MAX_SPEED
        # msg.angular.z = turn * MAX_TURN
    msg = Twist()
    
    msg.linear.x = drive * MAX_SPEED
    msg.angular.z = 1.5 * turn * MAX_TURN
    # else:
    #     msg.linear.x = 0
    #     msg.angular.z = 0

    global pub
    pub.publish(msg)

    # global r
    # r.sleep()

    msg.linear.x = 0
    msg.angular.z = 0

    pub.publish(msg)

def callback_stop_detection(data):
    # Five number array
    global blocked
    blocked = data.data


def main_ass2():
    rospy.init_node("ass2_cpu", anonymous=True)

    rospy.Subscriber('/key_points', Bool, callback_stop_detection)
    rospy.Subscriber('/road_info', RoadInfo, callback_road_info)

    global pub
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    try:
            main_ass2()
    except rospy.ROSInterruptException:
            rospy.loginfo("node is shut down.")
