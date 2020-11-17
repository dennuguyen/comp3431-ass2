#!/usr/bin/env python

import rospy, time
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from ass2_test.msg import RoadInfo
from std_msgs.msg import Bool

MAX_SPEED = 0.26
MAX_TURN = 1.0

msg = None
pub = None

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
        if abs(road_info_data[0] - road_info_data[4]) < 5:
            intersection_flag = 1
        else:
            intersection_flag = 0

        print(intersection_flag)
        # TODO: Intersection handler goes here
        #   Keep logic simple. i.e., move forward for x seconds to clear
        #   the line if no stop sign is detected

        # This is the turn value if we are not at an intersection
        # TODO: Keep turning even when both values are equal
        turn = (road_info_data[0] - road_info_data[4]) / 60.0

    # TODO: Zero out linear movement if another turtlebot is detected
    global msg
    msg.linear.x = drive * MAX_SPEED
    msg.angular.z = turn * MAX_TURN

    global pub
    pub.publish(msg)

# def callback_stop_detection(data):
#     # Five number array
#     stop_data = data.data
# 
#     if (data.data == 1):
#         stop_flag = 1
#     else:
#         stop_flag = 0
# 
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
#     if not moving:
#         #Not sure about the values
#         #Not sure how to pick where to turn
#         #Forward on long gap, left otherwise will manually complete the map
#         if road_info_data[2] > 80:
#             direction = Direction.FORWARD
#         else:
#             direction = Direction.LEFT
# 
#     if moving and not blocked:
#         if direction == Direction.FORWARD:
#             newMsg.linear.x = TURN_LINVEL
#             newMsg.angular.z = 0
#         elif direction == Direction.LEFT:
#             newMsg.linear.x = TURN_LINVEL
#             newMsg.angular.z = -TURN_ANGVEL
#         elif direction == Direction.RIGHT:
#             newMsg.linear.x = TURN_LINVEL
#             newMsg.angular.z = -TURN_ANGVEL
# 
#         cyclesTurned += 1
#         if cyclesTurned >= TURN_TOTAL_CYCLES:
#             moving = False
#             cyclesTurned = 0
# 
#     if newMsg != msg:
#         pub.publish(newMsg)
#         msg = newMsg

def main_ass2():
    global stop_flag
    stop_flag = 0

    global intersection_flag
    intersection_flag = 0

    rospy.init_node("ass2_cpu", anonymous=True)
    r = rospy.Rate(10)
    # rospy.Subscriber('/key_points', Bool, callback_stop_detection)
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
