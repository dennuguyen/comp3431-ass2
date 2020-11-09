import rospy

from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from ass2_test.msg import RoadInfo
from enum import Enum

class Direction(Enum):
    LEFT = 1
    FORWARD = 2
    RIGHT = 3

TURN_ANGVEL = 0.5
TURN_LINVEL = 0.1
TURN_TOTAL_CYCLES = 90

direction = Direction.FORWARD
moving = False
blocked = False
cyclesTurned = 0
msg = Twist()

def callback_road_info(data):
    # Five number array
    road_info_data = data.data
    global msg
    global intersection_vel
    global direction
    global moving
    global blocked
    global cyclesTurned

    newMsg = Twist()
    if not moving and not blocked:
        #Not sure about the values
        #Not sure how to pick where to turn
        #Forward on long gap, left otherwise will manually complete the map
        if road_info_data[2] > 80:
            direction = Direction.FORWARD
        else:
            direction = Direction.LEFT

    if moving:
        if direction = Direction.FORWARD:
            newMsg.linear.x = TURN_LINVEL
            newMsg.angular.z = 0
        elif direction = Direction.LEFT:
            newMsg.linear.x = TURN_LINVEL
            newMsg.angular.z = -TURN_ANGVEL
        elif direction = Direction.RIGHT:
            newMsg.linear.x = TURN_LINVEL
            newMsg.angular.z = -TURN_ANGVEL

        cyclesTurned += 1
        if cyclesTurned >= TURN_TOTAL_CYCLES:
            moving = False
            cyclesTurned = 0

    if newMessage != msg:
        intersection_vel.publish(msg)
        msg = newMessage

def callback_blocked(blocked):
    global blocked = blocked.data

def callback_interesection_detected(data)
    intersection_detected = data.data
    global moving
    global blocked
    global cyclesTurned

    if intersection_detected and not moving:
        moving = True
        cyclesTurned = 0

if __name__ == '__main__':
    rospy.Subscriber('/road_info', RoadInfo, callback_road_info)
    rospy.Subscriber('intersection_blocked', Bool, callback_blocked)
    rospy.Subscriber('intersection_detected', Bool, callback_interesection_detected)

    global intersection_vel
    intersection_vel = rospy.Publisher('intersection_vel', Twist, queue_size=1)

    rospy.init_node('intersection', anonymous=True)
    rospy.spin()
