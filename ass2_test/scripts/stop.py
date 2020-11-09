import rospy

from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from ass2_test.msg import RoadInfo

def callback_road_info(data):
    # Five number array
    road_info_data = data.data
    msg = Twist()
    msg.linear.x = 0

    msg.angular.z = 0
    global cmd_vel
    cmd_vel.publish(msg)

if __name__ == '__main__':
    rospy.Subscriber('/road_info', RoadInfo, callback_road_info)
    global cmd_vel
    cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.init_node('stop', anonymous=True)
    rospy.spin()

