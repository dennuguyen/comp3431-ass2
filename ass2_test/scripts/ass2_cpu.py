import rospy

from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from ass2_test.msg import RoadInfo

def callback_road_info(data):
    # Five number array
    road_info_data = data.data
    msg = Twist()
    if road_info_data[2] < 20:
        msg.linear.x = 0
    elif road_info_data[2] < 80:
        msg.linear.x = 0.16 * road_info_data[2] / 190
    else:
        msg.linear.x = 0.16

    
    if road_info_data[1] - road_info_data[3] < 10:
        msg.angular.z = -0.1
    elif road_info_data[1] - road_info_data[3] > 10:
        msg.angular.z = 0.1
    else:
        msg.angular.z = 0
    
    global cmd_vel
    cmd_vel.publish(msg)

if __name__ == '__main__':
    rospy.Subscriber('/road_info', RoadInfo, callback_road_info)
    global cmd_vel
    cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.init_node('ass2_cpu', anonymous=True)
    rospy.spin()

