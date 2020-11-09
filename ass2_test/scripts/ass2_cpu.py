import rospy

from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from ass2_test.msg import RoadInfo

def callback_road_info(data):
    # Five number array
    road_info_data = data.data


if __name__ == '__main__':
    rospy.Subscriber('/road_info', RoadInfo, callback_road_info)
    rospy.init_node('ass2_cpu', anonymous=True)
    rospy.spin()

