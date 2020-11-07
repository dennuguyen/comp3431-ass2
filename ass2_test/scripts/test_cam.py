#!/usr/bin/env python

import cv2 as cv
import numpy as np
import rospy

from ass2_test.msg import RoadInfo
from scipy import stats
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist

'''
NOTE: Camera position relative to:
base_footprint : 0.076 0.000 0.103
map            : x.xxx x.xxx 0.102
'''


road_info = None
cmd_vel = None
img_pub = None


def rpi_callback(data):
    # Read into numpy
    img = np.fromstring(data.data, np.uint8)
    
    # Read into OpenCV
    img = cv.imdecode(img, cv.IMREAD_COLOR)

    # Get all edges, dilate them, then get the inverse to get blobs
    dst = cv.Canny(img, 50, 200, None, 3)
    elem = cv.getStructuringElement(cv.MORPH_ELLIPSE, (14, 14), (7, 7))
    dst = 255 - cv.dilate(dst, elem)

    # Assign an integer to all connected blobs. The blob with the most pixels
    # at the bottom of the image is considered the lane
    _, dst = cv.connectedComponents(dst)
    road_id = stats.mode(dst[479])[0][0]

    dst = (dst == road_id).astype(np.uint8) * 255

    # Calculate the distances as described previously
    data = np.zeros(5, dtype=np.uint32)
    
    # The first pixel from the bottom left going up that is not part of the
    # lane
    tmp = dst[:, 0]    # First column
    data[0] = (1 - (tmp != 0)[::-1]).argmax()

    # Similar to data[0], but for the right side
    tmp = dst[:, -1]   # Last column
    data[4] = (1 - (tmp != 0)[::-1]).argmax()

    # The first pixel from the bottom left going right that is part of the
    # lane
    tmp = dst[-1]
    data[1] = (tmp != 0).argmax()

    # Similar to data[1], but for the right side
    tmp = dst[-1]
    data[3] = (tmp != 0)[::-1].argmax()

    # The length of the lane, checking lane pixels in the middle
    tmp = dst[:, 240]
    data[2] = (1 - (tmp != 0)[::-1]).argmax()

    # Publish the lane mask
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    msg.data = np.array(cv.imencode('.jpg', dst)[1]).tostring()
    
    global img_pub
    img_pub.publish(msg)

    # Publish the image distance info
    msg = RoadInfo()
    msg.header.stamp = rospy.Time.now()
    msg.data = data

    global road_info
    road_info.publish(msg)

    return

    ##################################################################
    msg = Twist()

    # Go full speed if at least 80 pixels of lane length remaining
    if m_u < 80:
        msg.linear.x = 0.16 * m_u / 190
    else:
        msg.linear.x = 0.16

    # Turn if left and right line markers are imbalanced
    if l_u - r_u < 0:
        msg.angular.z = -0.1
    elif l_u - r_u > 0:
        msg.angular.z = 0.1

    global cmd_vel
    cmd_vel.publish(msg)

    
def main():
    # rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, rpi_callback)
    rospy.Subscriber('/camera/rgb/image_raw/compressed', CompressedImage, rpi_callback)

    global road_info
    road_info = rospy.Publisher('/road_info', RoadInfo, queue_size=1)

    global cmd_vel
    cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    global img_pub
    img_pub = rospy.Publisher('/lane/image/compressed', CompressedImage, queue_size=1)

    rospy.init_node('test_cam', anonymous=True)
    
    rospy.spin()
    
    
if __name__ == '__main__':
    main()
