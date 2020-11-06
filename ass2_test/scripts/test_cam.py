#!/usr/bin/env python

import cv2 as cv
import numpy as np
import rospy

# from ass2_test.msg import RoadInfo
from scipy import stats
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist

'''
NOTE: Camera position relative to:
base_footprint : 0.076 0.000 0.103
map            : x.xxx x.xxx 0.102
'''


# road_info = None
cmd_vel = None


def rpi_callback(data):
    # Read into numpy
    img = np.fromstring(data.data, np.uint8)
    
    # Read into OpenCV
    img = cv.imdecode(img, cv.IMREAD_COLOR)

    dst = cv.Canny(img, 50, 200, None, 3)
    elem = cv.getStructuringElement(cv.MORPH_ELLIPSE, (14, 14), (7, 7))
    dst = 255 - cv.dilate(dst, elem)

    _, dst = cv.connectedComponents(dst)
    road_id = stats.mode(dst[479])[0][0]

    dst = (dst == road_id).astype(np.uint8) * 255

    '''
    # Hough line detection strategy
    tmp = cv.Canny(dst, 50, 200, None, 3)
    linesP = cv.HoughLinesP(tmp, 1, np.pi / 180, 50, None, 50, 10)
    cdstP = dst.copy()
    cdstP = cv.cvtColor(cdstP, cv.COLOR_GRAY2BGR)

    if linesP is not None:
        for i in range(0, len(linesP)):
            l = linesP[i][0]
            cv.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv.LINE_AA)

    1. Get all line segments with a slope <= 0
    2. Filter short lines
    3. Intersect with line projected from center of image
    4. Line with lowest intersection point is the dashed line
    5. Mask everything above that line with 0
    '''

    # Display image
    cv.imshow('img', dst)
    cv.waitKey(1)

    tmp1 = dst[:, 0]    # First column
    l_u = (1 - (tmp1 != 0)[::-1]).argmax()
    # The first pixel from the bottom left going up that is not part of the
    # lane

    tmp1 = dst[:, -1]   # Last column
    r_u = (1 - (tmp1 != 0)[::-1]).argmax()
    # Similar to  l_u, but for the right side

    tmp1 = dst[:, 240]
    m_u = (1 - (tmp1 != 0)[::-1]).argmax()
    # The length of the lane, checking lane pixels in the middle

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

    '''
    TODO: Filter dotted lines
    1. ???
    '''

    '''
    TODO: wallFollow image version
    Instead of balancing left and right lanes, use wallFollow logic,
        i.e. base movement off bottom left row and column
    If forward lane length is short and there's no where to turn, go into 
        intersection handler
    Smoothen forward movement function. Preferably sigmoidal
    Draw a map by warping image perspective
    '''

    '''
    TODO: Map generation
    1. Using the camera matrix, project 4 3D points located at positions known to
        be within the image. Store their 2D locations. Do not use distortion
    2. Create a homography matrix based on these

    LOOP
    3. Undistort the input image using the camera's distortion parameters
    4. Apply the homography matrix on the lane mask
    5. Translate the lane mask based on the tf from the camera to the map
    6. Adjust global map, either incrementally, or by running and AND over
        the global map and local map

    ALTERNATIVE
    1. Navigate using wallFollow image
    2. Record traversed areas. Any area previously traversed is a lane
    '''

    '''
    TODO: Intersection handling
    1. Warp the image's perspective to a birds eye view (possibly follow previous
        section's logic)
    2. Filter out the road color instead of just the road immediately in front of
        the robot
    3. Given the observed lane widths, check if you can turn left, right, or go
        forward, i.e. check which lanes are linked to the intersection box
    #. Do not exit the intersection handling state until it can be confirmed we're
        out of the intersection
    #. Include handler for oncoming traffic
    '''

    '''
    TODO: Stop sign handling
    1. If at an intersection and a red blob is detected, do not proceed with
        intersection handling function
    2. Alternatively use SIFT feature matching to find stop signs
    '''

    '''
    TODO: Traffic handling
    1. Already handled by base code
    2. Maybe back up if the black blob gets bigger
    '''

    '''
    TODO: Automatic navigation
    ???
    '''
    
    global cmd_vel
    cmd_vel.publish(msg)

    
def main():
    # rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, rpi_callback)
    rospy.Subscriber('/camera/rgb/image_raw/compressed', CompressedImage, rpi_callback)

    # global road_info
    # road_info = rospy.Publisher('callbackVision', RoadInfo, queue_size=1)

    global cmd_vel
    cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    rospy.init_node('test_cam', anonymous=True)
    
    rospy.spin()
    
    
if __name__ == '__main__':
    main()
