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


def solve_intersection(seg1, seg2):
    '''Solves for the intersection of two line segments. A line segment is
    defined as a numpy array of the form [x_start, y_start, x_end, y_end]'''
    pt1 = seg1[0:2]
    pt2 = seg1[2:4]

    pt3 = seg2[0:2]
    pt4 = seg2[2:4]

    q = pt3
    p = pt1
    r = pt2 - pt1
    s = pt4 - pt3

    t = np.cross(q - p, s) / np.cross(r, s)
    
    return p + t * r


'''
TODO:
- Filter out white regions, then perform a logical not with dst
    before performing and ya
'''


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
    tmp = dst[:, 320]
    data[2] = (1 - (tmp != 0)[::-1]).argmax()

    # Compute the Hough Line transform
    edges = cv.Canny(dst, 50, 200, None, 3)
    linesP = cv.HoughLinesP(edges, 1, np.pi / 180, 50, None, 50, 10)
    cdstP = np.copy(dst)

    if linesP is not None:
        # Get the index of downward sloped lines (note 1st quadrant coords are
        # flipped)
        linesP = linesP.astype(np.float32)
        s = (linesP[:,:,3] - linesP[:,:,1]) / (linesP[:,:,2] - linesP[:,:,0])
        ix = (s > 0)
        linesP = linesP[ix]

        # If the line segment begins and ends on the right side, it probably
        # isn't the right lane marker
        ix = (linesP[:, 2] > 320) & (linesP[:, 0] > 320)
        linesP = linesP[ix]

        # Define line segments for the left and right image borders
        segr = np.int32([640, 0, 640, 1])
        segl = np.int32([0, 0, 0, 1])
        
        for i in range(0, len(linesP)):
            seg1 = linesP[i]

            # Solve for where the line segment intersects the left and right
            # edges of the image
            tmpr = solve_intersection(seg1, segr)
            tmpl = solve_intersection(seg1, segl)

            # Mask everything above the line
            pts = np.array([[0, 0], tmpl, tmpr, [640, 0]], dtype=np.int32)
            cv.fillPoly(cdstP, [pts], (0, 0, 0), cv.LINE_AA)

    # cv.imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP)
    # cv.waitKey(1)

    # Publish the image distance info
    msg = RoadInfo()
    msg.header.stamp = rospy.Time.now()
    msg.data = data

    global road_info
    road_info.publish(msg)

    # Publish the lane mask
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    msg.data = np.array(cv.imencode('.jpg', cdstP)[1]).tostring()

    global img_pub
    img_pub.publish(msg)

    return

    
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
