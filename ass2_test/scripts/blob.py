#!/usr/bin/env python2

import cv2
import numpy as np
import rospy

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool


def configureParams(minThreshold=0,
                    maxThreshold=255,
                    filterByArea=True,
                    minArea=0,
                    maxArea=100000,
                    filterByColor=True,
                    blobColour=0,
                    filterByCircularity=True,
                    minCircularity=0,
                    maxCircularity=1,
                    filterByConvexity=True,
                    minConvexity=0,
                    maxConvexity=1,
                    filterByInertia=True,
                    minInertiaRatio=0,
                    maxInertiaRatio=1):
    """
    All parameters are true by default and detects black blobs by default.
    """
    # Create params checklist
    params = cv2.SimpleBlobDetector_Params()

    # Filters by threshold (how gray)
    params.minThreshold = minThreshold
    params.maxThreshold = maxThreshold

    # Filters by area
    params.filterByArea = filterByArea
    params.minArea = minArea
    params.maxArea = maxArea

    # Filters by colour (0 for black, 1 for white)
    params.filterByColor = filterByColor
    params.blobColor = blobColour

    # Filter by roundness
    params.filterByCircularity = filterByCircularity
    params.minCircularity = minCircularity
    params.maxCircularity = maxCircularity

    # Filter by convexity
    params.filterByConvexity = filterByConvexity
    params.minConvexity = minConvexity
    params.maxConvexity = maxConvexity

    # Filter by inertia (elongation)
    params.filterByInertia = filterByInertia
    params.minInertiaRatio = minInertiaRatio
    params.maxInertiaRatio = maxInertiaRatio

    return params


def detectStopSign(image):
    """
    Returns a bool if stop signs represented as blobs of red is detected.
    """
    # Convert from RGV to HSV
    image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Apply lower and upper masks for RED
    lower = cv2.inRange(image, (0, 50, 70), (10, 255, 255))
    upper = cv2.inRange(image, (170, 50, 0), (180, 255, 255))
    red = cv2.bitwise_or(lower, upper)

    # Invert image to detect black blobs
    mask = cv2.bitwise_not(red)

    # Morphologically close the mask
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # Configure blob detection parameters
    params = configureParams(minArea=600,
                             minCircularity=0.05,
                             maxCircularity=1,
                             minConvexity=0.12,
                             maxConvexity=1,
                             filterByInertia=False)

    # Create blob detector
    detector = cv2.SimpleBlobDetector_create(params)

    # Detect blobs in our image returned as keypoints
    keypoints = detector.detect(mask)

    # Draw the keypoints on an image
    # mask = cv2.drawKeypoints(mask, keypoints, np.array([]), (0, 0, 255),
    #                          cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    # cv2.imshow("Stop Sign Blobbing", mask)
    # if cv2.waitKey(0) == ord('q'):
    #     cv2.destroyAllWindows()

    # Check all keypoints for a valid keypoint to return stopped
    stopped = False
    for keypoint in keypoints:
        if keypoint.size > 10 and keypoint.pt[1] < 200 and keypoint.pt[
                0] < 550 and keypoint.pt[0] > 200:
            print("Detected stop sign at (", keypoint.pt[0], ",",
                  keypoint.pt[1], ")")
            stopped = True
            break

    return stopped


def detectTurtlebot(image):
    """
    Returns a bool if a turtlebot represented as a large black blob is detected.
    """
    # Configure blob detection parameters
    params = configureParams(minThreshold=1,
                             maxThreshold=114,
                             minArea=9000,
                             filterByCircularity=False,
                             filterByConvexity=False,
                             minInertiaRatio=0,
                             maxInertiaRatio=1)

    # Create blob detector
    detector = cv2.SimpleBlobDetector_create(params)

    # Detect blobs in our image returned as keypoints
    keypoints = detector.detect(image)

    # Draw the keypoints on an image
    # image = cv2.drawKeypoints(image, keypoints, np.array([]), (0, 0, 255),
    #                           cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    # cv2.imshow("Keypoints Image", image)
    # if cv2.waitKey(0) == ord('q'):
    #     cv2.destroyAllWindows()

    # Check all keypoints for a valid keypoint to return stopped
    stopped = False
    for keypoint in keypoints:
        if keypoint.size > 50 and keypoint.pt[1] < 200 and keypoint.pt[
                0] < 550 and keypoint.pt[0] > 200:
            print("Detected turtlebot at (", keypoint.pt[0], ",",
                  keypoint.pt[1], ")")
            stopped = True
            break

    return stopped


def callback(image, pub):
    """
    Callback function to publish to key_points
    """
    # Open image
    image = np.fromstring(image.data, np.uint8)
    image = cv2.imdecode(image, cv2.IMREAD_COLOR)  # decompress image

    # Detect stop sign or turtlebot
    stopped = Bool()
    stopped.data = detectTurtlebot(image) | detectStopSign(image)

    # Publish the topics
    pub.publish(stopped)


if __name__ == "__main__":
    ros_node_name = "colour_blob_detect"  # ros node name
    pub_topic = "key_points"  # topic to publish to
    sub_topic = "raspicam_node/image/compressed"  # topic to subscribe to

    rospy.init_node(ros_node_name)
    pub = rospy.Publisher(pub_topic, Bool, queue_size=1)
    rospy.Subscriber('/camera/rgb/image_raw/compressed', CompressedImage,
                     callback)
    sub = rospy.Subscriber(sub_topic,
                           CompressedImage,
                           callback, (pub),
                           queue_size=1)
    rospy.spin()

    # image = cv2.imread("../../images/lab.png", 1)
    # detectTurtlebot(image)
    # image = cv2.imread("../../images/stop.png", 1)
    # detectTurtlebot(image)
    # image = cv2.imread("../../images/stoplab.jpg", 1)
    # detectTurtlebot(image)
    # image = cv2.imread("../../images/graph.png", 1)
    # detectTurtlebot(image)
    # image = cv2.imread("../../images/roses.jpg", 1)
    # detectTurtlebot(image)
