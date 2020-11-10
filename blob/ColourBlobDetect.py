#!/usr/bin/env python2

import cv2
import numpy as np
import rospy

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool

def applyMask(image):
    """
    Detects regions of red and applies an inverted mask
    """
    # Convert from RGV to HSV
    # image = cv2.imread(image, 1)  # Uncomment if opening a file path directly
    img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Apply lower and upper masks for RED
    lower = cv2.inRange(img, (0, 50, 70), (10, 255, 255))
    upper = cv2.inRange(img, (170, 50, 0), (180, 255, 255))
    red = cv2.bitwise_or(lower, upper)

    # Invert image to detect black blobs
    mask = cv2.bitwise_not(red)

    # Merge mask and original image
    cropped = cv2.bitwise_and(img, img, mask=mask)

    # Convert back to RGB
    cropped = cv2.cvtColor(cropped, cv2.COLOR_HSV2BGR)

    # # Display
    # cv2.imshow("Black Mask", cropped)
    # if cv2.waitKey(0) == ord('q'):
    #     cv2.destroyAllWindows()

    return cropped


def detectBlob(image, pub):
    """
    Detect the blob
    """
    # Create params checklist
    params = cv2.SimpleBlobDetector_Params()

    # Filters by threshold (how gray)
    params.minThreshold = 1
    params.maxThreshold = 114

    # Filters by area
    params.filterByArea = True
    params.minArea = 600
    params.maxArea = 100000

    # Filters by colour (0 for black, 1 for white)
    params.filterByColor = True
    params.blobColor = 0

    # Filter by roundness
    params.filterByCircularity = True
    params.minCircularity = 0.05
    params.maxCircularity = 1

    # Filter by convexity
    params.filterByConvexity = True
    params.minConvexity = 0.12
    params.maxConvexity = 1

    # Filter by inertia (elongation)
    params.filterByInertia = False
    params.minInertiaRatio = 0
    params.maxInertiaRatio = 1

    # Create blob detector
    detector = cv2.SimpleBlobDetector_create(params)

    # Open image
    image = np.fromstring(frame.data, np.uint8)
    image = cv2.imdecode(image, cv2.IMREAD_COLOR)  # decompress image

    # Detect colour regions
    image = applyMask(image)

    # Morphologically close the image
    kernel = np.ones((5,5),np.uint8)
    image = cv2.morphologyEx(image, cv2.MORPH_CLOSE, kernel)
    image = cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel)

    # Detect blobs in our frame returned as keypoints
    keypoints = detector.detect(image)

    # Draw the keypoints on an image
    image = cv2.drawKeypoints(image, keypoints, np.array([]), (0, 0, 255),
                             cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    cv2.imshow("Keypoints Image", image)
    if cv2.waitKey(0) == ord('q'):
        cv2.destroyAllWindows()

    # Create the PoseArray
    stopped = Bool()
    stopped.data = False

    # # Extract all coordinates from keypoints and put it into PoseArray
    for keypoint in keypoints:
        # If stop sign is detected, set stopped to true
        if keypoint.size > 50 and keypoint.pt[1] < 200 and keypoint.pt[0] < 550 and keypoint.pt[0] > 200:
            stopped = True
            print("x: ", keypoint.pt[0], "y: ", keypoint.pt[1], "stop: ", stopped)
            break

    # Publish the topics
    pub.publish(stopped)
    

if __name__ == "__main__":
    ros_node_name = "colour_blob_detect"  # ros node name
    pub_topic = "key_points"  # topic to publish to
    sub_topic = "raspicam_node/image/compressed"  # topic to subscribe to

    rospy.init_node(ros_node_name)
    pub = rospy.Publisher(pub_topic, Bool, queue_size=1)
    rospy.Subscriber('/camera/rgb/image_raw/compressed', CompressedImage, detectBlob)
    sub = rospy.Subscriber(sub_topic,
                           CompressedImage,
                           detectBlob, (pub),
                           queue_size=1)
    rospy.spin()

    # image = cv2.imread("lab.png", 1)
    # detectBlob(image)
    # image = cv2.imread("stop.png", 1)
    # detectBlob(image)
    # image = cv2.imread("stoplab.jpg", 1)
    # detectBlob(image)
    # image = cv2.imread("graph.png", 1)
    # detectBlob(image)
    # image = cv2.imread("roses.jpg", 1)
    # detectBlob(image)
