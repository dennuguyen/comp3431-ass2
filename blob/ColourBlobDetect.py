#!/usr/bin/env python3

import cv2
import numpy as np
import rospy

from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray


def detectRed(frame):
    """
    docstring
    """
    img = cv2.imread("ColorChecker.png")
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    ## Gen lower mask (0-5) and upper mask (175-180) of RED
    mask1 = cv2.inRange(img_hsv, (0, 50, 20), (5, 255, 255))
    mask2 = cv2.inRange(img_hsv, (175, 50, 20), (180, 255, 255))

    ## Merge the mask and crop the red regions
    mask = cv2.bitwise_or(mask1, mask2)
    cropped = cv2.bitwise_and(img, img, mask=mask)

    ## Display
    cv2.imshow("mask", mask)
    cv2.imshow("cropped", cropped)
    cv2.waitKey()


def openImage(imagePath):
    """
    Opens the image given its path using cv2
    """
    image = cv2.imread(imagePath, cv2.IMREAD_COLOR)
    # if image == None:
    #     raise FileNotFoundError
    return image


def detectBlob(frame, pub):
    """
    Detect the blob
    """
    # Create params checklist
    params = cv2.SimpleBlobDetector_Params()
    params.minThreshold = 1
    params.maxThreshold = 200

    # Filters by area
    params.filterByArea = True
    params.minArea = 1000
    params.maxArea = 2000

    # Filters by colour
    params.filterByColor = True
    params.blobColor = 255

    # Filter by roundness
    params.filterByCircularity = True
    params.minCircularity = 0.1

    # Filter by convexity
    params.filterByConvexity = False
    params.minConvexity - 0.87

    # Filter by inertia
    params.filterByInertia = False
    params.minInertiaRatio = 0.1

    # Create blob detector
    detector = cv2.SimpleBlobDetector_create(params)

    # Detect blobs in our frame returned as keypoints
    img = np.fromstring(frame.data, np.uint8)
    img = cv2.imdecode(img, cv2.IMREAD_COLOR)
    keypoints = detector.detect(img)

    # Draw the keypoints on an image
    # image = cv2.drawKeypoints(frame, keypoints, np.array([]), (0, 0, 255),
    #                          cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    # cv2.imshow("Frame", image)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    # Create the PoseArray
    poses = PoseArray()
    poses.header.frame_id = "/raspicam_node/image/compressed"
    poses.header.stamp = rospy.Time.now()

    # Extract all coordinates from keypoints and put it into PoseArray
    for keypoint in keypoints:
        pose = Pose()
        pose.position.x = keypoint.pt[0]
        pose.position.y = keypoint.pt[1]
        pose.position.z = keypoint.size
        poses.poses.append(pose)
        print("x: ", pose.position.x, "\ty: ", pose.position.y, "\tsize: ",
              pose.position.z)

    # Publish the PoseArray
    pub.publish(poses)


if __name__ == "__main__":
    ros_node_name = "colour_blob_detect"  # ros node name
    pub_topic = "key_points"  # topic to publish to
    sub_topic = "raspicam_node/image/compressed"  # topic to subscribe to

    rospy.init_node(ros_node_name)
    pub = rospy.Publisher(pub_topic, PoseArray, queue_size=1)
    sub = rospy.Subscriber(sub_topic,
                           CompressedImage,
                           detectBlob, (pub),
                           queue_size=1)
    rospy.spin()
# detectBlob(openImage("turtlebot.jpg"), pub)
