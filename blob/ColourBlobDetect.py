#!/usr/bin/env python3

import cv2
import numpy as np
import rospy

from sensor_msgs.msg import CompressedImage
from ass2.msg import Blob


def openCamera(self, parameter_list):
    """
    docstring
    """
    try:
        imgStream = cv2.VideoCapture(-1)
        if not (imgStream.isOpened()):
            print("Can't open camera")
        while cv2.waitKey(1) & 0xFF == ord('q'):
            break
        imgStream.release()
        cv2.destroyAllWindows()
    except:
        pass


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
    params.minArea = 200

    # Filters by colour
    params.filterByColor = True
    params.blobColor = 0

    # Filter by roundness
    params.filterByCircularity = False
    params.minCircularity = 0.1

    # Filter by convexity
    params.filterByConvexity = False
    params.minConvexity - 0.87

    # Filter by inertia
    params.filterByInertia = True
    params.minInertiaRatio = 0.1

    # Create blob detector
    detector = cv2.SimpleBlobDetector_create(params)

    # Detect blobs in our frame returned as keypoints
    keypoints = detector.detect(frame)

    # Draw the keypoints on an image
    # image = cv2.drawKeypoints(frame, keypoints, np.array([]), (0, 0, 255),
    #                           cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    # cv2.imshow("Frame", image)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    # Extract all coordinates from keypoints and put it into PoseArray
    for keypoint in keypoints:
        pose = Pose()
        pose.position.x = keypoint.pt[0]
        pose.position.y = keypoint.pt[1]
        pose.position.z = keypoint.size
        poses.poses.append(pose)
        # print("x: ", pose.position.x, "\ty: ", pose.position.y, "\tsize: ", pose.position.z)

    # Publish the PoseArray
    pub.publish(blobs)


if __name__ == "__main__":
    ros_node_name = "colour_blob_detect"  # ros node name
    pub_topic = "key_points"  # topic to publish to
    sub_topic = "raspicam_node/image/compressed"  # topic to subscribe to

    rospy.init_node(ros_node_name)
    pub = rospy.Publisher(pub_topic, Blob)
    sub = rospy.Subscriber(sub_topic,
                           CompressedImage,
                           detectBlob,
                           queue_size=1)
    detectBlob(openImage("turtlebot.jpg"), pub)
