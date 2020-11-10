#!/usr/bin/env python2

import cv2
import numpy as np
import rospy

from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray


def detectRed(image):
    """
    Detects regions of red and applies an inverted mask
    """
    # Convert from RGV to HSV
    # img = cv2.imread("redstop.jpg", 1)  # Uncomment if opening a file path directly
    img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Apply lower and upper masks for RED
    lower = cv2.inRange(img, (0, 50, 50), (10, 255, 255))     # lower mask (0-5)
    upper = cv2.inRange(img, (170, 50, 20), (180, 255, 255)) # upper mask (175-180)

    # Combine masks (subjects become white blobs on black background)
    mask = cv2.bitwise_or(lower, upper)

    # Invert image to detect black blobs
    # mask = cv2.bitwise_not(mask)

    # Merge mask and original image
    cropped = cv2.bitwise_and(img, img, mask=mask)

    # Convert back to RGB
    cropped = cv2.cvtColor(cropped, cv2.COLOR_HSV2BGR)

    # Display
    # cv2.imshow("Red Mask", cropped)
    # cv2.waitKey(0)

    return mask


def detectBlob(frame):
    """
    Detect the blob
    """
    # Create params checklist
    params = cv2.SimpleBlobDetector_Params()
    params.minThreshold = 1
    params.maxThreshold = 200

    # Filters by area
    params.filterByArea = False
    params.minArea = 10
    # params.maxArea = 2000

    # Filters by colour
    params.filterByColor = True
    params.blobColor = 255

    # Filter by roundness
    params.filterByCircularity = False
    params.minCircularity = 0.1

    # Filter by convexity
    params.filterByConvexity = False
    params.minConvexity - 0.87

    # Filter by inertia
    params.filterByInertia = False
    params.minInertiaRatio = 0.1

    # Create blob detector
    detector = cv2.SimpleBlobDetector_create(params)

    # Open image
    img = np.fromstring(frame.data, np.uint8)
    img = cv2.imdecode(img, cv2.IMREAD_COLOR)   # decompress image

    # Detect red regions
    mask = detectRed(img)

    # morphologically close the image
    kernel = np.ones((5,5),np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # Detect blobs in our frame returned as keypoints
    keypoints = detector.detect(mask)

    # Draw the keypoints on an image
    # mask = cv2.drawKeypoints(mask, keypoints, np.array([]), (0, 0, 255),
    #                          cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    # cv2.imshow("stop sign", mask)
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
        # pose.position.z = keypoint.size
	pose.position.z = 0        
	poses.poses.append(pose)
	
	# if stop sign is detected, pose.position.z is set to 1
	if keypoint.size > 50 and keypoint.pt[1] < 200 and keypoint.pt[0] < 550 and keypoint.pt[0] > 200:
	    pose.position.z = 1
        print("x: ", pose.position.x, "y: ", pose.position.y, "stop: ",
              pose.position.z)

    # Publish the topics
    pub.publish(poses)
    

if __name__ == "__main__":
    ros_node_name = "colour_blob_detect"  # ros node name
    pub_topic = "key_points"  # topic to publish to
    sub_topic = "raspicam_node/image/compressed"  # topic to subscribe to

    rospy.init_node(ros_node_name)
    pub = rospy.Publisher(pub_topic, PoseArray, queue_size=1)
    rospy.Subscriber('/camera/rgb/image_raw/compressed', CompressedImage, detectBlob)
    sub = rospy.Subscriber(sub_topic,
                           CompressedImage,
                           detectBlob, (pub),
                           queue_size=1)
    rospy.spin()
    # image = cv2.imread("redstop.jpg", cv2.IMREAD_COLOR)
    # detectBlob(image, pub)