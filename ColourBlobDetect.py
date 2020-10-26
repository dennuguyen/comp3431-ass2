"""
Black colour blob detection using OpenCV
"""

import cv2
import numpy as np


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

def detectBlob(frame):
    """
    Detect the blob
    """

    params = cv2.SimpleBlobDetector_Params()
    params.minThreshold = 10
    params.maxThreshold = 200

    params.filterByArea = False
    params.minArea = 1500

    params.filterByCircularity = True
    params.minCircularity = 0.1

    params.filterByConvexity = False
    params.minConvexity - 0.87

    params.filterByInertia = False
    params.minInertiaRatio = 0.01

    detector = cv2.SimpleBlobDetector_create(params)
    keypoints = detector.detect(frame)
    image = cv2.drawKeypoints(frame,
                              keypoints,
                              np.array([]),
                              (0,0,50),
                              cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    cv2.imshow("Frame", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    image = openImage("turtlebot.jpg")
    detectBlob(image)