"""
Black colour blob detection using OpenCV
"""

import cv2


def openImageStream(self, parameter_list):
    """
    docstring
    """
    pass


def detectBlob(self, parameter_list):
    """
    docstring
    """
    pass


if __name__ == "__main__":
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