import cv2 as cv
import numpy as np

def main():
    # Load the image and resize to the calibrated parameters
    img = cv.imread('data/image4.png')
    img = cv.resize(img, (320, 240))

    # Load the camera matrix and distortion coefficients
    K = np.array([ 248.73233,    0.     ,  163.61402,
                     0.     ,  249.65376,  118.81648,
                     0.     ,    0.     ,    1.     ])
    K = K.reshape((3, 3))
    D = np.array([0.135629, -0.226556, -0.001079, 0.002116, 0.000000])

    # Undistort the image
    dst = cv.undistort(img, K, D)

    # 3D positions of arbitrarily chosen points.
    # X,Y,Z is relative to the image plane,
    # i.e., X->(left,right), Y->(up, down), Z->(distance from camera)
    # Positions are in meters
    pts3d = np.array([[0.100,  0.103, 0.250],
                      [0.100,  0.103, 0.500],
                      [-0.100, 0.103, 0.500],
                      [-0.100, 0.103, 0.250],
                     ])

    # Project the 3D points to their pixel locations
    pts2d = cv.projectPoints(pts3d, np.eye(3), np.zeros(3), K, np.zeros(5))
    pts2d = pts2d[0]

    # Load the mapping from points in pts2d to their locations
    # on the destination image. Analogous to drawing the points on
    # the original image, warping to a birds-eye view, then aligning
    # the points on the warped image to that of the canvas
    ptsdst = np.array([
                       [550, 1000],
                       [550, 825],
                       [450, 825],
                       [450, 1000],
                      ], dtype=np.float32)
    ptsdst = ptsdst / 5 * 2
    pts2d = np.squeeze(pts2d).astype(np.float32)

    # Creates the perspective tranformation matrix and warps the image
    # Ratio is about 200cm : 40px
    M = cv.getPerspectiveTransform(pts2d, ptsdst)
    dst = cv.warpPerspective(dst, M, (400, 400))

    # Attempts to sharpen the image
    #for _ in range(5):
    #    tmp = cv.GaussianBlur(dst, (0, 0), 2.0)
    #    dst = cv.addWeighted(dst, 1.5, tmp, -0.5, 0, dst)

    # Displays the result
    cv.imshow('img', img)
    cv.imshow('dst', dst)
    cv.waitKey(0)
    cv.destroyAllWindows()

if __name__ == '__main__':
    main()

