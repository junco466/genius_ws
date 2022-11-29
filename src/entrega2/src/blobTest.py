import cv2
import numpy as np

def circleBlob(im):

    img = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)

    lowerRed = np.array([0, 50, 20])
    upperRed = np.array([20, 255, 255])

    lowerRed2 = np.array([175, 100, 20])
    upperRed2 = np.array([179, 255, 255])

    lowerYellow = np.array([24, 50, 20])
    upperYellow = np.array([30, 255, 255])

    lowerBlue = np.array([90, 50, 20])
    upperBlue = np.array([120, 255, 255])

    maskRed1 = cv2.inRange(img, lowerRed, upperRed)
    maskRed2 = cv2.inRange(img, lowerRed2, upperRed2)
    mask_y = cv2.inRange(img, lowerYellow, upperYellow)
    mask_b = cv2.inRange(img, lowerBlue, upperBlue)
    mask_r = cv2.add(maskRed1, maskRed2)    

    _, maskR = cv2.threshold(mask_r, 200, 255, cv2.THRESH_BINARY_INV)
    _, maskY = cv2.threshold(mask_y, 200, 255, cv2.THRESH_BINARY_INV)
    _, maskB = cv2.threshold(mask_b, 200, 255, cv2.THRESH_BINARY_INV)
    params = cv2.SimpleBlobDetector_Params()

    params.filterByCircularity = True
    params.minCircularity = 0.5

    params.filterByConvexity = True
    params.minConvexity = 0.6

    params.filterByInertia = True
    params.minInertiaRatio = 0.5

    detector = cv2.SimpleBlobDetector_create(params)

    red = detector.detect(maskR)
    yellow = detector.detect(maskY)
    blue = detector.detect(maskB)

    blank = np.zeros((1, 1))

    if len(red)>=1:
        print("red")
        redblobs = cv2.drawKeypoints(img, red, blank, (0, 0, 255),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        cv2.imshow("Circular Blobs", redblobs)
        cv2.waitKey(1)

    elif len(yellow)>=1:
        print("yellow")
        yellowblobs = cv2.drawKeypoints(img, yellow, blank, (0, 0, 255),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        # cv2.imshow("Circular Blobs", yellowblobs)
        # cv2.waitKey(1)

    elif len(blue)>=1:
        print("Start")
        greenblobs = cv2.drawKeypoints(img, blue, blank, (0, 0, 255),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        # cv2.imshow("Circular Blobs", blank)
        # cv2.waitKey(3)
        # cv2.destroyAllWindows()