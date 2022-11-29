import cv2
import numpy as np

def detectarSemaforo(_image):
        
    img = _image  # Cargamos la imagen
    
    lowerRed = np.array([0, 0, 200])
    upperRed = np.array([0, 0, 255])
    lowerYellow = np.array([0, 200, 200])
    upperYellow = np.array([40, 255, 255])
    lowerGreen = np.array([0, 200, 0])
    upperGreen = np.array([40, 255, 40])
    
    mask_r = cv2.inRange(img, lowerRed, upperRed)
    mask_y = cv2.inRange(img, lowerYellow, upperYellow)
    mask_g = cv2.inRange(img, lowerGreen, upperGreen)
    
    _, maskR = cv2.threshold(mask_r, 200, 255, cv2.THRESH_BINARY_INV)
    _, maskY = cv2.threshold(mask_y, 200, 255, cv2.THRESH_BINARY_INV)
    _, maskG = cv2.threshold(mask_g, 200, 255, cv2.THRESH_BINARY_INV)
    params = cv2.SimpleBlobDetector_Params()

    params.filterByCircularity = True
    params.minCircularity = 0.8

    detector = cv2.SimpleBlobDetector_create(params)

    red = detector.detect(maskR)
    yellow = detector.detect(maskY)
    green = detector.detect(maskG)

    blank = np.zeros((1, 1))
    
    if len(red)>=1:
        print("red")
        redblobs = cv2.drawKeypoints(img, red, blank, (0, 0, 255),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        # cv2.imshow("Circular Blobs", redblobs)
        # cv2.waitKey(1)
        return False

    elif len(yellow)>=1:
        print("yellow")
        yellowblobs = cv2.drawKeypoints(img, yellow, blank, (0, 0, 255),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        # cv2.imshow("Circular Blobs", yellowblobs)
        # cv2.waitKey(1)
        return False

    elif len(green)>=1:
        print("Start")
        greenblobs = cv2.drawKeypoints(img, green, blank, (0, 0, 255),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        # cv2.imshow("Circular Blobs", blank)
        # cv2.waitKey(3)
        # cv2.destroyAllWindows()
        return True

    # cv2.imshow("Original Image",img)
    # #cv2.imshow("Circular Blobs Only", blobs)
    # cv2.waitKey(3)
        