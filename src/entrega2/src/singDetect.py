import rospy
import numpy as np
import cv2
from enum import Enum
from std_msgs.msg import UInt8
from time import sleep

class DetectSign():
    def __init__(self,_follow):

        #--OBJECTS , CONFIGS--
        self.fnPreproc()
        self.TrafficSign = Enum('TrafficSign', 'intersection left right tunnel prec prec2 construction barra barraUp parking')
        self.counter = 1
        self.follow = _follow

        #--FLAGS--
        self.dir_path = None
        self.yaGire = False
        self.Cruce = True
        self.obras = False
        self.consDetected = False
        self.barra = False
        self.tunnel = False
        self.barDetected = False
        self.parking = False
        self.parkingDetected = False

        #--COUNTERS--
        self.left_count = 0
        self.right_count = 0
        self.prec_count = 0
        self.prec_count2 = 0
        self.const_count = 0

    def fnPreproc(self):
        # Initiate SIFT detector
        self.sift = cv2.SIFT_create()
        self.dir_path = '/home/shorlak/genius_ws/src/entrega2/src/signs/'
        self.img_intersection = cv2.imread(self.dir_path + 'intersection.png',0)
        self.img_left  = cv2.imread(self.dir_path + 'left4.png',0)
        self.img_right = cv2.imread(self.dir_path + 'right3.png',0)
        self.img_construction = cv2.imread(self.dir_path + 'construction.png',0)
        self.img_parking = cv2.imread(self.dir_path + 'parking.png',0)
        self.img_stop = cv2.imread(self.dir_path + 'stop.png',0)
        self.img_tunnel = cv2.imread(self.dir_path + 'tunnel.png',0)
        self.img_prec = cv2.imread(self.dir_path + 'prec.png',0)
        self.img_prec2 = cv2.imread(self.dir_path + 'prec2.png',0)
        self.img_barra = cv2.imread(self.dir_path + 'barra.png',0)
        self.img_barraUp = cv2.imread(self.dir_path + 'barraUp.png',0)
        self.img_parking = cv2.imread(self.dir_path + 'parking3.png',0)


        self.kp_intersection, self.des_intersection  = self.sift.detectAndCompute(self.img_intersection, None)
        self.kp_left, self.des_left  = self.sift.detectAndCompute(self.img_left, None)
        self.kp_right, self.des_right = self.sift.detectAndCompute(self.img_right, None)
        self.kp_construction, self.des_construction = self.sift.detectAndCompute(self.img_construction, None)
        self.kp_parking, self.des_parking = self.sift.detectAndCompute(self.img_parking, None)
        self.kp_stop, self.des_stop = self.sift.detectAndCompute(self.img_stop, None)
        self.kp_tunnel, self.des_tunnel = self.sift.detectAndCompute(self.img_tunnel, None)
        self.kp_prec, self.des_prec = self.sift.detectAndCompute(self.img_prec, None)
        self.kp_prec2, self.des_prec2 = self.sift.detectAndCompute(self.img_prec2, None)
        self.kp_barra, self.des_barra = self.sift.detectAndCompute(self.img_barra, None)
        self.kp_barraUp, self.des_barraUp = self.sift.detectAndCompute(self.img_barraUp, None)
        self.kp_parking, self.des_parking = self.sift.detectAndCompute(self.img_parking, None)


        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks = 50)

        self.flann = cv2.FlannBasedMatcher(index_params, search_params)




    def circleTrafficSign(self, image_msg):
        
        # drop the frame to 1/5 (6fps) because of the processing speed. This is up to your computer's operating power.
        if self.counter % 2 != 0:
            self.counter += 1
            return
        else:
            self.counter = 1
            # cv_image_input = self.cvBridge.imgmsg_to_cv2(image_msg, "bgr8")
            cv_image_input = image_msg


        # find the keypoints and descriptors with SIFT
        kp1, des1 = self.sift.detectAndCompute(cv_image_input,None)

        matches_left = self.flann.knnMatch(des1,self.des_left,k=2)
        matches_right = self.flann.knnMatch(des1,self.des_right,k=2)
        matches_prec = self.flann.knnMatch(des1,self.des_prec,k=2)
        matches_prec2 = self.flann.knnMatch(des1,self.des_prec2,k=2)

        image_out_num = 1

        if self.yaGire is not True:

            MIN_MATCH_COUNT = 8 #9n left right tunnel prec')
            MIN_MSE_DECISION = 15000
            good_left = []
            for m,n in matches_left:
                if m.distance < 0.7*n.distance:
                    good_left.append(m)
            if len(good_left)>MIN_MATCH_COUNT:
                src_pts = np.float32([kp1[m.queryIdx].pt for m in good_left ]).reshape(-1,1,2)
                dst_pts = np.float32([self.kp_left[m.trainIdx].pt for m in good_left]).reshape(-1,1,2)

                M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
                matches_left = mask.ravel().tolist()

                mse = self.fnCalcMSE(src_pts, dst_pts)
                print('mse left: ', mse)
                if mse < MIN_MSE_DECISION and self.yaGire is not True:
                    msg_sign = UInt8()
                    msg_sign.data = self.TrafficSign.left.value

                    rospy.loginfo("detect left sign")
                    self.left_count = self.left_count + 1
                    print('conteo left: ', self.left_count)
                    if self.left_count > 2:
                        self.follow.Ladvice = True
                        self.yaGire = True 
                    image_out_num = 2

            else:
                matches_left = None
                rospy.loginfo("nothing detected")

            good_right = []
            for m,n in matches_right:
                if m.distance < 0.7*n.distance:
                    good_right.append(m)
            print(len(good_right))
            if len(good_right)>MIN_MATCH_COUNT:
                src_pts = np.float32([kp1[m.queryIdx].pt for m in good_right ]).reshape(-1,1,2)
                dst_pts = np.float32([self.kp_right[m.trainIdx].pt for m in good_right]).reshape(-1,1,2)

                M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
                matches_right = mask.ravel().tolist()

                mse = self.fnCalcMSE(src_pts, dst_pts)
                print('mse right: ', mse)
                if mse < MIN_MSE_DECISION and self.yaGire is not True:
                    msg_sign = UInt8()
                    msg_sign.data = self.TrafficSign.right.value

                    rospy.loginfo("detect right sign")
                    self.right_count = self.right_count + 1
                    print('conteo right: ', self.right_count)
                   
                    if self.right_count > 2:

                        self.follow.Radvice = True
                        self.yaGire = True 
                        
                    image_out_num = 3

            else:
                matches_right = None
                rospy.loginfo("nothing detected")


        if self.yaGire:

            MIN_MATCH_COUNT = 9 #9
            MIN_MSE_DECISION = 20000
            good_prec = []
            for m,n in matches_prec:
                if m.distance < 0.7*n.distance:
                    good_prec.append(m)
            # print(len(good_prec))
            if len(good_prec)>MIN_MATCH_COUNT:
                src_pts = np.float32([kp1[m.queryIdx].pt for m in good_prec ]).reshape(-1,1,2)
                dst_pts = np.float32([self.kp_prec[m.trainIdx].pt for m in good_prec]).reshape(-1,1,2)

                M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
                matches_prec = mask.ravel().tolist()

                mse = self.fnCalcMSE(src_pts, dst_pts)
                print('mse prec: ', mse)
                if mse < MIN_MSE_DECISION:
                    msg_sign = UInt8()
                    msg_sign.data = self.TrafficSign.prec.value

                    rospy.loginfo("detect prec sign")
                    self.prec_count = self.prec_count + 1
                    print('conteo prec: ', self.prec_count)
                    if self.prec_count > 2:
                        self.follow.Padvice = True
                        self.Cruce = False 
                        self.obras = True

                    image_out_num = 4

            else:
                matches_prec = None
                rospy.loginfo("nothing detected")
            

            good_prec2 = []
            for m,n in matches_prec2:
                if m.distance < 0.7*n.distance:
                    good_prec2.append(m)
            # print(len(good_prec))
            if len(good_prec2)>MIN_MATCH_COUNT:
                src_pts = np.float32([kp1[m.queryIdx].pt for m in good_prec2 ]).reshape(-1,1,2)
                dst_pts = np.float32([self.kp_prec2[m.trainIdx].pt for m in good_prec2]).reshape(-1,1,2)

                M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
                matches_prec2 = mask.ravel().tolist()

                mse = self.fnCalcMSE(src_pts, dst_pts)
                print('mse prec2: ', mse)
                if mse < MIN_MSE_DECISION:
                    msg_sign = UInt8()
                    msg_sign.data = self.TrafficSign.prec2.value

                    rospy.loginfo("detect prec sign")
                    self.prec_count2 = self.prec_count2 + 1
                    print('conteo prec: ', self.prec_count2)
                    if self.prec_count2 > 2:
                        self.follow.Padvice = True
                        self.Cruce = False 
                        self.obras = True

                    image_out_num = 5

            else:
                matches_prec2 = None
                rospy.loginfo("nothing detected")



        if image_out_num == 1:
            cv2.imshow('Input image', cv_image_input)
            cv2.waitKey(1)
            
        elif image_out_num == 2:
            draw_params_left = dict(matchColor = (255,0,0), # draw matches in green color
                        singlePointColor = None,
                        matchesMask = matches_left, # draw only inliers
                        flags = 2)

            final_left = cv2.drawMatches(cv_image_input,kp1,self.img_left,self.kp_left,good_left,None,**draw_params_left)
            cv2.imshow('Final left', final_left)
            cv2.waitKey(1)
            
        elif image_out_num == 3:
            draw_params_right = dict(matchColor = (255,0,0), # draw matches in green color
                            singlePointColor = None,
                            matchesMask = matches_right, # draw only inliers
                            flags = 2)

            final_right = cv2.drawMatches(cv_image_input,kp1,self.img_right,self.kp_right,good_right,None,**draw_params_right)
            cv2.imshow('Final right', final_right)
            cv2.waitKey(1)

        elif image_out_num == 4:
            draw_params_prec = dict(matchColor = (255,0,0), # draw matches in green color
                            singlePointColor = None,
                            matchesMask = matches_prec, # draw only inliers
                            flags = 2)

            final_prec = cv2.drawMatches(cv_image_input,kp1,self.img_prec,self.kp_prec,good_prec,None,**draw_params_prec)
            cv2.imshow('Final prec', final_prec)
            cv2.waitKey(1)

        elif image_out_num == 5:
            draw_params_prec2 = dict(matchColor = (255,0,0), # draw matches in green color
                            singlePointColor = None,
                            matchesMask = matches_prec2, # draw only inliers
                            flags = 2)

            final_prec2 = cv2.drawMatches(cv_image_input,kp1,self.img_prec2,self.kp_prec2,good_prec2,None,**draw_params_prec2)
            cv2.imshow('Final prec2', final_prec2)
            cv2.waitKey(1)




    def detectTunnel(self, image_msg):

        # drop the frame to 1/3 (6fps) because of the processing speed. This is up to your computer's operating power.
        if self.counter % 2 != 0:
            self.counter += 1
            return
        else:
            self.counter = 1
            # cv_image_input = self.cvBridge.imgmsg_to_cv2(image_msg, "bgr8")
            cv_image_input = image_msg

        cv_image_input = image_msg
        MIN_MATCH_COUNT = 9 #9
        MIN_MSE_DECISION = 50000

        # find the keypoints and descriptors with SIFT
        kp1, des1 = self.sift.detectAndCompute(cv_image_input,None)

        matches_tunnel = self.flann.knnMatch(des1,self.des_tunnel,k=2)

        image_out_num = 1

        good_tunnel = []
        for m,n in matches_tunnel:
            if m.distance < 0.7*n.distance:
                good_tunnel.append(m)
        if len(good_tunnel)>MIN_MATCH_COUNT:
            src_pts = np.float32([kp1[m.queryIdx].pt for m in good_tunnel ]).reshape(-1,1,2)
            dst_pts = np.float32([self.kp_tunnel[m.trainIdx].pt for m in good_tunnel]).reshape(-1,1,2)

            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
            matches_tunnel = mask.ravel().tolist()

            mse = self.fnCalcMSE(src_pts, dst_pts)
            if mse < MIN_MSE_DECISION:
                msg_sign = UInt8()
                msg_sign.data = self.TrafficSign.tunnel.value

                rospy.loginfo("detect tunnel sign")
                self.follow.victory = True
                image_out_num = 2
                # self.follow.linearSpeed = 0.03

        else:
            rospy.loginfo("tunnel not detected")


        if image_out_num == 1:
            cv2.imshow('Input image', cv_image_input)
            cv2.waitKey(1)
            
        elif image_out_num == 2:
            draw_params_tunnel = dict(matchColor = (255,0,0), # draw matches in green color
                        singlePointColor = None,
                        matchesMask = matches_tunnel, # draw only inliers
                        flags = 2)

            final_tunnel = cv2.drawMatches(cv_image_input,kp1,self.img_tunnel,self.kp_tunnel,good_tunnel,None,**draw_params_tunnel)
            cv2.imshow('Final tunnel', final_tunnel)
            cv2.waitKey(1)
        # self.Dbarra = True




    def detectConstruction(self,image_msg):

        # drop the frame to 1/3 (6fps) because of the processing speed. This is up to your computer's operating power.
        if self.counter % 3 != 0:
            self.counter += 1
            return
        else:
            self.counter = 1
            # cv_image_input = self.cvBridge.imgmsg_to_cv2(image_msg, "bgr8")
            cv_image_input = image_msg

        cv_image_input = image_msg
        MIN_MATCH_COUNT = 9 #9
        MIN_MSE_DECISION = 50000

        # find the keypoints and descriptors with SIFT
        kp1, des1 = self.sift.detectAndCompute(cv_image_input,None)

        matches_construction = self.flann.knnMatch(des1,self.des_construction,k=2)

        image_out_num = 1

        good_construction = []
        for m,n in matches_construction:
            if m.distance < 0.7*n.distance:
                good_construction.append(m)
        if len(good_construction)>MIN_MATCH_COUNT:
            src_pts = np.float32([kp1[m.queryIdx].pt for m in good_construction ]).reshape(-1,1,2)
            dst_pts = np.float32([self.kp_construction[m.trainIdx].pt for m in good_construction]).reshape(-1,1,2)

            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
            matches_construction = mask.ravel().tolist()

            mse = self.fnCalcMSE(src_pts, dst_pts)
            if mse < MIN_MSE_DECISION:
                msg_sign = UInt8()
                msg_sign.data = self.TrafficSign.construction.value

                rospy.loginfo("detect construction sign")
                self.const_count = self.const_count + 1
                print('conteo const: ', self.const_count)
                if self.const_count > 10:
                    print('prendi scan') 
                    self.consDetected = True
                image_out_num = 2
                # self.follow.linearSpeed = 0.03

        else:
            rospy.loginfo("construction not detected")


        if image_out_num == 1:
            cv2.imshow('Input image', cv_image_input)
            cv2.waitKey(1)
            
        elif image_out_num == 2:
            draw_params_cons = dict(matchColor = (255,0,0), # draw matches in green color
                        singlePointColor = None,
                        matchesMask = matches_construction, # draw only inliers
                        flags = 2)

            final_cons = cv2.drawMatches(cv_image_input,kp1,self.img_construction,self.kp_construction,good_construction,None,**draw_params_cons)
            cv2.imshow('Final construction', final_cons)
            cv2.waitKey(1)


    def detectParking(self,image_msg):
        
        # print('detectando parking')

        # drop the frame to 1/3 (6fps) because of the processing speed. This is up to your computer's operating power.
        if self.counter % 2 != 0:
            self.counter += 1
            return
        else:
            self.counter = 1
            # cv_image_input = self.cvBridge.imgmsg_to_cv2(image_msg, "bgr8")
            cv_image_input = image_msg

        cv_image_input = image_msg
        MIN_MATCH_COUNT = 9 #9
        MIN_MSE_DECISION = 50000

        # find the keypoints and descriptors with SIFT
        kp1, des1 = self.sift.detectAndCompute(cv_image_input,None)

        matches_parking = self.flann.knnMatch(des1,self.des_parking,k=2)

        image_out_num = 1

        good_parking = []
        for m,n in matches_parking:
            if m.distance < 0.7*n.distance:
                good_parking.append(m)
        if len(good_parking)>MIN_MATCH_COUNT:
            src_pts = np.float32([kp1[m.queryIdx].pt for m in good_parking ]).reshape(-1,1,2)
            dst_pts = np.float32([self.kp_parking[m.trainIdx].pt for m in good_parking]).reshape(-1,1,2)

            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
            matches_parking = mask.ravel().tolist()

            mse = self.fnCalcMSE(src_pts, dst_pts)
            if mse < MIN_MSE_DECISION:
                msg_sign = UInt8()
                msg_sign.data = self.TrafficSign.parking.value

                rospy.loginfo("detect parking sign")
                #---->comando de accion
                self.follow.park = True
                self.parkingDetected = True
                image_out_num = 2
                # self.follow.linearSpeed = 0.03

        else:
            rospy.loginfo("parking not detected")


        if image_out_num == 1:
            cv2.imshow('Input image', cv_image_input)
            cv2.waitKey(1)
            
        elif image_out_num == 2:
            draw_params_parking = dict(matchColor = (255,0,0), # draw matches in green color
                        singlePointColor = None,
                        matchesMask = matches_parking, # draw only inliers
                        flags = 2)

            final_parking = cv2.drawMatches(cv_image_input,kp1,self.img_parking,self.kp_parking,good_parking,None,**draw_params_parking)
            cv2.imshow('Final parking', final_parking)
            cv2.waitKey(1)


    def detectBar(self, img):

        print('detectando barra')
        # Read the template
        path = '/home/shorlak/genius_ws/src/entrega2/src/signs/barra.jpeg'
        template = cv2.imread(path)
        
        # Store width and height of template in w and h
        h,w = template.shape[:2]
        
        # Perform match operations.
        res = cv2.matchTemplate(img, template, cv2.TM_CCOEFF_NORMED)
        best_res=np.max(res)
        # Specify a threshold
        threshold = 0.7
        if(best_res>threshold):
            # Store the coordinates of matched area in a numpy array
            loc = np.where(res >= threshold)
            # Draw a rectangle around the matched region.
            print('detecte barra')
            self.follow.bar = True
            self.barDetected = True
            for pt in zip(*loc[::-1]):
                cv2.rectangle(img, pt, (pt[0] + w, pt[1] + h), (0, 255, 255), 2)
        else:
            if self.barDetected:
                self.tunnel = True
                self.barra = False
            self.follow.bar = False

        # Show the final image with the matched area.
        cv2.imshow('Detected', img)
        cv2.waitKey(1)




    def detectBarra(self, image_msg):

        # drop the frame to 1/3 (6fps) because of the processing speed. This is up to your computer's operating power.
        if self.counter % 2 != 0:
            self.counter += 1
            return
        else:
            self.counter = 1
            # cv_image_input = self.cvBridge.imgmsg_to_cv2(image_msg, "bgr8")
            cv_image_input = image_msg

        cv_image_input = image_msg
        MIN_MATCH_COUNT = 9 #9
        MIN_MSE_DECISION = 50000

        # find the keypoints and descriptors with SIFT
        kp1, des1 = self.sift.detectAndCompute(cv_image_input,None)

        matches_barra = self.flann.knnMatch(des1,self.des_barra,k=2)
        matches_barraUp = self.flann.knnMatch(des1,self.des_barraUp,k=2)

        image_out_num = 1

        good_barra = []
        for m,n in matches_barra:
            if m.distance < 0.7*n.distance:
                good_barra.append(m)
        if len(good_barra)>MIN_MATCH_COUNT:
            src_pts = np.float32([kp1[m.queryIdx].pt for m in good_barra ]).reshape(-1,1,2)
            dst_pts = np.float32([self.kp_barra[m.trainIdx].pt for m in good_barra]).reshape(-1,1,2)

            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
            matches_barra = mask.ravel().tolist()

            mse = self.fnCalcMSE(src_pts, dst_pts)
            if mse < MIN_MSE_DECISION:
                msg_sign = UInt8()
                msg_sign.data = self.TrafficSign.barra.value

                rospy.loginfo("detect barra sign")
                self.follow.bar = True
                image_out_num = 2
                # self.follow.linearSpeed = 0.03

        else:
            rospy.loginfo("barra not detected")


        if image_out_num == 1:
            cv2.imshow('Input image', cv_image_input)
            cv2.waitKey(1)
            
        elif image_out_num == 2:
            draw_params_barra = dict(matchColor = (255,0,0), # draw matches in green color
                        singlePointColor = None,
                        matchesMask = matches_barra, # draw only inliers
                        flags = 2)

            final_barra = cv2.drawMatches(cv_image_input,kp1,self.img_tunnel,self.kp_barra,good_barra,None,**draw_params_barra)
            cv2.imshow('Final barra', final_barra)
            cv2.waitKey(1)




    def fnCalcMSE(self, arr1, arr2):
            squared_diff = (arr1 - arr2) ** 2
            sum = np.sum(squared_diff)
            num_all = arr1.shape[0] * arr1.shape[1] #cv_image_input and 2 should have same shape
            err = sum / num_all
            return err





    def circle(self,im):

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

        cv2.imshow('thres:',maskR)
        cv2.waitKey(1)

        output = img.copy()

        # detect circles in the image
        Rcircles = cv2.HoughCircles(maskR, cv2.HOUGH_GRADIENT, 10, 500, maxRadius=50)
        Bcircles = cv2.HoughCircles(maskB, cv2.HOUGH_GRADIENT, 10, 500, maxRadius=50)

        # print('len circles:', len(circles))
        # ensure at least some circles were found

        if (Bcircles is not None and self.yaGire is not True) or (self.parking is True and Bcircles is not None):

            Bcircles = np.round(Bcircles[0, :]).astype("int")

            for (x, y, r) in Bcircles:
                # draw the circle in the output image, then draw a rectangle
                # corresponding to the center of the circle
                cv2.circle(output, (x, y), r, (0, 255, 0), 4)
                cv2.rectangle(output, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)

            # show the output image
            cv2.imshow('output:',output)
            cv2.waitKey(1)
            print('entre a azul')

            if self.parking is not True:
                self.circleTrafficSign(im)
            else:
                self.follow.finalParking = True
                self.parking = False
                self.barra = True

        elif Rcircles is not None and self.yaGire:
            # convert the (x, y) coordinates and radius of the circles to integers
            Rcircles = np.round(Rcircles[0, :]).astype("int")
            # loop over the (x, y) coordinates and radius of the circles
            for (x, y, r) in Rcircles:
                # draw the circle in the output image, then draw a rectangle
                # corresponding to the center of the circle
                cv2.circle(output, (x, y), r, (0, 255, 0), 4)
                cv2.rectangle(output, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
            
            # show the output image
            cv2.imshow('output:',output)
            cv2.waitKey(1)
            print('entre a rojo')
            self.circleTrafficSign(im)


        # cv2.imshow('circle blob:',maskR)
        # cv2.waitKey(1)



    def triangle(self, im):

        img = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
        lowerRed = np.array([0, 50, 20])
        upperRed = np.array([20, 255, 255])

        lowerRed2 = np.array([175, 100, 20])
        upperRed2 = np.array([179, 255, 255])

        maskRed1 = cv2.inRange(img, lowerRed, upperRed)
        maskRed2 = cv2.inRange(img, lowerRed2, upperRed2)
        mask_r = cv2.add(maskRed1, maskRed2)
        _, maskR = cv2.threshold(mask_r, 200, 255, cv2.THRESH_BINARY_INV) 

        # convert the image to grayscale
        # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # apply thresholding to convert the grayscale image to a binary image
        # ret,thresh = cv2.threshold(gray,50,255,0)

        kernel = np.ones((2, 2), np.uint8)
        thresh = cv2.morphologyEx(maskR,cv2.MORPH_ERODE,kernel)
        #thresh = cv2.dilate(img, kernel, iterations=1)
        # cv2.imshow("close",dilation_shape)
        # cv2.waitKey(0)

        # cv2.imshow("thresh:", thresh)
        # cv2.waitKey(1)


        # find the contours
        contours,hierarchy = cv2.findContours(thresh, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        # print("Number of contours detected:",len(contours))

        for cnt in contours:
            approx = cv2.approxPolyDP(cnt, 0.04*cv2.arcLength(cnt, True), True)
            if len(approx) == 3:

                # print('Encontre un triangulo')
                # self.follow.Tadvice = True
                self.detectConstruction(im)
                # self.detectTunnel(im)
                # compute the center of mass of the triangle
                # img = cv2.drawContours(img, [cnt], -1, (255,0,0), 3)
                # M = cv2.moments(cnt)
                # if M['m00'] != 0.0:
                #     x = int(M['m10']/M['m00'])
                #     y = int(M['m01']/M['m00'])
                
                # cv2.putText(img, 'Triangle', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        # cv2.imshow("Shapes", img)
        # cv2.waitKey(1)


    def cuadrado(self, im):
        

        # # read the input image
        # img = cv2.imread('parking.jpg')

        img = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
        lowerBlue = np.array([90, 50, 20])
        upperBlue = np.array([120, 255, 255])
        mask_b = cv2.inRange(img, lowerBlue, upperBlue)
        _, maskB = cv2.threshold(mask_b, 200, 255, cv2.THRESH_BINARY_INV)

        # convert the image to grayscale
        # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # apply thresholding to convert the grayscale image to a binary image
        # ret,thresh = cv2.threshold(gray,50,255,0)

        # gray = cv2.bitwise_not(gray) #invertir la imagen binaria

        kernel = np.ones((2, 2), np.uint8)
        thresh = cv2.morphologyEx(maskB,cv2.MORPH_ERODE,kernel)
        #thresh = cv2.dilate(img, kernel, iterations=1)
        # cv2.imshow("close",dilation_shape)
        # cv2.waitKey(0)
        cv2.imshow('thresh',thresh)
        cv2.waitKey(1)

        # find the contours
        contours,hierarchy = cv2.findContours(thresh, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        # print("Number of contours detected:",len(contours))

        for cnt in contours:
            approx = cv2.approxPolyDP(cnt, 0.02*cv2.arcLength(cnt, True), True)
            # print(f'aprox: {approx}')
            if len(approx) == 4:

                self.detectParking(im)

                # compute the center of mass of the triangle
                # img = cv2.drawContours(img, [cnt], -1, (0,0,255), 3)
                # M = cv2.moments(cnt)
                # if M['m00'] != 0.0:
                #     x = int(M['m10']/M['m00'])
                #     y = int(M['m01']/M['m00'])
                # cv2.putText(img, 'Parking', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        # cv2.imshow("Shapes", img)
        # cv2.waitKey(1)

















# def cbFindTrafficSign(self, image_msg):
#         # drop the frame to 1/5 (6fps) because of the processing speed. This is up to your computer's operating power.
#         if self.counter % 3 != 0:
#             self.counter += 1
#             return
#         else:
#             self.counter = 1
#             # cv_image_input = self.cvBridge.imgmsg_to_cv2(image_msg, "bgr8")
#             cv_image_input = image_msg

#         MIN_MATCH_COUNT = 9 #9
#         MIN_MSE_DECISION = 50000

#         # find the keypoints and descriptors with SIFT
#         kp1, des1 = self.sift.detectAndCompute(cv_image_input,None)

#         matches_intersection = self.flann.knnMatch(des1,self.des_intersection,k=2)
#         matches_left = self.flann.knnMatch(des1,self.des_left,k=2)
#         matches_right = self.flann.knnMatch(des1,self.des_right,k=2)

#         image_out_num = 1

#         good_intersection = []
#         for m,n in matches_intersection:
#             if m.distance < 0.7*n.distance:
#                 good_intersection.append(m)
#         if len(good_intersection)>MIN_MATCH_COUNT:
#             src_pts = np.float32([kp1[m.queryIdx].pt for m in good_intersection ]).reshape(-1,1,2)
#             dst_pts = np.float32([self.kp_intersection[m.trainIdx].pt for m in good_intersection]).reshape(-1,1,2)

#             M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
#             matches_intersection = mask.ravel().tolist()

#             mse = self.fnCalcMSE(src_pts, dst_pts)
#             if mse < MIN_MSE_DECISION:
#                 msg_sign = UInt8()
#                 msg_sign.data = self.TrafficSign.intersection.value

#                 rospy.loginfo("detect intersection sign")
#                 image_out_num = 2
#                 # self.follow.linearSpeed = 0.03

#         good_left = []
#         for m,n in matches_left:
#             if m.distance < 0.7*n.distance:
#                 good_left.append(m)
#         if len(good_left)>MIN_MATCH_COUNT:
#             src_pts = np.float32([kp1[m.queryIdx].pt for m in good_left ]).reshape(-1,1,2)
#             dst_pts = np.float32([self.kp_left[m.trainIdx].pt for m in good_left]).reshape(-1,1,2)

#             M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
#             matches_left = mask.ravel().tolist()

#             mse = self.fnCalcMSE(src_pts, dst_pts)
#             if mse < MIN_MSE_DECISION:
#                 msg_sign = UInt8()
#                 msg_sign.data = self.TrafficSign.left.value

#                 rospy.loginfo("detect left sign")
#                 image_out_num = 3

#         else:
#             matches_left = None
#             # rospy.loginfo("nothing detected")

#         good_right = []
#         for m,n in matches_right:
#             if m.distance < 0.7*n.distance:
#                 good_right.append(m)
#         print(len(good_right))
#         if len(good_right)>MIN_MATCH_COUNT:
#             src_pts = np.float32([kp1[m.queryIdx].pt for m in good_right ]).reshape(-1,1,2)
#             dst_pts = np.float32([self.kp_right[m.trainIdx].pt for m in good_right]).reshape(-1,1,2)

#             M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
#             matches_right = mask.ravel().tolist()

#             mse = self.fnCalcMSE(src_pts, dst_pts)
#             if mse < MIN_MSE_DECISION:
#                 msg_sign = UInt8()
#                 msg_sign.data = self.TrafficSign.right.value

#                 rospy.loginfo("detect right sign")
#                 image_out_num = 4

#         else:
#             matches_right = None
#             rospy.loginfo("nothing detected")

#         if image_out_num == 1:
#             cv2.imshow('Input image', cv_image_input)
#             cv2.waitKey(1)
            
#         elif image_out_num == 2:
#             draw_params_intersection = dict(matchColor = (255,0,0), # draw matches in green color
#                         singlePointColor = None,
#                         matchesMask = matches_intersection, # draw only inliers
#                         flags = 2)

#             final_intersection = cv2.drawMatches(cv_image_input,kp1,self.img_intersection,self.kp_intersection,good_intersection,None,**draw_params_intersection)
#             cv2.imshow('Final intersection', final_intersection)
#             cv2.waitKey(1)
            
#         elif image_out_num == 3:
#             draw_params_left = dict(matchColor = (255,0,0), # draw matches in green color
#                         singlePointColor = None,
#                         matchesMask = matches_left, # draw only inliers
#                         flags = 2)

#             final_left = cv2.drawMatches(cv_image_input,kp1,self.img_left,self.kp_left,good_left,None,**draw_params_left)
#             cv2.imshow('Final left', final_left)
#             cv2.waitKey(1)
            
#         elif image_out_num == 4:
#             draw_params_right = dict(matchColor = (255,0,0), # draw matches in green color
#                             singlePointColor = None,
#                             matchesMask = matches_right, # draw only inliers
#                             flags = 2)

#             final_right = cv2.drawMatches(cv_image_input,kp1,self.img_right,self.kp_right,good_right,None,**draw_params_right)
#             cv2.imshow('Final right', final_right)
#             cv2.waitKey(1)














