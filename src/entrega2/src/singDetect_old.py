import rospy
import numpy as np
import cv2
from enum import Enum
from std_msgs.msg import UInt8

class DetectSign():
    def __init__(self,_follow):
        self.fnPreproc()
        self.TrafficSign = Enum('TrafficSign', 'intersection left right')
        self.counter = 1
        self.follow = _follow

    def fnPreproc(self):
        # Initiate SIFT detector
        self.sift = cv2.SIFT_create()
        dir_path = '/home/shorlak/genius_ws/src/entrega2/src/signs/'
        self.img_intersection = cv2.imread(dir_path + 'intersection.png',0)
        self.img_left  = cv2.imread(dir_path + 'left.png',0)
        self.img_right = cv2.imread(dir_path + 'right.png',0)

        self.kp_intersection, self.des_intersection  = self.sift.detectAndCompute(self.img_intersection, None)
        self.kp_left, self.des_left  = self.sift.detectAndCompute(self.img_left, None)
        self.kp_right, self.des_right = self.sift.detectAndCompute(self.img_right, None)

        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks = 50)

        self.flann = cv2.FlannBasedMatcher(index_params, search_params)


    def cbFindTrafficSign(self, image_msg):
        # drop the frame to 1/5 (6fps) because of the processing speed. This is up to your computer's operating power.
        if self.counter % 3 != 0:
            self.counter += 1
            return
        else:
            self.counter = 1
            # cv_image_input = self.cvBridge.imgmsg_to_cv2(image_msg, "bgr8")
            cv_image_input = image_msg

        MIN_MATCH_COUNT = 9 #9
        MIN_MSE_DECISION = 50000

        # find the keypoints and descriptors with SIFT
        kp1, des1 = self.sift.detectAndCompute(cv_image_input,None)

        matches_intersection = self.flann.knnMatch(des1,self.des_intersection,k=2)
        matches_left = self.flann.knnMatch(des1,self.des_left,k=2)
        matches_right = self.flann.knnMatch(des1,self.des_right,k=2)

        image_out_num = 1

        good_intersection = []
        for m,n in matches_intersection:
            if m.distance < 0.7*n.distance:
                good_intersection.append(m)
        if len(good_intersection)>MIN_MATCH_COUNT:
            src_pts = np.float32([kp1[m.queryIdx].pt for m in good_intersection ]).reshape(-1,1,2)
            dst_pts = np.float32([self.kp_intersection[m.trainIdx].pt for m in good_intersection]).reshape(-1,1,2)

            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
            matches_intersection = mask.ravel().tolist()

            mse = self.fnCalcMSE(src_pts, dst_pts)
            if mse < MIN_MSE_DECISION:
                msg_sign = UInt8()
                msg_sign.data = self.TrafficSign.intersection.value

                rospy.loginfo("detect intersection sign")
                image_out_num = 2
                # self.follow.linearSpeed = 0.03

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
            if mse < MIN_MSE_DECISION:
                msg_sign = UInt8()
                msg_sign.data = self.TrafficSign.left.value

                rospy.loginfo("detect left sign")
                image_out_num = 3

        else:
            matches_left = None
            # rospy.loginfo("nothing detected")

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
            if mse < MIN_MSE_DECISION:
                msg_sign = UInt8()
                msg_sign.data = self.TrafficSign.right.value

                rospy.loginfo("detect right sign")
                image_out_num = 4

        else:
            matches_right = None
            rospy.loginfo("nothing detected")

        if image_out_num == 1:
            cv2.imshow('Input image', cv_image_input)
            cv2.waitKey(1)
            
        elif image_out_num == 2:
            draw_params_intersection = dict(matchColor = (255,0,0), # draw matches in green color
                        singlePointColor = None,
                        matchesMask = matches_intersection, # draw only inliers
                        flags = 2)

            final_intersection = cv2.drawMatches(cv_image_input,kp1,self.img_intersection,self.kp_intersection,good_intersection,None,**draw_params_intersection)
            cv2.imshow('Final intersection', final_intersection)
            cv2.waitKey(1)
            
        elif image_out_num == 3:
            draw_params_left = dict(matchColor = (255,0,0), # draw matches in green color
                        singlePointColor = None,
                        matchesMask = matches_left, # draw only inliers
                        flags = 2)

            final_left = cv2.drawMatches(cv_image_input,kp1,self.img_left,self.kp_left,good_left,None,**draw_params_left)
            cv2.imshow('Final left', final_left)
            cv2.waitKey(1)
            
        elif image_out_num == 4:
            draw_params_right = dict(matchColor = (255,0,0), # draw matches in green color
                            singlePointColor = None,
                            matchesMask = matches_right, # draw only inliers
                            flags = 2)

            final_right = cv2.drawMatches(cv_image_input,kp1,self.img_right,self.kp_right,good_right,None,**draw_params_right)
            cv2.imshow('Final right', final_right)
            cv2.waitKey(1)


    def fnCalcMSE(self, arr1, arr2):
            squared_diff = (arr1 - arr2) ** 2
            sum = np.sum(squared_diff)
            num_all = arr1.shape[0] * arr1.shape[1] #cv_image_input and 2 should have same shape
            err = sum / num_all
            return err
 

















