#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
import cv2
import os
import sys
import glob
import numpy as np
import math


from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist



class camera:
    def __init__(self):#, device, width, height, rates):
        #currentpath, _ = os.path.split(os.path.abspath(sys.argv[0]))
        #self.calibrationPath = os.path.join(currentpath, 'calib_pics')
        #self.testImgPath = os.path.join(currentpath, 'cc.jpg')
        
        self.camMat = []
        self.camDistortion = []

        self.cap = cv2.VideoCapture(1)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        
        rospy.init_node('laneline', anonymous=True)
        self.rate = rospy.Rate(10)

        self.cvb = CvBridge()        

    def calibrateAndTrans(self):
        cameraMatrix = np.array([[1.59416773e+03,0,1.04169331e+03],[0,1.55532826e+03,6.38753345e+02],[0,0,1]])
        cameraDistortion = np.array([[-0.55126622,  0.14143426, -0.02125189, -0.00292519,  0.44898094]])
        if cameraMatrix != []:
            self.camMat = cameraMatrix
            self.camDistortion = cameraDistortion
            print 'CALIBRATION SUCCESSED!'
        else:
            print 'CALIBRATION FAILED!'
        return 0

    def spin(self):
        global pub,aP,lastP
        while not rospy.is_shutdown():
            ret, img = self.cap.read()
            if ret == True:
                undstrt = cv2.undistort(img, self.camMat, self.camDistortion, None, self.camMat)
                
                gray = cv2.cvtColor(undstrt, cv2.COLOR_BGR2GRAY)
                gray_Blur = gray
                kernel = np.ones((10,10),np.uint8)
                gray_Blur = cv2.dilate(gray_Blur, kernel, iterations = 1)
                origin_thr = np.zeros_like(gray_Blur)
                origin_thr[(gray_Blur <= 100)] = 255 
                binary_warped = cv2.warpPerspective(origin_thr, M, img.shape[1::-1], flags=cv2.INTER_LINEAR)
                histogram = np.sum(binary_warped[binary_warped.shape[0]//2:,:], axis=0) #4/5
                #midpoint = int(histogram.shape[0]/2)
                
                lane_base = np.argmax(histogram)

                nwindows = 5
                window_height = int(binary_warped.shape[0]/nwindows)
                nonzero = binary_warped.nonzero()
                nonzeroy = np.array(nonzero[0])
                nonzerox = np.array(nonzero[1])
                lane_current = lane_base
                margin = 100
                minpix = 1
                lane_inds = []


                for window in range(nwindows):
                    win_y_low = binary_warped.shape[0] - (window + 1)*window_height
                    win_y_high = binary_warped.shape[0] - window*window_height 
                    win_x_low = lane_current - margin 
                    win_x_high = lane_current + margin 
                    good_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_x_low) & (nonzerox < win_x_high)).nonzero()[0]
                    
                    lane_inds.append(good_inds)
                    if len(good_inds) > minpix:
                        lane_current = int(np.mean(nonzerox[good_inds])) ####

                lane_inds = np.concatenate(lane_inds)

                pixelX = nonzerox[lane_inds]
                pixelY = nonzeroy[lane_inds]

                # calculate the aimPoint
                
                if (pixelX.size == 0):
                    continue
                a2,a1,a0 = np.polyfit(pixelX, pixelY, 2)
                
                
                aveX = np.average(pixelX)
                #区分左右车道线,以计算截距
                if (2*a2*aveX + a1) > 0 : #斜率大于0
                    if a2 > 0:
                        x_intertcept = (-a1+(abs(a1*a1-4*a2*(a0 - 1183.63))**0.5))/(2*a2) #求截距
                    else :
                        x_intertcept = (-a1-(abs(a1*a1-4*a2*(a0 - 1183.63))**0.5))/(2*a2)
                        
                else : #斜率小于0
                    if a2 > 0:
                        x_intertcept = (-a1-(abs(a1*a1-4*a2*(a0 - 1183.63))**0.5))/(2*a2) 
                    else :
                        x_intertcept = (-a1+(abs(a1*a1-4*a2*(a0 - 1183.63))**0.5))/(2*a2)

                if (x_intertcept > 639):
                    LorR = -1; #RightLane
                    #print('R')
                else:
                    LorR = 1; #LeftLane
                    #print('L')                


                frontDistance = np.argsort(pixelY)[int(len(pixelY)/5)]
                aimLaneP = [pixelX[frontDistance], pixelY[frontDistance]]
                
                #计算aimLaneP处斜率，从而得到目标点的像素坐标
                lanePk = 2*a2*aimLaneP[0] + a1
                if (lanePk > 500 or lanePk < -500):
                    aP[0] = aimLaneP[0] + LorR*roadWidth/2
                    aP[1] = aimLaneP[1]
                else :
                    k_ver = -1/lanePk
                    theta = math.atan(k_ver)
                    aP[0] = aimLaneP[0] + math.cos(theta)*(LorR)*roadWidth/2
                    aP[1] = aimLaneP[1] + math.sin(theta)*(LorR)*roadWidth/2

                aP[0] = (aP[0] - 639)*x_cmPerPixel
                aP[1] = (720 - aP[1])*y_cmPerPixel + y_offset
                
                #计算目标点的真实坐标
                if(lastP[0] > 0.001 and lastP[1] > 0.001):
                    if(((aP[0]-lastP[0])**2 + (aP[1]-lastP[1])**2 > 2500) and Timer < 4 ): #To avoid the mislead by walkers
                        aP = lastP
                        Timer = Timer + 1
                    else:
                        Timer = 0

                lastP = aP 
                steerAngle = math.atan(2*I*aP[0]/(aP[0]*aP[0]+(aP[1]+D)*(aP[1]+D)))
                cmd_vel.angular.z = k*steerAngle
                pub.publish(cmd_vel)
                    
            self.rate.sleep()

        self.cap.release()


        

if __name__ == '__main__':

    #透视变换
    startx = 256
    starty = 620
    length_pers = 766
    width_pers = int(length_pers*59/76)
    srcps = np.float32([[(20, 525), (306, 278), (967, 312), (1168, 536)]])
    dstps = np.float32([[(startx, starty), (startx, starty - width_pers), (startx + length_pers, starty - width_pers), (startx + length_pers, starty)]])
    M = cv2.getPerspectiveTransform(srcps, dstps)
    if M != []:
        print 'Transform Successed'
    else:
        print 'failed'

    #距离映射
    x_cmPerPixel = 76/766.0
    y_cmPerPixel = 76/766.0
    roadWidth = 80.0 / x_cmPerPixel #80.0
    y_offset = 46.0 #cm

    aP = [0.0, 0.0]
    lastP = [0.0, 0.0]
    Timer = 0

    #轴间距
    I = 58.0
    #摄像头坐标系与车中心间距
    D = 17.0

    k = -2.0     #tune this parameter

    #steerAngle, cmdSteer;
    cmd_vel = Twist()
    cmd_vel.linear.x = 0.3
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    try:
        cam = camera()
        cam.calibrateAndTrans()
        cam.spin()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    
