#youtube video: http://youtu.be/Mj_rb_ygdoQ
#student@ubuntu:~/projects/gazebo-examples/d_diff_drive_robot$ ./robot-view server
#./robot-view server
#!/usr/bin/env python
# /* -*-  indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
# /*
# Copyright (c) 2014, Daniel M. Lofaro <dan (at) danLofaro (dot) com>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the author nor the names of its contributors may
#       be used to endorse or promote products derived from this software
#       without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# */
import diff_drive
import ach
import sys
import time
from ctypes import *
import socket
import cv2.cv as cv
import cv2
import numpy as np

dd = diff_drive
ref = dd.H_REF()
tim = dd.H_TIME()

ROBOT_DIFF_DRIVE_CHAN   = 'robot-diff-drive'
ROBOT_CHAN_VIEW   = 'robot-vid-chan'
ROBOT_TIME_CHAN  = 'robot-time'
# CV setup 
cv.NamedWindow("wctrl", cv.CV_WINDOW_AUTOSIZE)
#capture = cv.CaptureFromCAM(0)
#capture = cv2.VideoCapture(0)

# added
##sock.connect((MCAST_GRP, MCAST_PORT))
newx = 320
newy = 240

nx = 640
ny = 480

r = ach.Channel(ROBOT_DIFF_DRIVE_CHAN)
r.flush()
v = ach.Channel(ROBOT_CHAN_VIEW)
v.flush()
t = ach.Channel(ROBOT_TIME_CHAN)
t.flush()

i=0


print '======================================'
print '============= Robot-View ============='
print '========== Daniel M. Lofaro =========='
print '========= dan@danLofaro.com =========='
print '======================================'


counting = 0
NumColor = 2



while True:
    # Get Frame
    img = np.zeros((newx,newy,3), np.uint8)
    c_image = img.copy()
    vid = cv2.resize(c_image,(newx,newy))
    [status, framesize] = v.get(vid, wait=False, last=True)
    if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
        vid2 = cv2.resize(vid,(nx,ny))
        img = cv2.cvtColor(vid2,cv2.COLOR_BGR2RGB)
        cv2.imshow("wctrl", img)
        cv2.waitKey(10)
    else:
        raise ach.AchException( v.result_string(status) )


    [status, framesize] = t.get(tim, wait=False, last=True)
    if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
        pass
        #print 'Sim Time = ', tim.sim[0]
    else:
        raise ach.AchException( v.result_string(status) )

#-----------------------------------------------------
#--------[ Do not edit above ]------------------------
#-----------------------------------------------------

    # Def:
    # ref.ref[0] = Right Wheel Velos
    # ref.ref[1] = Left Wheel Velos
    # tim.sim[0] = Sim Time
    # img        = cv image in BGR format
	

    
    # Take each frame
    #_, frame = img.read()
    frame = img

    # Convert BGR to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)



    # define range of colors in HSV
    lower_blue   = np.array([110, 50, 50], dtype=np.uint8)
    upper_blue   = np.array([130,255,255], dtype=np.uint8)

    lower_green = np.array([50, 50,  50], dtype=np.uint8)
    upper_green = np.array([70, 255, 255], dtype=np.uint8)

    lower_red    = np.array([ 0,  50,  50], dtype=np.uint8)
    upper_red    = np.array([10, 255, 255], dtype=np.uint8)


    #pick mask
    if(NumColor == 1):
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
 
    if(NumColor == 2):
        mask = cv2.inRange(hsv, lower_green, upper_green)

    if(NumColor == 3):
        mask = cv2.inRange(hsv, lower_red, upper_red)



    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame,frame, mask= mask)

    #cv2.imshow('frame',frame)
    cv2.imshow('mask',mask)
    #cv2.imshow('res',res)
    k = cv2.waitKey(5) & 0xFF
    
    contours,hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(mask,contours,-1,(0,255,0))

    img_mom = cv2.moments(mask,0)
    area = img_mom['m00']
    
    found = 0

    if((area > 7000)):
        #print area
        found = 1
    else:
        found = 0

    
    if(found):
        #print 'found'
        #print 'count:', counting
        max_area = 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > max_area:
                max_area = area
                best_cnt = cnt
        # finding centroids of best_cnt
        M = cv2.moments(best_cnt)
        cx,cy = int(M['m10']/M['m00']), int(M['m01']/M['m00']) # this is where the object target

        if(cx > 340):#slow down to track ball
            ref.ref[0] = -0.1
            ref.ref[1] = 0.1
            stoptime = tim.sim[0]
        else:
            #counting = counting + 1 #using simtime for count
            ref.ref[0] = -0.0
            ref.ref[1] = 0.0

    else:
        ref.ref[0] = -0.5
        ref.ref[1] = 0.5
        cx = 0
        cy = 0
        stoptime = tim.sim[0]





    
    if((tim.sim[0] - stoptime)>10):#(counting == 25):
        #counting = 0
	if(NumColor == 1):
            NumColor = 2
        else:
            if(NumColor == 2):
                NumColor = 3
            else:
                if(NumColor == 3):
                    NumColor = 1



    


    #print 'Sim Time = ', tim.sim[0], NumColor, cx, cy
    
    # Sets reference to robot
    r.put(ref);

    # Sleeps
    time.sleep(0.1)   
#-----------------------------------------------------
#--------[ Do not edit below ]------------------------
#-----------------------------------------------------
