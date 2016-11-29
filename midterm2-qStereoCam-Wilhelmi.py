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
import skimage.morphology as morphology
import matplotlib.pyplot as plt
import scipy

dd = diff_drive
ref = dd.H_REF()
tim = dd.H_TIME()

ROBOT_DIFF_DRIVE_CHAN   = 'robot-diff-drive'
ROBOT_CHAN_VIEW_R   = 'robot-vid-chan-r'
ROBOT_CHAN_VIEW_L   = 'robot-vid-chan-l'
ROBOT_TIME_CHAN  = 'robot-time'
# CV setup 
cv.NamedWindow("wctrl_L", cv.CV_WINDOW_AUTOSIZE)
cv.NamedWindow("wctrl_R", cv.CV_WINDOW_AUTOSIZE)
#capture = cv.CaptureFromCAM(0)
#capture = cv2.VideoCapture(0)

# added
##sock.connect((MCAST_GRP, MCAST_PORT))
newx = 320
newy = 240

nx = 320
ny = 240

r = ach.Channel(ROBOT_DIFF_DRIVE_CHAN)
r.flush()
vl = ach.Channel(ROBOT_CHAN_VIEW_L)
vl.flush()
vr = ach.Channel(ROBOT_CHAN_VIEW_R)
vr.flush()
t = ach.Channel(ROBOT_TIME_CHAN)
t.flush()

i=0

focal = .085 #meters
pixelSize = .000280 #meters
base = .4 #meters
width = 320.

print '======================================'
print '============= Robot-View ============='
print '========== Daniel M. Lofaro =========='
print '========= dan@danLofaro.com =========='
print '======================================'
while True:
    # Get Frame
    imgL = np.zeros((newx,newy,3), np.uint8)
    imgR = np.zeros((newx,newy,3), np.uint8)
    c_image = imgL.copy()
    c_image = imgR.copy()
    vidL = cv2.resize(c_image,(newx,newy))
    vidR = cv2.resize(c_image,(newx,newy))
    [status, framesize] = vl.get(vidL, wait=False, last=True)
    if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
        vid2 = cv2.resize(vidL,(nx,ny))
        imgL = cv2.cvtColor(vid2,cv2.COLOR_BGR2RGB)
        cv2.imshow("wctrl_L", imgL)
        cv2.waitKey(10)
    else:
        raise ach.AchException( v.result_string(status) )
    [status, framesize] = vr.get(vidR, wait=False, last=True)
    if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
        vid2 = cv2.resize(vidR,(nx,ny))
        imgR = cv2.cvtColor(vid2,cv2.COLOR_BGR2RGB)
        cv2.imshow("wctrl_R", imgR)
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
#-----------------------------------------------------
#-----------------------------------------------------
    # Def:
    # ref.ref[0] = Right Wheel Velos
    # ref.ref[1] = Left Wheel Velos
    # tim.sim[0] = Sim Time
    # imgL       = cv image in BGR format (Left Camera)
    # imgR       = cv image in BGR format (Right Camera)
    
    
    hsvImgR = cv2.cvtColor(imgR,cv2.COLOR_RGB2HSV_FULL)
    hsvImgL = cv2.cvtColor(imgL,cv2.COLOR_RGB2HSV_FULL)
    # print np.max(np.max(hsvImgR[:,:,0]))
    # hist , bin_= np.histogram(hsvImgR[:,:,0] , bins=np.arange(360))


    set1R = np.array(np.where(hsvImgR[:,:,0] > 50.))
    set2R = np.array(np.where(hsvImgR[:,:,0] < 200.))


    set1L = np.array(np.where(hsvImgL[:,:,0] > 50.))
    set2L = np.array(np.where(hsvImgL[:,:,0] < 200.))


    # set1 = np.reshape(set1,[np.shape(set1)[1],2])
    # set2 = np.reshape(set2,[np.shape(set2)[1],2])

    s1R = np.char.array(set1R[0,]) + '-' + np.char.array(set1R[1,])
    s2R = np.char.array(set2R[0,]) + '-' + np.char.array(set2R[1,])

    s1L = np.char.array(set1L[0,]) + '-' + np.char.array(set1L[1,])
    s2L = np.char.array(set2L[0,]) + '-' + np.char.array(set2L[1,])

    indsR = np.where(np.in1d(s1R,s2R))[0]
    indsL = np.where(np.in1d(s1L,s2L))[0]

    binImageR = np.zeros([np.shape(hsvImgR)[0],np.shape(hsvImgR)[1]])
    binImageL = np.zeros([np.shape(hsvImgL)[0],np.shape(hsvImgL)[1]])

    binImageR[set1R[0,indsR],set1R[1,indsR]] = 1
    binImageL[set1L[0,indsL],set1L[1,indsL]] = 1


    binImageR = morphology.dilation(morphology.erosion(binImageR,selem=morphology.disk(1)),selem=morphology.disk(1))
    binImageL = morphology.dilation(morphology.erosion(binImageL,selem=morphology.disk(1)),selem=morphology.disk(1))


    indsR = np.where(binImageR == 1)
    indsL = np.where(binImageL == 1)



    try:

        meanIndsR = np.mean(indsR[1])

        meanIndsL = np.mean(indsL[1])

        z = (base * focal) / (pixelSize * ((width/2.-meanIndsR) - (width/2.-meanIndsL)))

    except:
        z = 'NAN'


    print "The distance is {} meters away".format(z)




    # Sleeps
    time.sleep(0.1)   
#-----------------------------------------------------
#-----------------------------------------------------
#-----------------------------------------------------
