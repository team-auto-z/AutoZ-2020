#!/usr/bin/env python

import cv2
import numpy as np
import sys
import os
from PIL import Image

def road_lines(original_image):
	height,width,col =original_image.shape
	src = np.array([[0,0],[width,0],[width,height],[0,height]],dtype='float32')
	dest = np.array([[0,0],[width,0],[height,479],[210,height]],dtype='float32')
	h, status = cv2.findHomography(src, dest)
	imgd = np.zeros((height,width,col),dtype='uint8')
	top_view = cv2.warpPerspective(original_image, h, (width,height))
	red_chan=top_view[:,:,0]
	th, binary_warped = cv2.threshold(red_chan, 240, 255, cv2.THRESH_BINARY)
	image = cv2.medianBlur(binary_warped,11 )
	histogram = np.sum(image[np.int(binary_warped.shape[0]/2):,:], axis=0)
	out_img = np.dstack((binary_warped, binary_warped, binary_warped))*255
	midpoint = np.int(histogram.shape[0]/2)
	leftx_base = np.argmax(histogram[:midpoint])
	rightx_base = np.argmax(histogram[midpoint:]) + midpoint
	nwindows = 9
	window_height = np.int(binary_warped.shape[0]/nwindows)
	nonzero = binary_warped.nonzero()
	nonzeroy = np.array(nonzero[0])
	nonzerox = np.array(nonzero[1])
	leftx_current = leftx_base
	rightx_current = rightx_base
	margin = 100
	minpix = 50
	left_lane_inds = []
	right_lane_inds = []
	#nwind=0
	for window in range(nwindows):
		#nwind=nwind+1
		win_y_low = binary_warped.shape[0] - (window+1)*window_height
		win_y_high = binary_warped.shape[0] - window*window_height
		win_xleft_low = leftx_current - margin
		win_xleft_high = leftx_current + margin
		win_xright_low = rightx_current - margin
		win_xright_high = rightx_current + margin
		cv2.rectangle(out_img,(win_xleft_low,win_y_low),(win_xleft_high,win_y_high),(0,255,0), 2) 
		cv2.rectangle(out_img,(win_xright_low,win_y_low),(win_xright_high,win_y_high),(0,255,0), 2) 
		good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
		good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
		left_lane_inds.append(good_left_inds)
		right_lane_inds.append(good_right_inds)
		if len(good_left_inds) > minpix:
			leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
		if len(good_right_inds) > minpix:        
			rightx_current = np.int(np.mean(nonzerox[good_right_inds]))
	left_lane_inds = np.concatenate(left_lane_inds)
	right_lane_inds = np.concatenate(right_lane_inds)
	leftx = nonzerox[left_lane_inds]
	lefty = nonzeroy[left_lane_inds] 
	rightx = nonzerox[right_lane_inds]
	righty = nonzeroy[right_lane_inds] 
	if (len(leftx)==0 and len(lefty)==0):
		leftx=[0]*229
		lefty=list(range(229))
	if (len(rightx)==0 and len(righty)==0):
		rightx=[639]*229
		righty=list(range(229))
	left_fit = np.polyfit(lefty, leftx, 2)
	right_fit = np.polyfit(righty, rightx, 2)
	ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0])
	#ploty = np.linspace(binary_warped.shape[0],binary_warped.shape[0]-(nwind*window_height),binary_warped.shape[0] )
	left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
	right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
	left=np.column_stack((np.array(left_fitx),ploty))
	left = left.reshape((-1,1,2))
	right=np.column_stack((np.array(right_fitx),ploty))
	right = right.reshape((-1,1,2))
	pts=np.concatenate((left,right),axis=0)
	dvx=[]
	dvy=[]
	mid=(right+left)/2
	midx=mid[:,:,0]
	midy=mid[:,:,1]
	for i in range(len(midy)-1,1,-1):
		dvx.append(midx[i-1]-midx[i])
		dvy.append(midy[i]-midy[i-1])
	return(np.mean(np.array(dvx)),np.mean(np.array(dvy)))

# cap = cv2.VideoCapture('4.avi')





# while(True):
# 	cap=cv2.VideoCapture(0)
# 	ret,frame=cap.read()
# 	cv2.imshow("frame",frame)
# 	x,y=road_lines(frame)
# 	if(cv2.waitKey(1)&0xFF==ord('q')):
# 		break
# 	print("x=",x)
# 	print("y=",y)
# cap.release()
# cv2.destroyAllWindows()
