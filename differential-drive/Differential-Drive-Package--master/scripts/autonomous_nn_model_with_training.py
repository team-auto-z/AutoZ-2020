#!/usr/bin/env python
from image_sensor_publisher import ImageConverter
import rospy
from sensor_msgs.msg import Image
import sys
import cv2
converter = ImageConverter()
from geometry_msgs.msg import Twist
import numpy as np
import torch.nn as nn
import torch.nn.functional as F
import torch
from torchvision import models

LOOP = True


model = nn.Sequential(
		models.vgg19(),
		nn.Linear(1000,2),
	)

print(model)







class Data:
	def __init__(self):
		self.image = None
		self.command = None

	def assign(self,image,command):
		self.image = image
		self.command = command

	def is_valid(self):
		return self.image!=None and self.command!=None

	def __str__(self):
		return str(self.image) + ',' + str(self.command)


data = Data()



def image_callback(data1):
	global LOOP,data
	image = converter.convert_msg_to_image(data1)
	# print(image)

	cv2.imshow("frame---",image)
	data.image = image

	if cv2.waitKey(27) == ord('q'):
		LOOP = False
		
		cv2.destroyAllWindows()
	
def control_callback(data1):
	global data
	data.command = np.array([data1.linear.x,data1.angular.z])






def main():
	global LOOP,data

	device = torch.device("cpu")

	model.to(device)

	rospy.init_node("NNTraining")
	rospy.Subscriber("/front_view",Image,image_callback)
	rospy.Subscriber("/twist",Twist,control_callback)
	
	rate = rospy.Rate(rospy.get_param("~rate", 100))

	criterion = nn.MSELoss()
	opt = torch.optim.Adam(model.parameters(),lr=0.01)
	print("Running....")
	while LOOP:
		try:
			rate.sleep()
			if data.is_valid():
				X = torch.from_numpy(data.image.reshape((-1,3,640,480))).float().to(device)
				y = torch.from_numpy(data.command).float().to(device)
				# print(X,y)
				opt.zero_grad()
				yhat = model(X)
				loss_val = torch.log(criterion(yhat,y))
				loss_val.backward()
				opt.step()

				print(loss_val.item())




				# print(data.image.shape,data.command)
				
		

		except KeyboardInterrupt:
			cv2.destroyAllWindows()
			break
		



if __name__ == '__main__':
	main()