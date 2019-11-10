#!/usr/bin/env python
import os
import rospy
import copy
import numpy as np
import cv2
from duckietown import DTROS
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import json

class Detector(DTROS):

	def __init__(self, node_name):

		# Initialize the DTROS parent class
		super(Detector, self).__init__(node_name=node_name)

		#Get vehicle name
		self.veh_name = str(os.environ['VEHICLE_NAME'])

		#Load camera resolution
		try:
			self.res_w = int(os.environ['resolution_w'])
		except:
			rospy.loginfo("The environment variable resolution_w is not set. Resolution is set to default (320x240).")
			self.res_w = 320

		self.res_h = self.res_w*3/4
		rospy.set_param('/%s/camera_node/exposure_mode' %(self.veh_name) , 'off')
		rospy.set_param('/%s/camera_node/res_w' %(self.veh_name), self.res_w)
		rospy.set_param('/%s/camera_node/res_h' %(self.veh_name), self.res_h)

		#Initialize ROS topics
		self.pub = rospy.Publisher("/{}/coral_object_detection/image/compressed".format(self.veh_name), CompressedImage, queue_size=1)
		rospy.loginfo("Publishing image")
		self.sub_image = rospy.Subscriber("/{}/camera_node/image/compressed".format(self.veh_name), CompressedImage, self.callback, queue_size=1)

	def callback(self, data):
		#Convert compressed image to BGR
		np_arr = np.fromstring(data.data, np.uint8)
		orig = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

		frame = cv2.cvtColor(orig, cv2.COLOR_BGR2RGB)
		np.save('/code/catkin_ws/src/coral/packages/object_detection/src/temp_frame', frame)

		try:
			with open('/code/catkin_ws/src/coral/packages/object_detection/src/prediction') as json_file:
				prediction = json.load(json_file)

			for p in prediction:
				startX, startY, endX, endY = p['startX'], p['startY'], p['endX'], p['endY']
				label = p['label']
				score = p['score']

				# draw the bounding box and label on the image
				cv2.rectangle(orig, (startX, startY), (endX, endY), (0, 255, 0), 2)
				y = startY - 15 if startY - 15 > 15 else startY + 15
				text = "{}: {:.2f}%".format(label, score * 100)
				cv2.putText(orig, text, (startX, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

		except:
			pass

		compressed_img = br.cv2_to_compressed_imgmsg(orig, dst_format='jpg')

		self.pub.publish(compressed_img)

if __name__ == '__main__':
	# Initialize the node
	br = CvBridge()
	Detector_node = Detector(node_name='detector')
	# Keep it spinning to keep the node alive
	rospy.spin()