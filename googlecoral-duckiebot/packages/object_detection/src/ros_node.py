#!/usr/bin/env python
import os
import rospy
import copy
import numpy as np
import cv2
from duckietown import DTROS
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import BoolStamped
from cv_bridge import CvBridge
import json
import time


class Detector(DTROS):

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(Detector, self).__init__(node_name=node_name)

        # Get vehicle name
        self.veh_name = str(os.environ['VEHICLE_NAME'])

        # setting if object detection should activate emergency stops
        # if Duckie or Duckiebot are detected close-by
        self.objdet_emergency_stop = False
        if os.environ['OBJDET_EMERGENCY_STOP'] == 'true':
            self.objdet_emergency_stop = True

        # state is emergency stop is currently activated
        self.emergency_stop_activated = False

        self.fps = str(0)

        # Load camera resolution
        try:
            self.res_w = int(os.environ['resolution_w'])
        except:
            rospy.loginfo("The environment variable resolution_w is not set. Resolution is set to default (320x240).")
            self.res_w = 320

        self.res_h = self.res_w * 3 / 4
        rospy.set_param('/%s/camera_node/exposure_mode' % (self.veh_name), 'off')
        rospy.set_param('/%s/camera_node/res_w' % (self.veh_name), self.res_w)
        rospy.set_param('/%s/camera_node/res_h' % (self.veh_name), self.res_h)

        # Initialize ROS topics
        self.pub_coral_image = rospy.Publisher("/{}/coral_object_detection/image/compressed".format(self.veh_name),
                                               CompressedImage,
                                               queue_size=1)
        rospy.loginfo("Publishing image")
        self.sub_image = rospy.Subscriber("/{}/camera_node/image/compressed".format(self.veh_name), CompressedImage,
                                          self.callback, queue_size=1)
        self.pub_emergency_stop = rospy.Publisher("/{}/wheels_driver_node/emergency_stop", BoolStamped, queue_size=1)

    def callback(self, data):
        # start_time = time.time()
        # x = 1 # displays the frame rate every 1 second

        # Convert compressed image to BGR
        np_arr = np.fromstring(data.data, np.uint8)
        orig = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        frame = cv2.cvtColor(orig, cv2.COLOR_BGR2RGB)
        np.save('/code/catkin_ws/src/coral/packages/object_detection/src/temp_frame', frame)

        try:
            with open('/code/catkin_ws/src/coral/packages/object_detection/src/prediction') as json_file:
                predictions = json.load(json_file)

            for prediction in predictions:
                # Read FPS
                self.fps = str(prediction['FPS'])
                self.draw_prediction(orig, prediction)

                if self.objdet_emergency_stop is True:
                    self.assess_emergency_stop(prediction)

        except Exception as e:
            print(e)
            rospy.logerr(e)

        # Render FPS
        textfps = "Inference FPS: {}".format(self.fps)
        cv2.putText(orig, textfps, (10, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.25, (255, 255, 255), 1)

        # Publish image
        compressed_img = br.cv2_to_compressed_imgmsg(orig, dst_format='jpg')
        self.pub_coral_image.publish(compressed_img)

    def draw_prediction(self, img, p):
        # Read bounding box, label, and score
        startX, startY, endX, endY = p['startX'], p['startY'], p['endX'], p['endY']
        label = p['label']
        score = p['score']
        box_color = eval(p['color'])

        # Text specification
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.25
        font_color = (0, 0, 0)
        font_thickness = 1
        y_text = startY - 5
        text = "{}: {:.2f}%".format(label, score * 100)

        # Draw the bounding box and label on the image
        cv2.rectangle(img, (startX, startY), (endX, endY), box_color, 1)
        # Add text fill
        (w, h) = cv2.getTextSize(text, font, font_scale, font_thickness)[0]
        cv2.rectangle(img, (startX, startY), (startX + w, startY - h - 6), box_color, -1)

        # Render text
        cv2.putText(img, text, (startX, y_text), font, font_scale, font_color, font_thickness)

    def assess_emergency_stop(self, prediction):
        activate_emergency_stop = False
        deactivate_emergency_stop = True

        # we use centerpoint of lower edge of image for assessment, as ground projection works best for it
        u = int((prediction['startX'] + prediction['endX']) / 2)
        v = max(prediction['startY'], prediction['endY'])

        # prediction certainty should be higher than 75% for emergency stop
        if prediction['score'] > 0.75:
            # activate emergency stop if object is Duckie or Duckiebot
            if prediction['label'] == 'Duckie' or prediction['label'] == 'Duckiebot':
                # activate emergency stop if object is in horizontal center half and in lower quarter of image
                if v >= 3/4.0*self.res_h and 1 / 4.0 * self.res_w <= u <= 3 / 4.0 * self.res_w:
                    activate_emergency_stop = True
                    deactivate_emergency_stop = False

        if activate_emergency_stop is True or deactivate_emergency_stop is True:
            emergency_stop_msg = BoolStamped()
            if activate_emergency_stop is True:
                emergency_stop_msg.data = True
                self.emergency_stop_activated = True

            if deactivate_emergency_stop is True:
                emergency_stop_msg.data = False
                self.emergency_stop_activated = False

            self.pub_emergency_stop.publish(emergency_stop_msg)


if __name__ == '__main__':
    # Initialize the node
    br = CvBridge()
    Detector_node = Detector(node_name='detector')
    # Keep it spinning to keep the node alive
    rospy.spin()
