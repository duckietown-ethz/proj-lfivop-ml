#!/usr/bin/env python
import os
import rospy
import copy
import numpy as np
import cv2
from duckietown import DTROS
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import WheelsCmdStamped
from cv_bridge import CvBridge
import json
import time
import yaml

class Detector(DTROS):

    def __init__(self, node_name):

    	# Initialize the DTROS parent class
        super(Detector, self).__init__(node_name=node_name)

        # Get vehicle name
        self.veh_name = str(os.environ['VEHICLE_NAME'])

        # setting if object detection should activate emergency stops
        # if Duckie or Duckiebot are detected close-by
        self.objdet_emergency_stop = True

        # Initialize emergency stop object
        self.emergency_stop = False

        # Initialize FPS
        self.fps = str(0)
        
        # Initialize wheels command         
        self.msg_wheels_cmd = WheelsCmdStamped()

        # Use the kinematics calibration for the gain and trim
        self.parameters['~gain'] = None
        self.parameters['~trim'] = None
        self.parameters['~baseline'] = None
        self.parameters['~radius'] = None
        self.parameters['~k'] = None
        self.parameters['~limit'] = None

        # Set parameters using a robot-specific yaml file if such exists
        self.readParamFromFile()
        self.updateParameters()

        # Wait for the automatic gain control
        # of the camera to settle, before we stop it
        rospy.sleep(2)

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
        
        self.pub_wheels = rospy.Publisher("/{}/wheels_driver_node/wheels_cmd".format(self.veh_name), WheelsCmdStamped, queue_size=1)

    def callback(self, data):
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
                    self.emergency_stop = self.assess_emergency_stop(prediction)
                    if self.emergency_stop == True:
                        self.msg_wheels_cmd.vel_left = 0
                        self.msg_wheels_cmd.vel_right = 0
                        self.pub_wheels.publish(self.msg_wheels_cmd)

                    if self.emergency_stop == False:
                        correction_constant = 100/3
                        vel_left, vel_right = self.speedToCmd(0.15*correction_constant, 0.15*correction_constant)
                        self.msg_wheels_cmd.vel_left = vel_left
                        self.msg_wheels_cmd.vel_right = vel_right
                        self.pub_wheels.publish(self.msg_wheels_cmd)          	

        except Exception as e:
            correction_constant = 100/3
            vel_left, vel_right = self.speedToCmd(0.15*correction_constant, 0.15*correction_constant)
            self.msg_wheels_cmd.vel_left = vel_left
            self.msg_wheels_cmd.vel_right = vel_right
            self.pub_wheels.publish(self.msg_wheels_cmd)

        # Render FPS
        textfps = "Inference FPS: {}".format(self.fps)
        cv2.putText(orig, textfps, (10, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.25, (255, 255, 255), 1)

        # Publish image
        compressed_img = br.cv2_to_compressed_imgmsg(orig, dst_format='jpg')
        self.pub_coral_image.publish(compressed_img)
       
        if rospy.is_shutdown():
            self.rate = rospy.Rate(0.1)

            vel_left = 0
            vel_right = 0
            self.msg_wheels_cmd.vel_left = 0
            self.msg_wheels_cmd.vel_right = 0
            self.pub_wheels.publish(self.msg_wheels_cmd)
            rospy.sleep(10)

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
    	emergency_stop = False
        activate_emergency_stop = False
        deactivate_emergency_stop = True

        # we use centerpoint of lower edge of image for assessment, as ground projection works best for it
        u = int((prediction['startX'] + prediction['endX']) / 2)
        v = max(prediction['startY'], prediction['endY'])

        # prediction certainty should be higher than 60% for emergency stop
        if prediction['score'] > 0.60:
            # activate emergency stop if object is Duckie or Duckiebot
            if prediction['label'] == 'Duckie' or prediction['label'] == 'Duckiebot':
                # activate emergency stop if object is in horizontal center half and in lower quarter of image
                if v >= 2/3.0*self.res_h and 1 / 4.0 * self.res_w <= u <= 3 / 4.0 * self.res_w:
                    emergency_stop = True

        return emergency_stop

    def speedToCmd(self, speed_l, speed_r):
        """Applies the robot-specific gain and trim to the
        output velocities

        Applies the motor constant k to convert the deisred wheel speeds
        to wheel commands. Additionally, applies the gain and trim from
        the robot-specific kinematics configuration.

        Args:
            speed_l (:obj:`float`): Desired speed for the left
                wheel (e.g between 0 and 1)
            speed_r (:obj:`float`): Desired speed for the right
                wheel (e.g between 0 and 1)

        Returns:
            The respective left and right wheel commands that need to be
                packed in a `WheelsCmdStamped` message

        """

        # assuming same motor constants k for both motors
        k_r = self.parameters['~k']
        k_l = self.parameters['~k']

        # adjusting k by gain and trim
        k_r_inv = (self.parameters['~gain'] + self.parameters['~trim'])\
                  / k_r
        k_l_inv = (self.parameters['~gain'] - self.parameters['~trim'])\
                  / k_l

        # conversion from motor rotation rate to duty cycle
        u_r = speed_r * k_r_inv
        u_l = speed_l * k_l_inv

        # limiting output to limit, which is 1.0 for the duckiebot
        u_r_limited = self.trim(u_r,
                                -self.parameters['~limit'],
                                self.parameters['~limit'])
        u_l_limited = self.trim(u_l,
                                -self.parameters['~limit'],
                                self.parameters['~limit'])

        return u_l_limited, u_r_limited

    def readParamFromFile(self):
        """
        Reads the saved parameters from
        `/data/config/calibrations/kinematics/DUCKIEBOTNAME.yaml` or
        uses the default values if the file doesn't exist. Adjsuts
        the ROS paramaters for the node with the new values.

        """
        # Check file existence
        fname = self.getFilePath(self.veh_name)
        # Use the default values from the config folder if a
        # robot-specific file does not exist.
        if not os.path.isfile(fname):
            self.log("Kinematics calibration file %s does not "
                     "exist! Using the default file." % fname, type='warn')
            fname = self.getFilePath('default')

        with open(fname, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file)
            except yaml.YAMLError as exc:
                self.log("YAML syntax error. File: %s fname. Exc: %s"
                         %(fname, exc), type='fatal')
                rospy.signal_shutdown()
                return

        # Set parameters using value in yaml file
        if yaml_dict is None:
            # Empty yaml file
            return
        for param_name in ["gain", "trim", "baseline", "k", "radius", "limit"]:
            param_value = yaml_dict.get(param_name)
            if param_name is not None:
                rospy.set_param("~"+param_name, param_value)
            else:
                # Skip if not defined, use default value instead.
                pass

    def getFilePath(self, name):
        """
        Returns the path to the robot-specific configuration file,
        i.e. `/data/config/calibrations/kinematics/DUCKIEBOTNAME.yaml`.

        Args:
            name (:obj:`str`): the Duckiebot name

        Returns:
            :obj:`str`: the full path to the robot-specific
                calibration file

        """
        cali_file_folder = '/data/config/calibrations/kinematics/'
        cali_file = cali_file_folder + name + ".yaml"
        return cali_file

    def trim(self, value, low, high):
        """
        Trims a value to be between some bounds.

        Args:
            value: the value to be trimmed
            low: the minimum bound
            high: the maximum bound

        Returns:
            the trimmed value
        """

        return max(min(value, high), low)

    def onShutdown(self):
        """Shutdown procedure.
        Publishes a zero velocity command at shutdown."""

        # MAKE SURE THAT THE LAST WHEEL COMMAND YOU PUBLISH IS ZERO,
        # OTHERWISE YOUR DUCKIEBOT WILL CONTINUE MOVING AFTER
        # THE NODE IS STOPPED

        # PUT YOUR CODE HERE
        vel_left = 0
        vel_right = 0
        self.msg_wheels_cmd.vel_left = 0
        self.msg_wheels_cmd.vel_right = 0
        self.pub_wheels.publish(self.msg_wheels_cmd)
        super(Detector, self).onShutdown()               

if __name__ == '__main__':
    # Initialize the node
    br = CvBridge()
    Detector_node = Detector(node_name='detector')
    # Keep it spinning to keep the node alive
    rospy.spin()