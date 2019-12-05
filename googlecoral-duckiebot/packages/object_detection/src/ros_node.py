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
import math

from sensor_msgs.msg import CameraInfo
from duckietown_msgs.msg import (Pixel, Vector2D)
from image_geometry import PinholeCameraModel
from geometry_msgs.msg import Point
from duckietown_utils.path_utils import get_ros_package_path
from duckietown_utils.yaml_wrap import (yaml_load_file, yaml_write_to_file)
from duckietown_utils import (logger, get_duckiefleet_root)


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

        # our object detection works with raw, non-rectified images
        self.rectified_input = False

        # load camera info
        self.pcm_ = PinholeCameraModel()
        camera_info = self.load_camera_info()
        self.pcm_.width = camera_info.width
        self.pcm_.height = camera_info.height
        self.pcm_.K = camera_info.K
        self.pcm_.D = camera_info.D
        self.pcm_.R = camera_info.R
        self.pcm_.P = camera_info.P

        # Load homography
        self.H = self.load_homography()
        self.Hinv = np.linalg.inv(self.H)

        # find horizon
        self.horizon = self.find_horizon()

        # define parameters for emergency stop
        self.a_brake_max = 0.2 * 9.81  # m/s^2 --> 0.2*g
        self.v_max = 1  # m/s
        self.d_brake = 1 / 2 * self.v_max ** 2 / self.a_brake_max
        rospy.loginfo("Calculated a braking distance of d_brake: " + str(self.d_brake) + "m")

        # Bottom center point
        BC_pixel = Pixel()
        BC_pixel.u = self.camera_info['width'] / 2
        BC_pixel.v = self.camera_info['height']
        BC_ground = self.pixel2ground(BC_pixel)
        origin_r = BC_ground['x']
        rospy.loginfo('setted mu of weight r to: ' + str(origin_r) + 'm')

        self.threshold_emergency_stop_r = origin_r + 1.5 * self.d_brake
        self.threshold_emergency_stop_phi = 60 / 180 * math.pi  # [rad]

    def load_camera_info(self):
        '''Load camera intrinsics'''
        filename = (os.environ['DUCKIEFLEET_ROOT'] + "/calibrations/camera_intrinsic/" + self.veh_name + ".yaml")
        if not os.path.isfile(filename):
            logger.warn("no intrinsic calibration parameters for {}, trying default".format(self.veh_name))
            filename = (os.environ['DUCKIEFLEET_ROOT'] + "/calibrations/camera_intrinsic/default.yaml")
            if not os.path.isfile(filename):
                logger.error("can't find default either, something's wrong")
        calib_data = yaml_load_file(filename)
        #     logger.info(yaml_dump(calib_data))
        cam_info = CameraInfo()
        cam_info.width = calib_data['image_width']
        cam_info.height = calib_data['image_height']
        cam_info.K = np.array(calib_data['camera_matrix']['data']).reshape((3, 3))
        cam_info.D = np.array(calib_data['distortion_coefficients']['data']).reshape((1, 5))
        cam_info.R = np.array(calib_data['rectification_matrix']['data']).reshape((3, 3))
        cam_info.P = np.array(calib_data['projection_matrix']['data']).reshape((3, 4))
        cam_info.distortion_model = calib_data['distortion_model']
        logger.info(
            "Loaded camera calibration parameters for {} from {}".format(self.veh_name, os.path.basename(filename)))
        return cam_info

    def load_homography(self):
        '''Load homography (extrinsic parameters)'''
        filename = (get_duckiefleet_root() + "/calibrations/camera_extrinsic/" + self.veh_name + ".yaml")
        if not os.path.isfile(filename):
            logger.warn("no extrinsic calibration parameters for {}, trying default".format(self.veh_name))
            filename = (get_duckiefleet_root() + "/calibrations/camera_extrinsic/default.yaml")
            if not os.path.isfile(filename):
                logger.error("can't find default either, something's wrong")
            else:
                data = yaml_load_file(filename)
        else:
            rospy.loginfo("Using extrinsic calibration of " + self.veh_name)
            data = yaml_load_file(filename)
        logger.info("Loaded homography for {}".format(os.path.basename(filename)))
        return np.array(data['homography']).reshape((3, 3))

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
        pixel = Pixel()
        pixel.u = int((prediction['startX'] + prediction['endX']) / 2)
        pixel.v = max(prediction['startY'], prediction['endY'])

        # scale back to original camera image size
        pixel_scaled = Pixel()
        pixel_scaled.u = int(pixel.u * self.pcm_.width / self.res_w)
        pixel_scaled.v = int(pixel.v * self.pcm_.height / self.res_h)

        # prediction certainty should be higher than 75% for emergency stop
        if prediction['score'] > 0.75:
            # activate emergency stop if object is Duckie or Duckiebot
            if prediction['label'] == 'Duckie' or prediction['label'] == 'Duckiebot':
                # only project to ground, if pixel below horizon
                if pixel_scaled.v > self.horizon:
                    ground_point = self.pixel2ground(pixel)
                    cylinder_point = self.ground2cylinder(ground_point)

                    if cylinder_point['r'] < self.threshold_emergency_stop_r \
                            and abs(cylinder_point['phi']) < self.threshold_emergency_stop_phi:
                        activate_emergency_stop = True
                        deactivate_emergency_stop = False

                    # activate emergency stop if object is in horizontal center half and in lower quarter of image
                    # if pixel.v >= 3 / 4.0 * self.res_h and 1 / 4.0 * self.res_w <= pixel.u <= 3 / 4.0 * self.res_w:
                    #     activate_emergency_stop = True
                    #     deactivate_emergency_stop = False

        if activate_emergency_stop is True or deactivate_emergency_stop is True:
            emergency_stop_msg = BoolStamped()
            if activate_emergency_stop is True:
                emergency_stop_msg.data = True
                self.emergency_stop_activated = True

            if deactivate_emergency_stop is True:
                emergency_stop_msg.data = False
                self.emergency_stop_activated = False

            self.pub_emergency_stop.publish(emergency_stop_msg)

    def pixel2ground(self, pixel):
        uv_raw = np.array([pixel.u, pixel.v])
        if not self.rectified_input:
            uv_raw = self.pcm_.rectifyPoint(uv_raw)
        # uv_raw = [uv_raw, 1]
        uv_raw = np.append(uv_raw, np.array([1]))
        ground_point = np.dot(self.H, uv_raw)
        point = Point()
        x = ground_point[0]
        y = ground_point[1]
        z = ground_point[2]
        point.x = x / z
        point.y = y / z
        point.z = 0.0
        return point

    def ground2pixel(self, point):
        ground_point = np.array([point.x, point.y, 1.0])
        image_point = np.dot(self.Hinv, ground_point)
        image_point = image_point / image_point[2]

        pixel = Pixel()
        if not self.rectified_input:
            distorted_pixel = self.pcm_.project3dToPixel(image_point)
            pixel.u = distorted_pixel[0]
            pixel.v = distorted_pixel[1]
        else:
            pixel.u = image_point[0]
            pixel.v = image_point[1]

        return pixel

    def ground2cylinder(self, ground_point):
        world_arr = np.array([ground_point.x, ground_point.y, ground_point.z])
        r = np.linalg.norm(world_arr)
        phi = np.arctan2(ground_point.y, ground_point.x)
        z = ground_point.z

        cylinder_point = {'r': r, 'phi': phi, 'z': z}

        return cylinder_point

    def find_horizon(self):
        ground_point_horizon = Point()
        ground_point_horizon.x = 10000
        ground_point_horizon.y = 0
        ground_point_horizon.z = 0

        pixel = self.ground2pixel(ground_point_horizon)
        rospy.loginfo('Horizon is at v: ' + str(pixel.v) + 'px')

        return pixel.v


if __name__ == '__main__':
    # Initialize the node
    br = CvBridge()
    Detector_node = Detector(node_name='detector')
    # Keep it spinning to keep the node alive
    rospy.spin()
