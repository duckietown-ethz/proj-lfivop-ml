import os
import csv
import cv2
import matplotlib.pyplot as plt
import yaml
import json
import numpy as np
import math
from pprint import pprint


class DTLocationalWeightsBase:

    def __init__(self):
        #  # initialize workdir paths
        self.workdir_path = os.environ['WORKDIR_PATH']
        self.config_workdir_path = os.environ['CONFIG_WORKDIR_PATH']
        self.data_workdir_path = os.environ['DATA_WORKDIR_PATH']
        self.raw_data_workdir_path = os.environ['RAW_DATA_WORKDIR_PATH']

        self.calibrations_path = os.path.join(os.environ['REPO_CONFIG_PATH'], 'calibrations')

        # hostname of the duckiebot where the camera calibrations where done
        duckiebot_hostname = os.environ['DUCKIEBOT_CALIBRATION_HOSTNAME']

        camera_extrinsic_calib_name = duckiebot_hostname
        camera_intrinsic_calib_name = duckiebot_hostname

        self.camera_extrinsic_calib_path = os.path.join(self.calibrations_path, 'camera_extrinsic',
                                                        camera_extrinsic_calib_name + '.yaml')
        self.camera_intrinsic_calib_path = os.path.join(self.calibrations_path, 'camera_intrinsic',
                                                        camera_intrinsic_calib_name + '.yaml')

        # load camera info (intrinsic)
        self.camera_info = self.load_camera_info()

        # Load homography (extrinsic)
        self.H = self.load_homography()
        self.Hinv = np.linalg.inv(self.H)

        # pprint(self.camera_info)

        # find horizon
        self.horizon = self.find_horizon()

        # set path for locational weights json
        locational_weights_filename = os.environ['LOCATIONAL_WEIGHTS_JSON_FILENAME']
        self.locational_weights_path = os.path.join(self.config_workdir_path, locational_weights_filename+'.json')

        # set weight parameters
        self.set_weight_parameters()

    def set_weight_parameters(self):
        # define weight parameters
        self.a_brake_max = 0.2 * 9.81  # m/s^2 --> 0.2*g
        self.v_max = 1  # m/s
        self.d_brake = 1 / 2 * self.v_max ** 2 / self.a_brake_max
        print("Calculated a braking distance of d_brake: " + str(self.d_brake) + "m")

        self.amplitude_weight_r = 3
        self.sigma_weight_r = 1.5 * self.d_brake
        self.amplitude_weight_phi = 2
        self.sigma_weight_phi = 60  # degree

        # Bottom center point
        BC_pixel = {'u': self.camera_info['width'] / 2, 'v': self.camera_info['height']}
        BC_ground = self.pixel2ground(BC_pixel)
        self.mu_weight_r = BC_ground['x']
        print('setted mu of weight r to: ' + str(self.mu_weight_r) + 'm')

    def load_camera_info(self):
        '''Load camera intrinsics'''
        with open(self.camera_intrinsic_calib_path, 'r') as stream:
            calib_data = yaml.safe_load(stream)

        cam_info = {'width': calib_data['image_width'], 'height': calib_data['image_height'],
                    'K': np.array(calib_data['camera_matrix']['data']).reshape((3, 3)),
                    'D': np.array(calib_data['distortion_coefficients']['data']).reshape((1, 5)),
                    'R': np.array(calib_data['rectification_matrix']['data']).reshape((3, 3)),
                    'P': np.array(calib_data['projection_matrix']['data']).reshape((3, 4)),
                    'distortion_model': calib_data['distortion_model']}
        return cam_info

    def load_homography(self):
        '''Load homography (extrinsic parameters)'''
        with open(self.camera_extrinsic_calib_path, 'r') as stream:
            data = yaml.safe_load(stream)

        return np.array(data['homography']).reshape((3, 3))

    def rectify(self, cv_image_raw):
        '''Undistort image'''
        cv_image_rectified = np.zeros(np.shape(cv_image_raw))
        mapx = np.ndarray(shape=(self.camera_info['height'], self.camera_info['width'], 1), dtype='float32')
        mapy = np.ndarray(shape=(self.camera_info['height'], self.camera_info['width'], 1), dtype='float32')
        mapx, mapy = cv2.initUndistortRectifyMap(self.camera_info['K'], self.camera_info['D'], self.camera_info['R'],
                                                 self.camera_info['P'],
                                                 (self.camera_info['width'], self.camera_info['height']),
                                                 cv2.CV_32FC1, mapx, mapy)
        return cv2.remap(cv_image_raw, mapx, mapy, cv2.INTER_CUBIC, cv_image_rectified)

    def pixel2ground(self, pixel):
        uv_raw = np.array([pixel['u'], pixel['v']])
        # uv_raw = [uv_raw, 1]
        uv_raw = np.append(uv_raw, np.array([1]))
        world_arr = np.dot(self.H, uv_raw)
        x = world_arr[0] / world_arr[2]
        y = world_arr[1] / world_arr[2]
        z = 0.0
        world_point = {'x': x, 'y': y, 'z': z}
        return world_point

    def world2spherical(self, world_point):
        world_arr = np.array([world_point['x'], world_point['y'], world_point['z']])
        r = np.linalg.norm(world_arr)
        phi = np.arctan2(world_point['y'], world_point['x'])
        theta = np.arccos(world_point['z'] / r)

        spherical_point = {'r': r, 'phi': phi, 'theta': theta}
        return spherical_point

    def world2cylinder(self, world_point):
        world_arr = np.array([world_point['x'], world_point['y'], world_point['z']])
        r = np.linalg.norm(world_arr)
        phi = np.arctan2(world_point['y'], world_point['x'])
        z = world_point['z']

        cylinder_point = {'r': r, 'phi': phi, 'z': z}

        return cylinder_point

    def ground2pixel(self, ground_point):
        ground_arr = np.array([ground_point['x'], ground_point['y'], 1.0])
        image_point = np.dot(self.Hinv, ground_arr)
        image_point = image_point / image_point[2]

        pixel = {'u': image_point[0], 'v': image_point[1]}

        return pixel

    def find_horizon(self):
        ground_point_horizon = {'x': 10000, 'y': 0, 'z': 0}

        pixel = self.ground2pixel(ground_point_horizon)
        print('Horizon is at v: ' + str(pixel['v']) + 'px')

        return pixel['v']
