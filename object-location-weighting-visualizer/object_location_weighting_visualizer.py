import os
import csv
import cv2
import matplotlib.pyplot as plt
import yaml
import json
import numpy as np
import math
from pprint import pprint


class ObjectLocationWeightingVisualizer:

    def __init__(self):
        self.path = os.path.dirname(os.path.realpath(__file__))

        self.calibrations_path = os.path.join(self.path, 'config', 'calibrations')

        camera_extrinsic_calib_name = 'maxicar'
        camera_intrinsic_calib_name = 'maxicar'

        self.camera_extrinsic_calib_path = os.path.join(self.calibrations_path, 'camera_extrinsic',
                                                        camera_extrinsic_calib_name + '.yaml')
        self.camera_intrinsic_calib_path = os.path.join(self.calibrations_path, 'camera_intrinsic',
                                                        camera_intrinsic_calib_name + '.yaml')

        self.img_name = 'b_CR_doort_frame01771'
        self.img_path = os.path.join(self.path, 'test_data', self.img_name+'.jpg')
        self.overlayed_img_path = os.path.join(self.path, 'test_data', self.img_name+'_overlayed.jpg')

        # load camera info (intrinsic)
        self.camera_info = self.load_camera_info()

        # Load homography (extrinsic)
        self.H = self.load_homography()
        self.Hinv = np.linalg.inv(self.H)

        # pprint(self.camera_info)

        # find horizon
        self.horizon = self.find_horizon()

        # define weight parameterst
        self.set_weight_parameters()

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

    def set_weight_parameters(self):
        self.a_brake_max = 0.2*9.81 # m/s^2 --> 0.2*g
        self.v_max = 1 # m/s
        self.d_brake = 1/2*self.v_max**2/self.a_brake_max
        print("Calculated a braking distance of d_brake: "+str(self.d_brake)+"m")

        # Bottom center point
        BC_pixel = {'u': self.camera_info['width']/2, 'v': self.camera_info['height']}
        BC_ground = self.pixel2ground(BC_pixel)
        self.mu_weight_r = BC_ground['x']
        print('setted mu of weight r to: '+str(self.mu_weight_r)+'m')

        self.amplitude_weight_r = 3
        self.sigma_weight_r = 1.5*self.d_brake
        self.amplitude_weight_phi = 2
        self.sigma_weight_phi = 60 # degree

    def weight_object_location(self, pixel):
        if pixel['v'] < self.horizon:
            weight = 1
        else:
            ground_point = self.pixel2ground(pixel)
            cylinder_point = self.world2cylinder(ground_point)

            weight_r = self.get_weight_r(cylinder_point['r'])
            weight_phi = self.get_weight_phi(cylinder_point['phi'])

            weight = weight_r * weight_phi

        return weight

    def get_weight_r(self, r):
        weight = self.amplitude_weight_r * math.exp(-(float(r - self.mu_weight_r) / self.sigma_weight_r) ** 2 / 2.0)

        return weight

    def get_weight_phi(self, phi):
        # highest weight at phi = 0°
        mu = 0

        phi_deg = phi / math.pi * 180

        weight = self.amplitude_weight_phi * math.exp(
            -(float(phi_deg - mu) / self.sigma_weight_phi) ** 2 / 2.0)

        return weight

    def run(self):
        raw_img = cv2.imread(self.img_path)
        cv2.imshow('raw_img', raw_img)

        h = raw_img.shape[0]
        w = raw_img.shape[1]

        rectified_img = self.rectify(raw_img)
        cv2.imshow('rectified_image', rectified_img)

        u = w/2
        v_range = range(0, h, 10)
        x = []
        r = []
        phi = []
        weight = []
        for v in v_range:
            pixel = {'u': u, 'v': v}
            world_point = self.pixel2ground(pixel)
            cylinder_point = self.world2cylinder(world_point)
            x.append(world_point['x'])
            r.append(cylinder_point['r'])
            phi.append(cylinder_point['phi'] / math.pi * 180)
            weight.append(self.weight_object_location(pixel))

        # plt.plot(v_range, weight)
        # plt.plot(v_range, r)
        # plt.plot(v_range, phi)
        # plt.show()

        overlayed_img = self.process_image(rectified_img)
        cv2.imwrite(self.overlayed_img_path, overlayed_img)
        cv2.imshow('overlayed_img', overlayed_img)

        key = cv2.waitKey(30000)#pauses for 3 seconds before fetching next image
        if key == 27:#if ESC is pressed, exit loop
            cv2.destroyAllWindows()

    def process_image(self, rectified_image):
        h = rectified_image.shape[0]
        w = rectified_image.shape[1]

        norm_factor = 255 / (self.amplitude_weight_r * self.amplitude_weight_phi)

        # weights_img = np.zeros((h, w))
        weights_img = cv2.cvtColor(rectified_image, cv2.COLOR_BGR2GRAY)
        for u in range(0, w, 1):
            for v in range(0, h, 1):
                pixel = {'u': u, 'v': v}
                weight = norm_factor * self.weight_object_location(pixel)
                weights_img[v, u] = weight

        cv2.imshow('weights_img', weights_img)
        # apply the overlay
        overlay = cv2.applyColorMap(weights_img, cv2.COLORMAP_JET)
        alpha = 0.6
        overlayed_img = rectified_image.copy()
        cv2.addWeighted(overlay, alpha, rectified_image, 1 - alpha,
                        0, overlayed_img)

        return overlayed_img


if __name__ == '__main__':
    object_location_weighting_visualizer = ObjectLocationWeightingVisualizer()
    # run node
    object_location_weighting_visualizer.run()
