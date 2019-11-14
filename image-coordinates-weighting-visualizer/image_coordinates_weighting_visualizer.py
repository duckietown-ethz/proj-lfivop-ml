import os
import csv
import cv2
import matplotlib.pyplot as plt
import yaml
import json
import numpy as np
import math
from pprint import pprint


class ImageCoordinatesWeightingVisualizer:

    def __init__(self):
        self.path = os.path.dirname(os.path.realpath(__file__))

        self.calibrations_path = os.path.join(self.path, 'config', 'calibrations')

        camera_extrinsic_calib_name = 'default'
        camera_intrinsic_calib_name = 'default'

        self.camera_extrinsic_calib_path = os.path.join(self.calibrations_path, 'camera_extrinsic',
                                                        camera_extrinsic_calib_name + '.yaml')
        self.camera_intrinsic_calib_path = os.path.join(self.calibrations_path, 'camera_intrinsic',
                                                        camera_intrinsic_calib_name + '.yaml')

        self.img_path = os.path.join(self.path, 'test_data', 'b_CR_doort_frame01771.jpg')

        # load camera info (intrinsic)
        self.camera_info = self.load_camera_info()
        pprint(self.camera_info)

        # Load homography (extrinsic)
        self.H = self.load_homography()
        self.Hinv = np.linalg.inv(self.H)

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

    def pixel2world(self, pixel):
        uv_raw = np.array([pixel['u'], pixel['v']])
        # uv_raw = [uv_raw, 1]
        uv_raw = np.append(uv_raw, np.array([1]))
        world_arr = np.dot(self.H, uv_raw)
        world_point = {'x': world_arr[0], 'y': world_arr[1], 'z': world_arr[2]}
        return world_point

    def world2spherical(self, world_point):
        world_arr = np.array([world_point['x'], world_point['y'], world_point['z']])
        r = np.linalg.norm(world_arr)
        phi = np.arctan2(world_point['y'], world_point['x'])
        theta = np.arccos(world_point['z'] / r)

        spherical_point = {'r': r, 'phi': phi, 'theta': theta}
        return spherical_point

    def ground2pixel(self, ground_point):
        ground_arr = np.array([ground_point['x'], ground_point['y'], 1.0])
        image_point = np.dot(self.Hinv, ground_arr)
        image_point = image_point / image_point[2]

        pixel = {'u': image_point[0], 'v': image_point['v']}

        return pixel

    def run(self):
        print("run")
        raw_img = cv2.imread(self.img_path)

        rectified_img = self.rectify(raw_img)
        cv2.imshow('image', rectified_img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        pixel = {'u': 0, 'v': 0}
        ground_point = self.pixel2world(pixel)
        print("corner left-up has coordinates: (" + str(ground_point['x']) + ", " + str(ground_point['y']) + ", " + str(
            ground_point['z']) + ")")

        pixel = {'u': 480, 'v': 0}
        ground_point = self.pixel2world(pixel)
        print("corner left-down has coordinates: (" + str(ground_point['x']) + ", " + str(
            ground_point['y']) + ", " + str(ground_point['z']) + ")")

        pixel = {'u': 480, 'v': 640}
        ground_point = self.pixel2world(pixel)
        print("corner right-down has coordinates: (" + str(ground_point['x']) + ", " + str(
            ground_point['y']) + ", " + str(ground_point['z']) + ")")

        pixel = {'u': 0, 'v': 640}
        ground_point = self.pixel2world(pixel)
        print("corner right-up has coordinates: (" + str(ground_point['x']) + ", " + str(
            ground_point['y']) + ", " + str(ground_point['z']) + ")")

        v = 320
        u_range = range(0, 480, 10)
        r = []
        phi = []
        theta = []
        for u in u_range:
            pixel = {'u': u, 'v': v}
            world_point = self.pixel2world(pixel)
            pprint(world_point)
            spherical_point = self.world2spherical(world_point)
            r.append(spherical_point['r'])
            phi.append(spherical_point['phi']/math.pi*180)
            theta.append(spherical_point['theta']/math.pi*180)

        # plt.plot(u_range, r)
        plt.plot(u_range, phi)
        # plt.plot(u_range, theta)
        plt.show()
        pass

    def process_image(self, img, annotations):
        pass


if __name__ == '__main__':
    image_coordinates_weighting_visualizer = ImageCoordinatesWeightingVisualizer()
    # run node
    image_coordinates_weighting_visualizer.run()
