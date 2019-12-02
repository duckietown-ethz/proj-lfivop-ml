import os
import csv
import cv2
import matplotlib.pyplot as plt
import yaml
import json
import numpy as np
import math
from pprint import pprint

from dt_locational_weights_base import DTLocationalWeightsBase


# locational weights generator
class DTLocationalWeightsGen(DTLocationalWeightsBase):

    def __init__(self):
        super().__init__()

    def weight_object_location(self, pixel):
        if pixel['v'] < self.horizon:
            weight = 1
        else:
            ground_point = self.pixel2ground(pixel)
            cylinder_point = self.world2cylinder(ground_point)

            weight_r = self.get_weight_r(cylinder_point['r'])
            weight_phi = self.get_weight_phi(cylinder_point['phi'])

            weight_candidate = weight_r * weight_phi

            # we won't let the weight drop below 1
            weight = max(weight_candidate, 1)

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
        h = self.camera_info['height']
        w = self.camera_info['width']

        locational_weights = np.zeros((h, w))
        for u in range(0, w, 1):
            for v in range(0, h, 1):
                pixel = {'u': u, 'v': v}
                weight = self.weight_object_location(pixel)
                locational_weights[v, u] = weight

        json_data = locational_weights.tolist()

        json_content = json.dumps(json_data)

        with open(self.locational_weights_path, 'w') as outfile:
            json.dump(json_data, outfile)

        print("Finished writing to: "+self.locational_weights_path)


if __name__ == '__main__':
    dt_locational_weights_generator = DTLocationalWeightsGen()
    # run class
    dt_locational_weights_generator.run()
