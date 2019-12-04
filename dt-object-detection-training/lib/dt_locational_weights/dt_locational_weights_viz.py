from absl import app
from absl import flags
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


FLAGS = flags.FLAGS
flags.DEFINE_string('image_filename', 'b_CR_doort_frame01771', 'State the name of the raw_image which you want to '
                                                             'visualize (in /workdir/raw_data/images/{image_filename}.jpg)')


# locational weights vizualizer
class DTLocationalWeightsViz(DTLocationalWeightsBase):

    def __init__(self):
        super().__init__()

        # load locational weights configuration
        with open(self.locational_weights_path, 'r') as file:
            json_data = json.load(file)

        self.locational_weights = np.array(json_data)

        pprint(FLAGS.image_filename)

        self.img_path = os.path.join(self.raw_data_workdir_path, 'images', FLAGS.image_filename+'.jpg')
        self.overlayed_img_path = os.path.join(self.data_workdir_path, 'images', FLAGS.image_filename+'_lw_overlayed.jpg')

    def run(self):
        print("Visualizing locational weights of raw image: "+str(self.img_path))
        raw_img = cv2.imread(self.img_path)

        rectified_img = self.rectify(raw_img)

        overlayed_img, weights_img = self.process_image(rectified_img)

        cv2.imwrite(self.overlayed_img_path, overlayed_img)
        print("Wrote overlayed image with locational weights to: "+str(self.overlayed_img_path))

        # self.show_images(raw_img, rectified_img, weights_img, overlayed_img)

    def process_image(self, rectified_image):
        h = rectified_image.shape[0]
        w = rectified_image.shape[1]

        norm_factor = 255 / (self.amplitude_weight_r * self.amplitude_weight_phi)

        # weights_img = np.zeros((h, w))
        # weights_img = cv2.cvtColor(rectified_image, cv2.COLOR_BGR2GRAY)
        # for u in range(0, w, 1):
        #     for v in range(0, h, 1):
        #         pixel = {'u': u, 'v': v}
        #         weight = norm_factor * self.weight_object_location(pixel)
        #         weights_img[v, u] = weight
        weights_img = np.array(norm_factor*self.locational_weights, dtype=np.uint8)

        # apply the overlay
        overlay = cv2.applyColorMap(weights_img, cv2.COLORMAP_JET)
        alpha = 0.6
        overlayed_img = rectified_image.copy()
        cv2.addWeighted(overlay, alpha, rectified_image, 1 - alpha,
                        0, overlayed_img)

        return overlayed_img, weights_img

    def show_images(self, raw_img, rectified_img, weights_img, overlayed_img):
        cv2.imshow('raw_img', raw_img)
        cv2.imshow('rectified_image', rectified_img)
        cv2.imshow('weights_img', weights_img)
        cv2.imshow('overlayed_img', overlayed_img)

        key = cv2.waitKey(30000)  # pauses for 3 seconds before fetching next image
        if key == 27:  # if ESC is pressed, exit loop
            cv2.destroyAllWindows()

def main(_):
    locational_weight_visualizer = DTLocationalWeightsViz()
    # run class
    locational_weight_visualizer.run()

if __name__ == '__main__':
    app.run(main)
