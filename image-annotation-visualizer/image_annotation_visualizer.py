import os
import csv
import cv2
import json
import numpy as np

from pprint import pprint

class ImageAnnotationVisualizer:

    def __init__(self):
        self.path = os.path.dirname(os.path.realpath(__file__))

        self.data_folder = os.path.join(os.path.dirname(os.path.dirname(self.path)),'data/dt-proj-lfivop-ml_annotated_images_18/')
        self.annotation_csv_path = self.data_folder+'Annotations.csv'
        self.original_images_folder = self.data_folder+'images/'
        self.annotated_images_folder = self.data_folder+'images_annotated/'

        self.annotation_color = (0, 255, 0)
    def run(self):
        with open(self.annotation_csv_path) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            line_count = 0
            for row in csv_reader:
                if line_count == 0:
                    print(f'Column names are {", ".join(row)}')
                    line_count += 1
                else:
                    image_original_filename = row[6]
                    print(f'process image \t{image_original_filename}')
                    image_path = self.original_images_folder+image_original_filename
                    img = cv2.imread(image_path)

                    annotations_json = row[9]
                    annotations = json.loads(annotations_json)

                    annotated_img = self.process_image(img, annotations)

                    annotated_img_path = self.annotated_images_folder+image_original_filename
                    if not cv2.imwrite(annotated_img_path,annotated_img):
                        raise Exception("Could not write image")

                    line_count += 1
            print(f'Processed {line_count} lines.')

    def process_image(self, img, annotations):
        h, w, channels = img.shape

        annotated_img = img
        i = 0
        for i in range(len(annotations)):
            try:
                annotation = annotations[i]
                p1 = annotation['p1']
                p2 = annotation['p2']
                x1 = int(np.floor(w*float(p1['x'])))
                y1 = int(np.floor(h*float(p1['y'])))
                x2 = int(np.floor(w*float(p2['x'])))
                y2 = int(np.floor(h*float(p2['y'])))

                annotated_img = cv2.rectangle(annotated_img, (x1,y1), (x2,y2), self.annotation_color,2)

                label = str(annotation['label'])

                # org
                label_x = min(max(x1,0),w)
                label_y = min(max(y1-5,0),h)

                # font
                font_face = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 0.4

                cv2.putText(img, label, (label_x, label_y), font_face, font_scale, self.annotation_color)
            except Exception as e:
                pprint('New exception: '+str(e))
        print(f'Processed {i} annotations.')
        return annotated_img

if __name__ == '__main__':
    image_annotation_visualizer = ImageAnnotationVisualizer()
    # run node
    image_annotation_visualizer.run()