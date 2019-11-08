#followed from https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/using_your_own_dataset.md
import tensorflow as tf
import os
import csv
import cv2
from object_detection.utils import dataset_util
import json
import numpy as np

flags = tf.app.flags
flags.DEFINE_string('output_path', '', 'Path to output TFRecord')
flags.DEFINE_string('data_folder_path', '', 'Path to data folder having Annotations.csn and images in images folder')
FLAGS = flags.FLAGS

class ImageAnnotationToTFR():
    def __init__(self, writer):
        self.data_folder = FLAGS.data_folder_path
        self.annotation_csv_path = self.data_folder+'Annotations.csv'
        self.original_images_folder = self.data_folder+'images/'
        self.writer = writer
        self.duckie_classes = {'Duckie':0,
			'Duckiebot':1,
			'Traffic light':2,
			'QR code':3,
			'Stop sign':4,
			'Intersection sign':5,
			'Traffic light':6,
			'Signal sign':7,}
        
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
                    with tf.gfile.GFile(image_path, 'rb') as fid:
                        encoded_image_data = fid.read()
                    line_count += 1
                    height = 480
                    width = 640
                    filename = image_original_filename.split('.')[0].encode('utf8')
                    image_format = b'jpg'
                    xmins, xmaxs, ymins, ymaxs, classes_text, classes = self.process_image(img, annotations)
                    tf_example = tf.train.Example(features=tf.train.Features(feature={
                    	'image/height': dataset_util.int64_feature(height),
                    	'image/width': dataset_util.int64_feature(width),
                    	'image/filename': dataset_util.bytes_feature(filename),
                    	'image/source_id': dataset_util.bytes_feature(filename),
                    	'image/encoded': dataset_util.bytes_feature(encoded_image_data),
                    	'image/format': dataset_util.bytes_feature(image_format),
                    	'image/object/bbox/xmin': dataset_util.float_list_feature(xmins),
                    	'image/object/bbox/xmax': dataset_util.float_list_feature(xmaxs),
                    	'image/object/bbox/ymin': dataset_util.float_list_feature(ymins),
                    	'image/object/bbox/ymax': dataset_util.float_list_feature(ymaxs),
                    	'image/object/class/text': dataset_util.bytes_list_feature(classes_text),
                    	'image/object/class/label': dataset_util.int64_list_feature(classes),}))
                    print(f'Processed {line_count} lines.')
                    self.writer.write(tf_example.SerializeToString())
            
    def process_image(self, img, annotations):
        h, w, channels = img.shape
        x1, x2, y1, y2, label, label_id = [], [], [], [], [], []
        annotated_img = img
        i = 0
        for i in range(len(annotations)):
            try:
                annotation = annotations[i]
                p1 = annotation['p1']
                p2 = annotation['p2']
                x1.append(int(np.floor(w*float(p1['x']))))
                y1.append(int(np.floor(h*float(p1['y']))))
                x2.append(int(np.floor(w*float(p2['x']))))
                y2.append(int(np.floor(h*float(p2['y']))))
                label.append(str(annotation['label']).encode('utf8'))
                label_id.append(self.duckie_classes[str(annotation['label'])])
                #print(self.duckie_classes[str(annotation['label'])], str(annotation['label']))
            except Exception as e:
                print('New exception: '+str(e))
        print(f'Processed {i} annotations.')
        return x1, x2, y1, y2, label, label_id

def main(_):
    writer = tf.python_io.TFRecordWriter(FLAGS.output_path)
    image_annotation_to_tfr = ImageAnnotationToTFR(writer)
    # run node
    image_annotation_to_tfr.run()
    writer.close()

if __name__ == '__main__':
    tf.app.run()
