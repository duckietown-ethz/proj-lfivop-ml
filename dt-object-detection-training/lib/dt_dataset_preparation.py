# followed from https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/using_your_own_dataset.md
import tensorflow as tf
import os
import csv
import cv2
from object_detection.utils import dataset_util
import json
import numpy as np
from pprint import pprint

data_dir_default = os.environ.get('DATA_WORKDIR_PATH')
raw_data_dir_default = os.environ.get('RAW_DATA_WORKDIR_PATH')
val_1_of_n_images_default = os.environ.get('VAL_1_OF_N_IMAGES')
test_1_of_n_images_default = os.environ.get('VAL_1_OF_N_IMAGES')

flags = tf.app.flags
flags.DEFINE_string('data_dir', data_dir_default, 'Path of directory to output TFRecord')
flags.DEFINE_string('raw_data_dir', raw_data_dir_default,
                    'Path to data folder having Annotations.csv and images in images folder')
flags.DEFINE_string('val_1_of_n_images', val_1_of_n_images_default,
                    'Determines the ratio of images used for validation to total images')
flags.DEFINE_string('test_1_of_n_images', test_1_of_n_images_default,
                    'Determines the ratio of images used for testing to total images')
FLAGS = flags.FLAGS


train_record_path = os.path.join(FLAGS.data_dir, 'dt_mscoco_train.record')
val_record_path = os.path.join(FLAGS.data_dir, 'dt_mscoco_val.record')
test_record_path = os.path.join(FLAGS.data_dir, 'dt_mscoco_test.record')


class DTDatasetPreparation:
    def __init__(self, writer_train, writer_val, writer_test):
        self.raw_data_dir = FLAGS.raw_data_dir
        self.annotation_csv_path = os.path.join(self.raw_data_dir, 'Annotations.csv')
        self.raw_images_dir = os.path.join(self.raw_data_dir, 'images')

        self.writer_train = writer_train
        self.writer_val = writer_val
        self.writer_test = writer_test

        self.label_map_path = os.path.join(FLAGS.data_dir, 'dt_mscoco_label_map.pbtxt')

        self.dt_object_classes = {'Duckie': 1,
                                  'Duckiebot': 2,
                                  'Traffic light': 3,
                                  'QR code': 4,
                                  'Stop sign': 5,
                                  'Intersection sign': 6,
                                  'Signal sign': 7, }

        self.val_1_of_n_images = int(FLAGS.val_1_of_n_images)
        self.test_1_of_n_images = int(FLAGS.test_1_of_n_images)

    def run(self):
        self.write_label_map()
        self.write_TF_record()

    def write_label_map(self):
        # write label_map.pbtxt file
        end = '\n'
        s = ' '
        out = ''
        for name in self.dt_object_classes:
            ID = self.dt_object_classes[name]

            out += 'item' + s + '{' + end
            out += s*2 + 'id:' + ' ' + (str(ID)) + end
            out += s*2 + 'name:' + ' ' + '\"' + name + '\"' + end
            out += '}' + end*2

        with open(self.label_map_path, 'w') as f:
            f.write(out)

    def write_TF_record(self):
        # write TensorFlow .record file
        with open(self.annotation_csv_path) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            line_count = 0
            i = -1
            n_train = 0
            n_val = 0
            n_test = 0
            for row in csv_reader:
                if line_count == 0:
                    print(f'Column names are {", ".join(row)}')
                    line_count += 1
                else:
                    i = i+1
                    image_original_filename = row[6]
                    print(f'process image \t{image_original_filename}')
                    image_path = os.path.join(self.raw_images_dir, image_original_filename)
                    img = cv2.imread(image_path)
                    annotations_json = row[9]
                    annotations = json.loads(annotations_json)
                    with tf.io.gfile.GFile(image_path, 'rb') as fid:
                        encoded_image_data = fid.read()
                    line_count += 1

                    height, width, channel = img.shape

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
                        'image/object/class/label': dataset_util.int64_list_feature(classes), }))
                    tf_example_serialized = tf_example.SerializeToString()
                    print(f'Processed {line_count} lines.')
                    if i % self.val_1_of_n_images == 0:
                        # validation dataset
                        n_val += 1
                        self.writer_val.write(tf_example_serialized)
                    elif (i+1) % self.test_1_of_n_images == 0:
                        # test dataset
                        n_test += 1
                        self.writer_test.write(tf_example_serialized)
                    else:
                        # training dataset
                        n_train += 1
                        self.writer_train.write(tf_example_serialized)

            print(f'Finished processing dataset.')
            print(f'Written {n_train} images to training dataset.')
            print(f'Written {n_val} images to validation dataset.')
            print(f'Written {n_test} images to testing dataset.')

    def process_image(self, img, annotations):
        h, w, channels = img.shape
        x_mins, x_maxs, y_mins, y_maxs, label, label_id = [], [], [], [], [], []
        annotated_img = img
        i = 0
        for i in range(len(annotations)):
            try:
                annotation = annotations[i]
                p1 = annotation['p1']
                p2 = annotation['p2']
                x1 = float(p1['x'])
                y1 = float(p1['y'])
                x2 = float(p2['x'])
                y2 = float(p2['y'])

                x_min = min(x1, x2) # List of normalized left x coordinates in bounding box (1 per box)
                x_max = max(x1, x2) # List of normalized right x coordinates in bounding box (1 per box)
                y_min = min(y1, y2) # List of normalized top y coordinates in bounding box (1 per box)
                y_max = max(y1, y2) # List of normalized bottom y coordinates in bounding box (1 per box)

                # just to make sure
                if x_min >= 0 and x_max <= 1 and y_min >= 0 and y_max <= 1:
                    x_mins.append(x_min)
                    x_maxs.append(x_max)
                    y_mins.append(y_min)
                    y_maxs.append(y_max)
                    label.append(str(annotation['label']).encode('utf8'))
                    label_id.append(self.dt_object_classes[str(annotation['label'])])
                else:
                    raise Exception("x_min, x_max, y_min or y_max are not allowed")
                # print(self.duckie_classes[str(annotation['label'])], str(annotation['label']))
            except Exception as e:
                print('New exception: ' + str(e))
        print(f'Processed {i} annotations.')
        return x_mins, x_maxs, y_mins, y_maxs, label, label_id


def main(_):
    writer_train = tf.io.TFRecordWriter(train_record_path)
    writer_val = tf.io.TFRecordWriter(val_record_path)
    writer_test = tf.io.TFRecordWriter(test_record_path)
    image_annotation_to_tfr = DTDatasetPreparation(writer_train, writer_val, writer_test)
    # run node
    image_annotation_to_tfr.run()
    writer_train.close()
    writer_val.close()
    writer_test.close()


if __name__ == '__main__':
    tf.compat.v1.app.run()
