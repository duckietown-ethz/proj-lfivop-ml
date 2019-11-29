# followed from https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/using_your_own_dataset.md
import tensorflow as tf
import os
import csv
import cv2
from object_detection.utils import dataset_util
import json
import numpy as np

data_dir_default = os.path.join(os.environ.get('TF_WORKDIR_PATH'), 'data')
raw_data_dir_default = os.path.join(os.environ.get('TF_WORKDIR_PATH'), 'raw_data')
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


class DtDatasetPreparation:
    def __init__(self, writer_train, writer_val, writer_test):
        self.raw_data_dir = FLAGS.raw_data_dir
        self.annotation_csv_path = os.path.join(self.raw_data_dir, 'Annotations.csv')
        self.raw_images_dir = os.path.join(self.raw_data_dir, 'images')

        self.writer_train = writer_train
        self.writer_val = writer_val
        self.writer_test = writer_test

        self.dt_object_classes = {'Duckie': 1,
                                  'Duckiebot': 2,
                                  'Traffic light': 3,
                                  'QR code': 4,
                                  'Stop sign': 5,
                                  'Intersection sign': 6,
                                  'Signal sign': 7, }

        self.val_1_of_n_images = int(FLAGS.val_1_of_n_images)
        self.test_1_of_n_images = int(FLAGS.test_1_of_n_images)
        print(self.val_1_of_n_images)
        print(self.test_1_of_n_images)

    def run(self):
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
        x1, x2, y1, y2, label, label_id = [], [], [], [], [], []
        annotated_img = img
        i = 0
        for i in range(len(annotations)):
            try:
                annotation = annotations[i]
                p1 = annotation['p1']
                p2 = annotation['p2']
                x1.append(int(np.floor(w * float(p1['x']))))
                y1.append(int(np.floor(h * float(p1['y']))))
                x2.append(int(np.floor(w * float(p2['x']))))
                y2.append(int(np.floor(h * float(p2['y']))))
                label.append(str(annotation['label']).encode('utf8'))
                label_id.append(self.dt_object_classes[str(annotation['label'])])
                # print(self.duckie_classes[str(annotation['label'])], str(annotation['label']))
            except Exception as e:
                print('New exception: ' + str(e))
        print(f'Processed {i} annotations.')
        return x1, x2, y1, y2, label, label_id


def main(_):
    writer_train = tf.io.TFRecordWriter(train_record_path)
    writer_val = tf.io.TFRecordWriter(val_record_path)
    writer_test = tf.io.TFRecordWriter(test_record_path)
    image_annotation_to_tfr = DtDatasetPreparation(writer_train, writer_val, writer_test)
    # run node
    image_annotation_to_tfr.run()
    writer_train.close()
    writer_val.close()
    writer_test.close()


if __name__ == '__main__':
    tf.compat.v1.app.run()
