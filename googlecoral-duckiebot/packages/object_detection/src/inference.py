#!/usr/bin/env python
from edgetpu.detection.engine import DetectionEngine
from PIL import Image
import numpy as np
import json

# loop over the class labels file
labels = {}
for row in open("/code/catkin_ws/src/coral/packages/object_detection/src/mobilenet_ssd_v2/coco_labels.txt"):
	# unpack the row and update the labels dictionary
	(classID, label) = row.strip().split(maxsplit=1)
	labels[int(classID)] = label.strip()

# load the Google Coral object detection model
print("[INFO] loading Coral model...")
model = DetectionEngine("/code/catkin_ws/src/coral/packages/object_detection/src/mobilenet_ssd_v2/mobilenet_ssd_v2_coco_quant_postprocess_edgetpu.tflite")

while True:
	try:
		frame = np.load('/code/catkin_ws/src/coral/packages/object_detection/src/temp_frame.npy')

		frame = Image.fromarray(frame)
		results = model.detect_with_image(frame, threshold=0.3,	keep_aspect_ratio=True, relative_coord=False)

		d = {}
		l = []

		for r in results:
			box = r.bounding_box.flatten().astype("int")
			(startX, startY, endX, endY) = box
			label = labels[r.label_id]
			d['startX'] = int(startX)
			d['startY'] = int(startY)
			d['endX'] = int(endX)
			d['endY'] = int(endY)
			d['label'] = label
			d['score'] = float(r.score)
			l.append(d)

		with open('/code/catkin_ws/src/coral/packages/object_detection/src/prediction', 'w') as fout:
			json.dump(l, fout)

	except (ValueError, FileNotFoundError):
		pass

	


