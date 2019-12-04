#!/usr/bin/env python
from edgetpu.detection.engine import DetectionEngine
from PIL import Image
import numpy as np
import json
import time

# loop over the class labels file
labels = {}
colors = {}
for row in open("/code/catkin_ws/src/coral/packages/object_detection/src/duckie_labels_colors.txt"):
	# unpack the row and update labels and colors dictionaries
	(classID, description) = row.strip().split(maxsplit=1)
	(label,color) = description.strip().split(";")
	labels[int(classID)] = label.strip()
	colors[int(classID)] = color.strip()

# load the Google Coral object detection model
print("[INFO] loading Coral model...")
model = DetectionEngine("/code/catkin_ws/src/coral/packages/object_detection/src/dt_inference_models/ssdv2_standardweight_edgetpu.tflite")
FPS = []
AvgFPS = 0
while True:
	try:
		start_time = time.time()
		frame = np.load('/code/catkin_ws/src/coral/packages/object_detection/src/temp_frame.npy')

		frame = Image.fromarray(frame)
		results = model.detect_with_image(frame, threshold=0.3, top_k=20, keep_aspect_ratio=True, relative_coord=False)

		d = {}
		l = []
		

		for r in results:
			box = r.bounding_box.flatten().astype("int")
			(startX, startY, endX, endY) = box
			label = labels[r.label_id]
			color = colors[r.label_id]
			d['startX'] = int(startX)
			d['startY'] = int(startY)
			d['endX'] = int(endX)
			d['endY'] = int(endY)
			d['label'] = label
			d['color'] = color
			d['score'] = float(r.score)
			d['FPS'] = int(AvgFPS)
			l.append(d.copy())
			#print(l)

		with open('/code/catkin_ws/src/coral/packages/object_detection/src/prediction', 'w') as fout:
			json.dump(l, fout)

		diff = time.time() - start_time
		fps = 1/diff
		FPS.append(fps)
		
		# Average 20 FPS records
		if len(FPS)==20:
			AvgFPS = sum(FPS)/len(FPS)
			FPS = []

	except (ValueError, FileNotFoundError):
		pass

