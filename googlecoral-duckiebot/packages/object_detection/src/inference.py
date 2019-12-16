#!/usr/bin/env python
from edgetpu.detection.engine import DetectionEngine
from PIL import Image
import numpy as np
import json
import time
import os

# loop over the class labels file
labels = {}
colors = {}
for row in open("/code/catkin_ws/src/coral/packages/object_detection/src/duckie_labels_colors.txt"):
	# unpack the row and update labels and colors dictionaries
	(classID, label, color) = row.strip().split(";")
	labels[int(classID)] = label.strip()
	colors[int(classID)] = color.strip()

# load the Google Coral object detection model
print("[INFO] loading Edge TPU model...")

# Specify model name. Choices are: augmentation, class, localization, class_localization, vanilla 
try:
	model_name = str(os.environ['model_name'])
except:
    print("Model is not specified. To specify, set environment variable model_name to your model choice (e.g. -e model_name=MODEL_NAME).")
    print("Choices are: augmentation, class, localization, class_localization, vanilla")
    print("Model is set to default (class_localization)")
    
    model_name = "class_localization"

if model_name not in ["augmentation", "class", "localization", "class_localization", "vanilla"]:
    print("No correct model is chosen. Model is set to default (class_localization).")
    model_name = "class_localization"

model = DetectionEngine("/code/catkin_ws/src/coral/packages/object_detection/src/dt_inference_models/dt_{}_edgetpu.tflite".format(model_name))
print("[INFO] dt_{}_edgetpu.tflite is loaded successfully".format(model_name))
FPS = []
AvgFPS = 0
while True:
	try:
		start_time = time.time()
		
		# Read numpy array of image stream
		frame = np.load('/code/catkin_ws/src/coral/packages/object_detection/src/temp_frame.npy')
		frame = Image.fromarray(frame)

		# Execute inference
		results = model.detect_with_image(frame, threshold=0.5, top_k=15, keep_aspect_ratio=True, relative_coord=False)
		
		# Initialize empty dictionary and list
		d = {}
		l = []
		
		# Get inference results
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

		# Dump inference results to prediction.json
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