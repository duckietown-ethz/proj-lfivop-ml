# Object Detection Inference on a Duckiebot with Google Coral USB Accelerator

This respository details the steps on running object detection inference on a Duckiebot with Google Coral USB accelerator. Google Coral only works with Python3, while 2019 class of ETHZ Duckietown uses ROS, which is only compatible with Python2. Therefore, a workaround is applied, as described on the schematic below.

![Screenshot](https://github.com/duckietown-ethz/proj-lfivop-ml/wiki/images/googlecoral-duckiebot-schematic.png)

## Instructions to run the inference

### Pre-flight checklist:
* You can see the output of the camera
* Google Coral USB accelerator is plugged into Duckiebot

### 1. Clone this repository and go inside googlecoral-duckiebot directory
```bash
git clone https://github.com/duckietown-ethz/proj-lfivop-ml.git
cd googlecoral-duckiebot
```

### 2. Build docker image in Duckiebot
```bash
dts devel build -f --arch arm32v7 -H [ROBOT_NAME].local 
```
Building the image for the first time can take up to 30 minutes.

### 3. Make sure you have plugged in Google Coral USB Accelerator to Duckiebot, then run docker image with the following options
 
```bash
docker -H [ROBOT_NAME].local run -it --rm --net=host -v /dev/bus/usb:/dev/bus/usb -e model_name=MODEL_NAME --privileged duckietown/proj-lfivop-ml:master-arm32v7
```
If no model is specified, it will be set to "class_localization". Model name choices are:

| model_name  | Data augmentation | Classification loss | Localization loss
| ------------- | ------------- |  ------------- |  ------------- |
| class_localization  | extra  | safety-weighted | safety-weighted |
| class  |  extra  | safety-weighted | standard |
| localization  |  extra  | standard | safety-weighted |
| augmentation  | extra  | standard | standard |
| vanilla  | standard  | standard | standard |

Other environment variables you can set:

| environment variable  | description | default | 
| ------------- | ------------- |  ------------- | 
| resolution_w | Width of image stream resolution. Height is set to 3/4 of width. For example, add "-e resolution_w=640"  to the docker run command above to set the resolution to 640x480px. | 320 |
| threshold | Minimum confidence threshold for detected objects. For example, use 0.5 to receive only detected objects with a confidence equal-to or higher-than 0.5. | 0.5 |
| top_k | The maximum number of detected objects to return. | 15 |

Image stream with bounding boxes, scores, and labels is then published to topic:  
```
[ROBOT_NAME]/coral_object_detection/image/compressed
```
## Instructions to run emergency stop demo

### Pre-flight checklist:
* You can see the output of the camera
* Google Coral USB accelerator is plugged into Duckiebot
* Wheels calibration is completed

### 1. Make sure you have plugged in Google Coral USB Accelerator to Duckiebot, then run docker image with the following options

```bash
docker -H [ROBOT_NAME].local run -it --rm --net=host -v /dev/bus/usb:/dev/bus/usb -v /data:/data -e model_name=MODEL_NAME --privileged duckietown/proj-lfivop-ml:master-arm32v7 bash -c packages/launch_emergencystop_demo/emergencystop_demo.sh
```
Your Duckiebot will move in a straight line. It will stop when it detects Duckies or Duckiebots close upfront with confidence level setted in the environmental variables.

In order to improve robustness and stability of the emergency stop against single false negatives and false positives, 
a vote count was implemented. Thus, a specific vote threshold is required to activate or deactivate the emergency stop.

Additionally to the environmental variables in the standard inference, the following environmental variables can bet set
for the emergency stop demo:

| environment variable  | description | default | 
| ------------- | ------------- |  ------------- | 
| EMGSTOP_OBJDET_CONFIDENCE_THRESH | Required confidence in object detection, in order to activate emergency stop for a specific critical object. The default is 60%. | 0.6 |
| EMGSTOP_STOP_MODE | Defines the mode for the emergency stop. Can be either `image_box`, which specifies a specific box within the image or `ground_projection` which judges using cylindrical coordinates | image_box |
| EMGSTOP_STOP_BRAKING_DISTANCE_MULTIPLE | Trigger an emergency stop at the specified multiple of the braking distance | 3 |
| EMGSTOP_STOP_VOTE_MAX | The vote count for an emergency stop is limited at this maximal value. | 30 |
| EMGSTOP_STOP_VOTE_THRESH | The emergency stop is activated above and deactivated below this threshold. | 10 |

## Troubleshooting
**1. I run the object detection inference and lane following at the same time, but my Duckiebot moves bizarrelly, especially at the intersection**

Object detection inference uses up some CPU capacity of your Duckiebot, hence makes it slower in estimating lane. Decrease gain parameter to make it more stable (https://docs.duckietown.org/daffy/opmanual_duckiebot/out/wheel_calibration.html). Our test indicates lane following with gain parameter of 0.5 is stable in a well-lit robotarium. 

**2. I run emergency stop demo. When I stop the container by pressing ctrl+c, my Duckiebot still moves.**

Re-run and stop the container again until your Duckiebot stops.
