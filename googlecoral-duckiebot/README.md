# Object Detection Inference on a Duckiebot with Google Coral

This respository details the steps on running object detection inference on a Duckiebot with Google Coral. Google Coral only works with Python3, while 2019 class of ETHZ Duckietown uses ROS, which is only compatible with Python2. Therefore, a workaround is applied, as described on the schematic below.

![Screenshot](https://github.com/duckietown-ethz/proj-lfivop-ml/wiki/images/googlecoral-duckiebot-schematic.png)

## Instructions to run the inference

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

### 3. Run docker image in Duckiebot with the following options
```bash
docker -H [ROBOT_NAME].local run -it --rm --net=host -v /dev/bus/usb:/dev/bus/usb --privileged duckietown/googlecoral-duckiebot:v1-arm32v7
```
The image stream resolution is set to default at 320x240 px. You can change the resolution by adding environment variable 'resolution_w' on the Docker run command. For example, run the command below to set the resolution at 640x480px.

```bash
docker -H [ROBOT_NAME].local run -it --rm --net=host -e resolution_w=640 -v /dev/bus/usb:/dev/bus/usb --privileged duckietown/googlecoral-duckiebot:v1-arm32v7
```

Image stream with bounding boxes, scores, and lables is then published. To avoid Google Coral not detected by the Raspberry Pi, make sure to plug it in before powering up the Raspberry Pi. 

## Credit
As a test, the first version of the code uses a pre-trained tflite model by Adrian Rosebrock, taken from https://www.pyimagesearch.com/2019/05/13/object-detection-and-image-classification-with-google-coral-usb-accelerator/. 