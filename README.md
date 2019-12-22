# Robust Object Detection in Duckietown (proj-lfivop-ml)
Welcome to AMoD 2019 - Robust object detection repository! Two main sections of this repository include:
1. [dt-object-detection-training](https://github.com/duckietown-ethz/proj-lfivop-ml/tree/master/dt-object-detection-training): details the instructions to train the Duckietown object detection models
2. [googlecoral-duckiebot](https://github.com/duckietown-ethz/proj-lfivop-ml/tree/master/googlecoral-duckiebot): details the steps to run inference of the trained Duckietown object detection models on the Duckiebot with Google Coral USB accelerator

The comprehensive [report](https://github.com/duckietown-ethz/proj-lfivop-ml/tree/master/AMoD_Robust_Object_Detection_2019.pdf) of this project is organized in [Overleaf](https://www.overleaf.com/read/njpmmpyhmndz).

## Summary

We trained several quantized Single Shot Detector (SSD) MobileNet V2 models with an application specific for Duckietown objects.  Our goal is to produce an object detection model that is robust to different lighting conditions.  In brief, the contributions of this project are threefold:
1. Introduced  safety-weighted  loss  function,  where  misclassification  is  penalized  more severely for critical objects in critical areas
2.  Applied extra data augmentations,  namely random image scale,  random brightness,random contrast, and random saturation
3.  Deployed Google Coral USB Accelerator in Duckiebot to enable live inference. 

## Teaser

### Sample of inference results

![Screenshot](https://github.com/duckietown-ethz/proj-lfivop-ml/wiki/images/inference.jpg)

### Lane following with robust object detection

[![Vimeo](https://github.com/duckietown-ethz/proj-lfivop-ml/wiki/images/lanefollowing_objdet.jpg)](https://player.vimeo.com/video/380723135  "DEMO results - Click to Watch!")

### Emergency stop demo

![Screenshot](https://github.com/duckietown-ethz/proj-lfivop-ml/wiki/images/emergencystop.gif)

## Project Contributors

| Name | E-mail | GitHub username |
| ------------ | ------------ | ----------- |
| Maximilian Stölzle | mstoelzl@ethz.ch | [mstoelzle](https://github.com/mstoelzle/) |
| Stefan Lionar | slionar@student.ethz.ch | [splionar](https://github.com/splionar) |