# Duckietown Object Detection ML Training
This README will give you some instructions on how to run the Duckietown object detection training either on your local computer or on _IDSC Rudolf_. Additionally, it will give you some Troubleshooting advice.

## Instructions to prepare Workdir
1. Create `workdir` directory somewhere on your local drive
2. Copy contents of `workdir_sample` from REPO into local `workdir`
3. Place any additional annotations and raw images for the training into the `workdir/raw_data` folder 

## Instructions for running training on LOCALHOST

### Pre-flight checklist
Before you try to run a training for duckietown object detection on your localhost, make sure you have prepared the following things:
1. Install [Git](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git) and [Docker](https://docs.docker.com/install/)
2. Give Docker sufficient resources on your local computer (increase allocated memory and swap-storage), otherwise TensorFlow will kill the training
3. Clone this repository to your local computer: `git clone https://github.com/duckietown-ethz/proj-lfivop-ml.git`
4. Choose (pre-trained) model which you want to use for training. The model needs to be quantized in order for the inference to work on the Edge-TPU. Recommended is using the pre-trained `dt_ssd_mobilenet_v2_quantized_320x320_coco` model
5. Prepare workdir directory

### 1. Git submodule initialization
We use git submodules for the tensorflow/models and cocoapi repository. 
After cloning this repository, you need to initialize the git submodules first:
```
# initialize submodules
git submodule init && git submodule update

# fetch remote HEAD for submodules
git submodule update --remote
```

### 2. Build
Run the building commands from within the `proj-lfivop-ml/dt-object-detection-training` directory.

Build CPU version:
```
docker build -f ./Dockerfile -t mstoelzle/dt-object-detection-training:latest .
```
Build GPU version:
```
docker build --build-arg GPU=-gpu -f ./Dockerfile -t mstoelzle/dt-object-detection-training:latest-gpu .
```

### 3. Prepare Duckietown Dataset
Run Dataset preparation:
```
docker run -it -e VAL_1_OF_N_IMAGES=10  -e TEST_1_OF_N_IMAGES=10 -v YOUR_LOCAL_WORKDIR:/workdir mstoelzle/dt-object-detection-training:latest bash -c launch/dataset_preparation.sh
```

### 4. Run Training
You need to specify the number of training steps and the chosen model for the training.

Run TensorBoard:
```
docker run -u $(id -u):$(id -g) -it -p 6006:6006 -v YOUR_LOCAL_WORKDIR:/workdir mstoelzle/dt-object-detection-training:latest bash -c launch/tensorboard.sh
```

Access TensorBoard: http://localhost:6006/

Run Training:
```
docker run -u $(id -u):$(id -g) -it -e MODEL_NAME=dt_ssd_mobilenet_v2_quantized_320x320_coco -e NUM_TRAIN_STEPS=50000 -e DUCKIEBOT_CALIBRATION_HOSTNAME=maxicar -v YOUR_LOCAL_WORKDIR:/workdir mstoelzle/dt-object-detection-training:latest
```

### 5. Run export of Edge-TPU inference graph
After the training finished, choose the checkpoint, you want to use for exporting the inference graph.

```
docker run -u $(id -u):$(id -g) -it -e MODEL_NAME=dt_ssd_mobilenet_v2_quantized_320x320_coco -e CHECKPOINT_NUMBER=0 -v YOUR_LOCAL_WORKDIR:/workdir mstoelzle/dt-object-detection-training:latest bash -c launch/inference_graph_edgetpu_export.sh
```

## Instructions for running training on IDSC RUDOLF
### Pre-flight checklist
Before you try to run a training for duckietown object detection on your IDSC Rudolf, make sure you have prepared the following things:
1. Granted IDSC Rudolf account with Docker privileges
2. Pushed the `proj-lfivop-ml` repository to Github. _Github Actions_ will automatically build the docker container (both CPU and GPU versions) and push them to Docker Hub (_mstoelzle_ account)
3. Choose (pre-trained) model which you want to use for training. The model needs to be quantized in order for the inference to work on the Edge-TPU. Recommended is using the pre-trained `dt_ssd_mobilenet_v2_quantized_320x320_coco` model
4. Prepare workdir directory on your local computer
5. Run Dataset preparation for your local workdir (see Instructions for localhost)

### 1. Build:
Run the building commands from within the `proj-lfivop-ml/dt-object-detection-training` directory.

Build on localhost for GPU:
```
docker build --build-arg GPU=-gpu -f ./Dockerfile -t mstoelzle/dt-object-detection-training:latest-gpu .
```

Push to Docker Hub:
```
docker push mstoelzle/dt-object-detection-training:latest-gpu
```
### 2. Copy workdir to IDSC Rudolf
Copy the prepared `workdir` from localhost to RUDOLF:

```
scp -r YOUR_LOCAL_WORKDIR/ lfivop-ml@idsc-rudolf.ethz.ch:/home/lfivop-ml/workdir/
```

### 3. SSH into IDSC Rudolf:
SSH into IDSC Rudolf:
```
ssh lfivop-ml@idsc-rudolf.ethz.ch -L 6067:127.0.0.1:6067
```

### 4. Pull Docker Image
```
docker pull mstoelzle/dt-object-detection-training:latest-gpu
```

### 5. Run Training
**Start screen:**
```
screen
```

**Run TensorBoard:**
```
docker run -u $(id -u):$(id -g) -it -e MODEL_NAME=dt_ssd_mobilenet_v2_quantized_320x320_coco -p 6067:6067 -e TB_PORT=6067 -v /home/lfivop-ml/workdir:/workdir mstoelzle/dt-object-detection-training:latest-gpu bash -c launch/tensorboard.sh
```

Access TensorBoard: http://localhost:6067/

**Run Training:**

Open new screen window with `Ctrl-a c`

```
docker run -u $(id -u):$(id -g) -it -e MODEL_NAME=dt_ssd_mobilenet_v2_quantized_320x320_coco -e CUDA_VISIBLE_DEVICES=2 -e NUM_TRAIN_STEPS=50000 -e DUCKIEBOT_CALIBRATION_HOSTNAME=maxicar -v /home/lfivop-ml/workdir:/workdir mstoelzle/dt-object-detection-training:latest-gpu
```

**Detach Screen:**

Detach screen with `Ctrl-a d`.
This will ensure, that the training keeps running, even if your SSH tunnel is closed

### 6. Run export of Edge-TPU inference graph
After the training finished, choose the checkpoint, you want to use for exporting the inference graph.

```
docker run -u $(id -u):$(id -g) -it -e MODEL_NAME=dt_ssd_mobilenet_v2_quantized_320x320_coco -e CHECKPOINT_NUMBER=0 -v YOUR_LOCAL_WORKDIR:/workdir mstoelzle/dt-object-detection-training:latest-gpu bash -c launch/inference_graph_edgetpu_export.sh
```

### 7. Copy Workdir from IDSC Rudolf to local computer
Copy back checkpoints and training results to your local computer. This step might take a while.
```
scp -r lfivop-ml@idsc-rudolf.ethz.ch:/home/lfivop-ml/workdir/ YOUR_LOCAL_WORKDIR/
```

### 8. Clean-up on IDSC Rudolf
1. Stop all windows:
    1. Resume the screen session using `screen -r`
    2. Then cycle through all the windows (`Ctrl-a n`) and exit the window (`exit`) which will stop all running processes in the window
2. Stopping all remaining running Docker containers
    1. `docker ps` to find the container ids 
    2. `docker stop container_id` to stop the container
3. Remove the working directory with `rm -r workdir`

## General Instructions
### Run locational weights generator
Generate locational weights matrix for use in safety-modified loss-function.
This step is automatically done, when running the training. However, it can also be run separately:
```
docker run -u $(id -u):$(id -g) -it -e DUCKIEBOT_CALIBRATION_HOSTNAME=maxicar -v YOUR_LOCAL_WORKDIR:/workdir mstoelzle/dt-object-detection-training:latest bash -c launch/locational_weights_gen.sh
```

### Run locational weights visualizer
Visualize the locational weights on a test image. The test image needs to be placed in `workdir/raw_data/images/`
```
docker run -u $(id -u):$(id -g) -it -e DUCKIEBOT_CALIBRATION_HOSTNAME=maxicar -v YOUR_LOCAL_WORKDIR:/workdir mstoelzle/dt-object-detection-training:latest bash -c "launch/locational_weights_viz.sh --image_filename b_BR_doort_frame00328"
```

### Run container interactively
```
docker run -it -p 8888:8888 -p 6006:6006 -v YOUR_LOCAL_WORKDIR:/workdir mstoelzle/dt-object-detection-training:latest /bin/bash
```

### Run export of inference graph
```
docker run -u $(id -u):$(id -g) -it -e MODEL_NAME=dt_ssd_mobilenet_v2_quantized_320x320_coco -e CHECKPOINT_NUMBER=0 -v YOUR_LOCAL_WORKDIR:/workdir mstoelzle/dt-object-detection-training:latest bash -c launch/inference_graph_export.sh
```

### Use of screens
Screen can be used to manage multiple terminal sessions and detach them in order that the training keeps running:
https://kb.iu.edu/d/acuy
```
# start screen
screen
# create window
Ctrl-a c
# detach screen
Ctrl-a d
# resume screen
screen -r
# exit window
exit
```

## Configurations

### Included models
**dt_ssd_mobilenet_v2_coco**

The model.ckpt checkpoint was pre-trained on the Coco dataset and is used to fine-tune the neural neural network on initialization. 
This model is not _quantized_. 

Additional data augmentation (compared to the standard model) as for example 
- random_image_scale [0.75, 1.25]
- random_adjust_brightness [1.20]
- random_adjust_contrast [0.80, 1.25]
- random_adjust_saturation [0.80, 1.25]

was added the the pipeline configuration.

**dt_ssd_mobilenet_v2_quantized_320x320_coco**

The model.ckpt checkpoint was pre-trained on the Coco dataset and then (using dt_ssd_mobilenet_v2_coco and additional data augmentation) for 50000 steps on the Duckietown dataset.
It is used to fine-tune the neural neural network on initialization and reduce the convergence time while training with quantization. 

The pre-trained model is not quantized yet, however this model will quantize during training with a delay of 48000, 8 weight bits and 8 activation bits. 

Additional data augmentation (compared to the standard model) as for example 
- random_image_scale [0.75, 1.25]
- random_adjust_brightness [1.20]
- random_adjust_contrast [0.80, 1.25]
- random_adjust_saturation [0.80, 1.25]

was added the the pipeline configuration.

Compared to the standard *dt_ssd_mobilenet_v2_coco* model, the following changes were introduced:
- the image_resizer is adapted to 320x320 pixels
- the batch size was reduced to 12 (otherwise memory issues on IDSC Rudolf)
- `load_all_detection_checkpoint_vars` was set to `true` to speed up training
- graph rewriter was added for quantization

### Working directory
Structure of working directory which must be attached as volume to container.

`+` describes a directory and `-` a file
```
+workdir
    + config
      - dt_locational_weights.json
    +raw_data
      - Annotations.csv
      + images
    +data
      - dt_mscoco_label_map.pbtxt
      - dt_mscoco_train.record
      - dt_mscoco_val.record
      - dt_mscoco_test.record
    +models
      + model
        - pipeline.conf
        + train
        + eval
```

### Build Arguments for Docker image
| build argument  | description | default |
| ------------- | ------------- |  ------------- | 
| GPU | Specifies if image is built for the use with a GPU acceleration and includes tensorflow-gpu. For an empty string, the CPU version is built, while for a string "gpu" the GPU version is built. |  |
| REPO_NAME | The name of the repository directory in the container where the relevant files are copied to | dt-object-detection-training |
| TF_PACKAGE_VERSION | Specifies the used TensorFlow version. | 1.15.0 |

### Environment Variables for Docker container
| environment variable  | description | default |
| ------------- | ------------- |  ------------- | 
| MODEL_NAME | Name of the model directory in `workdir/models` which is used for training and inference graph generation | dt_ssd_mobilenet_v2_coco |
| TB_PORT | Specifies the port where the TensorBoard is published (inside container) | 6006 |
| CUDA_VISIBLE_DEVICES | The number of GPUs the TensorFlow will use during training on IDSC Rudolf | 1 |
| NUM_TRAIN_STEPS | The number of training steps before TensorFlow stops the training. | 50000 |
| CHECKPOINT_NUMBER | Specifies the training checkpoint, which should be used to export the inference graph (for example 50000) | 0 |
| VAL_1_OF_N_IMAGES | Specifies the split of the validation set. 1 of every N images are assigned to the validation set. VAL_1_OF_N_IMAGES specifies N. | 10 |
| TEST_1_OF_N_IMAGES | Specifies the split of the test set. 1 of every N images are assigned to the test set. TEST_1_OF_N_IMAGES specifies N. | 10 |
| DUCKIEBOT_CALIBRATION_HOSTNAME | Specifies the hostname of the Duckiebot used to export the intrinsic and extrinsic camera calibration files to generate locational weights | default |

## Troubleshooting and Tips 


>Symptom: While trying to run a Docker container on IDSC Rudolf, it displays permission errors.

**Resolution**: Make sure that your IDSC Rudolf user account was added to the Docker group

>Symptom: While running training, the terminal outputs plenty of warnings, but doesn't get killed.

**Resolution**: We are using TensorFlow 1.15 with `models/research/object_detection` to train the object detection 
model in Duckietown. Although TensorFlow 2.0 was released in October 2019, `models/research/object_detection` has not
been adapted yet for TensorFlow 2.0. This is why, TensorFlow displays many _deprecated_ warnings while running the training,
which disappointingly cannot be turned-off.

>Symptom: TensorBoard doesn't show any results even though the training is running.

**Resolution**: TensorBoard shows gradually more and more data. First it will show the model graph, 
and later after the first evaluation (about 10-20 min) also images which compare the ground-truth to the inference 
on the validation data set. After about 2000-3000 steps, TensorBoard will also show the coco metrics graphs (mAP etc.).
If TensorBoard after 10-20 min still doesn't show any data, probably a different model and / or working directory 
was specified for the training and the TensorBoard container instance.

>Symptom: The coco training metrics (mAP, classification loss) are staying stagnant or even getting worse after a 
>certain number of training steps

**Resolution**: Because of the relatively low number of images in the training set, the training is prone to overfit the data.
This results in a worse performance while evaluating the model on the validation set. You might want to try the following approaches:
1. Increase size of raw data set by adding more annotated images and image sequences
2. Increase the use of data augmentation by changing its configuration in `pipeline.conf` in the appropriate model directory
3. Increase weight of regularization loss in `pipeline.conf`
4. Experiment with the use and timing of quantization, which increases overfitting in some situations



