# Duckietown Object Detection ML Training
This README will give you some instructions on how to run the Duckietown object detection training either on your local computer or on _IDSC Rudolf_. Additionally, it will give you some Troubleshooting advice.

## Instructions for running training on LOCALHOST

### Pre-flight checklist
Before you try to run a training for duckietown object detection on your localhost, make sure you have prepared the following things:
1. Install [Git](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git) and [Docker](https://docs.docker.com/install/)
2. Give Docker sufficient resources on your local computer (increase allocated memory and swap-storage), otherwise TensorFlow will kill the training
3. Clone this repository to your local computer: `git clone https://github.com/duckietown-ethz/proj-lfivop-ml.git`
4. Initialize Git submodules
4. Choose (pre-trained) model which you want to use for training. The model needs to be quantized in order for the inference to work on the Edge-TPU. Recommended is using the pre-trained `ssd_mobilenet_v2_quantized_300x300_coco` model
6. Prepare working directory
7. Run the following commands from the `proj-lfivop-ml/dt-object-detection-training` directory:
    1. Build either the CPU or the GPU version of the Docker container, depending on if your graphic card supports _Nvidia CUDA_
    2. Run Dataset preparation command in the Docker container (`bash -c launch/dataset_preparation.sh`)
    3. Run TensorBoard command in the Docker container (`bash -c launch/tensorboard.sh`)
    4. Run Training command in the Docker container (`bash -c launch/training.sh`) while specifying number of training steps and the chosen model
    5. Monitor training progress on TensorBoard
    9. After the training finished, choose the checkpoint, you want to use for exporting the inference graph
    10. Run the Edge-TPU inference graph export command in the Docker container: (`bash -c launch/inference_graph_edgetpu_export.sh`) while specifying the checkpoint number

### Git submodule initialization
We use git submodules for the tensorflow/models and cocoapi repository. 
After cloning this repository, you need to initialize the git submodules first:
```
# initialize submodules
git submodule init && git submodule update

# fetch remote HEAD for submodules
git submodule update --remote
```

### Build
Build CPU version:
```
docker build -f ./Dockerfile -t mstoelzle/dt-object-detection-training:latest .
```
Build GPU version:
```
docker build --build-arg GPU=-gpu -f ./Dockerfile -t mstoelzle/dt-object-detection-training:latest-gpu .
```

### Prepare Duckietown Dataset
1. Create `workdir` directory somewhere on your local drive
2. Create directory `raw_data` in `workdir`
3. Copy `Annotations.csv` and `images` folder into `raw_data` directory

Run Dataset preparation:
```
docker run -it -e VAL_1_OF_N_IMAGES=10  -e TEST_1_OF_N_IMAGES=10 -v YOUR_LOCAL_WORKDIR:/workdir mstoelzle/dt-object-detection-training:latest bash -c launch/dataset_preparation.sh
```

### Prepare TensorFlow WORKDIR
1. Create `workdir` directory somewhere on your local drive
2. Copy contents of `tf_wordir_sample directory/models` from REPO into local `workdir/models`
3. If step _Prepare Duckietown Dataset_ was not run: Place `dt_mscoco_train.record` and `dt_mscoco_val` into `workdir/data` directory

### Run locational weights generator
Generate locational weights matrix for use in safety-modified loss-function.
This step is automatically done, when running the training. However, it can also be run separately:
```
docker run -u $(id -u):$(id -g) -it -e DUCKIEBOT_CALIBRATION_HOSTNAME=maxicar -v YOUR_LOCAL_WORKDIR:/workdir mstoelzle/dt-object-detection-training:latest bash -c launch/locational_weights_gen.sh
```

### Run locational weights visualizer
```
docker run -u $(id -u):$(id -g) -it -e DUCKIEBOT_CALIBRATION_HOSTNAME=maxicar -v YOUR_LOCAL_WORKDIR:/workdir mstoelzle/dt-object-detection-training:latest bash -c "launch/locational_weights_viz.sh --image_filename b_BR_doort_frame00328"
```

### Run
Run TensorBoard:
```
docker run -u $(id -u):$(id -g) -it -p 6006:6006 -v YOUR_LOCAL_WORKDIR:/workdir mstoelzle/dt-object-detection-training:latest bash -c launch/tensorboard.sh
```

Access Tensorboard: http://localhost:6006/

Run Training:
```
docker run -u $(id -u):$(id -g) -it -e MODEL_NAME=ssd_mobilenet_v2_quantized_300x300_coco -e NUM_TRAIN_STEPS=50000 -e DUCKIEBOT_CALIBRATION_HOSTNAME=maxicar -v YOUR_LOCAL_WORKDIR:/workdir mstoelzle/dt-object-detection-training:latest
```

### Run export of inference graph
```
docker run -u $(id -u):$(id -g) -it -e MODEL_NAME=ssd_mobilenet_v2_quantized_300x300_coco -e CHECKPOINT_NUMBER=0 -v YOUR_LOCAL_WORKDIR:/workdir mstoelzle/dt-object-detection-training:latest bash -c launch/inference_graph_export.sh
```

### Run export of Edge-TPU inference graph
```
docker run -u $(id -u):$(id -g) -it -e MODEL_NAME=ssd_mobilenet_v2_quantized_300x300_coco -e CHECKPOINT_NUMBER=0 -v YOUR_LOCAL_WORKDIR:/workdir mstoelzle/dt-object-detection-training:latest bash -c launch/inference_graph_edgetpu_export.sh
```

### Run container interactively
```
docker run -it -p 8888:8888 -p 6006:6006 -v YOUR_LOCAL_WORKDIR:/workdir mstoelzle/dt-object-detection-training:latest /bin/bash
```

## Instructions for running training on IDSC RUDOLF
### Pre-flight checklist
Before you try to run a training for duckietown object detection training on IDSC Rudolf, make sure you have prepared the following things:
1. Granted IDSC Rudolf account with Docker privileges
2. Pushed the `proj-lfivop-ml` repository to Github. _Github Actions_ will automatically build the docker container (both CPU and GPU versions) and push them to Docker Hub (_mstoelzle_ account)
3. Prepare working directory
4. Choose (pre-trained) model which you want to use for training. The model needs to be quantized in order for the inference to work on the Edge-TPU. Recommended is using the pre-trained `ssd_mobilenet_v2_quantized_300x300_coco` model
5. Copy working directory to IDSC Rudolf using `scp`
6. Run the following commands from the `proj-lfivop-ml/dt-object-detection-training` directory:
    1. Run Dataset preparation command in the Docker container (`bash -c launch/dataset_preparation.sh`)
    2. SSH into IDSC Rudolf while sharing the port 6067 with your local computer, in order that you can access TensorBoard in your browser
    3. Pull the recent Docker image from Docker Hub
    4. Start screen with `screen`
    5. Run TensorBoard command in the Docker container (`bash -c launch/tensorboard.sh`)
    6. Open new window in screen with `Ctrl-a c`
    7. Run Training command in the Docker container (`bash -c launch/training.sh`) while specifying number of training steps and the chosen model
    8. Detach screen with `Ctrl-a d`. This will ensure, that the training keeps running, even if your SSH tunnel is closed
    9. Monitor training progress on TensorBoard
    10. After the training finished, choose the checkpoint, you want to use for exporting the inference graph
    11. Run the Edge-TPU inference graph export command in the Docker container: (`bash -c launch/inference_graph_edgetpu_export.sh`) while specifying the checkpoint number
    12. Resume the screen session using `screen -r`, then cycle through all the windows (`Ctrl-a n`) and exit the window (`exit`) which will stop all running processes in the window
    13. Clean up IDSC Rudolf by stopping all running containers (`docker ps` to find the container ids and `docker stop container_id` to stop the container) and removing the working directory with `rm -r`
7. Copy the working directory from IDSC Rudolf back to your local computer including all the checkpoints using `scp` (might take a while)

### Build:
Build on localhost for GPU:
```
docker build --build-arg GPU=-gpu -f ./Dockerfile -t mstoelzle/dt-object-detection-training:latest-gpu .
```

Push to Docker Hub:
```
docker push mstoelzle/dt-object-detection-training:latest-gpu
```

### SSH into IDSC Rudolf:
SSH into IDSC Rudolf:
```
ssh lfivop-ml@idsc-rudolf.ethz.ch -L 6067:127.0.0.1:6067
```

### Use of screens on IDSC Rudolf
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

### Copy WORKDIR to IDSC Rudolf
Copy the prepared WORKDIR from localhost to RUDOLF:

```
scp -r YOUR_LOCAL_WORKDIR/ lfivop-ml@idsc-rudolf.ethz.ch:/home/lfivop-ml/workdir/
```

## Pull Docker Image
```
docker pull mstoelzle/dt-object-detection-training:latest-gpu
```

### Run Training
Run TensorBoard:
```
docker run -u $(id -u):$(id -g) -it -e MODEL_NAME=ssd_mobilenet_v2_quantized_300x300_coco -p 6067:6067 -e TB_PORT=6067 -v /home/lfivop-ml/workdir:/workdir mstoelzle/dt-object-detection-training:latest-gpu bash -c launch/tensorboard.sh
```

Access TensorBoard: http://localhost:6067/

Run Training:

```
docker run -u $(id -u):$(id -g) -it -e MODEL_NAME=ssd_mobilenet_v2_quantized_300x300_coco -e CUDA_VISIBLE_DEVICES=2 -e NUM_TRAIN_STEPS=50000 -e DUCKIEBOT_CALIBRATION_HOSTNAME=maxicar -v /home/lfivop-ml/workdir:/workdir mstoelzle/dt-object-detection-training:latest-gpu
```

### Copy WORKDIR from IDSC Rudolf to local computer
Copy back checkpoints and training results to your local computer. This step might take a while.
```
scp -r lfivop-ml@idsc-rudolf.ethz.ch:/home/lfivop-ml/workdir/ YOUR_LOCAL_WORKDIR/
```

## Config Working directory
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
