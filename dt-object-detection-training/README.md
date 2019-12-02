# Duckietown Object Detection ML Training
## Working directory
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
## LOCALHOST
We use Docker to run our scripts. Please execute these command in this directory.

**Important:** Don't forget to increase the allocated memory and swap-storage of docker, otherwise TensorFlow will get killed
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

Run Dataset preparation (Maxi):
```
docker run -it -e VAL_1_OF_N_IMAGES=10 -e TEST_1_OF_N_IMAGES=10 -v /Users/maximilianstoelzle/Documents/ethz/AMoD/workdir:/workdir mstoelzle/dt-object-detection-training:latest launch/dataset_preparation.sh
```

### Prepare TensorFlow WORKDIR
1. Create `workdir` directory somewhere on your local drive
2. Copy contents of `tf_wordir_sample directory/models` from REPO into local `workdir/models`
3. If step _Prepare Duckietown Dataset_ was not run: Place `dt_mscoco_train.record` and `dt_mscoco_val` into `workdir/data` directory

### Run locational weights generator
Generate locational weights matrix for use in safety-modified loss-function.
This step is automatically done, when running the training. However, it can also be run separately:
```
docker run -it -e DUCKIEBOT_CALIBRATION_HOSTNAME=maxicar -e VISUALIZE=True -v YOUR_LOCAL_WORKDIR:/workdir mstoelzle/dt-object-detection-training:latest bash -c launch/locational_weights_gen.sh
```

### Run
Run TensorBoard:
```
docker run -it -p 8888:8888 -p 6006:6006 -v YOUR_LOCAL_WORKDIR:/workdir mstoelzle/dt-object-detection-training:latest bash -c launch/tensorboard.sh
```

Run TensorBoard (Maxi):
```
docker run -it -p 8888:8888 -p 6006:6006 -v /Users/maximilianstoelzle/Documents/ethz/AMoD/workdir:/workdir mstoelzle/dt-object-detection-training:latest bash -c launch/tensorboard.sh
```

Access Tensorboard: http://localhost:6006/

Run Training:
```
docker run -it -e MODEL_NAME=ssd_mobilenet_v2_quantized_300x300_coco -e DUCKIEBOT_CALIBRATION_HOSTNAME=maxicar -v YOUR_LOCAL_WORKDIR:/workdir mstoelzle/dt-object-detection-training:latest
```

Run Training (Maxi):
```
docker run -it -e MODEL_NAME=ssd_mobilenet_v2_quantized_300x300_coco -e DUCKIEBOT_CALIBRATION_HOSTNAME=maxicar -v /Users/maximilianstoelzle/Documents/ethz/AMoD/workdir:/workdir mstoelzle/dt-object-detection-training:latest
```

### Run container interactively
```
docker run -it -p 8888:8888 -p 6006:6006 -v YOUR_LOCAL_WORKDIR:/workdir mstoelzle/dt-object-detection-training:latest /bin/bash
```

## IDSC RUDOLF
### Build:
Build on localhost for GPU:
```
docker build --build-arg GPU=-gpu -f ./Dockerfile -t mstoelzle/dt-object-detection-training:latest-gpu .
```

Push to Docker Hub:
```
docker push mstoelzle/dt-object-detection-training:latest-gpu
```

### Prepare TensorFlow WORKDIR

SSH into IDSC Rudolf:
```
ssh lfivop-ml@idsc-rudolf.ethz.ch -L 6067:127.0.0.1:6067
```

1. Create `workdir` directory in home directory of RUDOLF account
2. Copy contents of `tf_wordir_sample directory` from REPO into  `workdir`
3. Place `mscoco_train.record` into `workdir/data` directory

Copy from localhost to RUDOLF:

```
scp -r YOUR_LOCAL_WORKDIR/ lfivop-ml@idsc-rudolf.ethz.ch:/home/lfivop-ml/workdir/
```

### Run

Pull docker image:
```
docker pull mstoelzle/dt-object-detection-training:latest-gpu
```

Run TensorBoard:
```
docker run -it -e MODEL_NAME=ssd_mobilenet_v2_quantized_300x300_coco -p 6067:6067 -e TB_PORT=6067 -v /home/lfivop-ml/workdir:/workdir mstoelzle/dt-object-detection-training:latest-gpu bash -c launch/tensorboard.sh
```

Access TensorBoard: http://localhost:6067/

Run Training:

```
ssh lfivop-ml@idsc-rudolf.ethz.ch
docker run -it -e MODEL_NAME=ssd_mobilenet_v2_quantized_300x300_coco -e CUDA_VISIBLE_DEVICES=2 -e DUCKIEBOT_CALIBRATION_HOSTNAME=maxicar -v /home/lfivop-ml/workdir:/workdir mstoelzle/dt-object-detection-training:latest-gpu
```

### export training checkpoints
```
scp -r lfivop-ml@idsc-rudolf.ethz.ch:/home/lfivop-ml/workdir/ YOUR_LOCAL_WORKDIR/
```