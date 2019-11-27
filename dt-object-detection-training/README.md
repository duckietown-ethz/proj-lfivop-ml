# Duckietown Object Detection ML Training
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
### Prepare TensorFlow WORKDIR
Place `mscoco_train.record` into your local `data` directory, then run:

1. Create `tf_workdir` directory somewhere on your local drive
2. Copy contents of `tf_wordir_sample directory` from REPO into local `tf_workdir`
3. Place `mscoco_train.record` into `tf_workdir/data` directory

### Run

Run TensorBoard:
```
docker run -it -p 8888:8888 -p 6006:6006 -v YOUR_LOCAL_TF_WORKDIR:/tf_workdir mstoelzle/dt-object-detection-training:latest bash -c launch/tensorboard_launch.sh
```

Access Tensorboard: http://localhost:6006/

Run Training:
```
docker run -it -v YOUR_LOCAL_TF_WORKDIR:/tf_workdir mstoelzle/dt-object-detection-training:latest
```

Run TensorBoard (Maxi):
```
docker run -it -p 8888:8888 -p 6006:6006 -v /Users/maximilianstoelzle/Documents/ethz/AMoD/tf_workdir:/tf_workdir mstoelzle/dt-object-detection-training:latest bash -c launch/tensorboard_launch.sh
```

Run Training (Maxi):
```
docker run -it -p 8888:8888 -p 6006:6006 -v YOUR_LOCAL_TF_WORKDIR:/tf_workdir mstoelzle/dt-object-detection-training:latest /bin/bash
```

### Run container interactively
```
docker run -it -p 8888:8888 -p 6006:6006 -v YOUR_LOCAL_TF_WORKDIR:/tf_workdir mstoelzle/dt-object-detection-training:latest /bin/bash
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

1. Create `tf_workdir` directory in home directory of RUDOLF account
2. Copy contents of `tf_wordir_sample directory` from REPO into  `tf_workdir`
3. Place `mscoco_train.record` into `tf_workdir/data` directory

Copy from localhost to RUDOLF:

```
scp -r YOUR_LOCAL_TF_WORKDIR/ lfivop-ml@idsc-rudolf.ethz.ch:/home/lfivop-ml/tf_workdir/
```

### Run

Pull docker image:
```
docker pull mstoelzle/dt-object-detection-training:latest-gpu`
```

Run TensorBoard:
```
docker run -it -e MODEL_NAME=ssd_mobilenet_v2_quantized_300x300_coco -p 6067:6067 -e TB_PORT=6067 -v /home/lfivop-ml/tf_workdir:/tf_workdir mstoelzle/dt-object-detection-training:latest-gpu bash -c launch/tensorboard_launch.sh
```

Access TensorBoard: http://localhost:6067/

Run Training:

```
ssh lfivop-ml@idsc-rudolf.ethz.ch
docker run -it -e MODEL_NAME=ssd_mobilenet_v2_quantized_300x300_coco -e CUDA_VISIBLE_DEVICES=2 -v /home/lfivop-ml/tf_workdir:/tf_workdir mstoelzle/dt-object-detection-training:latest-gpu
```

### export training checkpoints
```
scp -r lfivop-ml@idsc-rudolf.ethz.ch:/home/lfivop-ml/tf_workdir/ YOUR_LOCAL_TF_WORKDIR/
```