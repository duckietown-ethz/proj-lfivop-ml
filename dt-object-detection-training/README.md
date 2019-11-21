# Duckietown Object Detection ML Training
## Docker
We use Docker to run our scripts. Please execute these command in this directory.

**Important:** Don't forget to increase the allocated memory and swap-storage of docker, otherwise TensorFlow will get killed
### Build
`docker build -f ./Dockerfile -t dt-object-detection-training:latest .`
### Run
Place `mscoco_train.record` into your local `data` directory, then run:

`docker run -it -p 8888:8888 -p 6006:6006 -v YOUR_LOCAL_DATA_DIR/dt-object-detection-training:/src/dt-object-detection-training/data dt-object-detection-training:latest`

#### Run with volume sharing (Maxi)
`docker run -it -p 8888:8888 -p 6006:6006 -v /Users/maximilianstoelzle/Documents/ethz/AMoD/data/dt-object-detection-training:/src/dt-object-detection-training/data dt-object-detection-training:latest`

### Run container interactively
`docker run -it -p 8888:8888 -p 6006:6006 -v YOUR_LOCAL_DATA_DIR:/src/dt-object-detection-training/data dt-object-detection-training:latest /bin/bash`

### Run on IDSC RUDOLF
Build:

`docker build -f ./Dockerfile -t mstoelzle/dt-object-detection-training:latest .`

Push to Docker Hub:

`docker push mstoelzle/dt-object-detection-training:latest`

Copy `mscoco_train.record` into `data` directory:

`scp -r YOUR_LOCAL_DATA_DIR/dt-object-detection-training lfivop-ml@idsc-rudolf.ethz.ch:/home/lfivop-ml/data`

SSH into IDSC Rudolf:

`ssh lfivop-ml@idsc-rudolf.ethz.ch -L 6006:127.0.0.1:6006`

Run docker container:

`docker run -it -p 8888:8888 -p 6006:6006 -v /home/lfivop-ml/data/dt-object-detection-training:/src/dt-object-detection-training/data mstoelzle/dt-object-detection-training:latest`


## TensorBoard
While running the container, you can access the TensorBoard to survey the training progress:
http://localhost:6006/