# Duckietown Object Detection ML Training
## Docker
We use Docker to run our scripts. Please execute these command in this directory.

**Important:** Don't forget to increase the allocated memory and swap-storage of docker, otherwise TensorFlow will get killed
### Build
`docker build -f ./Dockerfile -t dt-object-detection-training .`
### Run
Place `mscoco_train.record` into your local `data` directory, then run:

`docker run -it -p 8888:8888 -p 6006:6006 -v YOUR_LOCAL_DATA_DIR:/src/dt-object-detection-training/data dt-object-detection-training:latest`

### Run with volume sharing (Maxi)
`docker run -it -p 8888:8888 -p 6006:6006 -v /Users/maximilianstoelzle/Documents/ethz/AMoD/data/dt-object-detection-training:/src/dt-object-detection-training/data dt-object-detection-training:latest`

### Run container interactively
`docker run -it -p 8888:8888 -p 6006:6006 -v YOUR_LOCAL_DATA_DIR:/src/dt-object-detection-training/data dt-object-detection-training:latest /bin/bash`

## TensorBoard
While running the container, you can access the TensorBoard to survey the training progress:
http://localhost:6006/