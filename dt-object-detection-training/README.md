# Duckietown Object Detection ML Training
## Docker
We use Docker to run our scripts. Please execute these command in this directory.

**Important:** Don't forget to increase the allocated memory and swap-storage of docker, otherwise TensorFlow will get killed
### Build
`docker build -f ./Dockerfile -t dt-object-detection-training .`
### Run
Place `mscoco_train.record` into your local `data` directory, then run:

`docker run -it -p 8888:8888 -v YOUR_LOCAL_DATA_DIR:/data dt-object-detection-training:latest`

## Run with volume sharing (Maxi)
`docker run -it -p 8888:8888 -v /Users/maximilianstoelzle/Documents/ethz/AMoD/data:/data dt-object-detection-training:latest`

### Run container interactively
`docker run -it -p 8888:8888 dt-object-detection-training:latest /bin/bash`