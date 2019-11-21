# Duckietown Object Detection ML Training
## Docker
We use Docker to run our scripts. Please execute these command in this directory.

**Important:** Don't forget to increase the allocated memory and swap-storage of docker, otherwise TensorFlow will get killed
### Build
Build CPU version:
`docker build -f ./Dockerfile -t mstoelzle/dt-object-detection-training:latest .`
Build GPU version:
`docker build --build-arg GPU=-gpu -f ./Dockerfile -t mstoelzle/dt-object-detection-training:latest-gpu .`
### Run
Place `mscoco_train.record` into your local `data` directory, then run:

`docker run -it -p 8888:8888 -p 6006:6006 -v YOUR_LOCAL_DATA_DIR/dt-object-detection-training:/src/dt-object-detection-training/data mstoelzle/dt-object-detection-training:latest`

Run with volume sharing (Maxi)

`docker run -it -p 8888:8888 -p 6006:6006 -v /Users/maximilianstoelzle/Documents/ethz/AMoD/data/dt-object-detection-training:/src/dt-object-detection-training/data mstoelzle/dt-object-detection-training:latest`

### Run container interactively
`docker run -it -p 8888:8888 -p 6006:6006 -v YOUR_LOCAL_DATA_DIR:/src/dt-object-detection-training/data mstoelzle/dt-object-detection-training:latest /bin/bash`

### Run on IDSC RUDOLF
Build:

`docker build --build-arg GPU=-gpu -f ./Dockerfile -t mstoelzle/dt-object-detection-training:latest-gpu .`

Push to Docker Hub:

`docker push mstoelzle/dt-object-detection-training:latest-gpu`

Copy `mscoco_train.record` into `data` directory:

`scp -r YOUR_LOCAL_DATA_DIR/dt-object-detection-training lfivop-ml@idsc-rudolf.ethz.ch:/home/lfivop-ml/data`

SSH into IDSC Rudolf:

`ssh lfivop-ml@idsc-rudolf.ethz.ch -L 6067:127.0.0.1:6067`

Specify number of GPUs:
`export CUDA_DEVICE_ORDER=PCI_BUS_ID && export CUDA_VISIBLE_DEVICES=1`

Run docker container:

`docker run -it -p 8888:8888 -p 6067:6067 -e TB_PORT=6067 -v /home/lfivop-ml/data/dt-object-detection-training:/src/dt-object-detection-training/data mstoelzle/dt-object-detection-training:latest-gpu`


## TensorBoard
While running the container, you can access the TensorBoard to survey the training progress:
http://localhost:6006/

if the training is running on Rudolf, you can access it here: http://localhost:6067/