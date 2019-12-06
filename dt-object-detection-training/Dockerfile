# Options:

# Set --build-arg TF_PACKAGE_VERSION=1.11.0rc0 to install a specific version.
# Installs the version 1.15.0 by default.
ARG TF_PACKAGE_VERSION="1.15.0"

# Set --build-arg GPU=-gpu to use gpu
# For the 1.15 release, CPU and GPU support are included in a single package
ARG GPU=""

# we use python 3
FROM tensorflow/tensorflow:${TF_PACKAGE_VERSION}${GPU}-py3 as base

# See http://bugs.python.org/issue19846
ENV LANG C.UTF-8
ENV TZ=Europe/Zurich

# we use python 3
ARG _PY_SUFFIX=3
ENV PYTHON=python${_PY_SUFFIX}
ARG PIP=pip${_PY_SUFFIX}

# prevent interactive questions
ARG DEBIAN_FRONTEND=noninteractive

# install tensorboard
RUN ${PIP} install "tensorboard"${TF_PACKAGE_VERSION:+==${TF_PACKAGE_VERSION}}

ARG REPO_NAME="dt-object-detection-training"
ENV REPO_PATH="/src/${REPO_NAME}"

ENV MODEL_NAME="ssdlite_mobilenet_v2_coco"

# create repo directory
RUN mkdir -p "${REPO_PATH}"

WORKDIR "${REPO_PATH}"

# copy requirements files
COPY ./requirements-apt.txt "${REPO_PATH}/"
COPY ./requirements-py.txt "${REPO_PATH}/"

# add sources for google coral edge-tpu
RUN APT_KEY_DONT_WARN_ON_DANGEROUS_USAGE=1 && \
    curl https://packages.cloud.google.com/apt/doc/apt-key.gpg | apt-key add - && \
    echo "deb https://packages.cloud.google.com/apt coral-edgetpu-stable main" | tee /etc/apt/sources.list.d/coral-edgetpu.list

# install apt dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    $(awk -F: '/^[^#]/ { print $1 }' "${REPO_PATH}/"requirements-apt.txt | uniq) \
  && rm -rf /var/lib/apt/lists/*

# install python dependencies
RUN ${PIP} install -r ${REPO_PATH}/requirements-py.txt

RUN jupyter serverextension enable --py jupyter_http_over_ws

# configure tensorflow
ENV TF_PATH="${REPO_PATH}/tf"

# Install tensorflow models object detection
COPY "tf/models"  "${TF_PATH}/models"
# RUN git clone https://github.com/tensorflow/models ${TF_PATH}/models
# we want to debug and change some stuff
# RUN git clone https://github.com/mstoelzle/tensorflow-models ${TF_PATH}/models
# WORKDIR ${TF_PATH}/models
# RUN git checkout "tags/v1.13.0"

ENV TF_MODELS_RESEARCH_PATH="${TF_PATH}/models/research"

# install COCO API
WORKDIR ${TF_PATH}
COPY "tf/cocoapi"  "${TF_PATH}/cocoapi"
# RUN git clone https://github.com/cocodataset/cocoapi.git "${TF_PATH}/cocoapi"

WORKDIR "${TF_PATH}/cocoapi/PythonAPI"
RUN make
RUN make install
RUN ${PYTHON} setup.py install
RUN cp -r pycocotools ${TF_MODELS_RESEARCH_PATH}

# From tensorflow/models/research/
WORKDIR ${TF_MODELS_RESEARCH_PATH}
RUN protoc object_detection/protos/*.proto --python_out=.
RUN export PYTHONPATH="$PYTHONPATH:${TF_MODELS_RESEARCH_PATH}:${TF_MODELS_RESEARCH_PATH}/slim:${TF_MODELS_RESEARCH_PATH}/object_detection"
RUN ${PYTHON} setup.py build
RUN ${PYTHON} setup.py install
WORKDIR ${TF_MODELS_RESEARCH_PATH}/slim
RUN ${PYTHON} setup.py install

RUN apt-get autoremove -y

WORKDIR ${TF_PATH}

RUN ${PYTHON} -m ipykernel.kernelspec

# install tutorials
# RUN mkdir -p ${TF_PATH}/tensorflow-tutorials && chmod -R a+rwx ${TF_PATH}/
# RUN mkdir /.local && chmod a+rwx /.local
# WORKDIR ${TF_PATH}/tensorflow-tutorials
# RUN wget https://raw.githubusercontent.com/tensorflow/docs/master/site/en/tutorials/keras/classification.ipynb
# RUN wget https://raw.githubusercontent.com/tensorflow/docs/master/site/en/tutorials/keras/overfit_and_underfit.ipynb
# RUN wget https://raw.githubusercontent.com/tensorflow/docs/master/site/en/tutorials/keras/regression.ipynb
# RUN wget https://raw.githubusercontent.com/tensorflow/docs/master/site/en/tutorials/keras/save_and_load.ipynb
# RUN wget https://raw.githubusercontent.com/tensorflow/docs/master/site/en/tutorials/keras/text_classification.ipynb
# RUN wget https://raw.githubusercontent.com/tensorflow/docs/master/site/en/tutorials/keras/text_classification_with_hub.ipynb
# RUN wget https://raw.githubusercontent.com/tensorflow/models/master/research/object_detection/object_detection_tutorial.ipynb


WORKDIR ${REPO_PATH}

# copy the source code
# COPY . "${REPO_PATH}/"
COPY "config/"  "${REPO_PATH}/config/"
COPY "launch/"  "${REPO_PATH}/launch/"
COPY "lib/"  "${REPO_PATH}/lib/"

ENV REPO_LIB_PATH="${REPO_PATH}/lib" \
    REPO_CONFIG_PATH="${REPO_PATH}/config" \
    WORKDIR_PATH="/workdir"

ENV CONFIG_WORKDIR_PATH="${WORKDIR_PATH}/config" \
    DATA_WORKDIR_PATH="${WORKDIR_PATH}/data" \
    EXPORT_WORKDIR_PATH="${WORKDIR_PATH}/export" \
    MODELS_WORKDIR_PATH="${WORKDIR_PATH}/models" \
    RAW_DATA_WORKDIR_PATH="${WORKDIR_PATH}/raw_data"


# define launch scripts
ENV WORKDIR_INIT="${REPO_PATH}/launch/workdir_init.sh" \
    TRAINING_LAUNCHFILE="${REPO_PATH}/launch/training.sh" \
    INFERENCE_GRAPH_EXPORT_LAUNCHFILE="${REPO_PATH}/launch/inference_graph_export.sh" \
    INFERENCE_GRAPH_EDGETPU_EXPORT_LAUNCHFILE="${REPO_PATH}/launch/inference_graph_edgetpu_export.sh" \
    TENSORBOARD_LAUNCHFILE="${REPO_PATH}/launch/tensorboard.sh" \
    DATASET_PREPARATION_LAUNCHFILE="${REPO_PATH}/launch/dataset_preparation.sh" \
    LOCATIONAL_WEIGHTS_GEN_LAUNCHFILE="${REPO_PATH}/launch/locational_weights_gen.sh" \
    LOCATIONAL_WEIGHTS_VIZ_LAUNCHFILE="${REPO_PATH}/launch/locational_weights_viz.sh"
RUN chmod a+rwx ${WORKDIR_INIT} \
                ${TRAINING_LAUNCHFILE} \
                ${INFERENCE_GRAPH_EXPORT_LAUNCHFILE} \
                ${INFERENCE_GRAPH_EDGETPU_EXPORT_LAUNCHFILE} \
                ${TENSORBOARD_LAUNCHFILE} \
                ${DATASET_PREPARATION_LAUNCHFILE} \
                ${LOCATIONAL_WEIGHTS_GEN_LAUNCHFILE} \
                ${LOCATIONAL_WEIGHTS_VIZ_LAUNCHFILE}

# port for tensorboard
ENV TB_PORT 6006

# CUDA GPU config
ENV CUDA_DEVICE_ORDER="PCI_BUS_ID" \
    CUDA_VISIBLE_DEVICES=1

# Define splitting of dataset between training, validation and test set
ENV VAL_1_OF_N_IMAGES=10 \
    TEST_1_OF_N_IMAGES=10

# hostname of duckiebot which was used to generate camera calibration files for locational weights
ENV DUCKIEBOT_CALIBRATION_HOSTNAME='default' \
    LOCATIONAL_WEIGHTS_JSON_FILENAME='dt_locational_weights'

# training settings
ENV NUM_TRAIN_STEPS=50000

# export inference graph settings
ENV CHECKPOINT_NUMBER=0

# needed if jupyter is used
# EXPOSE 8888

# define command
# CMD ["bash", "-c", "source /etc/bash.bashrc &&  && jupyter notebook --notebook-dir=${TF_PATH} --ip 0.0.0.0 --no-browser --allow-root"]
CMD ["bash", "-c", "source /etc/bash.bashrc && ${TRAINING_LAUNCHFILE}"]

# maintainer
LABEL maintainer="Maximilian Stoelzle (maximilian@stoelzle.ch)"